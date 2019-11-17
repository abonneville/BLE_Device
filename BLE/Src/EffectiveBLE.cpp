/*
 * Copyright (C) 2019 Andrew Bonneville.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */


#include <EffectiveBLE.hpp>
#include <cstring>
#include <cstdint>
#include <algorithm>

// WPAN Core files
#include "ble_defs.h"
#include "dbg_trace.h"
#include "ble.h"
#include "tl.h"
#include "stm32_seq.h"
#include "shci.h"
#include "stm32_lpm.h"
#include "otp.h"
#include "common_blesvc.h"
extern "C" {
#include "hci_tl.h"
}

using namespace ble;

/* Typedef -----------------------------------------------------------*/
typedef struct
{
	char localName[AD_TYPE_COMPLETE_LOCAL_NAME + 1];
	uint8_t nameLength;
	uint16_t advIntervalMin;
	uint16_t advIntervalMax;

}Adv_t;



/* Define ------------------------------------------------------------*/

//TODO delete or replace
#define CFG_MAX_CONNECTION                8
#define BLE_DBG_SVCCTL_MSG std::printf
#define BLE_DBG_P2P_STM_MSG std::printf


/* Macro -------------------------------------------------------------*/

/* Variables ---------------------------------------------------------*/
static bool hasInitialized = false;
static Adv_t advParams {};
static std::array<GattHandler_t, 10> gattHandles {};
static std::size_t count {};



/**
 * Advertising Data
 */
static uint8_t manuf_data[14] = {
    sizeof(manuf_data)-1, AD_TYPE_MANUFACTURER_SPECIFIC_DATA,
    0x01/*SKD version */,
    CFG_DEV_ID_P2P_ROUTER /* STM32WB - P2P Router*/,
    0x00 /* GROUP A Feature  */,
    0x00 /* GROUP A Feature */,
    0x00 /* GROUP B Feature */,
    0x00 /* GROUP B Feature */,
    0x00, /* BLE MAC start -MSB */
    0x00,
    0x00,
    0x00,
    0x00,
    0x00, /* BLE MAC stop */
};

/* Function prototypes -----------------------------------------------*/
static size_t safe_strlen(const char *str, size_t max_len);
static void Adv_Request( void );
static SVCCTL_EvtAckStatus_t EventHandler(void *Event);
static ble::GattHandle_t addService(const ble::Uuid128 & uuid, size_t uuidSize, size_t quantity);
static ble::GattHandle_t addCharacteristic(const ble::Uuid128 & uuid, size_t uuidSize, uint16_t dataSize, ble::GattHandle_t serviceHandle);
static void SendNotification(GattHandle_t charHandle, uint8_t *payload, size_t size );
static void init(void);






/* External functions ------------------------------------------------*/



class EffectiveBLE::impl
{
public:
	impl(const char * name, uint16_t interval ) :
		advName(name),
		advInterval(interval) {};


	/**
	 * @brief Setup the ability to advertise device capability
	 * @param advName local name to advertise with
	 * @param advInterval advertising interval between packets, in mS
	 */
	void advertise()
	{

		// TODO add check to verify connection not already established


		/* advInternal, convert mS into multiples of 625uS */
		uint32_t advInterval = this->advInterval;
		advInterval = ( advInterval * 1000 ) / 625;
		advInterval = advInterval < ADV_INTERVAL_LOWEST_CONN ? ADV_INTERVAL_LOWEST_CONN : advInterval;
		advInterval = advInterval > ADV_INTERVAL_HIGHEST ? ADV_INTERVAL_HIGHEST : advInterval;

		advParams.advIntervalMin = (uint16_t)advInterval;

		// To minimize interference from WiFi, set a max interval
		advParams.advIntervalMax = (uint16_t)(advInterval * 1333 / 1000);
		advParams.advIntervalMax = advParams.advIntervalMax > ADV_INTERVAL_HIGHEST ?
				ADV_INTERVAL_HIGHEST : advParams.advIntervalMax;


		/* Local name is limited to 9-bytes (without null), truncate as required*/
		auto advName = this->advName;
		uint8_t length = (uint8_t)safe_strlen(advName, 10);
		if (length < 10)
		{
			advParams.localName[0] = AD_TYPE_COMPLETE_LOCAL_NAME;
			std::memcpy(&advParams.localName[1], advName, length);
			advParams.nameLength = (uint8_t)(length + 1);
		}
		else
		{
			advParams.localName[0] = AD_TYPE_SHORTENED_LOCAL_NAME;
			std::memcpy(&advParams.localName[1], advName, 9);
			advParams.nameLength = 9 + 1;
		}

		/**
		* Place advertising task in pending state, ready to run
		*/
		UTIL_SEQ_RegTask( 1<<CFG_TASK_START_ADV_ID, UTIL_SEQ_RFU, Adv_Request);
		UTIL_SEQ_SetTask(1 << CFG_TASK_START_ADV_ID, CFG_SCH_PRIO_0);
	}


	const char * advName;
	uint16_t advInterval;
};


/**
 * @brief Constructor
 */
EffectiveBLE::EffectiveBLE(const char * name, uint16_t interval) :
		pimpl{ std::make_unique<impl>(name, interval) }
		{}

EffectiveBLE::~EffectiveBLE()
{}


/**
 * @brief initializes and starts the BLE device
 */
void EffectiveBLE::begin()
{
	// TODO - non-blocking or add timeout? Or change priority so advertising is lower priority than initialization?
	while (hasInitialized == false)
	{
	    UTIL_SEQ_Run( UTIL_SEQ_DEFAULT );
	}


	init();

	pimpl->advertise();
}


/**
 * @brief Insert the requested GATT object into a central lookup table
 * @param service is the service UUID that the characteristic belongs to
 * @param serviceSize is how many bytes long the service UUID is
 * @param characteristic is the characteristic UUID for this specific object
 * @param characterisitcSize is how many bytes long the characteristic UUID is
 * @param dataSize is how many bytes long the data/field value is for this specific characteristic
 * @retval pointer to this objects entry in the GATT lookup table, read/writable.
 */
GattHandler_t * EffectiveBLE::addObject(
		const Uuid128 & service,
		size_t serviceSize,
		const Uuid128 & characteristic,
		size_t characteristicSize,
		uint8_t dataSize )
{

	if ( count < gattHandles.size() )
	{
		gattHandles[count].srvID = service;
		gattHandles[count].srvIDSize = serviceSize;
		gattHandles[count].charID = characteristic;
		gattHandles[count].charIDSize = characteristicSize;
		gattHandles[count].dataSize = dataSize;
		count++;
		return &gattHandles[count - 1];
	}

	return nullptr;
}


/**
 * @brief Populate GATT database with service and characteristic entries.
 */
static void init(void)
{

	SVCCTL_RegisterSvcHandler(EventHandler);

	/* GATT characteristic handles are acquired under a specific service UUID. Before
	 * requesting handles, we need to group by service UUID, and count how many
	 * characteristics belong to each service UUID.
	 */
	std::sort(gattHandles.data(), gattHandles.data() + count,
			[](const GattHandler_t & a, const GattHandler_t & b) -> bool
			{ return a.srvID < b.srvID; });

	for ( size_t counter = 0; counter < count; )
	{
		auto & service = gattHandles[counter].srvID;;
		auto quantity = std::count_if(gattHandles.data(), gattHandles.data() + count,
				[service](const GattHandler_t & a) -> bool
				{ return a.srvID == service; });

		/* Acquire a service handle */
		auto length = gattHandles[counter].srvIDSize;
		ble::GattHandle_t serviceHandle = addService(service, length, quantity);

		if (serviceHandle)
		{
			/* Acquire a characteristic handle(s) */
			for (auto item = 0; item < quantity; item++)
			{
				auto & gh = gattHandles[counter + item];
				gh.serviceHandle = serviceHandle;
				gh.charHandle = addCharacteristic(gh.charID, gh.charIDSize, gh.dataSize, serviceHandle);
				if (gh.userObject)
				{
					/* Assign handle to user object to facilitate client notification. */
					gh.userObject->setGattHandle(gh.charHandle, SendNotification );
				}
			}
		}

		counter += quantity;
	}


}

/**
 * @brief Request adding service UUID to GATT database
 * @param uuid to be added to GATT database
 * @param uuidSize how many bytes long is the UUID
 * @param quantity is how many GATT characteristic handles to be reserved in database
 * @retval GATT handle to database entry, 0 if invalid request
 */
static ble::GattHandle_t addService(const ble::Uuid128 & uuid, size_t uuidSize, size_t quantity)
{
	tBleStatus status = !BLE_STATUS_SUCCESS;
	ble::GattHandle_t handle = 0;

	/* Convert characteristic quantity into how many handles need to be reserved */
	quantity = ( quantity > 127 ) ? 0 : quantity;
	quantity *= 2; /* Each characteristic consumes 2 handles */
	quantity += 1; /* Each service will consume 1 handle */

	/* The UUIDs are stored in a byte array with MSB first. However, the BLE standard requires
	 * LSB (little endian) be sent first. Make a copy (protect original) and load into GATT database.
	 */
	ble::Uuid128 uuidReversed;
	std::reverse_copy( uuid.cbegin(), uuid.cbegin() + uuidSize, uuidReversed.begin() );

	uint8_t uuidType = (uuidSize == 2) ? UUID_TYPE_16 : UUID_TYPE_128;

	status = aci_gatt_add_service(
				uuidType,
				(Service_UUID_t *) uuidReversed.data(),
				PRIMARY_SERVICE,
				(uint8_t)quantity,
				&handle
				);

	return (status == BLE_STATUS_SUCCESS ? handle : 0);
}



/**
 * @brief Request adding characteristic UUID to GATT database
 * @param uuid to be added to GATT database
 * @param uuidSize how many bytes long is the UUID
 * @param dataSize is how many bytes long is the data value
 * @retval GATT handle to database entry, 0 if invalid request
 */
static ble::GattHandle_t addCharacteristic(const ble::Uuid128 & uuid, size_t uuidSize, uint16_t dataSize, ble::GattHandle_t serviceHandle)
{
	tBleStatus status = !BLE_STATUS_SUCCESS;
	ble::GattHandle_t handle = 0;

	/* The UUIDs are stored in a byte array with MSB first. However, the BLE standard requires
	 * LSB (little endian) be sent first. Make a copy (protect original) and load into GATT database.
	 */
	ble::Uuid128 uuidReversed;
	std::reverse_copy( uuid.cbegin(), uuid.cbegin() + uuidSize, uuidReversed.begin() );

	uint8_t uuidType = (uuidSize == 2) ? UUID_TYPE_16 : UUID_TYPE_128;

	status = aci_gatt_add_char(
				serviceHandle,
				uuidType,
				(Char_UUID_t *) uuidReversed.data(),
				dataSize,
				CHAR_PROP_WRITE_WITHOUT_RESP|CHAR_PROP_READ,
				ATTR_PERMISSION_NONE,
				GATT_NOTIFY_ATTRIBUTE_WRITE, /* gattEvtMask */
				10, /* encryKeySize */
				1, /* isVariable */
				&handle );

	return (status == BLE_STATUS_SUCCESS ? handle : 0);
}


/**
 * @brief determines the length of a null terminated string, up to max number of characters
 * @param str character string to be analyzed
 * @param max_len limit on how many characters to check for a null-terminator
 * @retval length of null-terminated character string, or max_len if not terminated
 */
static size_t safe_strlen(const char *str, size_t max_len)
{
    const char * end = (const char *)memchr(str, '\0', max_len);
    if (end == NULL)
        return max_len;
    else
        return end - str;
}


/**
 * @brief  Initiate advertising to a new client
 */
static void Adv_Request( void )
{

	/*Start Advertising*/
	tBleStatus result = aci_gap_set_discoverable(
    		ADV_IND,
    		advParams.advIntervalMin,
			advParams.advIntervalMax,
			PUBLIC_ADDR,
			NO_WHITE_LIST_USE, /* use white list */
			advParams.nameLength,
			(uint8_t*)advParams.localName,
			0,
			NULL,
			0,
			0);

	if ( result == BLE_STATUS_SUCCESS )
	{
		/* Send Advertising data */
		result = aci_gap_update_adv_data(sizeof(manuf_data), (uint8_t*) manuf_data);
	}

	if (result == BLE_STATUS_SUCCESS)
	{
	  APP_DBG_MSG("\r\nInfo: Started advertising\r\n");
	}
	else
	{
	  APP_DBG_MSG("\r\nFail: Advertising - [%s][%d][%s]\r\n", __FUNCTION__,__LINE__, __FILE__);
	}
}


/**
 * @brief Update characteristic value and send notification to client(s)
 * @param charHandle identifies which characteristic to send notification under
 * @param buffer is the data to be sent
 * @param size is the amount of data to be sent
 */
static void SendNotification(GattHandle_t charHandle, uint8_t *payload, size_t size )
{
	tBleStatus result = !BLE_STATUS_SUCCESS;

	auto gh = gattHandles;

	/* Find matching characteristic */
	auto entry = std::find_if(gh.begin(), gh.end(),
			[charHandle](const GattHandler_t & gatt) { return gatt.charHandle == charHandle; } );

	if ( ( entry != gh.end() ) &&
		 ( entry->dataSize == size) )
	{
		/* Send/publish notification of value change*/
		result = aci_gatt_update_char_value(
								entry->serviceHandle,
	                            charHandle,
	                             0, /* charValOffset */
	                            entry->dataSize, /* charValueLen */
	                            payload);
	}

	if ( result != BLE_STATUS_SUCCESS )
	{
	      BLE_DBG_SVCCTL_MSG(" -- Notification of value change failed.\n");
	}
}















/**************************************************************************************************
 * Below this line is the legacy STM32 WPAN
 */


#define APPBLE_GAP_DEVICE_NAME_LENGTH 7

#define BD_ADDR_SIZE_LOCAL    6



typedef enum
{
  P2P_SERVER1_CONN_HANDLE_EVT,
  P2P_SERVER1_DISCON_HANDLE_EVT,
  SMART_PHONE1_CONN_HANDLE_EVT,
  SMART_PHONE1_DISCON_HANDLE_EVT,
} P2P_Opcode_Notification_evt_t;



/**
 * security parameters structure
 */
typedef struct _tSecurityParams
{
  /**
   * IO capability of the device
   */
  uint8_t ioCapability;

  /**
   * Authentication requirement of the device
   * Man In the Middle protection required?
   */
  uint8_t mitm_mode;

  /**
   * bonding mode of the device
   */
  uint8_t bonding_mode;

  /**
   * Flag to tell whether OOB data has
   * to be used during the pairing process
   */
  uint8_t OOB_Data_Present;

  /**
   * OOB data to be used in the pairing process if
   * OOB_Data_Present is set to TRUE
   */
  uint8_t OOB_Data[16];

  /**
   * this variable indicates whether to use a fixed pin
   * during the pairing process or a passkey has to be
   * requested to the application during the pairing process
   * 0 implies use fixed pin and 1 implies request for passkey
   */
  uint8_t Use_Fixed_Pin;

  /**
   * minimum encryption key size requirement
   */
  uint8_t encryptionKeySizeMin;

  /**
   * maximum encryption key size requirement
   */
  uint8_t encryptionKeySizeMax;

  /**
   * fixed pin to be used in the pairing process if
   * Use_Fixed_Pin is set to 1
   */
  uint32_t Fixed_Pin;

  /**
   * this flag indicates whether the host has to initiate
   * the security, wait for pairing or does not have any security
   * requirements.\n
   * 0x00 : no security required
   * 0x01 : host should initiate security by sending the slave security
   *        request command
   * 0x02 : host need not send the clave security request but it
   * has to wait for paiirng to complete before doing any other
   * processing
   */
  uint8_t initiateSecurity;
} tSecurityParams;

/**
 * global context
 * contains the variables common to all
 * services
 */
typedef struct _tBLEProfileGlobalContext
{

  /**
   * security requirements of the host
   */
  tSecurityParams bleSecurityParam;

  /**
   * gap service handle
   */
  uint16_t gapServiceHandle;

  /**
   * device name characteristic handle
   */
  uint16_t devNameCharHandle;

  /**
   * appearance characteristic handle
   */
  uint16_t appearanceCharHandle;

  /**
   * connection handle of the current active connection
   * When not in connection, the handle is set to 0xFFFF
   */
  uint16_t connectionHandle[CFG_MAX_CONNECTION];

  /**
   * length of the UUID list to be used while advertising
   */
  uint8_t advtServUUIDlen;

  /**
   * the UUID list to be used while advertising
   */
  uint8_t advtServUUID[100];

} BleGlobalContext_t;

typedef enum
    {
      APP_BLE_IDLE,
      APP_BLE_FAST_ADV,
      APP_BLE_LP_ADV,
      APP_BLE_SCAN,
      APP_BLE_CONNECTING,
      APP_BLE_CONNECTED,

      APP_BLE_DISCOVER_SERVICES,
      APP_BLE_DISCOVER_CHARACS,
      APP_BLE_DISCOVER_LED_CHAR_DESC,
      APP_BLE_DISCOVER_BUTTON_CHAR_DESC,
      APP_BLE_DISCOVER_NOTIFICATION_CHAR_DESC,
      APP_BLE_ENABLE_NOTIFICATION_BUTTON_DESC,
      APP_BLE_DISABLE_NOTIFICATION_TX_DESC
    } APP_BLE_ConnStatus_t;


typedef struct
{
  BleGlobalContext_t BleApplicationContext_legacy;
  /**
   * used to identify the GAP State
   */
  APP_BLE_ConnStatus_t SmartPhone_Connection_Status;

  /**
   * used to identify the GAP State
   */
  APP_BLE_ConnStatus_t EndDevice_Connection_Status[6];
  /**
   * connection handle with the Central connection (Smart Phone)
   * When not in connection, the handle is set to 0xFFFF
   */
  uint16_t connectionHandleCentral;
  /**
   * connection handle with the Server 1 connection (End Device 1)
   * When not in connection, the handle is set to 0xFFFF
   */
  uint16_t connectionHandleEndDevice1;

  /**
   * used when doing advertising to find end device 1
   */
  uint8_t EndDevice1Found;

} BleApplicationContext_t;

typedef struct
{
  P2P_Opcode_Notification_evt_t P2P_Evt_Opcode;
  uint16_t ConnectionHandle;

} P2P_ConnHandle_Not_evt_t;



typedef struct
{
  uint16_t Connection_Handle;
  uint8_t  Identifier;
  uint16_t L2CAP_Length;
  uint16_t Interval_Min;
  uint16_t Interval_Max;
  uint16_t Slave_Latency;
  uint16_t Timeout_Multiplier;
} APP_BLE_p2p_Conn_Update_req_t;







static void Ble_Tl_Init( void );
static void Ble_Hci_Gap_Gatt_Init(void);
static void BLE_StatusNot( HCI_TL_CmdStatus_t status );
static void BLE_UserEvtRx( void * pPayload );
const uint8_t* BleGetBdAddress( void );

P2P_ConnHandle_Not_evt_t handleNotification;

PLACE_IN_SECTION("MB_MEM1") ALIGN(4) static TL_CmdPacket_t BleCmdBuffer;
PLACE_IN_SECTION("BLE_APP_CONTEXT") APP_BLE_p2p_Conn_Update_req_t APP_BLE_p2p_Conn_Update_req;

uint16_t connection_handle;

PLACE_IN_SECTION("BLE_APP_CONTEXT") static BleApplicationContext_t BleApplicationContext;



static const uint8_t M_bd_addr[BD_ADDR_SIZE_LOCAL] =
    {
        (uint8_t)((CFG_ADV_BD_ADDRESS & 0x0000000000FF)),
        (uint8_t)((CFG_ADV_BD_ADDRESS & 0x00000000FF00) >> 8),
        (uint8_t)((CFG_ADV_BD_ADDRESS & 0x000000FF0000) >> 16),
        (uint8_t)((CFG_ADV_BD_ADDRESS & 0x0000FF000000) >> 24),
        (uint8_t)((CFG_ADV_BD_ADDRESS & 0x00FF00000000) >> 32),
        (uint8_t)((CFG_ADV_BD_ADDRESS & 0xFF0000000000) >> 40)
    };


static uint8_t bd_addr_udn[BD_ADDR_SIZE_LOCAL];

/**
*   Identity root key used to derive LTK and CSRK
*/
static const uint8_t BLE_CFG_IR_VALUE[16] = CFG_BLE_IRK;

/**
* Encryption root key used to derive LTK and CSRK
*/
static const uint8_t BLE_CFG_ER_VALUE[16] = CFG_BLE_ERK;


/**
 * BD Address of SERVER1 & SERVER 2 - to be connected once discovered
 */
tBDAddr P2P_SERVER1_BDADDR;


/**
 * @brief On receive of the System ready event from CPU2, this method is called to
 * initialize the application layer.
 */
extern "C" void APP_BLE_Init( void )
{
  SHCI_C2_Ble_Init_Cmd_Packet_t ble_init_cmd_packet =
  {
    {{0,0,0}},                          /**< Header unused */
    {0,                                 /** pBleBufferAddress not used */
    0,                                  /** BleBufferSize not used */
    CFG_BLE_NUM_GATT_ATTRIBUTES,
    CFG_BLE_NUM_GATT_SERVICES,
    CFG_BLE_ATT_VALUE_ARRAY_SIZE,
    CFG_BLE_NUM_LINK,
    CFG_BLE_DATA_LENGTH_EXTENSION,
    CFG_BLE_PREPARE_WRITE_LIST_SIZE,
    CFG_BLE_MBLOCK_COUNT,
    CFG_BLE_MAX_ATT_MTU,
    CFG_BLE_SLAVE_SCA,
    CFG_BLE_MASTER_SCA,
    CFG_BLE_LSE_SOURCE,
    CFG_BLE_MAX_CONN_EVENT_LENGTH,
    CFG_BLE_HSE_STARTUP_TIME,
    CFG_BLE_VITERBI_MODE,
    CFG_BLE_LL_ONLY,
    0}
  };


  /**
   * Initialize Ble Transport Layer
   */
  Ble_Tl_Init( );

  /**
   * Do not allow standby in the application
   */
  UTIL_LPM_SetOffMode(1 << CFG_LPM_APP_BLE, UTIL_LPM_DISABLE);

/**
   * Register the hci transport layer to handle BLE User Asynchronous Events
   */
 UTIL_SEQ_RegTask( 1<<CFG_TASK_HCI_ASYNCH_EVT_ID, UTIL_SEQ_RFU, hci_user_evt_proc);

  /**
   * Starts the BLE Stack on CPU2
   */
  SHCI_C2_BLE_Init( &ble_init_cmd_packet );

  /**
   * Initialization of HCI & GATT & GAP layer
   */
  Ble_Hci_Gap_Gatt_Init();

  /**
   * Initialization of the BLE Services
   */
  SVCCTL_Init();

  /**
   * From here, all initialization are BLE application specific
   */

  /**
   * Initialization of the BLE App Context
   */
  BleApplicationContext.SmartPhone_Connection_Status = APP_BLE_IDLE;
  BleApplicationContext.EndDevice_Connection_Status[0] = APP_BLE_IDLE;
  BleApplicationContext.EndDevice1Found = 0x00;

  /*
   * CPU2 and BLE stack are now initialized
   */
  hasInitialized = true;

  return;
}

/**
 * @brief Event handler
 * @param Event address of the buffer holding the event
 * @retval indicates if the Event has been handled or not
 */
static SVCCTL_EvtAckStatus_t EventHandler(void *Event)
{
	/**
	 * This is where the GATT layer informs of client updates; writing to characteristic value(s), changing
	 * notification (enable/disable), etc...
	 */
	printf("\r\n [%s][%s][%d] \r\n",__FILE__,__FUNCTION__,__LINE__);

	SVCCTL_EvtAckStatus_t retval = SVCCTL_EvtNotAck;
	hci_event_pckt *event_pckt;
	evt_blue_aci *blue_evt;
	aci_gatt_attribute_modified_event_rp0    * attribute_modified;

	event_pckt = (hci_event_pckt *)(((hci_uart_pckt*)Event)->data);

	switch(event_pckt->evt)
	{
		case EVT_VENDOR:
		{
			blue_evt = (evt_blue_aci*)event_pckt->data;
			switch(blue_evt->ecode)
			{
				case EVT_BLUE_GATT_ATTRIBUTE_MODIFIED:
				{
					attribute_modified = (aci_gatt_attribute_modified_event_rp0*)blue_evt->data;

					/* Locate a matching characteristic handle and set the value in memory. */
					for (size_t index = 0; index < count; index++ )
					{
						auto & gh = gattHandles[index];
						if ( gh.charHandle == ( attribute_modified->Attr_Handle - 1) )
						{
							if (gh.dataSize == attribute_modified->Attr_Data_Length )
							{
								BLE_DBG_P2P_STM_MSG("-- GATT : Attribute modified, invoke callback \n");
								gh.userObject->setValue(attribute_modified->Attr_Data);
							}
							else
							{
								/* Invalid length, discard data packet */
								BLE_DBG_P2P_STM_MSG("-- GATT : Attribute modified, short packet discarded \n");
							}

							/* Match found, event handled */
							retval = SVCCTL_EvtAckFlowEnable;
							break;
						}
					}
				}
				break;

			default:
				break;
			}
		}
		break; /* EVT_VENDOR */

		default:
			break;
	}

	return(retval);
}




extern "C" void hci_notify_asynch_evt(void* pdata)
{
	pdata = pdata;
	UTIL_SEQ_SetTask(1 << CFG_TASK_HCI_ASYNCH_EVT_ID, CFG_SCH_PRIO_0);
}

extern "C" void hci_cmd_resp_release(uint32_t flag)
{
	flag = flag;
	UTIL_SEQ_SetEvt(1 << CFG_IDLEEVT_HCI_CMD_EVT_RSP_ID);
}

extern "C" void hci_cmd_resp_wait(uint32_t timeout)
{
	timeout = timeout;
	UTIL_SEQ_WaitEvt(1 << CFG_IDLEEVT_HCI_CMD_EVT_RSP_ID);
}


/**
 * @brief GAP event handler
 *
 */
extern "C" SVCCTL_UserEvtFlowStatus_t SVCCTL_App_Notification(void *pckt)
{
  hci_event_pckt *event_pckt;
  evt_le_meta_event *meta_evt;
  evt_blue_aci *blue_evt;
  hci_le_advertising_report_event_rp0 * le_advertising_event;
  event_pckt = (hci_event_pckt*) ((hci_uart_pckt *) pckt)->data;
  uint8_t result;
  uint8_t event_type, event_data_size;
  int k = 0;
  uint8_t *adv_report_data;
  uint8_t adtype, adlength;

  switch (event_pckt->evt)
  {
    /* USER CODE BEGIN evt */

    /* USER CODE END evt */
    case EVT_VENDOR:
    {
      handleNotification.P2P_Evt_Opcode = P2P_SERVER1_DISCON_HANDLE_EVT;
      blue_evt = (evt_blue_aci*) event_pckt->data;
      /* USER CODE BEGIN EVT_VENDOR */

      /* USER CODE END EVT_VENDOR */
      switch (blue_evt->ecode)
      {
      /* USER CODE BEGIN ecode */

      /* USER CODE END ecode */
        case EVT_BLUE_GAP_PROCEDURE_COMPLETE:
        {
          /* USER CODE BEGIN EVT_BLUE_GAP_PROCEDURE_COMPLETE */

          /* USER CODE END EVT_BLUE_GAP_PROCEDURE_COMPLETE */
          aci_gap_proc_complete_event_rp0 *gap_evt_proc_complete = (aci_gap_proc_complete_event_rp0 *) (void*) blue_evt->data;
          /* CHECK GAP GENERAL DISCOVERY PROCEDURE COMPLETED & SUCCEED */
          if (gap_evt_proc_complete->Procedure_Code == GAP_GENERAL_DISCOVERY_PROC
              && gap_evt_proc_complete->Status == 0x00)
          {
              /* USER CODE BEGIN GAP_GENERAL_DISCOVERY_PROC */

              /* USER CODE END GAP_GENERAL_DISCOVERY_PROC */

            APP_DBG_MSG("-- GAP GENERAL DISCOVERY PROCEDURE_COMPLETED\n");
            /*if a device found, connect to it, device 1 being chosen first if both found*/
            if (BleApplicationContext.EndDevice1Found == 0x01
                && BleApplicationContext.EndDevice_Connection_Status[0] != APP_BLE_CONNECTED)
            {
              UTIL_SEQ_SetTask(1 << CFG_TASK_CONN_DEV_1_ID, CFG_SCH_PRIO_0);
            }
          }

        }
        break; /* EVT_BLUE_GAP_PAIRING_CMPLT */

        case EVT_BLUE_L2CAP_CONNECTION_UPDATE_REQ:
        {
         /* USER CODE BEGIN EVT_BLUE_L2CAP_CONNECTION_UPDATE_REQ */

          /* USER CODE END EVT_BLUE_L2CAP_CONNECTION_UPDATE_REQ */
          aci_l2cap_connection_update_req_event_rp0 *pr = (aci_l2cap_connection_update_req_event_rp0 *) blue_evt->data;
          APP_BLE_p2p_Conn_Update_req.Connection_Handle = pr->Connection_Handle;
          APP_BLE_p2p_Conn_Update_req.Identifier = pr->Identifier;
          APP_BLE_p2p_Conn_Update_req.L2CAP_Length = pr->L2CAP_Length;
          APP_BLE_p2p_Conn_Update_req.Interval_Min = pr->Interval_Min;
          APP_BLE_p2p_Conn_Update_req.Interval_Max = pr->Interval_Max;
          APP_BLE_p2p_Conn_Update_req.Slave_Latency = pr->Slave_Latency;
          APP_BLE_p2p_Conn_Update_req.Timeout_Multiplier = pr->Timeout_Multiplier;

          result = aci_l2cap_connection_parameter_update_resp(APP_BLE_p2p_Conn_Update_req.Connection_Handle,
                                                              APP_BLE_p2p_Conn_Update_req.Interval_Min,
                                                              APP_BLE_p2p_Conn_Update_req.Interval_Max,
                                                              APP_BLE_p2p_Conn_Update_req.Slave_Latency,
                                                              APP_BLE_p2p_Conn_Update_req.Timeout_Multiplier,
                                                              CONN_L1,
                                                              CONN_L2,
                                                              APP_BLE_p2p_Conn_Update_req.Identifier,
                                                              0x00);
          APP_DBG_MSG("\r\n\r** NO UPDATE \n");
          if(result != BLE_STATUS_SUCCESS) {
              /* USER CODE BEGIN BLE_STATUS_SUCCESS */

              /* USER CODE END BLE_STATUS_SUCCESS */
          }

        }

        break;

        default:
          /* USER CODE BEGIN ecode_default */

          /* USER CODE END ecode_default */
          break;
      }
    }
    break; /* EVT_VENDOR */

    case EVT_DISCONN_COMPLETE:
        APP_DBG_MSG("-- Disconnected from client.\n");
        //TODO temp until disconnect callback is in place
        UTIL_SEQ_SetTask(1 << CFG_TASK_START_ADV_ID, CFG_SCH_PRIO_0);
      break; /* EVT_DISCONN_COMPLETE */

    case EVT_LE_META_EVENT:
      meta_evt = (evt_le_meta_event*) event_pckt->data;

      switch (meta_evt->subevent)
      {
        case EVT_LE_CONN_COMPLETE:
            APP_DBG_MSG("-- Connected to client.\n");
          break; /* HCI_EVT_LE_CONN_COMPLETE */

        case EVT_LE_ADVERTISING_REPORT:

          /* USER CODE BEGIN EVT_LE_ADVERTISING_REPORT */

          /* USER CODE END EVT_LE_ADVERTISING_REPORT */
          le_advertising_event = (hci_le_advertising_report_event_rp0 *) meta_evt->data;

          event_type = le_advertising_event->Advertising_Report[0].Event_Type;

          event_data_size = le_advertising_event->Advertising_Report[0].Length_Data;

          adv_report_data = (uint8_t*)(&le_advertising_event->Advertising_Report[0].Length_Data) + 1;
          k = 0;

          /* search AD TYPE 0x09 (Complete Local Name) */
          /* search AD Type 0x02 (16 bits UUIDS) */
          if (event_type == ADV_IND)
          {

            /* ISOLATION OF BD ADDRESS AND LOCAL NAME */

            while(k < event_data_size)
            {
              adlength = adv_report_data[k];
              adtype = adv_report_data[k + 1];
              switch (adtype)
              {
                case 0x01: /* now get flags */
                /* USER CODE BEGIN get_flags */

                /* USER CODE END get_flags */
                break;
                case 0x09: /* now get local name */
                /* USER CODE BEGIN get_local_name */

                /* USER CODE END get_local_name */
                  break;
                case 0x02: /* now get UID */
                /* USER CODE BEGIN get_UID */

                /* USER CODE END get_UID */
                  break;
                case 0x0A: /* Tx power level */
                /* USER CODE BEGIN Tx_power_level */

                /* USER CODE END Tx_power_level */
                  break;
                case 0xFF: /* Manufacturer Specific */
                /* USER CODE BEGIN Manufactureur_Specific */

                /* USER CODE END Manufactureur_Specific */
                  if (adlength >= 7 && adv_report_data[k + 2] == 0x01)
                  { /* ST VERSION ID 01 */
                    APP_DBG_MSG("--- ST MANUFACTURER ID --- \n");
                    switch (adv_report_data[k + 3])
                    {
                      case CFG_DEV_ID_P2P_SERVER1:
                        APP_DBG_MSG("-- P2P SERVER 1 DETECTED -- VIA MAN ID\n");
                        BleApplicationContext.EndDevice1Found = 0x01;
                        P2P_SERVER1_BDADDR[0] = le_advertising_event->Advertising_Report[0].Address[0];
                        P2P_SERVER1_BDADDR[1] = le_advertising_event->Advertising_Report[0].Address[1];
                        P2P_SERVER1_BDADDR[2] = le_advertising_event->Advertising_Report[0].Address[2];
                        P2P_SERVER1_BDADDR[3] = le_advertising_event->Advertising_Report[0].Address[3];
                        P2P_SERVER1_BDADDR[4] = le_advertising_event->Advertising_Report[0].Address[4];
                        P2P_SERVER1_BDADDR[5] = le_advertising_event->Advertising_Report[0].Address[5];
                        break;
                      default:
                    break;
                    }

                  }
                  break;
                case 0x16:
                  /* USER CODE BEGIN AD_TYPE_SERVICE_DATA */

                  /* USER CODE END AD_TYPE_SERVICE_DATA */
                  break;
                default:
                  /* USER CODE BEGIN adtype_default */

                  /* USER CODE END adtype_default */
                  break;
              }
              k += adlength + 1;
            }

          }

          break;

        default:
        	break;

      }

      break; /* HCI_EVT_LE_META_EVENT */

      default:
    	  break;
    }
  return (SVCCTL_UserEvtFlowEnable);
}


static void Ble_Tl_Init( void )
{
  HCI_TL_HciInitConf_t Hci_Tl_Init_Conf;

  Hci_Tl_Init_Conf.p_cmdbuffer = (uint8_t*)&BleCmdBuffer;
  Hci_Tl_Init_Conf.StatusNotCallBack = BLE_StatusNot;
  hci_init(BLE_UserEvtRx, (void*) &Hci_Tl_Init_Conf);

  return;
}

static void BLE_UserEvtRx( void * pPayload )
{
  SVCCTL_UserEvtFlowStatus_t svctl_return_status;
  tHCI_UserEvtRxParam *pParam;

  pParam = (tHCI_UserEvtRxParam *)pPayload;

  svctl_return_status = SVCCTL_UserEvtRx((void *)&(pParam->pckt->evtserial));
  if (svctl_return_status != SVCCTL_UserEvtFlowDisable)
  {
    pParam->status = HCI_TL_UserEventFlow_Enable;
  }
  else
  {
    pParam->status = HCI_TL_UserEventFlow_Disable;
  }
}


static void Ble_Hci_Gap_Gatt_Init(void){

  uint8_t role;
  uint8_t index;
  uint16_t gap_service_handle, gap_dev_name_char_handle, gap_appearance_char_handle;
  const uint8_t *bd_addr;
  uint32_t srd_bd_addr[2];
//TODO  uint16_t appearance[1] = { BLE_CFG_UNKNOWN_APPEARANCE };
  uint16_t appearance[1] = { 0 };

  /**
   * Initialize HCI layer
   */
  /*HCI Reset to synchronise BLE Stack*/
  hci_reset();

  /**
   * Write the BD Address
   */

  bd_addr = BleGetBdAddress();
  aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET,
                            CONFIG_DATA_PUBADDR_LEN,
                            (uint8_t*) bd_addr);
  /* BLE MAC in ADV Packet */
  manuf_data[ sizeof(manuf_data)-6] = bd_addr[5];
  manuf_data[ sizeof(manuf_data)-5] = bd_addr[4];
  manuf_data[ sizeof(manuf_data)-4] = bd_addr[3];
  manuf_data[ sizeof(manuf_data)-3] = bd_addr[2];
  manuf_data[ sizeof(manuf_data)-2] = bd_addr[1];
  manuf_data[ sizeof(manuf_data)-1] = bd_addr[0];


  /**
   * Write Identity root key used to derive LTK and CSRK
   */
    aci_hal_write_config_data(CONFIG_DATA_IR_OFFSET,
    CONFIG_DATA_IR_LEN,
                            (uint8_t*) BLE_CFG_IR_VALUE);

   /**
   * Write Encryption root key used to derive LTK and CSRK
   */
    aci_hal_write_config_data(CONFIG_DATA_ER_OFFSET,
    CONFIG_DATA_ER_LEN,
                            (uint8_t*) BLE_CFG_ER_VALUE);

  /**
   * Static random Address
   * The two upper bits shall be set to 1
   * The lowest 32bits is read from the UDN to differentiate between devices
   * The RNG may be used to provide a random number on each power on
   */
  srd_bd_addr[1] =  0x0000ED6E;
  srd_bd_addr[0] =  LL_FLASH_GetUDN( );
  aci_hal_write_config_data( CONFIG_DATA_RANDOM_ADDRESS_OFFSET, CONFIG_DATA_RANDOM_ADDRESS_LEN, (uint8_t*)srd_bd_addr );

  /**
   * Write Identity root key used to derive LTK and CSRK
   */
    aci_hal_write_config_data( CONFIG_DATA_IR_OFFSET, CONFIG_DATA_IR_LEN, (uint8_t*)BLE_CFG_IR_VALUE );

   /**
   * Write Encryption root key used to derive LTK and CSRK
   */
    aci_hal_write_config_data( CONFIG_DATA_ER_OFFSET, CONFIG_DATA_ER_LEN, (uint8_t*)BLE_CFG_ER_VALUE );

  /**
   * Set TX Power to 0dBm.
   */
  aci_hal_set_tx_power_level(1, CFG_TX_POWER);

  /**
   * Initialize GATT interface
   */
  aci_gatt_init();

  /**
   * Initialize GAP interface
   */
  role = GAP_PERIPHERAL_ROLE | GAP_CENTRAL_ROLE;

  if (role > 0)
  {
    const uint8_t name[] = "STM32WB";

    aci_gap_init(role, 0,
                 APPBLE_GAP_DEVICE_NAME_LENGTH,
                 &gap_service_handle, &gap_dev_name_char_handle, &gap_appearance_char_handle);

    if (aci_gatt_update_char_value(gap_service_handle, gap_dev_name_char_handle, 0, sizeof(name) - 1, (uint8_t *)name))
    {
      BLE_DBG_SVCCTL_MSG("Device Name aci_gatt_update_char_value failed.\n");
    }
  }

  if(aci_gatt_update_char_value(gap_service_handle,
                                gap_appearance_char_handle,
                                0,
                                2,
                                (uint8_t *)&appearance))
  {
    BLE_DBG_SVCCTL_MSG("Appearance aci_gatt_update_char_value failed.\n");
  }

  /**
   * Initialize IO capability
   */
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.ioCapability = CFG_IO_CAPABILITY_DISPLAY_ONLY;
  aci_gap_set_io_capability(BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.ioCapability);

  /**
   * Initialize authentication
   */
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.mitm_mode = CFG_MITM_PROTECTION_REQUIRED;
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.OOB_Data_Present = 0;
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMin = 8;
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMax = 16;
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.Use_Fixed_Pin = 1;
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.Fixed_Pin = 111111;
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.bonding_mode = 1;
  for (index = 0; index < 16; index++)
  {
    BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.OOB_Data[index] = (uint8_t) index;
  }

  aci_gap_set_authentication_requirement(BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.bonding_mode,
                                         BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.mitm_mode,
                                         1,
                                         0,
                                         BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMin,
                                         BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMax,
                                         BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.Use_Fixed_Pin,
                                         BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.Fixed_Pin,
                                         0
  );

  /**
   * Initialize whitelist
   */
   if (BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.bonding_mode)
   {
     aci_gap_configure_whitelist();
   }

}

static void BLE_StatusNot( HCI_TL_CmdStatus_t status )
{
  uint32_t task_id_list;
  switch (status)
  {
    case HCI_TL_CmdBusy:
      /**
       * All tasks that may send an aci/hci commands shall be listed here
       * This is to prevent a new command is sent while one is already pending
       */
      task_id_list = (1 << CFG_LAST_TASK_ID_WITH_HCICMD) - 1;
      UTIL_SEQ_PauseTask(task_id_list);

      break;

    case HCI_TL_CmdAvailable:
      /**
       * All tasks that may send an aci/hci commands shall be listed here
       * This is to prevent a new command is sent while one is already pending
       */
      task_id_list = (1 << CFG_LAST_TASK_ID_WITH_HCICMD) - 1;
      UTIL_SEQ_ResumeTask(task_id_list);

      break;

    default:
      break;
  }
  return;
}


const uint8_t* BleGetBdAddress( void )
{
  uint8_t *otp_addr;
  const uint8_t *bd_addr;
  uint32_t udn;
  uint32_t company_id;
  uint32_t device_id;

  udn = LL_FLASH_GetUDN();

  if(udn != 0xFFFFFFFF)
  {
    company_id = LL_FLASH_GetSTCompanyID();
    device_id = LL_FLASH_GetDeviceID();

    bd_addr_udn[0] = (uint8_t)(udn & 0x000000FF);
    bd_addr_udn[1] = (uint8_t)( (udn & 0x0000FF00) >> 8 );
    bd_addr_udn[2] = (uint8_t)( (udn & 0x00FF0000) >> 16 );
    bd_addr_udn[3] = (uint8_t)device_id;
    bd_addr_udn[4] = (uint8_t)(company_id & 0x000000FF);;
    bd_addr_udn[5] = (uint8_t)( (company_id & 0x0000FF00) >> 8 );

    bd_addr = (const uint8_t *)bd_addr_udn;
  }
  else
  {
    otp_addr = OTP_Read(0);
    if(otp_addr)
    {
      bd_addr = ((OTP_ID0_t*)otp_addr)->bd_address;
    }
    else
    {
      bd_addr = M_bd_addr;
    }

  }

  return bd_addr;
}
