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

extern "C" {
#include <stdint.h>
#include "stm32wbxx_hal.h"
#include "main.h"
#include "stm32_seq.h"
#include "core_cm4.h"
}
#include <stdio.h>
#include <errno.h>

#include "EffectiveBLE.hpp"

/* Typedef -----------------------------------------------------------*/

/* Define ------------------------------------------------------------*/

/* Macro -------------------------------------------------------------*/

#define PRINT_MESG_DBG(...)     do{printf("\r\nInfo: "); printf(__VA_ARGS__);printf(" [%s][%s][%d] ", __FILE__, __FUNCTION__,__LINE__);}while(0);


/* Variables ---------------------------------------------------------*/
ble::EffectiveBLE bleDevice("Effective", 200);

ble::Uuid128 ServiceUuid {0x00,0x00,0xfe,0x40,0xcc,0x7a,0x48,0x2a,0x98,0x4a,0x7f,0x2e,0xd5,0xb3,0xe5,0x8f};
ble::Uuid128 LedCharUuid {0x00,0x00,0xfe,0x41,0x8e,0x22,0x45,0x41,0x9d,0x4c,0x21,0xed,0xae,0x82,0xed,0x19};
ble::Uuid128 ButCharUuid {0x00,0x00,0xfe,0x42,0x8e,0x22,0x45,0x41,0x9d,0x4c,0x21,0xed,0xae,0x82,0xed,0x19};

/* Function prototypes -----------------------------------------------*/

/* External functions ------------------------------------------------*/

static void LedCallback(uint16_t fromClient);

/* Define control/status variables */
auto button ( bleDevice.addChar<uint16_t>(0, ServiceUuid, ButCharUuid) );
auto led ( bleDevice.addChar<uint16_t>(0, ServiceUuid,  LedCharUuid, LedCallback) );

static void LedCallback(uint16_t fromClient)
{
	fromClient = fromClient;

	HAL_GPIO_TogglePin(GPIOB, LED_BLUE_Pin);
	button = (uint16_t)(button + 1);
}



extern "C" void StartApplication()
{


	/* Initiate BLE */
	bleDevice.begin();


	while(1)
	{
		/*
		 * Run all pending tasks
		 */
	    UTIL_SEQ_Run( UTIL_SEQ_DEFAULT );

	    /*
		  HAL_GPIO_TogglePin(GPIOB, LED_BLUE_Pin);
		  HAL_Delay(200);
		  HAL_GPIO_TogglePin(GPIOB, LED_GREEN_Pin);
		  HAL_Delay(200);
		  HAL_GPIO_TogglePin(GPIOB, LED_RED_Pin);
		  HAL_Delay(200);
		  */
	}
}


/**
 * Temporary location until HAL extension layer is to be ported/implemented.
 */

/**
 * @brief Redirects stdout to the JTAG serial wire viewer (SWV)
 * @param fd, not used. File/device ID (stdout = 0)
 * @param buffer is data to be sent out
 * @param len is how many bytes to be sent out
 * @retval number of bytes sent
 */
extern "C" int _write(int fd, char *ptr, int len)
{
	fd = fd;

	for (int DataIdx = 0; DataIdx < len; DataIdx++)
	{
		ITM_SendChar(*ptr++);
	}
	return len;
}


/**
 _sbrk
 Increase program data space. Malloc and related functions depend on this
**/
extern "C" caddr_t _sbrk(int incr)
{
	extern char end asm("end");
	extern char _end asm("_end");
	const char * const heap_limit =&_end;
	static char *heap_end;
	char *prev_heap_end;

	if (heap_end == 0)
		heap_end = &end;

	prev_heap_end = heap_end;
	if (heap_end + incr > heap_limit)
	{
		errno = ENOMEM;
		return (caddr_t) -1;
	}

	heap_end += incr;

	return (caddr_t) prev_heap_end;
}
