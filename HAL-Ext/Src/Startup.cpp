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

/* Variables ---------------------------------------------------------*/
ble::EffectiveBLE bleDevice("Effective", 200);


/* Function prototypes -----------------------------------------------*/

/* External functions ------------------------------------------------*/

extern "C" void StartApplication()
{
	printf("\r\n [%s][%s][%d] \r\n",__FILE__,__FUNCTION__,__LINE__);

	//auto myVar = ble.readOnly<uint8_t, 100>(0x1234, 0x4567, true);

	bleDevice.begin();

	ble::Uuid16 srvID {0x12, 0x34};
	ble::Uuid16 charID {0x56, 0x78};
	ble::Uuid128 bigID {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};

	//ble::Characteristic<uint32_t> timer (0, srvID, charID);
	auto timer ( bleDevice.addChar<uint32_t>(0, srvID, charID) );
	auto apple ( bleDevice.addChar<uint64_t>(0, 0x7893, bigID) );

	timer = 3;
	apple = 1;

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
	int DataIdx;

	for (DataIdx = 0; DataIdx < len; DataIdx++)
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
