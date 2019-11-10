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
#include "stm32wbxx_hal.h"
#include "core_cm4.h"
}

#include <cstdio>
#include <cstring>
#include <errno.h>


#include "CppUTest/CommandLineTestRunner.h"
#include "CppUTest/TestHarness.h"
#include "CppUTest/TestPlugin.h"
#include "CppUTest/TestRegistry.h"


/* Typedef -----------------------------------------------------------*/

/* Define ------------------------------------------------------------*/

/* Macro -------------------------------------------------------------*/

/* Variables ---------------------------------------------------------*/


/* Function prototypes -----------------------------------------------*/
static void PrepNewlib( void );


/* External functions ------------------------------------------------*/


/**
 * @brief Test/checks performed here run pre/post each and every test case
 */
class BaseTest : public TestPlugin {
public:
	BaseTest(const SimpleString& name)
	: TestPlugin(name)
	{
		/* Runs one-time before any testing is performed
		 */
	}

	~BaseTest()
	{
		/* Runs one-time after all testing has completed
		 */
	}

	virtual void preTestAction(UtestShell & /*test*/, TestResult & /*result*/)
	{
		/* Runs before each and every test case */


		errno = 0;
	}

	void postTestAction(UtestShell& test, TestResult& result)
	{
		test = test;
		/* Runs after each and every test case */


		CHECK_EQUAL(errno, 0);

	}
protected:
private:
};








extern "C" void StartApplication()
{
	/* Prepare optional library features */
	PrepNewlib();

	/* Register tests that run before and after every test case */
	BaseTest bt("BaseTest");
	TestRegistry::getCurrentRegistry()->installPlugin(&bt);

	const char *argv[] = {"", "-v", NULL };
	const int argc = sizeof( argv ) / sizeof( char* ) - 1;

	std::printf("\n**** Starting test now! ****\n\n");
	std::fflush(stdout);

	std::printf("\n Results: %d \n", RUN_ALL_TESTS(argc, argv));
	std::printf("**** Testing complete! ****\n");
	std::fflush(stdout);

	while(true){}

}




/**
 * @brief	By design, the newlib library reduces its default footprint size by not statically
 * allocating all its RAM requirements during compilation. If and only if an optional feature is
 * used, then the newlib library acquires the memory via the heap. This behavior impacts unit
 * testing, because when the memory is acquired for the first time, it is never released back
 * to the memory pool and results in a "false" memory leak being detected.
 *
 * This method is used to stimulate optional features prior to testing, so that the
 * heap memory is acquired prior to testing.
 *
 * @note	This has to be repeated for each test thread, since each thread maintains its own
 * control structures
 */
static void PrepNewlib( void )
{
	/* stdio is already addressed when the terminal interface is setup and used for reporting */

	/* strtok() */
	char str[] = "Brown foxes are cool";
	char *token = std::strtok( str, " " );
	volatile int count = 0;
	while ( token != nullptr )
	{
		++count;
		token = std::strtok( nullptr, " " );
	}

	/* rand() */
	std::srand ( count );
	volatile int randomValue = std::rand();
	++randomValue;

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

