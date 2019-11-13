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
#include "CppUTest/TestHarness.h"

/* Typedef -----------------------------------------------------------*/

/* Define ------------------------------------------------------------*/

/* Macro -------------------------------------------------------------*/

/* Variables ---------------------------------------------------------*/

/* Function prototypes -----------------------------------------------*/

/* External functions ------------------------------------------------*/


TEST_GROUP(Uuid16) {};
TEST_GROUP(Uuid128) {};

TEST(Uuid16, uint16_t)
{
	uint16_t id1 = 0xFF33;
	ble::Uuid128 uuid1 ( ble::to_uuid128(id1) );
	ble::Uuid128 uuid2 { ble::to_uuid128(0xFF33) };
	ble::Uuid128 uuid3 ( ble::to_uuid128(0) );

	CHECK( uuid1 == uuid2 );
	CHECK( uuid1 != uuid3 );

	uuid3 = ble::to_uuid128(9834);
	CHECK( uuid3[0] == 0x26 );
	CHECK( uuid3[1] == 0x6A );

	uuid3[15] = 99;
	uuid3 = ble::to_uuid128(76);
	CHECK( uuid3[0] == 0 );
	CHECK( uuid3[1] == 76 );
	CHECK( uuid3[2] == 0 );
	CHECK( uuid3[3] == 0 );
	CHECK( uuid3[4] == 0 );
	CHECK( uuid3[5] == 0 );
	CHECK( uuid3[6] == 0 );
	CHECK( uuid3[7] == 0 );
	CHECK( uuid3[8] == 0 );
	CHECK( uuid3[9] == 0 );
	CHECK( uuid3[10] == 0 );
	CHECK( uuid3[11] == 0 );
	CHECK( uuid3[12] == 0 );
	CHECK( uuid3[13] == 0 );
	CHECK( uuid3[14] == 0 );
	CHECK( uuid3[15] == 0 );
}


TEST(Uuid128, byteArray)
{
	const ble::Uuid128 id1
		{
			127, 0, 22, 33,
			127, 0, 22, 33,
			127, 0, 22, 33,
			127, 0, 22, 33
		};
	ble::Uuid128 uuid1 {id1};
	ble::Uuid128 uuid2 (id1);
	ble::Uuid128 uuid3 {};
	ble::Uuid128 uuid4 ({id1});

	CHECK( uuid1 == uuid2 );
	CHECK( uuid1 != uuid3 );
	CHECK( uuid1 == uuid4 );

	uuid4 = {234, 76};
	CHECK( uuid4[0] == 234 );
	CHECK( uuid4[1] == 76 );
	CHECK( uuid4[2] == 0  );
	CHECK( uuid4[3] == 0  );
	CHECK( uuid4[4] == 0  );
	CHECK( uuid4[5] == 0  );
	CHECK( uuid4[6] == 0  );
	CHECK( uuid4[7] == 0  );
	CHECK( uuid4[8] == 0  );
	CHECK( uuid4[9] == 0  );
	CHECK( uuid4[10] == 0  );
	CHECK( uuid4[11] == 0  );
	CHECK( uuid4[12] == 0  );
	CHECK( uuid4[13] == 0  );
	CHECK( uuid4[14] == 0  );
	CHECK( uuid4[15] == 0  );

	uuid4[1] = 44;
	CHECK( uuid4[0] == 234 );
	CHECK( uuid4[1] == 44 );
}


TEST(Uuid128, otherOperators)
{

	/* size() */
	ble::Uuid128 uuid1;
	CHECK( 16 == uuid1.size() );

	/* copy() */
	uint8_t id1[16] {};
	uuid1 = {125, 99};

	std::copy(uuid1.begin(), uuid1.end(), id1);
	//uuid1.copy( id1 );
	CHECK( 125 == id1[0] );
	CHECK( 99  == id1[1] );

	CHECK( 0   == id1[2] );
	CHECK( 0   == id1[3] );

	CHECK( 0   == id1[4] );
	CHECK( 0   == id1[5] );

	CHECK( 0   == id1[6] );
	CHECK( 0   == id1[7] );

	CHECK( 0   == id1[8] );
	CHECK( 0   == id1[9] );

	CHECK( 0   == id1[10] );
	CHECK( 0   == id1[11] );

	CHECK( 0   == id1[12] );
	CHECK( 0   == id1[13] );

	CHECK( 0   == id1[14] );
	CHECK( 0   == id1[15] );

	/* Verify access to raw pointer */
	CHECK( uuid1.data() == &uuid1[0] );
}


