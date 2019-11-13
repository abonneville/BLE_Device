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
typedef struct
{
	ble::GattHandle_t handle;
	uint8_t *payload;
	size_t size;
} Payload_t;

typedef struct
{
	uint16_t systolic;
	uint16_t diastolic;
	uint16_t mean;
} BloodPressure_t;


/* Define ------------------------------------------------------------*/

/* Macro -------------------------------------------------------------*/

/* Variables ---------------------------------------------------------*/
static uint8_t callbackValue {};
static BloodPressure_t callbackUserType {};
static Payload_t networkValue {};

/* Function prototypes -----------------------------------------------*/
static void TestCallback(uint8_t value);

/* External functions ------------------------------------------------*/

static void TestCallback(uint8_t value)
{
	callbackValue = value;
}

static void TestUserTypeCallback(BloodPressure_t value)
{
	callbackUserType = value;
}

/**
 * @brief Mock, replace driver implementation with this local definition.
 */
static void updateClientHook(ble::GattHandle_t charHandle, uint8_t *payload, size_t size )
{
	networkValue.handle = charHandle;
	networkValue.payload = payload;
	networkValue.size = size;
}





TEST_GROUP(Characteristic) {};

TEST(Characteristic, instantiation)
{
	ble::GattHandler_t gattHandle {};
	ble::Characteristic<uint8_t>test(45, &gattHandle, TestCallback);

	CHECK(test == 45);
	CHECK(gattHandle.userObject != nullptr);
}


TEST(Characteristic, callback)
{
	ble::GattHandler_t gattHandle {};
	ble::Characteristic<uint8_t>test(0, &gattHandle, TestCallback);

	/* Invoke callback */
	CHECK(test == 0);
	uint8_t local = 56;
	gattHandle.userObject->setValue(&local);
	CHECK(test == 56);
	CHECK(test == callbackValue);
}


TEST(Characteristic, notification)
{
	networkValue = {};

	ble::GattHandler_t gattHandle {};
	ble::Characteristic<uint8_t>test(0, &gattHandle, TestCallback);
	gattHandle.userObject->setGattHandle(36, updateClientHook);

	/* Update network/database value */
	test = 128;


	CHECK(test == networkValue.payload[0]);
	CHECK(1 == networkValue.size);
	CHECK(36 == networkValue.handle);
}

TEST(Characteristic, userType)
{
	networkValue = {};

	ble::GattHandler_t gattHandle {};
	ble::Characteristic<BloodPressure_t>test({}, &gattHandle, TestUserTypeCallback);
	gattHandle.userObject->setGattHandle(65535, updateClientHook);

	/* Update shadow copy */
	BloodPressure_t local {};
	local.diastolic = 7;
	local.systolic = 100;
	local.mean = 1900;

	/* Update network/database value */
	test = local;

	BloodPressure_t *result = (BloodPressure_t *)networkValue.payload;

	CHECK(7 == result->diastolic);
	CHECK(100 == result->systolic);
	CHECK(1900 == result->mean);
	CHECK(6 == networkValue.size);
	CHECK(65535 == networkValue.handle);
}
