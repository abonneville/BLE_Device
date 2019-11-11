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

#ifndef EFFECTIVEBLE_HPP_
#define EFFECTIVEBLE_HPP_

#include <memory>
#include <cstdint>

#include "EffectiveDef.hpp"

namespace ble
{

class EffectiveBLE
{
public:
	explicit EffectiveBLE(const char * name, uint16_t interval = 1000);

	~EffectiveBLE();

	EffectiveBLE(const EffectiveBLE &) = delete;
	EffectiveBLE & operator=(const EffectiveBLE &) = delete;

	void begin();
	void end();
	void advertise();

	template< typename T >
	Characteristic<T> addChar(
			T v,
			const ble::Uuid128 & service,
			const ble::Uuid128 & characteristic,
			void(*aCallback)(T) = nullptr)
	{

		auto gattHandle = addObject(service, sizeof(service), characteristic, sizeof(characteristic), sizeof(T) );
		if (gattHandle)  return {v, gattHandle, aCallback};
		else return {};
	}

	template< typename T >
	Characteristic<T> addChar(
			T v,
			uint16_t service,
			const ble::Uuid128 & characteristic,
			void(*aCallback)(T) = nullptr)
	{
		auto gattHandle = addObject(to_uuid128(service), sizeof(service), characteristic, sizeof(characteristic), sizeof(T) );
		if (gattHandle)  return {v, gattHandle, aCallback};
		else return {};
	}

	template< typename T >
	Characteristic<T> addChar(
			T v,
			const ble::Uuid128 & service,
			uint16_t characteristic,
			void(*aCallback)(T) = nullptr)
	{
		auto gattHandle = addObject(service, sizeof(service), to_uuid128(characteristic), sizeof(characteristic), sizeof(T) );
		if (gattHandle)  return {v, gattHandle, aCallback};
		else return {};
	}

	template< typename T >
	Characteristic<T> addChar(
			T v,
			uint16_t service,
			uint16_t characteristic,
			void(*aCallback)(T) = nullptr)
	{
		auto gattHandle = addObject(to_uuid128(service), sizeof(service), to_uuid128(characteristic), sizeof(characteristic), sizeof(T) );
		if (gattHandle)  return {v, gattHandle, aCallback};
		else return {};
	}

	std::array<GattHandler_t, 10> gattHandles;
	std::size_t count;

private:
	class impl;
	std::unique_ptr<impl> pimpl;

	void init(void);

	GattHandler_t * addObject(
			const Uuid128 & service,
			size_t serviceSize,
			const Uuid128 & characteristic,
			size_t characteristicSize,
			uint8_t dataSize );


};



} /* namespace ble */

#endif /* EFFECTIVEBLE_HPP_ */
