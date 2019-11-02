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
	EffectiveBLE(const char * name, uint16_t interval = 1000);

	~EffectiveBLE();

	void begin();
	void end();
	void advertise();

	template< typename T, typename US = ble::Uuid16, typename UC = ble::Uuid16>
	Characteristic<T, US, UC> addChar(
			T v,
			const US service,
			const UC characteristic,
			void(*aCallback)(T) = nullptr,
			bool updateNow = true)
	{
		return {v, service, characteristic, aCallback, updateNow};
	}

	template< typename T, typename UC = ble::Uuid16>
	Characteristic<T, Uuid16, UC> addChar(
			T v,
			uint16_t service,
			const UC characteristic,
			void(*aCallback)(T) = nullptr,
			bool updateNow = true)
	{
		return {v, to_uuid16(service), characteristic, aCallback, updateNow};
	}

	template< typename T, typename US = ble::Uuid16>
	Characteristic<T, US, Uuid16> addChar(
			T v,
			const US service,
			uint16_t characteristic,
			void(*aCallback)(T) = nullptr,
			bool updateNow = true)
	{
		return {v, service, to_uuid16(characteristic), aCallback, updateNow};
	}

	template< typename T>
	Characteristic<T, Uuid16, Uuid16> addChar(
			T v,
			uint16_t service,
			uint16_t characteristic,
			void(*aCallback)(T) = nullptr,
			bool updateNow = true)
	{
		return {v, to_uuid16(service), to_uuid16(characteristic), aCallback, updateNow};
	}


private:
	class impl;
	std::unique_ptr<impl> pimpl;
};



} /* namespace ble */

#endif /* EFFECTIVEBLE_HPP_ */
