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

#ifndef EFFECTIVEDEF_HPP_
#define EFFECTIVEDEF_HPP_

namespace ble
{
/**
 * Universally Unique IDentification (UUID)
 */
using Uuid16 = std::array<uint8_t, 2>;
using Uuid128 = std::array<uint8_t, 16>;

constexpr Uuid16 to_uuid16(const uint16_t value)
{
	return {(uint8_t)(value >> 8), (uint8_t)value};
}

/**
 * Service & characteristic definition, instantiate one per characteristic field. Linked together
 * and they represent a complete service & characteristic object with one or more fields.
 */

template< typename T, typename US = ble::Uuid16, typename UC = ble::Uuid16>
class Characteristic
{
public:
	Characteristic(T v, const US service, const UC characteristic, void(*aCallback)(T) = nullptr, bool updateNow = true ) :
		value(v),
		size(sizeof(T)),
		service(service),
		characteristic(characteristic),
		callback(aCallback),
		updateNow(updateNow) {}

	~Characteristic() {}

//	Characteristic(const Characteristic &) = delete;
	Characteristic& operator=(const Characteristic &) = delete;

	/*
	 * User methods & operators
	 */
	T operator =( T v)
	{
		value = v;
		update(v);
		return v;
	}

#if 0
	T operator +=( T v)
	{
		value += v;
		update(value);
		return value;
	}

	T operator -=( T v)
	{
		value -= v;
		update(value);
		return value;
	}
#endif

	operator T() const
	{
		return value;
	}


	/*
	 * GATT methods
	 */
	void setValue( T newValue )
	{
		value = newValue;
		if (callback)
			callback(newValue);
	}

	void update(T v) {
		if (updateNow)
		{
		}
	}

private:
	T value;
	size_t size;

	const US service;
	const UC characteristic;

	void(*callback)(T);
	bool updateNow;

};

} /* namespace ble*/

#endif /* EFFECTIVEDEF_HPP_ */
