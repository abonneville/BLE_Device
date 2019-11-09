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
using Uuid16 = std::uint16_t;
using Uuid128 = std::array<uint8_t, 16>;

#if 0
constexpr Uuid16 to_uuid16(const uint16_t value)
{
	return {(uint8_t)(value >> 8), (uint8_t)value};
}
#endif


/* GATT identifier used to access a specific database entry */
using GattHandle_t = std::uint16_t;


void SendNotification(GattHandle_t charHandle, uint8_t *payload, size_t size );


/**
 * @brief Non-template base class that defines an interface to member functions, which will
 * be be overridden by template subclasses.
 */
class CharBase
{
public:
	virtual void setValue( const std::uint8_t *newValue ) = 0;
	virtual void setGattHandle(GattHandle_t gh) = 0;
};

/**
 * GATT object handler, used to manage each user defined GATT characteristic object.
 */
struct GattHandler_t {
	//TODO reference of value
	Uuid128 srvID;
	Uuid128 charID;

	/* UUID is how many bytes long, 2 or 16 */
	std::size_t srvIDSize;
	std::size_t charIDSize;

	/* Data value is how many bytes long */
	std::uint8_t dataSize;

	GattHandle_t serviceHandle;
	GattHandle_t charHandle;

	CharBase * userObject;
} ;

/**
 * Service & characteristic definition, instantiate one per characteristic field. Linked together
 * and they represent a complete service & characteristic object with one or more fields.
 */
template<typename T>
class Characteristic : public CharBase
{
public:
	Characteristic(T v, GattHandler_t *gattHandler, void(*aCallback)(T)) :
		value(v),
		size(sizeof(T)),
		gattHandle(0),
		callback(aCallback)
		{
			gattHandler->userObject = this;
		}

	/* Default initialize for empty object*/
	Characteristic() :
		value(0),
		size(sizeof(T)),
		gattHandle(0),
		callback(nullptr) {}

	~Characteristic() {}

//	Characteristic(const Characteristic &) = delete;
	Characteristic& operator=(const Characteristic &) = delete;

	/*
	 * User methods & operators
	 */
	T operator =( T v)
	{
		value = v;
		updateClient();
		return value;
	}

#if 0
	T operator +=( T v)
	{
		value += v;
		updateClient();
		return value;
	}

	T operator -=( T v)
	{
		value -= v;
		updateClient();
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

	/**
	 * @brief Used by GATT interface to update user variable.
	 * @param newValue to be assigned to user variable
	 */
	virtual void setValue( const std::uint8_t *newValue )
	{
		value = ((T*)newValue) [0];
		if (callback)
			callback(value);
	}

	/**
	 * @brief Asserted when user variable is updated by application, and needs to notify
	 * 		GATT interface of new value.
	 */
	void updateClient() {
		if (gattHandle)
		{
			SendNotification(gattHandle, (uint8_t *)&value, sizeof(T) );
		}
	}

	/* @brief GATT handles are not known during instantiation. However, the GATT handle
	 * 		must be set when the application needs to update/notify client.
	 * @param gh sets internal copy of gatt handle
	 */
	virtual void setGattHandle(GattHandle_t gh)
	{
		gattHandle = gh;
	}

private:
	T value;
	size_t size;

	GattHandle_t gattHandle;
	void(*callback)(T);
};

} /* namespace ble*/

#endif /* EFFECTIVEDEF_HPP_ */
