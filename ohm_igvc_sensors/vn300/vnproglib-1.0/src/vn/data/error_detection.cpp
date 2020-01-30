// VectorNav Programming Library v1.0.0.1
// Copyright (c) 2015 VectorNav Technologies, LLC
#include "vn/data/error_detection.h"

namespace vn {
namespace data {
namespace integrity {

uint8_t Checksum8::compute(char const data[], size_t length)
{
	uint8_t xorVal = 0;

	for (size_t i = 0; i < length; i++)
	{
		xorVal ^= data[i];
	}

	return xorVal;
}

uint16_t Crc16::compute(char const data[], size_t length)
{
	uint32_t i;
	uint16_t crc = 0;

	for (i = 0; i < length; i++)
	{
		crc = static_cast<uint16_t>((crc >> 8) | (crc << 8));

		crc ^= static_cast<uint8_t>(data[i]);
		crc ^= static_cast<uint16_t>(static_cast<uint8_t>(crc & 0xFF) >> 4);
		crc ^= static_cast<uint16_t>((crc << 8) << 4);
		crc ^= static_cast<uint16_t>(((crc & 0xFF) << 4) << 1);
	}

	return crc;
}

}
}
}
