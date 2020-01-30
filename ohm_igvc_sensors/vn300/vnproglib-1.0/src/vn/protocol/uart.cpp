// VectorNav Programming Library v1.0.0.1
// Copyright (c) 2015 VectorNav Technologies, LLC
#include "vn/protocol/uart.h"

#include <queue>
#include <list>
#include <string>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "vn/utilities.h"
#include "vn/data/error_detection.h"

#define NEXT result = getNextData(_data, parseIndex); \
	if (result == NULL) \
		return;

#define ATOFF static_cast<float>(atof(result))
#define ATOFD atof(result)
#define ATOU32 static_cast<uint32_t>(atoi(result))
#define ATOU16 static_cast<uint16_t>(atoi(result))
#define ATOU8 static_cast<uint8_t>(atoi(result))

using namespace std;
using namespace vn::math;
using namespace vn::data::integrity;

namespace vn {
namespace protocol {
namespace uart {

char* vnstrtok(char* str, size_t& startIndex);

const unsigned char Packet::BinaryGroupLengths[sizeof(uint8_t)*8][sizeof(uint16_t)*8] = {
	{ 8, 8,	 8,  12, 16, 12, 24, 12, 12, 24, 20, 28, 2,  4, 8, 0 },		// Group 1
	{ 8, 8,  8,  2,  8,  8,  8,  4,  0,  0,  0,  0,  0,  0, 0, 0 },		// Group 2
	{ 2, 12, 12, 12, 4,  4,  16, 12, 12, 12, 12, 2,  40, 0, 0, 0 },		// Group 3
	{ 8, 8,  2,  1,  1,  24, 24, 12, 12, 12, 4,  4,  32, 0, 0, 0 },		// Group 4
	{ 2, 12, 16, 36, 12, 12, 12, 12, 12, 12, 28, 24, 0,  0, 0, 0 },		// Group 5
	{ 2, 24, 24, 12, 12, 12, 12, 12, 12, 4,  4,  68, 64, 0, 0, 0 },		// Group 6
	{ 0, 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 0, 0 },		// Invalid group
	{ 0, 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 0, 0 }		// Invalid group
};

Packet::Packet() :
	_isPacketDataMine(false),
	_length(0)
{
}

Packet::Packet(Type ptype, char* fullPacketData, size_t length) :
	_type(ptype),
	_isPacketDataMine(false),
	_length(length),
	_data(fullPacketData),
	_curExtractLoc(0)
{
}

Packet::Packet(Packet const& toCopy) :
	_type(toCopy._type),
	_isPacketDataMine(true),
	_length(toCopy._length),
	_data(new char[toCopy._length]),
	_curExtractLoc(0)
{
	copy(toCopy._data, toCopy._data + toCopy._length, _data);
}

Packet::~Packet()
{
	if (_isPacketDataMine)
		delete [] _data;
}

Packet& Packet::operator=(Packet const& from)
{
	if (_isPacketDataMine)
		delete [] _data;

	_isPacketDataMine = true;
	_data = new char[from._length];
	_type = from._type;
	_length = from._length;
	_curExtractLoc = from._curExtractLoc;

	copy(from._data, from._data + from._length, _data);

	return *this;
}

string Packet::datastr()
{
	return string(_data, _length);
}

Packet::Type Packet::type()
{
	return _type;
}

bool Packet::isValid()
{
	if (_length == 0)
		return false;

	if (type() == TYPE_ASCII)
	{
		// First determine if this packet does not have a checksum or CRC.
		if (_data[_length - 3] == 'X' && _data[_length - 4] == 'X')
			return true;

		// First determine if this packet has an 8-bit checksum or a 16-bit CRC.
		if (_data[_length - 5] == '*')
		{
			// Appears we have an 8-bit checksum packet.
			uint8_t expectedChecksum = toUint8FromHexStr(_data + _length - 4);

			uint8_t computedChecksum = Checksum8::compute(_data + 1, _length - 6);

			return expectedChecksum == computedChecksum;
		}
		else if (_data[_length - 7] == '*')
		{
			// Appears we have a 16-bit CRC packet.
			uint16_t packetCrc = to_uint16_from_hexstr(_data + _length - 6);

			uint16_t computedCrc = Crc16::compute(_data + 1, _length - 8);

			return packetCrc == computedCrc;
		}
		else
		{
			// Don't know what we have.
			return false;
		}
	}
	else if (type() == TYPE_BINARY)
	{
		uint16_t computedCrc = Crc16::compute(_data + 1, _length - 1);

		return computedCrc == 0;
	}
	else
	{
		throw not_implemented();
	}
}

bool Packet::isError()
{
	return strncmp(_data + 3, "ERR", 3) == 0;
}

bool Packet::isResponse()
{
	if (strncmp(_data + 3, "WRG", 3) == 0)
		return true;
	if (strncmp(_data + 3, "RRG", 3) == 0)
		return true;
	if (strncmp(_data + 3, "WNV", 3) == 0)
		return true;
	if (strncmp(_data + 3, "RFS", 3) == 0)
		return true;
	if (strncmp(_data + 3, "RST", 3) == 0)
		return true;
	if (strncmp(_data + 3, "FWU", 3) == 0)
		return true;
	if (strncmp(_data + 3, "CMD", 3) == 0)
		return true;
	if (strncmp(_data + 3, "ASY", 3) == 0)
		return true;

	return false;
}

bool Packet::isAsciiAsync()
{
	// Pointer to the unique asynchronous data type identifier.
	char* pAT = _data + 3;

	if (strncmp(pAT, "YPR", 3) == 0)
		return true;
	if (strncmp(pAT, "QTN", 3) == 0)
		return true;
	if (strncmp(pAT, "QMR", 3) == 0)
		return true;
	if (strncmp(pAT, "MAG", 3) == 0)
		return true;
	if (strncmp(pAT, "ACC", 3) == 0)
		return true;
	if (strncmp(pAT, "GYR", 3) == 0)
		return true;
	if (strncmp(pAT, "MAR", 3) == 0)
		return true;
	if (strncmp(pAT, "YMR", 3) == 0)
		return true;
	if (strncmp(pAT, "YBA", 3) == 0)
		return true;
	if (strncmp(pAT, "YIA", 3) == 0)
		return true;
	if (strncmp(pAT, "IMU", 3) == 0)
		return true;
	if (strncmp(pAT, "GPS", 3) == 0)
		return true;
	if (strncmp(pAT, "GPE", 3) == 0)
		return true;
	if (strncmp(pAT, "INS", 3) == 0)
		return true;
	if (strncmp(pAT, "INE", 3) == 0)
		return true;
	if (strncmp(pAT, "ISL", 3) == 0)
		return true;
	if (strncmp(pAT, "ISE", 3) == 0)
		return true;
	if (strncmp(pAT, "DTV", 3) == 0)
		return true;
	else
		return false;
}

AsciiAsync Packet::determineAsciiAsyncType()
{
	// Pointer to the unique asynchronous data type identifier.
	char* pAT = _data + 3;

	if (strncmp(pAT, "YPR", 3) == 0)
		return VNYPR;
	if (strncmp(pAT, "QTN", 3) == 0)
		return VNQTN;
	if (strncmp(pAT, "QMR", 3) == 0)
		return VNQMR;
	if (strncmp(pAT, "MAG", 3) == 0)
		return VNMAG;
	if (strncmp(pAT, "ACC", 3) == 0)
		return VNACC;
	if (strncmp(pAT, "GYR", 3) == 0)
		return VNGYR;
	if (strncmp(pAT, "MAR", 3) == 0)
		return VNMAR;
	if (strncmp(pAT, "YMR", 3) == 0)
		return VNYMR;
	if (strncmp(pAT, "YBA", 3) == 0)
		return VNYBA;
	if (strncmp(pAT, "YIA", 3) == 0)
		return VNYIA;
	if (strncmp(pAT, "IMU", 3) == 0)
		return VNIMU;
	if (strncmp(pAT, "GPS", 3) == 0)
		return VNGPS;
	if (strncmp(pAT, "GPE", 3) == 0)
		return VNGPE;
	if (strncmp(pAT, "INS", 3) == 0)
		return VNINS;
	if (strncmp(pAT, "INE", 3) == 0)
		return VNINE;
	if (strncmp(pAT, "ISL", 3) == 0)
		return VNISL;
	if (strncmp(pAT, "ISE", 3) == 0)
		return VNISE;
	if (strncmp(pAT, "DTV", 3) == 0)
		return VNDTV;
	else
		throw unknown_error();
}

bool Packet::isCompatible(CommonGroup commonGroup, TimeGroup timeGroup, ImuGroup imuGroup, GpsGroup gpsGroup, AttitudeGroup attitudeGroup, InsGroup insGroup)
{
	// First make sure the appropriate groups are specified.
	uint8_t groups = _data[1];
	char *curField = _data + 2;

	if (commonGroup)
	{
		if (*reinterpret_cast<uint16_t*>(curField) != commonGroup)
			// Not the expected collection of field data types.
			return false;

		curField += 2;
	}
	else if (groups & 0x01)
	{
		// There is unexpected Common Group data.
		return false;
	}

	if (timeGroup)
	{
		if (*reinterpret_cast<uint16_t*>(curField) != timeGroup)
			// Not the expected collection of field data types.
			return false;

		curField += 2;
	}
	else if (groups & 0x02)
	{
		// There is unexpected Time Group data.
		return false;
	}

	if (imuGroup)
	{
		if (*reinterpret_cast<uint16_t*>(curField) != imuGroup)
			// Not the expected collection of field data types.
			return false;

		curField += 2;
	}
	else if (groups & 0x04)
	{
		// There is unexpected IMU Group data.
		return false;
	}

	if (gpsGroup)
	{
		if (*reinterpret_cast<uint16_t*>(curField) != gpsGroup)
			// Not the expected collection of field data types.
			return false;

		curField += 2;
	}
	else if (groups & 0x08)
	{
		// There is unexpected GPS Group data.
		return false;
	}

	if (attitudeGroup)
	{
		if (*reinterpret_cast<uint16_t*>(curField) != attitudeGroup)
			// Not the expected collection of field data types.
			return false;

		curField += 2;
	}
	else if (groups & 0x10)
	{
		// There is unexpected Attitude Group data.
		return false;
	}

	if (insGroup)
	{
		if (*reinterpret_cast<uint16_t*>(curField) != insGroup)
			// Not the expected collection of field data types.
			return false;

		curField += 2;
	}
	else if (groups & 0x20)
	{
		// There is unexpected INS Group data.
		return false;
	}

	// Everything checks out.
	return true;
}

char* startAsciiPacketParse(char* packetStart, size_t& index)
{
	index = 7;

	return vnstrtok(packetStart, index);
}

char* startAsciiResponsePacketParse(char* packetStart, size_t& index)
{
	startAsciiPacketParse(packetStart, index);

	return vnstrtok(packetStart, index);
}

char* getNextData(char* str, size_t& startIndex)
{
	return vnstrtok(str, startIndex);
}

char* vnstrtok(char* str, size_t& startIndex)
{
	size_t origIndex = startIndex;

	while (str[startIndex] != ',' && str[startIndex] != '*')
		startIndex++;

	str[startIndex++] = '\0';

	return str + origIndex;
}

void Packet::ensureCanExtract(size_t numOfBytes)
{
	if (_curExtractLoc == 0)
		// Determine the location to start extracting.
		_curExtractLoc = countSetBits(_data[1]) * 2 + 2;

	if (_curExtractLoc + numOfBytes > _length - 2)
		// About to overrun data.
		throw invalid_operation();
}

uint8_t Packet::extractUint8()
{
	ensureCanExtract(sizeof(uint8_t));

	uint8_t d = *reinterpret_cast<uint8_t*>(_data + _curExtractLoc);

	_curExtractLoc += sizeof(uint8_t);

	return d;
}

int8_t Packet::extractInt8()
{
	ensureCanExtract(sizeof(int8_t));

	int8_t d = *reinterpret_cast<int8_t*>(_data + _curExtractLoc);

	_curExtractLoc += sizeof(int8_t);

	return d;
}

uint16_t Packet::extractUint16()
{
	ensureCanExtract(sizeof(uint16_t));

	uint16_t d;

	memcpy(&d, _data + _curExtractLoc, sizeof(uint16_t));

	_curExtractLoc += sizeof(uint16_t);

	return stoh(d);
}

uint32_t Packet::extractUint32()
{
	ensureCanExtract(sizeof(uint32_t));

	uint32_t d;

	memcpy(&d, _data + _curExtractLoc, sizeof(uint32_t));

	_curExtractLoc += sizeof(uint32_t);

	return stoh(d);
}

uint64_t Packet::extractUint64()
{
	ensureCanExtract(sizeof(uint64_t));

	uint64_t d;

	memcpy(&d, _data + _curExtractLoc, sizeof(uint64_t));

	_curExtractLoc += sizeof(uint64_t);

	return stoh(d);
}

float Packet::extractFloat()
{
	ensureCanExtract(sizeof(float));

	float f;

	memcpy(&f, _data + _curExtractLoc, sizeof(float));

	_curExtractLoc += sizeof(float);

	return f;
}

vec3f Packet::extractVec3f()
{
	ensureCanExtract(3 * sizeof(float));

	vec3f d;

	memcpy(&d.x, _data + _curExtractLoc, sizeof(float));
	memcpy(&d.y, _data + _curExtractLoc + sizeof(float), sizeof(float));
	memcpy(&d.z, _data + _curExtractLoc + 2 * sizeof(float), sizeof(float));

	_curExtractLoc += 3 * sizeof(float);

	return d;
}

vec3d Packet::extractVec3d()
{
	ensureCanExtract(3 * sizeof(double));

	vec3d d;

	memcpy(&d.x, _data + _curExtractLoc, sizeof(double));
	memcpy(&d.y, _data + _curExtractLoc + sizeof(double), sizeof(double));
	memcpy(&d.z, _data + _curExtractLoc + 2 * sizeof(double), sizeof(double));

	_curExtractLoc += 3 * sizeof(double);

	return d;
}

vec4f Packet::extractVec4f()
{
	ensureCanExtract(4 * sizeof(float));

	vec4f d;

	memcpy(&d.x, _data + _curExtractLoc, sizeof(float));
	memcpy(&d.y, _data + _curExtractLoc + sizeof(float), sizeof(float));
	memcpy(&d.z, _data + _curExtractLoc + 2 * sizeof(float), sizeof(float));
	memcpy(&d.w, _data + _curExtractLoc + 3 * sizeof(float), sizeof(float));

	_curExtractLoc += 4 * sizeof(float);

	return d;
}

mat3f Packet::extractMat3f()
{
	ensureCanExtract(9 * sizeof(float));

	mat3f m;

	memcpy(&m.e00, _data + _curExtractLoc, sizeof(float));
	memcpy(&m.e10, _data + _curExtractLoc + sizeof(float), sizeof(float));
	memcpy(&m.e20, _data + _curExtractLoc + 2 * sizeof(float), sizeof(float));
	memcpy(&m.e01, _data + _curExtractLoc + 3 * sizeof(float), sizeof(float));
	memcpy(&m.e11, _data + _curExtractLoc + 4 * sizeof(float), sizeof(float));
	memcpy(&m.e21, _data + _curExtractLoc + 5 * sizeof(float), sizeof(float));
	memcpy(&m.e02, _data + _curExtractLoc + 6 * sizeof(float), sizeof(float));
	memcpy(&m.e12, _data + _curExtractLoc + 7 * sizeof(float), sizeof(float));
	memcpy(&m.e22, _data + _curExtractLoc + 8 * sizeof(float), sizeof(float));

	_curExtractLoc += 9 * sizeof(float);

	return m;
}

size_t Packet::finalizeCommand(ErrorDetectionMode errorDetectionMode, char *packet, size_t length)
{
	if (errorDetectionMode == ERRORDETECTIONMODE_CHECKSUM)
	{
		length += sprintf(packet + length, "*%02X\r\n", Checksum8::compute(packet + 1, length - 1));
	}
	else if (errorDetectionMode == ERRORDETECTIONMODE_CRC)
	{
		length += sprintf(packet + length, "*%04X\r\n", Crc16::compute(packet + 1, length - 1));
	}
	else
	{
		length += sprintf(packet + length, "*XX\r\n");
	}

	return length;
}

size_t Packet::genReadBinaryOutput1(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	size_t length = sprintf(buffer, "$VNRRG,75");

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadBinaryOutput2(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	size_t length = sprintf(buffer, "$VNRRG,76");

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadBinaryOutput3(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	size_t length = sprintf(buffer, "$VNRRG,77");

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t writeBinaryOutput(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, uint8_t binaryOutputNumber, uint16_t asyncMode, uint16_t rateDivisor, uint16_t commonField, uint16_t timeField, uint16_t imuField, uint16_t gpsField, uint16_t attitudeField, uint16_t insField)
{
	// First determine which groups are present.
	uint16_t groups = 0;
	if (commonField)
		groups |= 0x0001;
	if (timeField)
		groups |= 0x0002;
	if (imuField)
		groups |= 0x0004;
	if (gpsField)
		groups |= 0x0008;
	if (attitudeField)
		groups |= 0x0010;
	if (insField)
		groups |= 0x0020;

	int length = sprintf(buffer, "$VNWRG,%u,%u,%u,%X", 74 + binaryOutputNumber, asyncMode, rateDivisor, groups);

	if (commonField)
		length += sprintf(buffer + length, ",%X", commonField);
	if (timeField)
		length += sprintf(buffer + length, ",%X", timeField);
	if (imuField)
		length += sprintf(buffer + length, ",%X", imuField);
	if (gpsField)
		length += sprintf(buffer + length, ",%X", gpsField);
	if (attitudeField)
		length += sprintf(buffer + length, ",%X", attitudeField);
	if (insField)
		length += sprintf(buffer + length, ",%X", insField);

	return Packet::finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genWriteBinaryOutput1(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, uint16_t asyncMode, uint16_t rateDivisor, uint16_t commonField, uint16_t timeField, uint16_t imuField, uint16_t gpsField, uint16_t attitudeField, uint16_t insField)
{
	return writeBinaryOutput(errorDetectionMode, buffer, size, 1, asyncMode, rateDivisor, commonField, timeField, imuField, gpsField, attitudeField, insField);
}

size_t Packet::genWriteBinaryOutput2(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, uint16_t asyncMode, uint16_t rateDivisor, uint16_t commonField, uint16_t timeField, uint16_t imuField, uint16_t gpsField, uint16_t attitudeField, uint16_t insField)
{
	return writeBinaryOutput(errorDetectionMode, buffer, size, 2, asyncMode, rateDivisor, commonField, timeField, imuField, gpsField, attitudeField, insField);
}

size_t Packet::genWriteBinaryOutput3(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, uint16_t asyncMode, uint16_t rateDivisor, uint16_t commonField, uint16_t timeField, uint16_t imuField, uint16_t gpsField, uint16_t attitudeField, uint16_t insField)
{
	return writeBinaryOutput(errorDetectionMode, buffer, size, 3, asyncMode, rateDivisor, commonField, timeField, imuField, gpsField, attitudeField, insField);
}

size_t Packet::genWriteSettings(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	size_t length = sprintf(buffer, "$VNWNV");

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genRestoreFactorySettings(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	size_t length = sprintf(buffer, "$VNRFS");

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReset(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	size_t length = sprintf(buffer, "$VNRST");

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadUserTag(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	size_t length = sprintf(buffer, "$VNRRG,00");

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genWriteUserTag(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, string tag)
{
	size_t length = sprintf(buffer, "$VNWRG,00,%s",
		tag.c_str());

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadModelNumber(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	size_t length = sprintf(buffer, "$VNRRG,01");

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadHardwareRevision(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	size_t length = sprintf(buffer, "$VNRRG,02");

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadSerialNumber(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	size_t length = sprintf(buffer, "$VNRRG,03");

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadFirmwareVersion(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	size_t length = sprintf(buffer, "$VNRRG,04");

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadSerialBaudRate(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	size_t length = sprintf(buffer, "$VNRRG,05");

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genWriteSerialBaudRate(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, uint32_t baudrate)
{
	size_t length = sprintf(buffer, "$VNWRG,05,%u",
		baudrate);

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadAsyncDataOutputType(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	size_t length = sprintf(buffer, "$VNRRG,06");

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genWriteAsyncDataOutputType(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, uint32_t ador)
{
	size_t length = sprintf(buffer, "$VNWRG,06,%u",
		ador);

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadAsyncDataOutputFrequency(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	size_t length = sprintf(buffer, "$VNRRG,07");

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genWriteAsyncDataOutputFrequency(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, uint32_t adof)
{
	size_t length = sprintf(buffer, "$VNWRG,07,%u",
		adof);

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadSynchronizationControl(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	size_t length = sprintf(buffer, "$VNRRG,32");

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genWriteSynchronizationControl(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, uint8_t syncInMode, uint8_t syncInEdge, uint16_t syncInSkipFactor, uint32_t reserved1, uint8_t syncOutMode, uint8_t syncOutPolarity, uint16_t syncOutSkipFactor, uint32_t syncOutPulseWidth, uint32_t reserved2)
{
	size_t length = sprintf(buffer, "$VNWRG,32,%u,%u,%u,%u,%u,%u,%u,%u,%u",
		syncInMode,
		syncInEdge,
		syncInSkipFactor,
		reserved1,
		syncOutMode,
		syncOutPolarity,
		syncOutSkipFactor,
		syncOutPulseWidth,
		reserved2);

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadCommunicationProtocolControl(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	size_t length = sprintf(buffer, "$VNRRG,30");

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genWriteCommunicationProtocolControl(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, uint8_t serialCount, uint8_t serialStatus, uint8_t spiCount, uint8_t spiStatus, uint8_t serialChecksum, uint8_t spiChecksum, uint8_t errorMode)
{
	size_t length = sprintf(buffer, "$VNWRG,30,%u,%u,%u,%u,%u,%u,%u",
		serialCount,
		serialStatus,
		spiCount,
		spiStatus,
		serialChecksum,
		spiChecksum,
		errorMode);

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadSynchronizationStatus(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	size_t length = sprintf(buffer, "$VNRRG,33");

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genWriteSynchronizationStatus(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, uint32_t syncInCount, uint32_t syncInTime, uint32_t syncOutCount)
{
	size_t length = sprintf(buffer, "$VNWRG,33,%u,%u,%u",
		syncInCount,
		syncInTime,
		syncOutCount);

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadImuMeasurements(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	size_t length = sprintf(buffer, "$VNRRG,54");

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadDeltaThetaAndDeltaVelocity(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	size_t length = sprintf(buffer, "$VNRRG,80");

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadMagnetometerCompensation(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	size_t length = sprintf(buffer, "$VNRRG,23");

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genWriteMagnetometerCompensation(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, mat3f c, vec3f b)
{
	size_t length = sprintf(buffer, "$VNWRG,23,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
		c.e00,
		c.e01,
		c.e02,
		c.e10,
		c.e11,
		c.e12,
		c.e20,
		c.e21,
		c.e22,
		b.x,
		b.y,
		b.z);

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadAccelerationCompensation(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	size_t length = sprintf(buffer, "$VNRRG,25");

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genWriteAccelerationCompensation(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, mat3f c, vec3f b)
{
	size_t length = sprintf(buffer, "$VNWRG,25,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
		c.e00,
		c.e01,
		c.e02,
		c.e10,
		c.e11,
		c.e12,
		c.e20,
		c.e21,
		c.e22,
		b.x,
		b.y,
		b.z);

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadGyroCompensation(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	size_t length = sprintf(buffer, "$VNRRG,84");

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genWriteGyroCompensation(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, mat3f c, vec3f b)
{
	size_t length = sprintf(buffer, "$VNWRG,84,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
		c.e00,
		c.e01,
		c.e02,
		c.e10,
		c.e11,
		c.e12,
		c.e20,
		c.e21,
		c.e22,
		b.x,
		b.y,
		b.z);

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadReferenceFrameRotation(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	size_t length = sprintf(buffer, "$VNRRG,26");

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genWriteReferenceFrameRotation(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, mat3f c)
{
	size_t length = sprintf(buffer, "$VNWRG,26,%f,%f,%f,%f,%f,%f,%f,%f,%f",
		c.e00,
		c.e01,
		c.e02,
		c.e10,
		c.e11,
		c.e12,
		c.e20,
		c.e21,
		c.e22);

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadImuFilteringConfiguration(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	size_t length = sprintf(buffer, "$VNRRG,85");

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genWriteImuFilteringConfiguration(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, uint16_t magWindowSize, uint16_t accelWindowSize, uint16_t gyroWindowSize, uint16_t tempWindowSize, uint16_t presWindowSize, uint8_t magFilterMode, uint8_t accelFilterMode, uint8_t gyroFilterMode, uint8_t tempFilterMode, uint8_t presFilterMode)
{
	size_t length = sprintf(buffer, "$VNWRG,85,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u",
		magWindowSize,
		accelWindowSize,
		gyroWindowSize,
		tempWindowSize,
		presWindowSize,
		magFilterMode,
		accelFilterMode,
		gyroFilterMode,
		tempFilterMode,
		presFilterMode);

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadDeltaThetaAndDeltaVelocityConfiguration(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	size_t length = sprintf(buffer, "$VNRRG,82");

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genWriteDeltaThetaAndDeltaVelocityConfiguration(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, uint8_t integrationFrame, uint8_t gyroCompensation, uint8_t accelCompensation, uint8_t reserved1, uint16_t reserved2)
{
	size_t length = sprintf(buffer, "$VNWRG,82,%u,%u,%u,%u,%u",
		integrationFrame,
		gyroCompensation,
		accelCompensation,
		reserved1,
		reserved2);

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadYawPitchRoll(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	size_t length = sprintf(buffer, "$VNRRG,08");

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadAttitudeQuaternion(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	size_t length = sprintf(buffer, "$VNRRG,09");

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadYawPitchRollMagneticAccelerationAndAngularRates(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	size_t length = sprintf(buffer, "$VNRRG,27");

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadQuaternionMagneticAccelerationAndAngularRates(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	size_t length = sprintf(buffer, "$VNRRG,15");

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadMagneticMeasurements(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	size_t length = sprintf(buffer, "$VNRRG,17");

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadAccelerationMeasurements(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	size_t length = sprintf(buffer, "$VNRRG,18");

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadAngularRateMeasurements(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	size_t length = sprintf(buffer, "$VNRRG,19");

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadMagneticAccelerationAndAngularRates(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	size_t length = sprintf(buffer, "$VNRRG,20");

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadYawPitchRollTrueBodyAccelerationAndAngularRates(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	size_t length = sprintf(buffer, "$VNRRG,239");

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadYawPitchRollTrueInertialAccelerationAndAngularRates(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	size_t length = sprintf(buffer, "$VNRRG,240");

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadVpeBasicControl(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	size_t length = sprintf(buffer, "$VNRRG,35");

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genWriteVpeBasicControl(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, uint8_t enable, uint8_t headingMode, uint8_t filteringMode, uint8_t tuningMode)
{
	size_t length = sprintf(buffer, "$VNWRG,35,%u,%u,%u,%u",
		enable,
		headingMode,
		filteringMode,
		tuningMode);

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadVpeMagnetometerBasicTuning(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	size_t length = sprintf(buffer, "$VNRRG,36");

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genWriteVpeMagnetometerBasicTuning(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, vec3f baseTuning, vec3f adaptiveTuning, vec3f adaptiveFiltering)
{
	size_t length = sprintf(buffer, "$VNWRG,36,%f,%f,%f,%f,%f,%f,%f,%f,%f",
		baseTuning.x,
		baseTuning.y,
		baseTuning.z,
		adaptiveTuning.x,
		adaptiveTuning.y,
		adaptiveTuning.z,
		adaptiveFiltering.x,
		adaptiveFiltering.y,
		adaptiveFiltering.z);

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadVpeAccelerometerBasicTuning(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	size_t length = sprintf(buffer, "$VNRRG,38");

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genWriteVpeAccelerometerBasicTuning(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, vec3f baseTuning, vec3f adaptiveTuning, vec3f adaptiveFiltering)
{
	size_t length = sprintf(buffer, "$VNWRG,38,%f,%f,%f,%f,%f,%f,%f,%f,%f",
		baseTuning.x,
		baseTuning.y,
		baseTuning.z,
		adaptiveTuning.x,
		adaptiveTuning.y,
		adaptiveTuning.z,
		adaptiveFiltering.x,
		adaptiveFiltering.y,
		adaptiveFiltering.z);

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadMagnetometerCalibrationControl(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	size_t length = sprintf(buffer, "$VNRRG,44");

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genWriteMagnetometerCalibrationControl(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, uint8_t hsiMode, uint8_t hsiOutput, uint8_t convergeRate)
{
	size_t length = sprintf(buffer, "$VNWRG,44,%u,%u,%u",
		hsiMode,
		hsiOutput,
		convergeRate);

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadCalculatedMagnetometerCalibration(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	size_t length = sprintf(buffer, "$VNRRG,47");

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadVelocityCompensationControl(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	size_t length = sprintf(buffer, "$VNRRG,51");

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genWriteVelocityCompensationControl(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, uint8_t mode, float velocityTuning, float rateTuning)
{
	size_t length = sprintf(buffer, "$VNWRG,51,%u,%f,%f",
		mode,
		velocityTuning,
		rateTuning);

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadVelocityCompensationStatus(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	size_t length = sprintf(buffer, "$VNRRG,52");

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadVelocityCompensationMeasurement(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	size_t length = sprintf(buffer, "$VNRRG,50");

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genWriteVelocityCompensationMeasurement(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, vec3f velocity)
{
	size_t length = sprintf(buffer, "$VNWRG,50,%f,%f,%f",
		velocity.x,
		velocity.y,
		velocity.z);

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadMagneticAndGravityReferenceVectors(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	size_t length = sprintf(buffer, "$VNRRG,21");

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genWriteMagneticAndGravityReferenceVectors(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, vec3f magRef, vec3f accRef)
{
	size_t length = sprintf(buffer, "$VNWRG,21,%f,%f,%f,%f,%f,%f",
		magRef.x,
		magRef.y,
		magRef.z,
		accRef.x,
		accRef.y,
		accRef.z);

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadReferenceVectorConfiguration(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	size_t length = sprintf(buffer, "$VNRRG,83");

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genWriteReferenceVectorConfiguration(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, uint8_t useMagModel, uint8_t useGravityModel, uint8_t resv1, uint8_t resv2, uint32_t recalcThreshold, float year, vec3d position)
{
	size_t length = sprintf(buffer, "$VNWRG,83,%u,%u,%u,%u,%u,%f,%f,%f,%f",
		useMagModel,
		useGravityModel,
		resv1,
		resv2,
		recalcThreshold,
		year,
		position.x,
		position.y,
		position.z);

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadGpsSolutionLla(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	size_t length = sprintf(buffer, "$VNRRG,58");

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadGpsSolutionEcef(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	size_t length = sprintf(buffer, "$VNRRG,59");

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadGpsConfiguration(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	size_t length = sprintf(buffer, "$VNRRG,55");

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genWriteGpsConfiguration(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, uint8_t mode, uint8_t ppsSource, uint8_t reserved1, uint8_t reserved2, uint8_t reserved3)
{
	size_t length = sprintf(buffer, "$VNWRG,55,%u,%u,%u,%u,%u",
		mode,
		ppsSource,
		reserved1,
		reserved2,
		reserved3);

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadGpsAntennaOffset(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	size_t length = sprintf(buffer, "$VNRRG,57");

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genWriteGpsAntennaOffset(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, vec3f position)
{
	size_t length = sprintf(buffer, "$VNWRG,57,%f,%f,%f",
		position.x,
		position.y,
		position.z);

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadGpsCompassBaseline(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	size_t length = sprintf(buffer, "$VNRRG,93");

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genWriteGpsCompassBaseline(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, vec3f position, vec3f uncertainty)
{
	size_t length = sprintf(buffer, "$VNWRG,93,%f,%f,%f,%f,%f,%f",
		position.x,
		position.y,
		position.z,
		uncertainty.x,
		uncertainty.y,
		uncertainty.z);

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadGpsCompassEstimatedBaseline(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	size_t length = sprintf(buffer, "$VNRRG,97");

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadInsSolutionLla(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	size_t length = sprintf(buffer, "$VNRRG,63");

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadInsSolutionEcef(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	size_t length = sprintf(buffer, "$VNRRG,64");

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadInsStateLla(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	size_t length = sprintf(buffer, "$VNRRG,72");

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadInsStateEcef(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	size_t length = sprintf(buffer, "$VNRRG,73");

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadInsBasicConfiguration(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	size_t length = sprintf(buffer, "$VNRRG,67");

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genWriteInsBasicConfiguration(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, uint8_t scenario, uint8_t ahrsAiding, uint8_t estBaseline, uint8_t resv2)
{
	size_t length = sprintf(buffer, "$VNWRG,67,%u,%u,%u,%u",
		scenario,
		ahrsAiding,
		estBaseline,
		resv2);

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadStartupFilterBiasEstimate(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	size_t length = sprintf(buffer, "$VNRRG,74");

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genWriteStartupFilterBiasEstimate(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, vec3f gyroBias, vec3f accelBias, float pressureBias)
{
	size_t length = sprintf(buffer, "$VNWRG,74,%f,%f,%f,%f,%f,%f,%f",
		gyroBias.x,
		gyroBias.y,
		gyroBias.z,
		accelBias.x,
		accelBias.y,
		accelBias.z,
		pressureBias);

	return finalizeCommand(errorDetectionMode, buffer, length);
}


void Packet::parseVNYPR(vec3f* yawPitchRoll)
{
	size_t parseIndex;

	char* result = startAsciiPacketParse(_data, parseIndex);

	yawPitchRoll->x = ATOFF; NEXT
	yawPitchRoll->y = ATOFF; NEXT
	yawPitchRoll->z = ATOFF;
}

void Packet::parseVNQTN(vec4f* quaternion)
{
	size_t parseIndex;

	char* result = startAsciiPacketParse(_data, parseIndex);

	quaternion->x = ATOFF; NEXT
	quaternion->y = ATOFF; NEXT
	quaternion->z = ATOFF; NEXT
	quaternion->w = ATOFF;
}

void Packet::parseVNQMR(vec4f* quaternion, vec3f* magnetic, vec3f* acceleration, vec3f* angularRate)
{
	size_t parseIndex;

	char* result = startAsciiPacketParse(_data, parseIndex);

	quaternion->x = ATOFF; NEXT
	quaternion->y = ATOFF; NEXT
	quaternion->z = ATOFF; NEXT
	quaternion->w = ATOFF; NEXT
	magnetic->x = ATOFF; NEXT
	magnetic->y = ATOFF; NEXT
	magnetic->z = ATOFF; NEXT
	acceleration->x = ATOFF; NEXT
	acceleration->y = ATOFF; NEXT
	acceleration->z = ATOFF; NEXT
	angularRate->x = ATOFF; NEXT
	angularRate->y = ATOFF; NEXT
	angularRate->z = ATOFF;
}

void Packet::parseVNMAG(vec3f* magnetic)
{
	size_t parseIndex;

	char* result = startAsciiPacketParse(_data, parseIndex);

	magnetic->x = ATOFF; NEXT
	magnetic->y = ATOFF; NEXT
	magnetic->z = ATOFF;
}

void Packet::parseVNACC(vec3f* acceleration)
{
	size_t parseIndex;

	char* result = startAsciiPacketParse(_data, parseIndex);

	acceleration->x = ATOFF; NEXT
	acceleration->y = ATOFF; NEXT
	acceleration->z = ATOFF;
}

void Packet::parseVNGYR(vec3f* angularRate)
{
	size_t parseIndex;

	char* result = startAsciiPacketParse(_data, parseIndex);

	angularRate->x = ATOFF; NEXT
	angularRate->y = ATOFF; NEXT
	angularRate->z = ATOFF;
}

void Packet::parseVNMAR(vec3f* magnetic, vec3f* acceleration, vec3f* angularRate)
{
	size_t parseIndex;

	char* result = startAsciiPacketParse(_data, parseIndex);

	magnetic->x = ATOFF; NEXT
	magnetic->y = ATOFF; NEXT
	magnetic->z = ATOFF; NEXT
	acceleration->x = ATOFF; NEXT
	acceleration->y = ATOFF; NEXT
	acceleration->z = ATOFF; NEXT
	angularRate->x = ATOFF; NEXT
	angularRate->y = ATOFF; NEXT
	angularRate->z = ATOFF;
}

void Packet::parseVNYMR(vec3f* yawPitchRoll, vec3f* magnetic, vec3f* acceleration, vec3f* angularRate)
{
	size_t parseIndex;

	char* result = startAsciiPacketParse(_data, parseIndex);

	yawPitchRoll->x = ATOFF; NEXT
	yawPitchRoll->y = ATOFF; NEXT
	yawPitchRoll->z = ATOFF; NEXT
	magnetic->x = ATOFF; NEXT
	magnetic->y = ATOFF; NEXT
	magnetic->z = ATOFF; NEXT
	acceleration->x = ATOFF; NEXT
	acceleration->y = ATOFF; NEXT
	acceleration->z = ATOFF; NEXT
	angularRate->x = ATOFF; NEXT
	angularRate->y = ATOFF; NEXT
	angularRate->z = ATOFF;
}

void Packet::parseVNYBA(vec3f* yawPitchRoll, vec3f* accelerationBody, vec3f* angularRate)
{
	size_t parseIndex;

	char* result = startAsciiPacketParse(_data, parseIndex);

	yawPitchRoll->x = ATOFF; NEXT
	yawPitchRoll->y = ATOFF; NEXT
	yawPitchRoll->z = ATOFF; NEXT
	accelerationBody->x = ATOFF; NEXT
	accelerationBody->y = ATOFF; NEXT
	accelerationBody->z = ATOFF; NEXT
	angularRate->x = ATOFF; NEXT
	angularRate->y = ATOFF; NEXT
	angularRate->z = ATOFF;
}

void Packet::parseVNYIA(vec3f* yawPitchRoll, vec3f* accelerationInertial, vec3f* angularRate)
{
	size_t parseIndex;

	char* result = startAsciiPacketParse(_data, parseIndex);

	yawPitchRoll->x = ATOFF; NEXT
	yawPitchRoll->y = ATOFF; NEXT
	yawPitchRoll->z = ATOFF; NEXT
	accelerationInertial->x = ATOFF; NEXT
	accelerationInertial->y = ATOFF; NEXT
	accelerationInertial->z = ATOFF; NEXT
	angularRate->x = ATOFF; NEXT
	angularRate->y = ATOFF; NEXT
	angularRate->z = ATOFF;
}

void Packet::parseVNIMU(vec3f* magneticUncompensated, vec3f* accelerationUncompensated, vec3f* angularRateUncompensated, float* temperature, float* pressure)
{
	size_t parseIndex;

	char* result = startAsciiPacketParse(_data, parseIndex);

	magneticUncompensated->x = ATOFF; NEXT
	magneticUncompensated->y = ATOFF; NEXT
	magneticUncompensated->z = ATOFF; NEXT
	accelerationUncompensated->x = ATOFF; NEXT
	accelerationUncompensated->y = ATOFF; NEXT
	accelerationUncompensated->z = ATOFF; NEXT
	angularRateUncompensated->x = ATOFF; NEXT
	angularRateUncompensated->y = ATOFF; NEXT
	angularRateUncompensated->z = ATOFF; NEXT
	*temperature = ATOFF; NEXT
	*pressure = ATOFF;
}

void Packet::parseVNGPS(double* time, uint16_t* week, uint8_t* gpsFix, uint8_t* numSats, vec3d* lla, vec3f* nedVel, vec3f* nedAcc, float* speedAcc, float* timeAcc)
{
	size_t parseIndex;

	char* result = startAsciiPacketParse(_data, parseIndex);

	*time = ATOFD; NEXT
	*week = static_cast<uint16_t>(atoi(result)); NEXT
	*gpsFix = static_cast<uint8_t>(atoi(result)); NEXT
	*numSats = static_cast<uint8_t>(atoi(result)); NEXT
	lla->x = ATOFD; NEXT
	lla->y = ATOFD; NEXT
	lla->z = ATOFD; NEXT
	nedVel->x = ATOFF; NEXT
	nedVel->y = ATOFF; NEXT
	nedVel->z = ATOFF; NEXT
	nedAcc->x = ATOFF; NEXT
	nedAcc->y = ATOFF; NEXT
	nedAcc->z = ATOFF; NEXT
	*speedAcc = ATOFF; NEXT
	*timeAcc = ATOFF;
}

void Packet::parseVNINS(double* time, uint16_t* week, uint16_t* status, vec3f* yawPitchRoll, vec3d* lla, vec3f* nedVel, float* attUncertainty, float* posUncertainty, float* velUncertainty)
{
	size_t parseIndex;

	char* result = startAsciiPacketParse(_data, parseIndex);

	*time = ATOFD; NEXT
	*week = static_cast<uint16_t>(atoi(result)); NEXT
	*status = static_cast<uint16_t>(atoi(result)); NEXT
	yawPitchRoll->x = ATOFF; NEXT
	yawPitchRoll->y = ATOFF; NEXT
	yawPitchRoll->z = ATOFF; NEXT
	lla->x = ATOFD; NEXT
	lla->y = ATOFD; NEXT
	lla->z = ATOFD; NEXT
	nedVel->x = ATOFF; NEXT
	nedVel->y = ATOFF; NEXT
	nedVel->z = ATOFF; NEXT
	*attUncertainty = ATOFF; NEXT
	*posUncertainty = ATOFF; NEXT
	*velUncertainty = ATOFF;
}

void Packet::parseVNGPE(double* tow, uint16_t* week, uint8_t* gpsFix, uint8_t* numSats, vec3d* position, vec3f* velocity, vec3f* posAcc, float* speedAcc, float* timeAcc)
{
	size_t parseIndex;

	char* result = startAsciiPacketParse(_data, parseIndex);

	*tow = ATOFD; NEXT
	*week = static_cast<uint16_t>(atoi(result)); NEXT
	*gpsFix = static_cast<uint8_t>(atoi(result)); NEXT
	*numSats = static_cast<uint8_t>(atoi(result)); NEXT
	position->x = ATOFD; NEXT
	position->y = ATOFD; NEXT
	position->z = ATOFD; NEXT
	velocity->x = ATOFF; NEXT
	velocity->y = ATOFF; NEXT
	velocity->z = ATOFF; NEXT
	posAcc->x = ATOFF; NEXT
	posAcc->y = ATOFF; NEXT
	posAcc->z = ATOFF; NEXT
	*speedAcc = ATOFF; NEXT
	*timeAcc = ATOFF;
}

void Packet::parseVNDTV(float* deltaTime, vec3f* deltaTheta, vec3f* deltaVelocity)
{
	size_t parseIndex;

	char* result = startAsciiPacketParse(_data, parseIndex);

	*deltaTime = ATOFF; NEXT
	deltaTheta->x = ATOFF; NEXT
	deltaTheta->y = ATOFF; NEXT
	deltaTheta->z = ATOFF; NEXT
	deltaVelocity->x = ATOFF; NEXT
	deltaVelocity->y = ATOFF; NEXT
	deltaVelocity->z = ATOFF;
}

size_t Packet::computeBinaryPacketLength(char const* startOfPossibleBinaryPacket)
{
	char groupsPresent = startOfPossibleBinaryPacket[1];
	size_t runningPayloadLength = 2;	// Start of packet character plus groups present field.
	const char* pCurrentGroupField = startOfPossibleBinaryPacket + 2;

	if (groupsPresent & 0x01)
	{
		runningPayloadLength += 2 + computeNumOfBytesForBinaryGroupPayload(Packet::Common, stoh(*reinterpret_cast<const uint16_t*>(pCurrentGroupField)));
		pCurrentGroupField += 2;
	}

	if (groupsPresent & 0x02)
	{
		runningPayloadLength += 2 + computeNumOfBytesForBinaryGroupPayload(Packet::Time, stoh(*reinterpret_cast<const uint16_t*>(pCurrentGroupField)));
		pCurrentGroupField += 2;
	}

	if (groupsPresent & 0x04)
	{
		runningPayloadLength += 2 + computeNumOfBytesForBinaryGroupPayload(Packet::Imu, stoh(*reinterpret_cast<const uint16_t*>(pCurrentGroupField)));
		pCurrentGroupField += 2;
	}

	if (groupsPresent & 0x08)
	{
		runningPayloadLength += 2 + computeNumOfBytesForBinaryGroupPayload(Packet::Gps, stoh(*reinterpret_cast<const uint16_t*>(pCurrentGroupField)));
		pCurrentGroupField += 2;
	}

	if (groupsPresent & 0x10)
	{
		runningPayloadLength += 2 + computeNumOfBytesForBinaryGroupPayload(Packet::Attitude, stoh(*reinterpret_cast<const uint16_t*>(pCurrentGroupField)));
		pCurrentGroupField += 2;
	}

	if (groupsPresent & 0x20)
	{
		runningPayloadLength += 2 + computeNumOfBytesForBinaryGroupPayload(Packet::Ins, stoh(*reinterpret_cast<const uint16_t*>(pCurrentGroupField)));
		pCurrentGroupField += 2;
	}

	return runningPayloadLength + 2;	// Add 2 bytes for CRC.
}

size_t Packet::computeNumOfBytesForBinaryGroupPayload(BinaryGroupType group, uint16_t groupField)
{
	size_t runningLength = 0;

	// Determine which group is present.
	size_t groupIndex = 0;
	for (size_t i = 0; i < 8; i++, groupIndex++)
	{
		if ((static_cast<size_t>(group) >> i) & 0x01)
			break;
	}

	for (size_t i = 0; i < sizeof(uint16_t) * 8; i++)
	{
		if ((groupField >> i) & 1)
		{
			runningLength += BinaryGroupLengths[groupIndex][i];
		}
	}

	return runningLength;
}

SensorError Packet::parseError()
{
	size_t parseIndex;

	char* result = startAsciiPacketParse(_data, parseIndex);

	return static_cast<SensorError>(atoi(result));
}

struct PacketFinder::Impl
{
	static const size_t DefaultReceiveBufferSize = 512;
	static const uint8_t AsciiStartChar = '$';
	static const uint8_t BinaryStartChar = 0xFA;
	static const uint8_t AsciiEndChar1 = '\r';
	static const uint8_t AsciiEndChar2 = '\n';
	static const size_t MaximumSizeExpectedForBinaryPacket = 256;
	static const size_t MaximumSizeForBinaryStartAndAllGroupData = 18;
	static const size_t MaximumSizeForAsciiPacket = 256;

	struct AsciiTracker
	{
		bool currentlyBuildingAsciiPacket;
		size_t possibleStartOfPacketIndex;
		bool asciiEndChar1Found;
		size_t runningDataIndexOfStart;

		AsciiTracker() :
			currentlyBuildingAsciiPacket(false),
			possibleStartOfPacketIndex(0),
			asciiEndChar1Found(false),
			runningDataIndexOfStart(0)
		{ }

		void reset()
		{
			currentlyBuildingAsciiPacket = false;
			possibleStartOfPacketIndex = 0;
			asciiEndChar1Found = false;
			runningDataIndexOfStart = 0;
		}
	};

	struct BinaryTracker
	{
		size_t possibleStartIndex;
		bool groupsPresentFound;
		uint8_t groupsPresent;
		uint8_t numOfBytesRemainingToHaveAllGroupFields;
		size_t numOfBytesRemainingForCompletePacket;
		bool startFoundInProvidedDataBuffer;
		size_t runningDataIndexOfStart;

		explicit BinaryTracker(size_t possibleStartIndex, size_t runningDataIndex) :
			possibleStartIndex(possibleStartIndex),
			groupsPresentFound(false),
			numOfBytesRemainingToHaveAllGroupFields(0),
			numOfBytesRemainingForCompletePacket(0),
			startFoundInProvidedDataBuffer(true),
			runningDataIndexOfStart(runningDataIndex)
		{ }

		bool operator==(const BinaryTracker &rhs)
		{
			return
				possibleStartIndex == rhs.possibleStartIndex &&
				groupsPresentFound == rhs.groupsPresentFound &&
				groupsPresent == rhs.groupsPresent &&
				numOfBytesRemainingToHaveAllGroupFields == rhs.numOfBytesRemainingToHaveAllGroupFields &&
				numOfBytesRemainingForCompletePacket == rhs.numOfBytesRemainingForCompletePacket;
		}
	};

	PacketFinder* _backReference;
	uint8_t* _buffer;
	const size_t _bufferSize;
	size_t _bufferAppendLocation;
	AsciiTracker _asciiOnDeck;
	list<BinaryTracker> _binaryOnDeck;	// Collection of possible binary packets we are checking.
	size_t _runningDataIndex;			// Used for correlating raw data with where the packet was found for the end user.
	void* _possiblePacketFoundUserData;
	ValidPacketFoundHandler _possiblePacketFoundHandler;

	explicit Impl(PacketFinder* backReference) :
		_backReference(backReference),
		_buffer(new uint8_t[DefaultReceiveBufferSize]),
		_bufferSize(DefaultReceiveBufferSize),
		_bufferAppendLocation(0),
		_runningDataIndex(0),
		_possiblePacketFoundUserData(NULL),
		_possiblePacketFoundHandler(NULL)
	{ }

	Impl(PacketFinder* backReference, size_t internalReceiveBufferSize) :
		_backReference(backReference),
		_buffer(new uint8_t[internalReceiveBufferSize]),
		_bufferSize(internalReceiveBufferSize),
		_bufferAppendLocation(0),
		_runningDataIndex(0),
		_possiblePacketFoundUserData(NULL),
		_possiblePacketFoundHandler(NULL)
	{ }

	~Impl()
	{
		delete [] _buffer;
	}

	void resetTracking()
	{
		_asciiOnDeck.reset();
		_binaryOnDeck.clear();
		_bufferAppendLocation = 0;
	}

	void dataReceived(uint8_t data[], size_t length)
	{
		bool asciiStartFoundInProvidedBuffer = false;

		// Assume that since the _runningDataIndex is unsigned, any overflows
		// will naturally go to zero, which is the behavior that we want.
		for (size_t i = 0; i < length; i++, _runningDataIndex++)
		{
			if (data[i] == AsciiStartChar)
			{
				_asciiOnDeck.reset();
				_asciiOnDeck.currentlyBuildingAsciiPacket = true;
				_asciiOnDeck.possibleStartOfPacketIndex = i;
				_asciiOnDeck.runningDataIndexOfStart = _runningDataIndex;

				asciiStartFoundInProvidedBuffer = true;
			}
			else if (_asciiOnDeck.currentlyBuildingAsciiPacket && data[i] == AsciiEndChar1)
			{
				_asciiOnDeck.asciiEndChar1Found = true;
			}
			else if (_asciiOnDeck.asciiEndChar1Found)
			{
				if (data[i] == AsciiEndChar2)
				{
					// We have a possible data packet.
					Packet p;
					size_t runningIndexOfPacketStart = _asciiOnDeck.runningDataIndexOfStart;
					uint8_t* startOfAsciiPacket = NULL;
					size_t packetLength = 0;

					if (asciiStartFoundInProvidedBuffer)
					{
						// All the packet was in this data buffer so we don't
						// need to do any copying.

						startOfAsciiPacket = data + _asciiOnDeck.possibleStartOfPacketIndex;
						packetLength = i - _asciiOnDeck.possibleStartOfPacketIndex + 1;
					}
					else
					{
						// The packet was split between the running data buffer
						// the current data buffer. We need to copy the data
						// over before further processing.

						if (_bufferAppendLocation + i < _bufferSize)
						{
							memcpy(_buffer + _bufferAppendLocation, data, i + 1);

							startOfAsciiPacket = _buffer + _asciiOnDeck.possibleStartOfPacketIndex;
							packetLength = _bufferAppendLocation + i + 1 - _asciiOnDeck.possibleStartOfPacketIndex;
						}
						else
						{
							// We are about to overflow our buffer. Just fall
							// through to reset tracking.
						}
					}

					p = Packet(Packet::TYPE_ASCII, reinterpret_cast<char*>(startOfAsciiPacket), packetLength);

					if (p.isValid())
						dispatchPacket(p, runningIndexOfPacketStart);
				}
				
				// Either this is an invalid packet or was a packet that was processed.
				if (_binaryOnDeck.empty())
					resetTracking();
				else
					_asciiOnDeck.reset();
				asciiStartFoundInProvidedBuffer = false;
			}
			else if (i + 1 > MaximumSizeForAsciiPacket)
			{
				// This must not be a valid ASCII packet.
				if (_binaryOnDeck.empty())
				{
					resetTracking();
				}
				else
				{
					_asciiOnDeck.reset();
					asciiStartFoundInProvidedBuffer = false;
				}
			}

			// Update all of our binary packets on deck.
			queue<BinaryTracker> invalidPackets;
			for (list<BinaryTracker>::iterator it = _binaryOnDeck.begin(); it != _binaryOnDeck.end(); ++it)
			{
				BinaryTracker &ez = (*it);

				if (!ez.groupsPresentFound)
				{
					// This byte must be the groups present.
					ez.groupsPresentFound = true;
					ez.groupsPresent = data[i];
					ez.numOfBytesRemainingToHaveAllGroupFields = 2 * countSetBits(data[i]);

					continue;
				}

				if (ez.numOfBytesRemainingToHaveAllGroupFields != 0)
				{
					// We found another byte belonging to this possible binary packet.
					ez.numOfBytesRemainingToHaveAllGroupFields--;

					if (ez.numOfBytesRemainingToHaveAllGroupFields == 0)
					{
						// We have all of the group fields now.
						size_t remainingBytesForCompletePacket;
						if (ez.startFoundInProvidedDataBuffer)
						{
							size_t headerLength = i - ez.possibleStartIndex + 1;
							remainingBytesForCompletePacket = Packet::computeBinaryPacketLength(reinterpret_cast<char*>(data) + ez.possibleStartIndex) - headerLength;
						}
						else
						{
							// Not all of the packet's group is inside the caller's provided buffer.

							// Temporarily copy the rest of the packet to the receive buffer
							// for computing the size of the packet.

							size_t numOfBytesToCopyIntoReceiveBuffer = i + 1;
							size_t headerLength = _bufferAppendLocation - ez.possibleStartIndex + numOfBytesToCopyIntoReceiveBuffer;

							if (_bufferAppendLocation + numOfBytesToCopyIntoReceiveBuffer < _bufferSize)
							{
								copy(data, data + numOfBytesToCopyIntoReceiveBuffer, _buffer + _bufferAppendLocation);

								remainingBytesForCompletePacket = Packet::computeBinaryPacketLength(reinterpret_cast<char*>(_buffer) + ez.possibleStartIndex) - headerLength;
							}
							else
							{
								// About to overrun our receive buffer!
								invalidPackets.push(ez);

								// TODO: Should we just go ahead and clear the ASCII tracker
								//       and buffer append location?

								continue;
							}
						}

						if (remainingBytesForCompletePacket > MaximumSizeExpectedForBinaryPacket)
						{
							// Must be a bad possible binary packet.
							invalidPackets.push(ez);
						}
						else
						{
							ez.numOfBytesRemainingForCompletePacket = remainingBytesForCompletePacket;
						}
					}

					continue;
				}

				// We are currently collecting data for our packet.

				ez.numOfBytesRemainingForCompletePacket--;

				if (ez.numOfBytesRemainingForCompletePacket == 0)
				{
					// We have a possible binary packet!

					uint8_t* packetStart;
					size_t packetLength;
					Packet p;

					if (ez.startFoundInProvidedDataBuffer)
					{
						// The binary packet exists completely in the user's provided buffer.
						packetStart = data + ez.possibleStartIndex;
						packetLength = i - ez.possibleStartIndex + 1;
					}
					else
					{
						// The packet is split between our receive buffer and the user's buffer.
						size_t numOfBytesToCopyIntoReceiveBuffer = i + 1;

						if (_bufferAppendLocation + numOfBytesToCopyIntoReceiveBuffer < _bufferSize)
						{
							copy(data, data + numOfBytesToCopyIntoReceiveBuffer, _buffer + _bufferAppendLocation);

							packetStart = _buffer + ez.possibleStartIndex;
							packetLength = _bufferAppendLocation - ez.possibleStartIndex + i + 1;
						}
						else
						{
							// About to overrun our receive buffer!
							invalidPackets.push(ez);

							continue;
						}
					}

					p = Packet(Packet::TYPE_BINARY, reinterpret_cast<char*>(packetStart), packetLength);

					if (!p.isValid())
					{
						// Invalid packet!
						invalidPackets.push(ez);
					}
					else
					{
						// We have a valid binary packet!!!.
						size_t packetStartRunningIndex = (ez).runningDataIndexOfStart;
						invalidPackets = queue<BinaryTracker>();
						resetTracking();

						dispatchPacket(p, packetStartRunningIndex);

						break;
					}
				}
			}

			// Remove any invalid packets.
			while (!invalidPackets.empty())
			{
				_binaryOnDeck.remove(invalidPackets.front());
				invalidPackets.pop();
			}

			if (_binaryOnDeck.empty() && !_asciiOnDeck.currentlyBuildingAsciiPacket)
			{
				_bufferAppendLocation = 0;
			}

			if (data[i] == BinaryStartChar)
			{
				// Possible start of a binary packet.
				_binaryOnDeck.push_back(BinaryTracker(i, _runningDataIndex));
			}
		}

		if (_binaryOnDeck.empty() && !_asciiOnDeck.currentlyBuildingAsciiPacket)
			// No data to copy over.
			return;

		// Perform any data copying to our receive buffer.

		size_t dataIndexToStartCopyingFrom = 0;
		bool binaryDataToCopyOver = false;
		size_t binaryDataMoveOverIndexAdjustment = 0;

		if (!_binaryOnDeck.empty())
		{
			binaryDataToCopyOver = true;

			if (_binaryOnDeck.front().startFoundInProvidedDataBuffer)
			{
				dataIndexToStartCopyingFrom = _binaryOnDeck.front().possibleStartIndex;
				binaryDataMoveOverIndexAdjustment = dataIndexToStartCopyingFrom;
			}
		}

		if (_asciiOnDeck.currentlyBuildingAsciiPacket && asciiStartFoundInProvidedBuffer)
		{
			if (_asciiOnDeck.possibleStartOfPacketIndex < dataIndexToStartCopyingFrom)
			{
				binaryDataMoveOverIndexAdjustment -= binaryDataMoveOverIndexAdjustment - _asciiOnDeck.possibleStartOfPacketIndex;
				dataIndexToStartCopyingFrom = _asciiOnDeck.possibleStartOfPacketIndex;
			}
			else if (!binaryDataToCopyOver)
			{
				dataIndexToStartCopyingFrom = _asciiOnDeck.possibleStartOfPacketIndex;
			}

			// Adjust our ASCII index to be based on the recieve buffer.
			_asciiOnDeck.possibleStartOfPacketIndex = _bufferAppendLocation + _asciiOnDeck.possibleStartOfPacketIndex - dataIndexToStartCopyingFrom;
		}

		// Adjust any binary packet indexes we are currently building.
		for (list<BinaryTracker>::iterator it = _binaryOnDeck.begin(); it != _binaryOnDeck.end(); ++it)
		{
			if ((*it).startFoundInProvidedDataBuffer)
			{
				(*it).startFoundInProvidedDataBuffer = false;
				(*it).possibleStartIndex = (*it).possibleStartIndex - binaryDataMoveOverIndexAdjustment + _bufferAppendLocation;
			}
		}

		if (_bufferAppendLocation + length - dataIndexToStartCopyingFrom < _bufferSize)
		{
			// Safe to copy over the data.

			size_t numOfBytesToCopyOver = length - dataIndexToStartCopyingFrom;
			uint8_t *copyFromStart = data + dataIndexToStartCopyingFrom;

			copy(copyFromStart, copyFromStart + numOfBytesToCopyOver, _buffer + _bufferAppendLocation);

			_bufferAppendLocation += numOfBytesToCopyOver;
		}
		else
		{
			// We are about to overflow our buffer.
			resetTracking();
		}
	}

	void dispatchPacket(Packet &packet, size_t runningDataIndexAtPacketStart)
	{
		if (_possiblePacketFoundHandler == NULL)
			return;

		_possiblePacketFoundHandler(_possiblePacketFoundUserData, packet, runningDataIndexAtPacketStart);
	}
};

#if defined(_MSC_VER) && _MSC_VER <= 1600
	#pragma warning(push)
	// Disable VS2010 warning for 'this' used in base member initializer list.
	#pragma warning(disable:4355)
#endif

PacketFinder::PacketFinder() :
	_pi(new Impl(this))
{
}

PacketFinder::PacketFinder(size_t internalReceiveBufferSize) :
	_pi(new Impl(this, internalReceiveBufferSize))
{
}

#if defined(_MSC_VER) && _MSC_VER <= 1600
	#pragma warning(pop)
#endif

PacketFinder::~PacketFinder()
{
	delete _pi;
}

void PacketFinder::processReceivedData(char data[], size_t length)
{
	_pi->dataReceived(reinterpret_cast<uint8_t*>(data), length);
}

void PacketFinder::registerPossiblePacketFoundHandler(void* userData, ValidPacketFoundHandler handler)
{
	if (_pi->_possiblePacketFoundHandler != NULL)
		throw invalid_operation();

	_pi->_possiblePacketFoundHandler = handler;
	_pi->_possiblePacketFoundUserData = userData;
}

void PacketFinder::unregisterPossiblePacketFoundHandler()
{
	if (_pi->_possiblePacketFoundHandler == NULL)
		throw invalid_operation();

	_pi->_possiblePacketFoundHandler = NULL;
	_pi->_possiblePacketFoundUserData = NULL;
}

void Packet::parseBinaryOutput(
	uint16_t* asyncMode,
	uint16_t* rateDivisor,
	uint16_t* outputGroup,
	uint16_t* commonField,
	uint16_t* timeField,
	uint16_t* imuField,
	uint16_t* gpsField,
	uint16_t* attitudeField,
	uint16_t* insField)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	*commonField = 0;
	*timeField = 0;
	*imuField = 0;
	*gpsField = 0;
	*attitudeField = 0;
	*insField = 0;

	*asyncMode = ATOU16; NEXT
	*rateDivisor = ATOU16; NEXT
	*outputGroup = ATOU16;
	if (*outputGroup & 0x0001)
	{
		NEXT
		*commonField = ATOU16;
	}
	if (*outputGroup & 0x0002)
	{
		NEXT
		*timeField = ATOU16;
	}
	if (*outputGroup & 0x0004)
	{
		NEXT
		*imuField = ATOU16;
	}
	if (*outputGroup & 0x0008)
	{
		NEXT
		*gpsField = ATOU16;
	}
	if (*outputGroup & 0x0010)
	{
		NEXT
		*attitudeField = ATOU16;
	}
	if (*outputGroup & 0x0020)
	{
		NEXT
		*insField = ATOU16;
	}
}

void Packet::parseUserTag(char* tag)
{
	size_t parseIndex;

	char* next = startAsciiPacketParse(_data, parseIndex);

	if (*(next + strlen(next) + 1) == '*')
	{
		tag[0] = '\0';
		return;
	}

	next = getNextData(_data, parseIndex);
	strcpy(tag, next);
}

void Packet::parseModelNumber(char* productName)
{
	size_t parseIndex;

	char* next = startAsciiPacketParse(_data, parseIndex);

	if (*(next + strlen(next) + 1) == '*')
	{
		productName[0] = '\0';
		return;
	}

	next = getNextData(_data, parseIndex);
	strcpy(productName, next);
}

void Packet::parseHardwareRevision(uint32_t* revision)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	*revision = ATOU32;
}

void Packet::parseSerialNumber(uint32_t* serialNum)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	*serialNum = ATOU32;
}

void Packet::parseFirmwareVersion(char* firmwareVersion)
{
	size_t parseIndex;

	char* next = startAsciiPacketParse(_data, parseIndex);

	if (*(next + strlen(next) + 1) == '*')
	{
		firmwareVersion[0] = '\0';
		return;
	}

	next = getNextData(_data, parseIndex);
	strcpy(firmwareVersion, next);
}

void Packet::parseSerialBaudRate(uint32_t* baudrate)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	*baudrate = ATOU32;
}

void Packet::parseAsyncDataOutputType(uint32_t* ador)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	*ador = ATOU32;
}

void Packet::parseAsyncDataOutputFrequency(uint32_t* adof)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	*adof = ATOU32;
}

void Packet::parseSynchronizationControl(uint8_t* syncInMode, uint8_t* syncInEdge, uint16_t* syncInSkipFactor, uint8_t* syncOutMode, uint8_t* syncOutPolarity, uint16_t* syncOutSkipFactor, uint32_t* syncOutPulseWidth)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	*syncInMode = ATOU8; NEXT
	*syncInEdge = ATOU8; NEXT
	*syncInSkipFactor = ATOU16; NEXT
	NEXT
	*syncOutMode = ATOU8; NEXT
	*syncOutPolarity = ATOU8; NEXT
	*syncOutSkipFactor = ATOU16; NEXT
	*syncOutPulseWidth = ATOU32; NEXT
	NEXT
}

void Packet::parseCommunicationProtocolControl(uint8_t* serialCount, uint8_t* serialStatus, uint8_t* spiCount, uint8_t* spiStatus, uint8_t* serialChecksum, uint8_t* spiChecksum, uint8_t* errorMode)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	*serialCount = ATOU8; NEXT
	*serialStatus = ATOU8; NEXT
	*spiCount = ATOU8; NEXT
	*spiStatus = ATOU8; NEXT
	*serialChecksum = ATOU8; NEXT
	*spiChecksum = ATOU8; NEXT
	*errorMode = ATOU8;
}

void Packet::parseSynchronizationStatus(uint32_t* syncInCount, uint32_t* syncInTime, uint32_t* syncOutCount)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	*syncInCount = ATOU32; NEXT
	*syncInTime = ATOU32; NEXT
	*syncOutCount = ATOU32;
}

void Packet::parseImuMeasurements(vec3f* mag, vec3f* accel, vec3f* gyro, float* temp, float* pressure)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	mag->x = ATOFF; NEXT
	mag->y = ATOFF; NEXT
	mag->z = ATOFF; NEXT
	accel->x = ATOFF; NEXT
	accel->y = ATOFF; NEXT
	accel->z = ATOFF; NEXT
	gyro->x = ATOFF; NEXT
	gyro->y = ATOFF; NEXT
	gyro->z = ATOFF; NEXT
	*temp = ATOFF; NEXT
	*pressure = ATOFF;
}

void Packet::parseDeltaThetaAndDeltaVelocity(float* deltaTime, vec3f* deltaTheta, vec3f* deltaVelocity)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	*deltaTime = ATOFF; NEXT
	deltaTheta->x = ATOFF; NEXT
	deltaTheta->y = ATOFF; NEXT
	deltaTheta->z = ATOFF; NEXT
	deltaVelocity->x = ATOFF; NEXT
	deltaVelocity->y = ATOFF; NEXT
	deltaVelocity->z = ATOFF;
}

void Packet::parseMagnetometerCompensation(mat3f* c, vec3f* b)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	c->e00 = ATOFF; NEXT
	c->e01 = ATOFF; NEXT
	c->e02 = ATOFF; NEXT
	c->e10 = ATOFF; NEXT
	c->e11 = ATOFF; NEXT
	c->e12 = ATOFF; NEXT
	c->e20 = ATOFF; NEXT
	c->e21 = ATOFF; NEXT
	c->e22 = ATOFF; NEXT
	b->x = ATOFF; NEXT
	b->y = ATOFF; NEXT
	b->z = ATOFF;
}

void Packet::parseAccelerationCompensation(mat3f* c, vec3f* b)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	c->e00 = ATOFF; NEXT
	c->e01 = ATOFF; NEXT
	c->e02 = ATOFF; NEXT
	c->e10 = ATOFF; NEXT
	c->e11 = ATOFF; NEXT
	c->e12 = ATOFF; NEXT
	c->e20 = ATOFF; NEXT
	c->e21 = ATOFF; NEXT
	c->e22 = ATOFF; NEXT
	b->x = ATOFF; NEXT
	b->y = ATOFF; NEXT
	b->z = ATOFF;
}

void Packet::parseGyroCompensation(mat3f* c, vec3f* b)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	c->e00 = ATOFF; NEXT
	c->e01 = ATOFF; NEXT
	c->e02 = ATOFF; NEXT
	c->e10 = ATOFF; NEXT
	c->e11 = ATOFF; NEXT
	c->e12 = ATOFF; NEXT
	c->e20 = ATOFF; NEXT
	c->e21 = ATOFF; NEXT
	c->e22 = ATOFF; NEXT
	b->x = ATOFF; NEXT
	b->y = ATOFF; NEXT
	b->z = ATOFF;
}

void Packet::parseReferenceFrameRotation(mat3f* c)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	c->e00 = ATOFF; NEXT
	c->e01 = ATOFF; NEXT
	c->e02 = ATOFF; NEXT
	c->e10 = ATOFF; NEXT
	c->e11 = ATOFF; NEXT
	c->e12 = ATOFF; NEXT
	c->e20 = ATOFF; NEXT
	c->e21 = ATOFF; NEXT
	c->e22 = ATOFF;
}

void Packet::parseImuFilteringConfiguration(uint16_t* magWindowSize, uint16_t* accelWindowSize, uint16_t* gyroWindowSize, uint16_t* tempWindowSize, uint16_t* presWindowSize, uint8_t* magFilterMode, uint8_t* accelFilterMode, uint8_t* gyroFilterMode, uint8_t* tempFilterMode, uint8_t* presFilterMode)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	*magWindowSize = ATOU16; NEXT
	*accelWindowSize = ATOU16; NEXT
	*gyroWindowSize = ATOU16; NEXT
	*tempWindowSize = ATOU16; NEXT
	*presWindowSize = ATOU16; NEXT
	*magFilterMode = ATOU8; NEXT
	*accelFilterMode = ATOU8; NEXT
	*gyroFilterMode = ATOU8; NEXT
	*tempFilterMode = ATOU8; NEXT
	*presFilterMode = ATOU8;
}

void Packet::parseDeltaThetaAndDeltaVelocityConfiguration(uint8_t* integrationFrame, uint8_t* gyroCompensation, uint8_t* accelCompensation)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	*integrationFrame = ATOU8; NEXT
	*gyroCompensation = ATOU8; NEXT
	*accelCompensation = ATOU8; NEXT
	NEXT
	NEXT
}

void Packet::parseYawPitchRoll(vec3f* yawPitchRoll)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	yawPitchRoll->x = ATOFF; NEXT
	yawPitchRoll->y = ATOFF; NEXT
	yawPitchRoll->z = ATOFF;
}

void Packet::parseAttitudeQuaternion(vec4f* quat)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	quat->x = ATOFF; NEXT
	quat->y = ATOFF; NEXT
	quat->z = ATOFF; NEXT
	quat->w = ATOFF;
}

void Packet::parseYawPitchRollMagneticAccelerationAndAngularRates(vec3f* yawPitchRoll, vec3f* mag, vec3f* accel, vec3f* gyro)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	yawPitchRoll->x = ATOFF; NEXT
	yawPitchRoll->y = ATOFF; NEXT
	yawPitchRoll->z = ATOFF; NEXT
	mag->x = ATOFF; NEXT
	mag->y = ATOFF; NEXT
	mag->z = ATOFF; NEXT
	accel->x = ATOFF; NEXT
	accel->y = ATOFF; NEXT
	accel->z = ATOFF; NEXT
	gyro->x = ATOFF; NEXT
	gyro->y = ATOFF; NEXT
	gyro->z = ATOFF;
}

void Packet::parseQuaternionMagneticAccelerationAndAngularRates(vec4f* quat, vec3f* mag, vec3f* accel, vec3f* gyro)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	quat->x = ATOFF; NEXT
	quat->y = ATOFF; NEXT
	quat->z = ATOFF; NEXT
	quat->w = ATOFF; NEXT
	mag->x = ATOFF; NEXT
	mag->y = ATOFF; NEXT
	mag->z = ATOFF; NEXT
	accel->x = ATOFF; NEXT
	accel->y = ATOFF; NEXT
	accel->z = ATOFF; NEXT
	gyro->x = ATOFF; NEXT
	gyro->y = ATOFF; NEXT
	gyro->z = ATOFF;
}

void Packet::parseMagneticMeasurements(vec3f* mag)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	mag->x = ATOFF; NEXT
	mag->y = ATOFF; NEXT
	mag->z = ATOFF;
}

void Packet::parseAccelerationMeasurements(vec3f* accel)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	accel->x = ATOFF; NEXT
	accel->y = ATOFF; NEXT
	accel->z = ATOFF;
}

void Packet::parseAngularRateMeasurements(vec3f* gyro)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	gyro->x = ATOFF; NEXT
	gyro->y = ATOFF; NEXT
	gyro->z = ATOFF;
}

void Packet::parseMagneticAccelerationAndAngularRates(vec3f* mag, vec3f* accel, vec3f* gyro)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	mag->x = ATOFF; NEXT
	mag->y = ATOFF; NEXT
	mag->z = ATOFF; NEXT
	accel->x = ATOFF; NEXT
	accel->y = ATOFF; NEXT
	accel->z = ATOFF; NEXT
	gyro->x = ATOFF; NEXT
	gyro->y = ATOFF; NEXT
	gyro->z = ATOFF;
}

void Packet::parseYawPitchRollTrueBodyAccelerationAndAngularRates(vec3f* yawPitchRoll, vec3f* bodyAccel, vec3f* gyro)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	yawPitchRoll->x = ATOFF; NEXT
	yawPitchRoll->y = ATOFF; NEXT
	yawPitchRoll->z = ATOFF; NEXT
	bodyAccel->x = ATOFF; NEXT
	bodyAccel->y = ATOFF; NEXT
	bodyAccel->z = ATOFF; NEXT
	gyro->x = ATOFF; NEXT
	gyro->y = ATOFF; NEXT
	gyro->z = ATOFF;
}

void Packet::parseYawPitchRollTrueInertialAccelerationAndAngularRates(vec3f* yawPitchRoll, vec3f* inertialAccel, vec3f* gyro)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	yawPitchRoll->x = ATOFF; NEXT
	yawPitchRoll->y = ATOFF; NEXT
	yawPitchRoll->z = ATOFF; NEXT
	inertialAccel->x = ATOFF; NEXT
	inertialAccel->y = ATOFF; NEXT
	inertialAccel->z = ATOFF; NEXT
	gyro->x = ATOFF; NEXT
	gyro->y = ATOFF; NEXT
	gyro->z = ATOFF;
}

void Packet::parseVpeBasicControl(uint8_t* enable, uint8_t* headingMode, uint8_t* filteringMode, uint8_t* tuningMode)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	*enable = ATOU8; NEXT
	*headingMode = ATOU8; NEXT
	*filteringMode = ATOU8; NEXT
	*tuningMode = ATOU8;
}

void Packet::parseVpeMagnetometerBasicTuning(vec3f* baseTuning, vec3f* adaptiveTuning, vec3f* adaptiveFiltering)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	baseTuning->x = ATOFF; NEXT
	baseTuning->y = ATOFF; NEXT
	baseTuning->z = ATOFF; NEXT
	adaptiveTuning->x = ATOFF; NEXT
	adaptiveTuning->y = ATOFF; NEXT
	adaptiveTuning->z = ATOFF; NEXT
	adaptiveFiltering->x = ATOFF; NEXT
	adaptiveFiltering->y = ATOFF; NEXT
	adaptiveFiltering->z = ATOFF;
}

void Packet::parseVpeAccelerometerBasicTuning(vec3f* baseTuning, vec3f* adaptiveTuning, vec3f* adaptiveFiltering)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	baseTuning->x = ATOFF; NEXT
	baseTuning->y = ATOFF; NEXT
	baseTuning->z = ATOFF; NEXT
	adaptiveTuning->x = ATOFF; NEXT
	adaptiveTuning->y = ATOFF; NEXT
	adaptiveTuning->z = ATOFF; NEXT
	adaptiveFiltering->x = ATOFF; NEXT
	adaptiveFiltering->y = ATOFF; NEXT
	adaptiveFiltering->z = ATOFF;
}

void Packet::parseMagnetometerCalibrationControl(uint8_t* hsiMode, uint8_t* hsiOutput, uint8_t* convergeRate)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	*hsiMode = ATOU8; NEXT
	*hsiOutput = ATOU8; NEXT
	*convergeRate = ATOU8;
}

void Packet::parseCalculatedMagnetometerCalibration(mat3f* c, vec3f* b)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	c->e00 = ATOFF; NEXT
	c->e01 = ATOFF; NEXT
	c->e02 = ATOFF; NEXT
	c->e10 = ATOFF; NEXT
	c->e11 = ATOFF; NEXT
	c->e12 = ATOFF; NEXT
	c->e20 = ATOFF; NEXT
	c->e21 = ATOFF; NEXT
	c->e22 = ATOFF; NEXT
	b->x = ATOFF; NEXT
	b->y = ATOFF; NEXT
	b->z = ATOFF;
}

void Packet::parseVelocityCompensationControl(uint8_t* mode, float* velocityTuning, float* rateTuning)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	*mode = ATOU8; NEXT
	*velocityTuning = ATOFF; NEXT
	*rateTuning = ATOFF;
}

void Packet::parseVelocityCompensationStatus(float* x, float* xDot, vec3f* accelOffset, vec3f* omega)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	*x = ATOFF; NEXT
	*xDot = ATOFF; NEXT
	accelOffset->x = ATOFF; NEXT
	accelOffset->y = ATOFF; NEXT
	accelOffset->z = ATOFF; NEXT
	omega->x = ATOFF; NEXT
	omega->y = ATOFF; NEXT
	omega->z = ATOFF;
}

void Packet::parseVelocityCompensationMeasurement(vec3f* velocity)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	velocity->x = ATOFF; NEXT
	velocity->y = ATOFF; NEXT
	velocity->z = ATOFF;
}

void Packet::parseMagneticAndGravityReferenceVectors(vec3f* magRef, vec3f* accRef)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	magRef->x = ATOFF; NEXT
	magRef->y = ATOFF; NEXT
	magRef->z = ATOFF; NEXT
	accRef->x = ATOFF; NEXT
	accRef->y = ATOFF; NEXT
	accRef->z = ATOFF;
}

void Packet::parseReferenceVectorConfiguration(uint8_t* useMagModel, uint8_t* useGravityModel, uint32_t* recalcThreshold, float* year, vec3d* position)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	*useMagModel = ATOU8; NEXT
	*useGravityModel = ATOU8; NEXT
	NEXT
	NEXT
	*recalcThreshold = ATOU32; NEXT
	*year = ATOFF; NEXT
	position->x = ATOFD; NEXT
	position->y = ATOFD; NEXT
	position->z = ATOFD;
}

void Packet::parseGpsSolutionLla(double* time, uint16_t* week, uint8_t* gpsFix, uint8_t* numSats, vec3d* lla, vec3f* nedVel, vec3f* nedAcc, float* speedAcc, float* timeAcc)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	*time = ATOFD; NEXT
	*week = ATOU16; NEXT
	*gpsFix = ATOU8; NEXT
	*numSats = ATOU8; NEXT
	lla->x = ATOFD; NEXT
	lla->y = ATOFD; NEXT
	lla->z = ATOFD; NEXT
	nedVel->x = ATOFF; NEXT
	nedVel->y = ATOFF; NEXT
	nedVel->z = ATOFF; NEXT
	nedAcc->x = ATOFF; NEXT
	nedAcc->y = ATOFF; NEXT
	nedAcc->z = ATOFF; NEXT
	*speedAcc = ATOFF; NEXT
	*timeAcc = ATOFF;
}

void Packet::parseGpsSolutionEcef(double* tow, uint16_t* week, uint8_t* gpsFix, uint8_t* numSats, vec3d* position, vec3f* velocity, vec3f* posAcc, float* speedAcc, float* timeAcc)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	*tow = ATOFD; NEXT
	*week = ATOU16; NEXT
	*gpsFix = ATOU8; NEXT
	*numSats = ATOU8; NEXT
	position->x = ATOFD; NEXT
	position->y = ATOFD; NEXT
	position->z = ATOFD; NEXT
	velocity->x = ATOFF; NEXT
	velocity->y = ATOFF; NEXT
	velocity->z = ATOFF; NEXT
	posAcc->x = ATOFF; NEXT
	posAcc->y = ATOFF; NEXT
	posAcc->z = ATOFF; NEXT
	*speedAcc = ATOFF; NEXT
	*timeAcc = ATOFF;
}

void Packet::parseGpsConfiguration(uint8_t* mode, uint8_t* ppsSource)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	*mode = ATOU8; NEXT
	*ppsSource = ATOU8; NEXT
	NEXT
	NEXT
	NEXT
}

void Packet::parseGpsAntennaOffset(vec3f* position)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	position->x = ATOFF; NEXT
	position->y = ATOFF; NEXT
	position->z = ATOFF;
}

void Packet::parseGpsCompassBaseline(vec3f* position, vec3f* uncertainty)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	position->x = ATOFF; NEXT
	position->y = ATOFF; NEXT
	position->z = ATOFF; NEXT
	uncertainty->x = ATOFF; NEXT
	uncertainty->y = ATOFF; NEXT
	uncertainty->z = ATOFF;
}

void Packet::parseGpsCompassEstimatedBaseline(uint8_t* estBaselineUsed, uint16_t* numMeas, vec3f* position, vec3f* uncertainty)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	*estBaselineUsed = ATOU8; NEXT
	NEXT
	*numMeas = ATOU16; NEXT
	position->x = ATOFF; NEXT
	position->y = ATOFF; NEXT
	position->z = ATOFF; NEXT
	uncertainty->x = ATOFF; NEXT
	uncertainty->y = ATOFF; NEXT
	uncertainty->z = ATOFF;
}

void Packet::parseInsSolutionLla(double* time, uint16_t* week, uint16_t* status, vec3f* yawPitchRoll, vec3d* position, vec3f* nedVel, float* attUncertainty, float* posUncertainty, float* velUncertainty)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	*time = ATOFD; NEXT
	*week = ATOU16; NEXT
	*status = ATOU16; NEXT
	yawPitchRoll->x = ATOFF; NEXT
	yawPitchRoll->y = ATOFF; NEXT
	yawPitchRoll->z = ATOFF; NEXT
	position->x = ATOFD; NEXT
	position->y = ATOFD; NEXT
	position->z = ATOFD; NEXT
	nedVel->x = ATOFF; NEXT
	nedVel->y = ATOFF; NEXT
	nedVel->z = ATOFF; NEXT
	*attUncertainty = ATOFF; NEXT
	*posUncertainty = ATOFF; NEXT
	*velUncertainty = ATOFF;
}

void Packet::parseInsSolutionEcef(double* time, uint16_t* week, uint16_t* status, vec3f* yawPitchRoll, vec3d* position, vec3f* velocity, float* attUncertainty, float* posUncertainty, float* velUncertainty)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	*time = ATOFD; NEXT
	*week = ATOU16; NEXT
	*status = ATOU16; NEXT
	yawPitchRoll->x = ATOFF; NEXT
	yawPitchRoll->y = ATOFF; NEXT
	yawPitchRoll->z = ATOFF; NEXT
	position->x = ATOFD; NEXT
	position->y = ATOFD; NEXT
	position->z = ATOFD; NEXT
	velocity->x = ATOFF; NEXT
	velocity->y = ATOFF; NEXT
	velocity->z = ATOFF; NEXT
	*attUncertainty = ATOFF; NEXT
	*posUncertainty = ATOFF; NEXT
	*velUncertainty = ATOFF;
}

void Packet::parseInsStateLla(vec3f* yawPitchRoll, vec3d* position, vec3f* velocity, vec3f* accel, vec3f* angularRate)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	yawPitchRoll->x = ATOFF; NEXT
	yawPitchRoll->y = ATOFF; NEXT
	yawPitchRoll->z = ATOFF; NEXT
	position->x = ATOFD; NEXT
	position->y = ATOFD; NEXT
	position->z = ATOFD; NEXT
	velocity->x = ATOFF; NEXT
	velocity->y = ATOFF; NEXT
	velocity->z = ATOFF; NEXT
	accel->x = ATOFF; NEXT
	accel->y = ATOFF; NEXT
	accel->z = ATOFF; NEXT
	angularRate->x = ATOFF; NEXT
	angularRate->y = ATOFF; NEXT
	angularRate->z = ATOFF;
}

void Packet::parseInsStateEcef(vec3f* yawPitchRoll, vec3d* position, vec3f* velocity, vec3f* accel, vec3f* angularRate)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	yawPitchRoll->x = ATOFF; NEXT
	yawPitchRoll->y = ATOFF; NEXT
	yawPitchRoll->z = ATOFF; NEXT
	position->x = ATOFD; NEXT
	position->y = ATOFD; NEXT
	position->z = ATOFD; NEXT
	velocity->x = ATOFF; NEXT
	velocity->y = ATOFF; NEXT
	velocity->z = ATOFF; NEXT
	accel->x = ATOFF; NEXT
	accel->y = ATOFF; NEXT
	accel->z = ATOFF; NEXT
	angularRate->x = ATOFF; NEXT
	angularRate->y = ATOFF; NEXT
	angularRate->z = ATOFF;
}

void Packet::parseInsBasicConfiguration(uint8_t* scenario, uint8_t* ahrsAiding)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	*scenario = ATOU8; NEXT
	*ahrsAiding = ATOU8; NEXT
	NEXT
	NEXT
}

void Packet::parseInsBasicConfiguration(uint8_t* scenario, uint8_t* ahrsAiding, uint8_t* estBaseline)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	*scenario = ATOU8; NEXT
	*ahrsAiding = ATOU8; NEXT
	*estBaseline = ATOU8; NEXT
	NEXT
}

void Packet::parseStartupFilterBiasEstimate(vec3f* gyroBias, vec3f* accelBias, float* pressureBias)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	gyroBias->x = ATOFF; NEXT
	gyroBias->y = ATOFF; NEXT
	gyroBias->z = ATOFF; NEXT
	accelBias->x = ATOFF; NEXT
	accelBias->y = ATOFF; NEXT
	accelBias->z = ATOFF; NEXT
	*pressureBias = ATOFF;
}

string str(AsciiAsync val)
{
	switch (val)
	{
		case VNOFF:
			return "OFF";
		case VNYPR:
			return "VNYPR";
		case VNQTN:
			return "VNQTN";
		case VNQMR:
			return "VNQMR";
		case VNMAG:
			return "VNMAG";
		case VNACC:
			return "VNACC";
		case VNGYR:
			return "VNGYR";
		case VNMAR:
			return "VNMAR";
		case VNYMR:
			return "VNYMR";
		case VNYBA:
			return "VNYBA";
		case VNYIA:
			return "VNYIA";
		case VNIMU:
			return "VNIMU";
		case VNGPS:
			return "VNGPS";
		case VNGPE:
			return "VNGPE";
		case VNINS:
			return "VNINS";
		case VNINE:
			return "VNINE";
		case VNISL:
			return "VNISL";
		case VNISE:
			return "VNISE";
		case VNDTV:
			return "VNDTV";
		default:
			throw invalid_argument("val");
	}
}

ostream& operator<<(ostream& out, AsciiAsync e)
{
	out << str(e);
	return out;
}

string str(SensorError val)
{
	switch (val)
	{
	case ERR_HARD_FAULT:
		return "HardFault";
	case ERR_SERIAL_BUFFER_OVERFLOW:
		return "SerialBufferOverflow";
	case ERR_INVALID_CHECKSUM:
		return "InvalidChecksum";
	case ERR_INVALID_COMMAND:
		return "InvalidCommand";
	case ERR_NOT_ENOUGH_PARAMETERS:
		return "NotEnoughParameters";
	case ERR_TOO_MANY_PARAMETERS:
		return "TooManyParameters";
	case ERR_INVALID_PARAMETER:
		return "InvalidParameter";
	case ERR_INVALID_REGISTER:
		return "InvalidRegister";
	case ERR_UNAUTHORIZED_ACCESS:
		return "UnauthorizedAccess";
	case ERR_WATCHDOG_RESET:
		return "WatchdogReset";
	case ERR_OUTPUT_BUFFER_OVERFLOW:
		return "OutputBufferOverflow";
	case ERR_INSUFFICIENT_BAUD_RATE:
		return "InsufficientBaudRate";
	case ERR_ERROR_BUFFER_OVERFLOW:
		return "ErrorBufferOverflow";
	default:
		throw invalid_argument("val");
	}
}

ostream& operator<<(ostream& out, SensorError e)
{
	out << str(e);
	return out;
}

CommonGroup operator|(CommonGroup lhs, CommonGroup rhs)
{
	return CommonGroup(int(lhs) | int(rhs));
}

TimeGroup operator|(TimeGroup lhs, TimeGroup rhs)
{
	return TimeGroup(int(lhs) | int(rhs));
}

ImuGroup operator|(ImuGroup lhs, ImuGroup rhs)
{
	return ImuGroup(int(lhs) | int(rhs));
}

GpsGroup operator|(GpsGroup lhs, GpsGroup rhs)
{
	return GpsGroup(int(lhs) | int(rhs));
}

AttitudeGroup operator|(AttitudeGroup lhs, AttitudeGroup rhs)
{
	return AttitudeGroup(int(lhs) | int(rhs));
}

InsGroup operator|(InsGroup lhs, InsGroup rhs)
{
	return InsGroup(int(lhs) | int(rhs));
}

string str(SyncInMode val)
{
	switch (val)
	{
		case SYNCINMODE_COUNT:
			 return "Count";
		case SYNCINMODE_IMU:
			 return "Imu";
		case SYNCINMODE_ASYNC:
			 return "Async";
		default:
			throw invalid_argument("val");
	}
}

string str(SyncInEdge val)
{
	switch (val)
	{
		case SYNCINEDGE_RISING:
			 return "Rising";
		case SYNCINEDGE_FALLING:
			 return "Falling";
		default:
			throw invalid_argument("val");
	}
}

string str(SyncOutMode val)
{
	switch (val)
	{
		case SYNCOUTMODE_NONE:
			 return "None";
		case SYNCOUTMODE_ITEMSTART:
			 return "ItemStart";
		case SYNCOUTMODE_IMUREADY:
			 return "ImuReady";
		case SYNCOUTMODE_INS:
			 return "Ins";
		case SYNCOUTMODE_GPSPPS:
			 return "GpsPps";
		default:
			throw invalid_argument("val");
	}
}

string str(SyncOutPolarity val)
{
	switch (val)
	{
		case SYNCOUTPOLARITY_NEGATIVE:
			 return "Negative";
		case SYNCOUTPOLARITY_POSITIVE:
			 return "Positive";
		default:
			throw invalid_argument("val");
	}
}

string str(CountMode val)
{
	switch (val)
	{
		case COUNTMODE_NONE:
			 return "None";
		case COUNTMODE_SYNCINCOUNT:
			 return "SyncInCount";
		case COUNTMODE_SYNCINTIME:
			 return "SyncInTime";
		case COUNTMODE_SYNCOUTCOUNTER:
			 return "SyncOutCounter";
		case COUNTMODE_GPSPPS:
			 return "GpsPps";
		default:
			throw invalid_argument("val");
	}
}

string str(StatusMode val)
{
	switch (val)
	{
		case STATUSMODE_OFF:
			 return "Off";
		case STATUSMODE_VPESTATUS:
			 return "VpeStatus";
		case STATUSMODE_INSSTATUS:
			 return "InsStatus";
		default:
			throw invalid_argument("val");
	}
}

string str(ChecksumMode val)
{
	switch (val)
	{
		case CHECKSUMMODE_OFF:
			 return "Off";
		case CHECKSUMMODE_CHECKSUM:
			 return "Checksum";
		case CHECKSUMMODE_CRC:
			 return "Crc";
		default:
			throw invalid_argument("val");
	}
}

string str(ErrorMode val)
{
	switch (val)
	{
		case ERRORMODE_IGNORE:
			 return "Ignore";
		case ERRORMODE_SEND:
			 return "Send";
		case ERRORMODE_SENDANDOFF:
			 return "SendAndOff";
		default:
			throw invalid_argument("val");
	}
}

string str(FilterMode val)
{
	switch (val)
	{
		case FILTERMODE_NOFILTERING:
			 return "NoFiltering";
		case FILTERMODE_ONLYRAW:
			 return "OnlyRaw";
		case FILTERMODE_ONLYCOMPENSATED:
			 return "OnlyCompensated";
		case FILTERMODE_BOTH:
			 return "Both";
		default:
			throw invalid_argument("val");
	}
}

string str(IntegrationFrame val)
{
	switch (val)
	{
		case INTEGRATIONFRAME_BODY:
			 return "Body";
		case INTEGRATIONFRAME_NED:
			 return "Ned";
		default:
			throw invalid_argument("val");
	}
}

string str(CompensationMode val)
{
	switch (val)
	{
		case COMPENSATIONMODE_NONE:
			 return "None";
		case COMPENSATIONMODE_BIAS:
			 return "Bias";
		default:
			throw invalid_argument("val");
	}
}

string str(GpsFix val)
{
	switch (val)
	{
		case GPSFIX_NOFIX:
			 return "NoFix";
		case GPSFIX_TIMEONLY:
			 return "TimeOnly";
		case GPSFIX_2D:
			 return "2D";
		case GPSFIX_3D:
			 return "3D";
		default:
			throw invalid_argument("val");
	}
}

string str(GpsMode val)
{
	switch (val)
	{
		case GPSMODE_ONBOARDGPS:
			 return "OnBoardGps";
		case GPSMODE_EXTERNALGPS:
			 return "ExternalGps";
		case GPSMODE_EXTERNALVN200GPS:
			 return "ExternalVn200Gps";
		default:
			throw invalid_argument("val");
	}
}

string str(PpsSource val)
{
	switch (val)
	{
		case PPSSOURCE_GPSPPSRISING:
			 return "GpsPpsRising";
		case PPSSOURCE_GPSPPSFALLING:
			 return "GpsPpsFalling";
		case PPSSOURCE_SYNCINRISING:
			 return "SyncInRising";
		case PPSSOURCE_SYNCINFALLING:
			 return "SyncInFalling";
		default:
			throw invalid_argument("val");
	}
}

string str(VpeEnable val)
{
	switch (val)
	{
		case VPEENABLE_DISABLE:
			 return "Disable";
		case VPEENABLE_ENABLE:
			 return "Enable";
		default:
			throw invalid_argument("val");
	}
}

string str(HeadingMode val)
{
	switch (val)
	{
		case HEADINGMODE_ABSOLUTE:
			 return "Absolute";
		case HEADINGMODE_RELATIVE:
			 return "Relative";
		case HEADINGMODE_INDOOR:
			 return "Indoor";
		default:
			throw invalid_argument("val");
	}
}

string str(VpeMode val)
{
	switch (val)
	{
		case VPEMODE_OFF:
			 return "Off";
		case VPEMODE_MODE1:
			 return "Mode1";
		default:
			throw invalid_argument("val");
	}
}

string str(Scenario val)
{
	switch (val)
	{
		case SCENARIO_AHRS:
			 return "Ahrs";
		case SCENARIO_INSWITHPRESSURE:
			 return "InsWithPressure";
		case SCENARIO_INSWITHOUTPRESSURE:
			 return "InsWithoutPressure";
		case SCENARIO_GPSMOVINGBASELINEDYNAMIC:
			 return "GpsMovingBaselineDynamic";
		case SCENARIO_GPSMOVINGBASELINESTATIC:
			 return "GpsMovingBaselineStatic";
		default:
			throw invalid_argument("val");
	}
}

string str(HsiMode val)
{
	switch (val)
	{
		case HSIMODE_OFF:
			 return "Off";
		case HSIMODE_RUN:
			 return "Run";
		case HSIMODE_RESET:
			 return "Reset";
		default:
			throw invalid_argument("val");
	}
}

string str(HsiOutput val)
{
	switch (val)
	{
		case HSIOUTPUT_NOONBOARD:
			 return "NoOnboard";
		case HSIOUTPUT_USEONBOARD:
			 return "UseOnboard";
		default:
			throw invalid_argument("val");
	}
}

ostream& operator<<(ostream& out, SyncInMode e)
{
	out << str(e);
	return out;
}

ostream& operator<<(ostream& out, SyncInEdge e)
{
	out << str(e);
	return out;
}

ostream& operator<<(ostream& out, SyncOutMode e)
{
	out << str(e);
	return out;
}

ostream& operator<<(ostream& out, SyncOutPolarity e)
{
	out << str(e);
	return out;
}

ostream& operator<<(ostream& out, CountMode e)
{
	out << str(e);
	return out;
}

ostream& operator<<(ostream& out, StatusMode e)
{
	out << str(e);
	return out;
}

ostream& operator<<(ostream& out, ChecksumMode e)
{
	out << str(e);
	return out;
}

ostream& operator<<(ostream& out, ErrorMode e)
{
	out << str(e);
	return out;
}

ostream& operator<<(ostream& out, FilterMode e)
{
	out << str(e);
	return out;
}

ostream& operator<<(ostream& out, IntegrationFrame e)
{
	out << str(e);
	return out;
}

ostream& operator<<(ostream& out, CompensationMode e)
{
	out << str(e);
	return out;
}

ostream& operator<<(ostream& out, GpsFix e)
{
	out << str(e);
	return out;
}

ostream& operator<<(ostream& out, GpsMode e)
{
	out << str(e);
	return out;
}

ostream& operator<<(ostream& out, PpsSource e)
{
	out << str(e);
	return out;
}

ostream& operator<<(ostream& out, VpeEnable e)
{
	out << str(e);
	return out;
}

ostream& operator<<(ostream& out, HeadingMode e)
{
	out << str(e);
	return out;
}

ostream& operator<<(ostream& out, VpeMode e)
{
	out << str(e);
	return out;
}

ostream& operator<<(ostream& out, Scenario e)
{
	out << str(e);
	return out;
}

ostream& operator<<(ostream& out, HsiMode e)
{
	out << str(e);
	return out;
}

ostream& operator<<(ostream& out, HsiOutput e)
{
	out << str(e);
	return out;
}

}
}
}
