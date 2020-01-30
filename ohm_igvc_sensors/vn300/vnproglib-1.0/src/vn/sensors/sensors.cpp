// VectorNav Programming Library v1.0.0.1
// Copyright (c) 2015 VectorNav Technologies, LLC
#include "vn/sensors/sensors.h"

#include <string>
#include <queue>
#include <string.h>
#include <stdio.h>

#include "vn/xplat/serialport.h"
#include "vn/xplat/criticalsection.h"
#include "vn/xplat/time.h"
#include "vn/xplat/event.h"
#include "vn/exceptions.h"
#include "vn/data/error_detection.h"
#include "vn/math/vector.h"
#include "vn/math/matrix.h"
#include "vn/xplat/thread.h"

using namespace std;
using namespace vn::math;
using namespace vn::common;
using namespace vn::xplat;
using namespace vn::protocol::uart;
using namespace vn::data::integrity;

#define COMMAND_MAX_LENGTH 0x100

namespace vn {
namespace sensors {

sensor_error::sensor_error()
{ }

sensor_error::sensor_error(SensorError e) :
	exception(),
	error(e),
	_errorMessage(NULL)
{ }

sensor_error::sensor_error(sensor_error const& e) :
	exception(),
	error(e.error),
	_errorMessage(NULL)
{ }

sensor_error::~sensor_error() throw()
{
	if (_errorMessage != NULL)
		delete[] _errorMessage;
}

char const* sensor_error::what() const throw()
{
	if (_errorMessage != NULL)
		return _errorMessage;

	string errorMsg = "received sensor error '" + str(error) + "'";

	const_cast<char*&>(_errorMessage) = new char[errorMsg.size() + 1];
	strcpy(_errorMessage, errorMsg.c_str());

	return _errorMessage;
}

struct VnSensor::Impl
{
	static const size_t DefaultReadBufferSize = 256;
	static const uint16_t DefaultResponseTimeoutMs = 500;
	static const uint16_t DefaultRetransmitDelayMs = 200;

	SerialPort *pSerialPort;
	ISimplePort* SimplePort;
	bool SimplePortIsOurs;
	bool DidWeOpenSimplePort;
	RawDataReceivedHandler _rawDataReceivedHandler;
	void* _rawDataReceivedUserData;
	PossiblePacketFoundHandler _possiblePacketFoundHandler;
	void* _possiblePacketFoundUserData;
	PacketFinder _packetFinder;
	size_t _dataRunningIndex;
	AsyncPacketReceivedHandler _asyncPacketReceivedHandler;
	void* _asyncPacketReceivedUserData;
	ErrorDetectionMode _sendErrorDetectionMode;
	VnSensor* BackReference;
	queue<Packet> _receivedResponses;
	CriticalSection _transactionCS;
	bool _waitingForResponse;
	ErrorPacketReceivedHandler _errorPacketReceivedHandler;
	void* _errorPacketReceivedUserData;
	uint16_t _responseTimeoutMs;
	uint16_t _retransmitDelayMs;
	Event _newResponsesEvent;

	explicit Impl(VnSensor* backReference) :
		pSerialPort(NULL),
		SimplePort(NULL),
		SimplePortIsOurs(false),
		DidWeOpenSimplePort(false),
		_rawDataReceivedHandler(NULL),
		_rawDataReceivedUserData(NULL),
		_possiblePacketFoundHandler(NULL),
		_possiblePacketFoundUserData(NULL),
		_dataRunningIndex(0),
		_asyncPacketReceivedHandler(NULL),
		_asyncPacketReceivedUserData(NULL),
		_sendErrorDetectionMode(ERRORDETECTIONMODE_CHECKSUM),
		BackReference(backReference),
		_waitingForResponse(false),
		_errorPacketReceivedHandler(NULL),
		_errorPacketReceivedUserData(NULL),
		_responseTimeoutMs(DefaultResponseTimeoutMs),
		_retransmitDelayMs(DefaultRetransmitDelayMs)
	{
		_packetFinder.registerPossiblePacketFoundHandler(this, possiblePacketFoundHandler);
	}

	~Impl()
	{
		_packetFinder.unregisterPossiblePacketFoundHandler();
	}

	void onPossiblePacketFound(Packet& possiblePacket, size_t packetStartRunningIndex)
	{
		if (_possiblePacketFoundHandler != NULL)
			_possiblePacketFoundHandler(_possiblePacketFoundUserData, possiblePacket, packetStartRunningIndex);
	}

	void onAsyncPacketReceived(Packet& asciiPacket, size_t runningIndex)
	{
		if (_asyncPacketReceivedHandler != NULL)
			_asyncPacketReceivedHandler(_asyncPacketReceivedUserData, asciiPacket, runningIndex);
	}

	void onErrorPacketReceived(Packet& errorPacket, size_t runningIndex)
	{
		if (_errorPacketReceivedHandler != NULL)
			_errorPacketReceivedHandler(_errorPacketReceivedUserData, errorPacket, runningIndex);
	}

	static void possiblePacketFoundHandler(void* userData, Packet& possiblePacket, size_t packetStartRunningIndex)
	{
		Impl* pThis = static_cast<Impl*>(userData);

		pThis->onPossiblePacketFound(possiblePacket, packetStartRunningIndex);

		if (!possiblePacket.isValid())
			return;

		if (possiblePacket.isError())
		{
			if (pThis->_waitingForResponse)
			{
				pThis->_transactionCS.enter();
				pThis->_receivedResponses.push(possiblePacket);
				pThis->_newResponsesEvent.signal();
				pThis->_transactionCS.leave();
			}

			pThis->onErrorPacketReceived(possiblePacket, packetStartRunningIndex);

			return;
		}

		if (possiblePacket.isResponse() && pThis->_waitingForResponse)
		{
			pThis->_transactionCS.enter();
			pThis->_receivedResponses.push(possiblePacket);
			pThis->_newResponsesEvent.signal();
			pThis->_transactionCS.leave();

			return;
		}

		// This wasn't anything else. We assume it is an async packet.
		pThis->onAsyncPacketReceived(possiblePacket, packetStartRunningIndex);
	}

	static void dataReceivedHandler(void* userData)
	{
		static char readBuffer[DefaultReadBufferSize];
		Impl *pi = static_cast<Impl*>(userData);

		size_t numOfBytesRead = 0;

		pi->SimplePort->read(
			readBuffer,
			DefaultReadBufferSize,
			numOfBytesRead);

		if (numOfBytesRead == 0)
			return;

		if (pi->_rawDataReceivedHandler != NULL)
			pi->_rawDataReceivedHandler(pi->_rawDataReceivedUserData, reinterpret_cast<char*>(readBuffer), numOfBytesRead, pi->_dataRunningIndex);

		pi->_packetFinder.processReceivedData(reinterpret_cast<char*>(readBuffer), numOfBytesRead);

		pi->_dataRunningIndex += numOfBytesRead;
	}

	bool isConnected()
	{
		return SimplePort != NULL && SimplePort->isOpen();
	}

	size_t finalizeCommandToSend(char *toSend, size_t length)
	{
		if (_sendErrorDetectionMode == ERRORDETECTIONMODE_CHECKSUM)
		{
			length += sprintf(toSend + length, "%02X\r\n", Checksum8::compute(toSend + 1, length - 2));
		}
		else if (_sendErrorDetectionMode == ERRORDETECTIONMODE_CRC)
		{
			length += sprintf(toSend + length, "%04X\r\n", Crc16::compute(toSend + 1, length - 2));
		}
		else
		{
			length += sprintf(toSend + length, "XX\r\n");
		}

		return length;
	}

	Packet transactionWithWait(char* toSend, size_t length, uint16_t responseTimeoutMs, uint16_t retransmitDelayMs)
	{
		// Make sure we don't have any existing responses.
		_transactionCS.enter();

		#if (defined(_MSC_VER) && _MSC_VER <= 1500) || (__cplusplus < 201103L)
		// This C++ version doesn't support swap().
		while (!_receivedResponses.empty()) _receivedResponses.pop();
		#else
		queue<Packet> empty;
		_receivedResponses.swap(empty);
		#endif
		_waitingForResponse = true;
		_transactionCS.leave();

		// Send the command and continue sending if retransmits are enabled
		// until we receive the response or timeout.
		Stopwatch timeoutSw;

		SimplePort->write(toSend, length);
		float curElapsedTime = timeoutSw.elapsedMs();

		while (true)
		{
			bool shouldRetransmit = false;

			// Compute how long we should wait for a response before taking
			// more action.
			float responseWaitTime = responseTimeoutMs - curElapsedTime;
			if (responseWaitTime > retransmitDelayMs)
			{
				responseWaitTime = retransmitDelayMs;
				shouldRetransmit = true;
			}

			// See if we have time left.
			if (responseWaitTime < 0)
			{
				_waitingForResponse = false;
				throw timeout();
			}

			// Wait for any new responses that come in or until it is time to
			// send a new retransmit.
			Event::WaitResult waitResult = _newResponsesEvent.waitUs(static_cast<uint32_t>(responseWaitTime * 1000));

			queue<Packet> responsesToProcess;

			if (waitResult == Event::WAIT_TIMEDOUT)
			{
				if (!shouldRetransmit)
				{
					_waitingForResponse = false;
					throw timeout();
				}
			}

			if (waitResult == Event::WAIT_SIGNALED)
			{
				// Get the current collection of responses.
				_transactionCS.enter();
				#if (defined(_MSC_VER) && _MSC_VER <= 1500) || (__cplusplus < 201103L)
				// This C++ version doesn't support swap().
				while (!_receivedResponses.empty())
				{
					responsesToProcess.push(_receivedResponses.front());
					_receivedResponses.pop();
				}
				#else
				_receivedResponses.swap(responsesToProcess);
				#endif
				_transactionCS.leave();
			}

			// Process the collection of responses we have.
			while (!responsesToProcess.empty())
			{
				Packet p = responsesToProcess.front();
				responsesToProcess.pop();

				if (p.isError())
				{
					_waitingForResponse = false;
					throw sensor_error(p.parseError());
				}

				// We must have a response packet.
				_waitingForResponse = false;
				return p;
			}

			// Retransmit.
			SimplePort->write(toSend, length);
			curElapsedTime = timeoutSw.elapsedMs();
		}
	}

	void transactionNoFinalize(char* toSend, size_t length, bool waitForReply, Packet *response, uint16_t responseTimeoutMs, uint16_t retransmitDelayMs)
	{
		if (!isConnected())
			throw invalid_operation();

		if (waitForReply)
		{
			*response = transactionWithWait(toSend, length, responseTimeoutMs, retransmitDelayMs);

			if (response->isError())
				throw sensor_error(response->parseError());
		}
		else
		{
			SimplePort->write(toSend, length);
		}
	}

	void transactionNoFinalize(char* toSend, size_t length, bool waitForReply, Packet *response)
	{
		transactionNoFinalize(toSend, length, waitForReply, response, _responseTimeoutMs, _retransmitDelayMs);
	}

	void transaction(char* toSend, size_t length, bool waitForReply, Packet *response, uint16_t responseTimeoutMs, uint16_t retransmitDelayMs)
	{
		length = finalizeCommandToSend(toSend, length);

		transactionNoFinalize(toSend, length, waitForReply, response, responseTimeoutMs, retransmitDelayMs);
	}

	// Performs a communication transaction with the sensor. If waitForReply is
	// set to true, we will retransmit the message until we receive a response
	// or until we time out, depending on current settings.
	void transaction(char* toSend, size_t length, bool waitForReply, Packet *response)
	{
		transaction(toSend, length, waitForReply, response, _responseTimeoutMs, _retransmitDelayMs);
	}

	BinaryOutputRegister readBinaryOutput(uint8_t binaryOutputNumber)
	{
		char toSend[17];
		Packet response;
		uint16_t asyncMode, rateDivisor, outputGroup, commonField, timeField, imuField, gpsField, attitudeField, insField;

		int length = sprintf(toSend, "$VNRRG,%u*", 74 + binaryOutputNumber);

		transaction(toSend, length, true, &response);

		response.parseBinaryOutput(
			&asyncMode,
			&rateDivisor,
			&outputGroup,
			&commonField,
			&timeField,
			&imuField,
			&gpsField,
			&attitudeField,
			&insField);

		return BinaryOutputRegister(
			static_cast<AsyncMode>(asyncMode),
			rateDivisor,
			static_cast<CommonGroup>(commonField),
			static_cast<TimeGroup>(timeField),
			static_cast<ImuGroup>(imuField),
			static_cast<GpsGroup>(gpsField),
			static_cast<AttitudeGroup>(attitudeField),
			static_cast<InsGroup>(insField));
	}

	void writeBinaryOutput(uint8_t binaryOutputNumber, BinaryOutputRegister &fields, bool waitForReply)
	{
		char toSend[256];
		Packet response;

		// First determine which groups are present.
		uint16_t groups = 0;
		if (fields.commonField)
			groups |= 0x0001;
		if (fields.timeField)
			groups |= 0x0002;
		if (fields.imuField)
			groups |= 0x0004;
		if (fields.gpsField)
			groups |= 0x0008;
		if (fields.attitudeField)
			groups |= 0x0010;
		if (fields.insField)
			groups |= 0x0020;

		int length = sprintf(toSend, "$VNWRG,%u,%u,%u,%X", 74 + binaryOutputNumber, fields.asyncMode, fields.rateDivisor, groups);

		if (fields.commonField)
			length += sprintf(toSend + length, ",%X", fields.commonField);
		if (fields.timeField)
			length += sprintf(toSend + length, ",%X", fields.timeField);
		if (fields.imuField)
			length += sprintf(toSend + length, ",%X", fields.imuField);
		if (fields.gpsField)
			length += sprintf(toSend + length, ",%X", fields.gpsField);
		if (fields.attitudeField)
			length += sprintf(toSend + length, ",%X", fields.attitudeField);
		if (fields.insField)
			length += sprintf(toSend + length, ",%X", fields.insField);

		length += sprintf(toSend + length, "*");

		transaction(toSend, length, waitForReply, &response);
	}
};

vector<uint32_t> VnSensor::supportedBaudrates()
{
	uint32_t br[] = {
		9600,
		19200,
		38400,
		57600,
		115200,
		128000,
		230400,
		460800,
		921600 };

	return vector<uint32_t>(br, br + sizeof(br));
}

#if defined(_MSC_VER) && _MSC_VER <= 1600
	#pragma warning(push)
	// Disable VS2010 warning for 'this' used in base member initializer list.
	#pragma warning(disable:4355)
#endif

VnSensor::VnSensor() :
	_pi(new Impl(this))
{
}

#if defined(_MSC_VER) && _MSC_VER <= 1600
	#pragma warning(pop)
#endif

VnSensor::~VnSensor()
{
	if (_pi->SimplePortIsOurs && _pi->DidWeOpenSimplePort && isConnected())
		disconnect();

	delete _pi;
}

ErrorDetectionMode VnSensor::sendErrorDetectionMode()
{
	return _pi->_sendErrorDetectionMode;
}

void VnSensor::setSendErrorDetectionMode(ErrorDetectionMode mode)
{
	_pi->_sendErrorDetectionMode = mode;
}

bool VnSensor::isConnected()
{
	return _pi->isConnected();
}

uint16_t VnSensor::responseTimeoutMs()
{
	return _pi->_responseTimeoutMs;
}

void VnSensor::setResponseTimeoutMs(uint16_t timeout)
{
	_pi->_responseTimeoutMs = timeout;
}

uint16_t VnSensor::retransmitDelayMs()
{
	return _pi->_retransmitDelayMs;
}

void VnSensor::setRetransmitDelayMs(uint16_t delay)
{
	_pi->_retransmitDelayMs = delay;
}

bool VnSensor::verifySensorConnectivity()
{
	try
	{
		readModelNumber();

		return true;
	}
	catch (...) {	}

	return false;
}

void VnSensor::connect(const string &portName, uint32_t baudrate)
{
	_pi->pSerialPort = new SerialPort(portName, baudrate);

	connect(dynamic_cast<ISimplePort*>(_pi->pSerialPort));

	_pi->SimplePortIsOurs = true;
}

void VnSensor::connect(ISimplePort* simplePort)
{
	_pi->SimplePort = simplePort;
	_pi->SimplePortIsOurs = false;

	_pi->SimplePort->registerDataReceivedHandler(_pi, Impl::dataReceivedHandler);

	if (!_pi->SimplePort->isOpen())
	{
		_pi->SimplePort->open();
		_pi->DidWeOpenSimplePort = true;
	}
}

void VnSensor::disconnect()
{
	if (_pi->SimplePort == NULL || !_pi->SimplePort->isOpen())
		throw invalid_operation();

	_pi->SimplePort->unregisterDataReceivedHandler();

	if (_pi->DidWeOpenSimplePort)
	{
		_pi->SimplePort->close();
	}

	_pi->DidWeOpenSimplePort = false;

	if (_pi->SimplePortIsOurs)
	{
		delete _pi->SimplePort;

		_pi->SimplePort = NULL;
	}
}

string VnSensor::transaction(string toSend)
{
	char buffer[COMMAND_MAX_LENGTH];
	size_t finalLength = toSend.length();
	Packet response;

	// Copy over what was provided.
	copy(toSend.begin(), toSend.end(), buffer);

	// First see if an '*' is present.
	if (toSend.find('*') == string::npos)
	{
		buffer[toSend.length()] = '*';
		finalLength = _pi->finalizeCommandToSend(buffer, toSend.length() + 1);
	}
	else if (toSend[toSend.length() - 2] != '\r' && toSend[toSend.length() - 1] != '\n')
	{
		buffer[toSend.length()] = '\r';
		buffer[toSend.length() + 1] = '\n';
		finalLength += 2;
	}

	_pi->transactionNoFinalize(buffer, finalLength, true, &response);

	return response.datastr();
}

string VnSensor::send(string toSend, bool waitForReply, ErrorDetectionMode errorDetectionMode)
{
	Packet p;
	char *buffer = new char[toSend.size() + 8];	// Extra room for possible additions.
	size_t curToSendLength = toSend.size();

	// See if a '$' needs to be prepended.
	if (toSend[0] == '$')
	{
		toSend.copy(buffer, toSend.size());
	}
	else
	{
		buffer[0] = '$';
		toSend.copy(buffer + 1, toSend.size());
		curToSendLength++;
	}

	// Locate any '*' in the command.
	size_t astrickLocation = string::npos;
	for (size_t i = 0; i < curToSendLength; i++)
	{
		if (buffer[i] == '*')
		{
			astrickLocation = i;
			break;
		}
	}

	// Do we need to add a '*'?
	if (astrickLocation == string::npos)
	{
		buffer[curToSendLength] = '*';
		astrickLocation = curToSendLength++;
	}

	// Do we need to add a checksum/CRC?
	if (astrickLocation == curToSendLength - 1)
	{
		if (errorDetectionMode == ERRORDETECTIONMODE_CHECKSUM)
		{
			curToSendLength += sprintf(buffer + curToSendLength, "%02X\r\n", Checksum8::compute(buffer + 1, curToSendLength - 2));
		}
		else if (errorDetectionMode == ERRORDETECTIONMODE_CRC)
		{
			curToSendLength += sprintf(buffer + curToSendLength, "%04X\r\n", Crc16::compute(buffer + 1, curToSendLength - 2));
		}
		else
		{
			curToSendLength += sprintf(buffer + curToSendLength, "*XX\r\n");
		}
	}
	// Do we need to add "\r\n"?
	else if (buffer[curToSendLength - 1] != '\n')
	{
		buffer[curToSendLength++] = '\r';
		buffer[curToSendLength++] = '\n';
	}

	_pi->transactionNoFinalize(buffer, curToSendLength, waitForReply, &p, _pi->_responseTimeoutMs, _pi->_retransmitDelayMs);

	delete [] buffer;

	return p.datastr();
}

void VnSensor::registerRawDataReceivedHandler(void* userData, RawDataReceivedHandler handler)
{
	if (_pi->_rawDataReceivedHandler != NULL)
		throw invalid_operation();

	_pi->_rawDataReceivedHandler = handler;
	_pi->_rawDataReceivedUserData = userData;
}

void VnSensor::unregisterRawDataReceivedHandler()
{
	if (_pi->_rawDataReceivedHandler == NULL)
		throw invalid_operation();

	_pi->_rawDataReceivedHandler = NULL;
	_pi->_rawDataReceivedUserData = NULL;
}

void VnSensor::registerPossiblePacketFoundHandler(void* userData, PossiblePacketFoundHandler handler)
{
	if (_pi->_possiblePacketFoundHandler != NULL)
		throw invalid_operation();

	_pi->_possiblePacketFoundHandler = handler;
	_pi->_possiblePacketFoundUserData = userData;
}

void VnSensor::unregisterPossiblePacketFoundHandler()
{
	if (_pi->_possiblePacketFoundHandler == NULL)
		throw invalid_operation();

	_pi->_possiblePacketFoundHandler = NULL;
	_pi->_possiblePacketFoundUserData = NULL;
}

void VnSensor::registerAsyncPacketReceivedHandler(void* userData, AsyncPacketReceivedHandler handler)
{
	if (_pi->_asyncPacketReceivedHandler != NULL)
		throw invalid_operation();

	_pi->_asyncPacketReceivedHandler = handler;
	_pi->_asyncPacketReceivedUserData = userData;
}

void VnSensor::unregisterAsyncPacketReceivedHandler()
{
	if (_pi->_asyncPacketReceivedHandler == NULL)
		throw invalid_operation();

	_pi->_asyncPacketReceivedHandler = NULL;
	_pi->_asyncPacketReceivedUserData = NULL;
}

void VnSensor::registerErrorPacketReceivedHandler(void* userData, ErrorPacketReceivedHandler handler)
{
	if (_pi->_errorPacketReceivedHandler != NULL)
		throw invalid_operation();

	_pi->_errorPacketReceivedHandler = handler;
	_pi->_errorPacketReceivedUserData = userData;
}

void VnSensor::unregisterErrorPacketReceivedHandler()
{
	if (_pi->_errorPacketReceivedHandler == NULL)
		throw invalid_operation();

	_pi->_errorPacketReceivedHandler = NULL;
	_pi->_errorPacketReceivedUserData = NULL;
}

void VnSensor::writeSettings(bool waitForReply)
{
	char toSend[37];

	size_t length = Packet::genWriteSettings(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;

	// Write settings sometimes takes a while to do and receive a response
	// from the sensor.
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response, 2500, 1000);
}

void VnSensor::restoreFactorySettings(bool waitForReply)
{
	char toSend[37];

	size_t length = Packet::genRestoreFactorySettings(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;

	// Restore factory settings sometimes takes a while to do and receive a
	// response from the sensor.
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response, 2500, 1000);
}

void VnSensor::reset(bool waitForReply)
{
	char toSend[37];

	size_t length = Packet::genReset(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;

	// Reset sometimes takes a while to do and receive a response from the
	// sensor.
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response, 2500, 1000);
}

VnSensor::Family VnSensor::determineDeviceFamily()
{
	string mn = readModelNumber();

	return determineDeviceFamily(mn);
}

VnSensor::Family VnSensor::determineDeviceFamily(std::string modelNumber)
{
	if (modelNumber.find("VN-100") == 0)
		return VnSensor_Family_Vn100;

	if (modelNumber.find("VN-200") == 0)
		return VnSensor_Family_Vn200;

	if (modelNumber.find("VN-300") == 0)
		return VnSensor_Family_Vn300;

	return VnSensor_Family_Unknown;
}

BinaryOutputRegister VnSensor::readBinaryOutput1()
{
	return _pi->readBinaryOutput(1);
}

BinaryOutputRegister VnSensor::readBinaryOutput2()
{
	return _pi->readBinaryOutput(2);
}

BinaryOutputRegister VnSensor::readBinaryOutput3()
{
	return _pi->readBinaryOutput(3);
}

void VnSensor::writeBinaryOutput1(BinaryOutputRegister &fields, bool waitForReply)
{
	_pi->writeBinaryOutput(1, fields, waitForReply);
}

void VnSensor::writeBinaryOutput2(BinaryOutputRegister &fields, bool waitForReply)
{
	_pi->writeBinaryOutput(2, fields, waitForReply);
}

void VnSensor::writeBinaryOutput3(BinaryOutputRegister &fields, bool waitForReply)
{
	_pi->writeBinaryOutput(3, fields, waitForReply);
}

string VnSensor::readUserTag()
{
	char toSend[17];

	size_t length = Packet::genReadUserTag(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	char tagBuffer[50];
	response.parseUserTag(tagBuffer);

	return tagBuffer;
}

void VnSensor::writeUserTag(const string &tag, bool waitForReply)
{
	char toSend[37];

	size_t length = Packet::genWriteUserTag(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), tag.c_str());

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

string VnSensor::readModelNumber()
{
	char toSend[17];

	size_t length = Packet::genReadModelNumber(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	char productNameBuffer[50];
	response.parseModelNumber(productNameBuffer);

	return productNameBuffer;
}

uint32_t VnSensor::readHardwareRevision()
{
	char toSend[17];

	size_t length = Packet::genReadHardwareRevision(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	uint32_t reg;
	response.parseHardwareRevision(&reg);

	return reg;
}

uint32_t VnSensor::readSerialNumber()
{
	char toSend[17];

	size_t length = Packet::genReadSerialNumber(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	uint32_t reg;
	response.parseSerialNumber(&reg);

	return reg;
}

string VnSensor::readFirmwareVersion()
{
	char toSend[17];

	size_t length = Packet::genReadFirmwareVersion(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	char firmwareVersionBuffer[50];
	response.parseFirmwareVersion(firmwareVersionBuffer);

	return firmwareVersionBuffer;
}

uint32_t VnSensor::readSerialBaudRate()
{
	char toSend[17];

	size_t length = Packet::genReadSerialBaudRate(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	uint32_t reg;
	response.parseSerialBaudRate(&reg);

	return reg;
}

void VnSensor::writeSerialBaudRate(const uint32_t &baudrate, bool waitForReply)
{
	char toSend[20];

	size_t length = Packet::genWriteSerialBaudRate(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), baudrate);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

AsciiAsync VnSensor::readAsyncDataOutputType()
{
	char toSend[17];

	size_t length = Packet::genReadAsyncDataOutputType(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	uint32_t reg;
	response.parseAsyncDataOutputType(&reg);

	return static_cast<AsciiAsync>(reg);
}

void VnSensor::writeAsyncDataOutputType(AsciiAsync ador, bool waitForReply)
{
	char toSend[19];

	size_t length = Packet::genWriteAsyncDataOutputType(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), ador);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

uint32_t VnSensor::readAsyncDataOutputFrequency()
{
	char toSend[17];

	size_t length = Packet::genReadAsyncDataOutputFrequency(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	uint32_t reg;
	response.parseAsyncDataOutputFrequency(&reg);

	return reg;
}

void VnSensor::writeAsyncDataOutputFrequency(const uint32_t &adof, bool waitForReply)
{
	char toSend[26];

	size_t length = Packet::genWriteAsyncDataOutputFrequency(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), adof);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

SynchronizationControlRegister VnSensor::readSynchronizationControl()
{
	char toSend[17];

	size_t length = Packet::genReadSynchronizationControl(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	SynchronizationControlRegister reg;
	uint8_t syncInMode;
	uint8_t syncInEdge;
	uint8_t syncOutMode;
	uint8_t syncOutPolarity;
	response.parseSynchronizationControl(
		&syncInMode,
		&syncInEdge,
		&reg.syncInSkipFactor,
		&syncOutMode,
		&syncOutPolarity,
		&reg.syncOutSkipFactor,
		&reg.syncOutPulseWidth);

	reg.syncInMode = static_cast<SyncInMode>(syncInMode);
	reg.syncInEdge = static_cast<SyncInEdge>(syncInEdge);
	reg.syncOutMode = static_cast<SyncOutMode>(syncOutMode);
	reg.syncOutPolarity = static_cast<SyncOutPolarity>(syncOutPolarity);

	return reg;
}

void VnSensor::writeSynchronizationControl(SynchronizationControlRegister &fields, bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteSynchronizationControl(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), fields.syncInMode, fields.syncInEdge, fields.syncInSkipFactor, 0, fields.syncOutMode, fields.syncOutPolarity, fields.syncOutSkipFactor, fields.syncOutPulseWidth, 0);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

void VnSensor::writeSynchronizationControl(
	SyncInMode syncInMode,
	SyncInEdge syncInEdge,
	const uint16_t &syncInSkipFactor,
	SyncOutMode syncOutMode,
	SyncOutPolarity syncOutPolarity,
	const uint16_t &syncOutSkipFactor,
	const uint32_t &syncOutPulseWidth,
	bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteSynchronizationControl(
		_pi->_sendErrorDetectionMode,
		toSend,
		sizeof(toSend),
		syncInMode,
		syncInEdge,
		syncInSkipFactor,
		0,
		syncOutMode,
		syncOutPolarity,
		syncOutSkipFactor,
		syncOutPulseWidth,
		0);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}
CommunicationProtocolControlRegister VnSensor::readCommunicationProtocolControl()
{
	char toSend[17];

	size_t length = Packet::genReadCommunicationProtocolControl(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	CommunicationProtocolControlRegister reg;
	uint8_t serialCount;
	uint8_t serialStatus;
	uint8_t spiCount;
	uint8_t spiStatus;
	uint8_t serialChecksum;
	uint8_t spiChecksum;
	uint8_t errorMode;
	response.parseCommunicationProtocolControl(
		&serialCount,
		&serialStatus,
		&spiCount,
		&spiStatus,
		&serialChecksum,
		&spiChecksum,
		&errorMode);

	reg.serialCount = static_cast<CountMode>(serialCount);
	reg.serialStatus = static_cast<StatusMode>(serialStatus);
	reg.spiCount = static_cast<CountMode>(spiCount);
	reg.spiStatus = static_cast<StatusMode>(spiStatus);
	reg.serialChecksum = static_cast<ChecksumMode>(serialChecksum);
	reg.spiChecksum = static_cast<ChecksumMode>(spiChecksum);
	reg.errorMode = static_cast<ErrorMode>(errorMode);

	return reg;
}

void VnSensor::writeCommunicationProtocolControl(CommunicationProtocolControlRegister &fields, bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteCommunicationProtocolControl(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), fields.serialCount, fields.serialStatus, fields.spiCount, fields.spiStatus, fields.serialChecksum, fields.spiChecksum, fields.errorMode);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

void VnSensor::writeCommunicationProtocolControl(
	CountMode serialCount,
	StatusMode serialStatus,
	CountMode spiCount,
	StatusMode spiStatus,
	ChecksumMode serialChecksum,
	ChecksumMode spiChecksum,
	ErrorMode errorMode,
	bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteCommunicationProtocolControl(
		_pi->_sendErrorDetectionMode,
		toSend,
		sizeof(toSend),
		serialCount,
		serialStatus,
		spiCount,
		spiStatus,
		serialChecksum,
		spiChecksum,
		errorMode);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}
SynchronizationStatusRegister VnSensor::readSynchronizationStatus()
{
	char toSend[17];

	size_t length = Packet::genReadSynchronizationStatus(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	SynchronizationStatusRegister reg;
	response.parseSynchronizationStatus(
		&reg.syncInCount,
		&reg.syncInTime,
		&reg.syncOutCount);

	return reg;
}

void VnSensor::writeSynchronizationStatus(SynchronizationStatusRegister &fields, bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteSynchronizationStatus(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), fields.syncInCount, fields.syncInTime, fields.syncOutCount);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

void VnSensor::writeSynchronizationStatus(
	const uint32_t &syncInCount,
	const uint32_t &syncInTime,
	const uint32_t &syncOutCount,
	bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteSynchronizationStatus(
		_pi->_sendErrorDetectionMode,
		toSend,
		sizeof(toSend),
		syncInCount,
		syncInTime,
		syncOutCount);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}
ImuMeasurementsRegister VnSensor::readImuMeasurements()
{
	char toSend[17];

	size_t length = Packet::genReadImuMeasurements(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	ImuMeasurementsRegister reg;
	response.parseImuMeasurements(
		&reg.mag,
		&reg.accel,
		&reg.gyro,
		&reg.temp,
		&reg.pressure);

	return reg;
}

DeltaThetaAndDeltaVelocityRegister VnSensor::readDeltaThetaAndDeltaVelocity()
{
	char toSend[17];

	size_t length = Packet::genReadDeltaThetaAndDeltaVelocity(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	DeltaThetaAndDeltaVelocityRegister reg;
	response.parseDeltaThetaAndDeltaVelocity(
		&reg.deltaTime,
		&reg.deltaTheta,
		&reg.deltaVelocity);

	return reg;
}

MagnetometerCompensationRegister VnSensor::readMagnetometerCompensation()
{
	char toSend[17];

	size_t length = Packet::genReadMagnetometerCompensation(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	MagnetometerCompensationRegister reg;
	response.parseMagnetometerCompensation(
		&reg.c,
		&reg.b);

	return reg;
}

void VnSensor::writeMagnetometerCompensation(MagnetometerCompensationRegister &fields, bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteMagnetometerCompensation(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), fields.c, fields.b);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

void VnSensor::writeMagnetometerCompensation(
	const math::mat3f &c,
	const math::vec3f &b,
	bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteMagnetometerCompensation(
		_pi->_sendErrorDetectionMode,
		toSend,
		sizeof(toSend),
		c,
		b);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}
AccelerationCompensationRegister VnSensor::readAccelerationCompensation()
{
	char toSend[17];

	size_t length = Packet::genReadAccelerationCompensation(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	AccelerationCompensationRegister reg;
	response.parseAccelerationCompensation(
		&reg.c,
		&reg.b);

	return reg;
}

void VnSensor::writeAccelerationCompensation(AccelerationCompensationRegister &fields, bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteAccelerationCompensation(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), fields.c, fields.b);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

void VnSensor::writeAccelerationCompensation(
	const math::mat3f &c,
	const math::vec3f &b,
	bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteAccelerationCompensation(
		_pi->_sendErrorDetectionMode,
		toSend,
		sizeof(toSend),
		c,
		b);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}
GyroCompensationRegister VnSensor::readGyroCompensation()
{
	char toSend[17];

	size_t length = Packet::genReadGyroCompensation(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	GyroCompensationRegister reg;
	response.parseGyroCompensation(
		&reg.c,
		&reg.b);

	return reg;
}

void VnSensor::writeGyroCompensation(GyroCompensationRegister &fields, bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteGyroCompensation(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), fields.c, fields.b);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

void VnSensor::writeGyroCompensation(
	const math::mat3f &c,
	const math::vec3f &b,
	bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteGyroCompensation(
		_pi->_sendErrorDetectionMode,
		toSend,
		sizeof(toSend),
		c,
		b);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}
mat3f VnSensor::readReferenceFrameRotation()
{
	char toSend[17];

	size_t length = Packet::genReadReferenceFrameRotation(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	mat3f reg;
	response.parseReferenceFrameRotation(&reg);

	return reg;
}

void VnSensor::writeReferenceFrameRotation(const mat3f &c, bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteReferenceFrameRotation(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), c);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

ImuFilteringConfigurationRegister VnSensor::readImuFilteringConfiguration()
{
	char toSend[17];

	size_t length = Packet::genReadImuFilteringConfiguration(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	ImuFilteringConfigurationRegister reg;
	uint8_t magFilterMode;
	uint8_t accelFilterMode;
	uint8_t gyroFilterMode;
	uint8_t tempFilterMode;
	uint8_t presFilterMode;
	response.parseImuFilteringConfiguration(
		&reg.magWindowSize,
		&reg.accelWindowSize,
		&reg.gyroWindowSize,
		&reg.tempWindowSize,
		&reg.presWindowSize,
		&magFilterMode,
		&accelFilterMode,
		&gyroFilterMode,
		&tempFilterMode,
		&presFilterMode);

	reg.magFilterMode = static_cast<FilterMode>(magFilterMode);
	reg.accelFilterMode = static_cast<FilterMode>(accelFilterMode);
	reg.gyroFilterMode = static_cast<FilterMode>(gyroFilterMode);
	reg.tempFilterMode = static_cast<FilterMode>(tempFilterMode);
	reg.presFilterMode = static_cast<FilterMode>(presFilterMode);

	return reg;
}

void VnSensor::writeImuFilteringConfiguration(ImuFilteringConfigurationRegister &fields, bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteImuFilteringConfiguration(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), fields.magWindowSize, fields.accelWindowSize, fields.gyroWindowSize, fields.tempWindowSize, fields.presWindowSize, fields.magFilterMode, fields.accelFilterMode, fields.gyroFilterMode, fields.tempFilterMode, fields.presFilterMode);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

void VnSensor::writeImuFilteringConfiguration(
	const uint16_t &magWindowSize,
	const uint16_t &accelWindowSize,
	const uint16_t &gyroWindowSize,
	const uint16_t &tempWindowSize,
	const uint16_t &presWindowSize,
	FilterMode magFilterMode,
	FilterMode accelFilterMode,
	FilterMode gyroFilterMode,
	FilterMode tempFilterMode,
	FilterMode presFilterMode,
	bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteImuFilteringConfiguration(
		_pi->_sendErrorDetectionMode,
		toSend,
		sizeof(toSend),
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

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}
DeltaThetaAndDeltaVelocityConfigurationRegister VnSensor::readDeltaThetaAndDeltaVelocityConfiguration()
{
	char toSend[17];

	size_t length = Packet::genReadDeltaThetaAndDeltaVelocityConfiguration(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	DeltaThetaAndDeltaVelocityConfigurationRegister reg;
	uint8_t integrationFrame;
	uint8_t gyroCompensation;
	uint8_t accelCompensation;
	response.parseDeltaThetaAndDeltaVelocityConfiguration(
		&integrationFrame,
		&gyroCompensation,
		&accelCompensation);

	reg.integrationFrame = static_cast<IntegrationFrame>(integrationFrame);
	reg.gyroCompensation = static_cast<CompensationMode>(gyroCompensation);
	reg.accelCompensation = static_cast<CompensationMode>(accelCompensation);

	return reg;
}

void VnSensor::writeDeltaThetaAndDeltaVelocityConfiguration(DeltaThetaAndDeltaVelocityConfigurationRegister &fields, bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteDeltaThetaAndDeltaVelocityConfiguration(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), fields.integrationFrame, fields.gyroCompensation, fields.accelCompensation, 0, 0);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

void VnSensor::writeDeltaThetaAndDeltaVelocityConfiguration(
	IntegrationFrame integrationFrame,
	CompensationMode gyroCompensation,
	CompensationMode accelCompensation,
	bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteDeltaThetaAndDeltaVelocityConfiguration(
		_pi->_sendErrorDetectionMode,
		toSend,
		sizeof(toSend),
		integrationFrame,
		gyroCompensation,
		accelCompensation,
		0,
		0);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}
vec3f VnSensor::readYawPitchRoll()
{
	char toSend[17];

	size_t length = Packet::genReadYawPitchRoll(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	vec3f reg;
	response.parseYawPitchRoll(&reg);

	return reg;
}

vec4f VnSensor::readAttitudeQuaternion()
{
	char toSend[17];

	size_t length = Packet::genReadAttitudeQuaternion(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	vec4f reg;
	response.parseAttitudeQuaternion(&reg);

	return reg;
}

YawPitchRollMagneticAccelerationAndAngularRatesRegister VnSensor::readYawPitchRollMagneticAccelerationAndAngularRates()
{
	char toSend[17];

	size_t length = Packet::genReadYawPitchRollMagneticAccelerationAndAngularRates(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	YawPitchRollMagneticAccelerationAndAngularRatesRegister reg;
	response.parseYawPitchRollMagneticAccelerationAndAngularRates(
		&reg.yawPitchRoll,
		&reg.mag,
		&reg.accel,
		&reg.gyro);

	return reg;
}

QuaternionMagneticAccelerationAndAngularRatesRegister VnSensor::readQuaternionMagneticAccelerationAndAngularRates()
{
	char toSend[17];

	size_t length = Packet::genReadQuaternionMagneticAccelerationAndAngularRates(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	QuaternionMagneticAccelerationAndAngularRatesRegister reg;
	response.parseQuaternionMagneticAccelerationAndAngularRates(
		&reg.quat,
		&reg.mag,
		&reg.accel,
		&reg.gyro);

	return reg;
}

vec3f VnSensor::readMagneticMeasurements()
{
	char toSend[17];

	size_t length = Packet::genReadMagneticMeasurements(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	vec3f reg;
	response.parseMagneticMeasurements(&reg);

	return reg;
}

vec3f VnSensor::readAccelerationMeasurements()
{
	char toSend[17];

	size_t length = Packet::genReadAccelerationMeasurements(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	vec3f reg;
	response.parseAccelerationMeasurements(&reg);

	return reg;
}

vec3f VnSensor::readAngularRateMeasurements()
{
	char toSend[17];

	size_t length = Packet::genReadAngularRateMeasurements(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	vec3f reg;
	response.parseAngularRateMeasurements(&reg);

	return reg;
}

MagneticAccelerationAndAngularRatesRegister VnSensor::readMagneticAccelerationAndAngularRates()
{
	char toSend[17];

	size_t length = Packet::genReadMagneticAccelerationAndAngularRates(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	MagneticAccelerationAndAngularRatesRegister reg;
	response.parseMagneticAccelerationAndAngularRates(
		&reg.mag,
		&reg.accel,
		&reg.gyro);

	return reg;
}

YawPitchRollTrueBodyAccelerationAndAngularRatesRegister VnSensor::readYawPitchRollTrueBodyAccelerationAndAngularRates()
{
	char toSend[17];

	size_t length = Packet::genReadYawPitchRollTrueBodyAccelerationAndAngularRates(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	YawPitchRollTrueBodyAccelerationAndAngularRatesRegister reg;
	response.parseYawPitchRollTrueBodyAccelerationAndAngularRates(
		&reg.yawPitchRoll,
		&reg.bodyAccel,
		&reg.gyro);

	return reg;
}

YawPitchRollTrueInertialAccelerationAndAngularRatesRegister VnSensor::readYawPitchRollTrueInertialAccelerationAndAngularRates()
{
	char toSend[17];

	size_t length = Packet::genReadYawPitchRollTrueInertialAccelerationAndAngularRates(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	YawPitchRollTrueInertialAccelerationAndAngularRatesRegister reg;
	response.parseYawPitchRollTrueInertialAccelerationAndAngularRates(
		&reg.yawPitchRoll,
		&reg.inertialAccel,
		&reg.gyro);

	return reg;
}

VpeBasicControlRegister VnSensor::readVpeBasicControl()
{
	char toSend[17];

	size_t length = Packet::genReadVpeBasicControl(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	VpeBasicControlRegister reg;
	uint8_t enable;
	uint8_t headingMode;
	uint8_t filteringMode;
	uint8_t tuningMode;
	response.parseVpeBasicControl(
		&enable,
		&headingMode,
		&filteringMode,
		&tuningMode);

	reg.enable = static_cast<VpeEnable>(enable);
	reg.headingMode = static_cast<HeadingMode>(headingMode);
	reg.filteringMode = static_cast<VpeMode>(filteringMode);
	reg.tuningMode = static_cast<VpeMode>(tuningMode);

	return reg;
}

void VnSensor::writeVpeBasicControl(VpeBasicControlRegister &fields, bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteVpeBasicControl(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), fields.enable, fields.headingMode, fields.filteringMode, fields.tuningMode);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

void VnSensor::writeVpeBasicControl(
	VpeEnable enable,
	HeadingMode headingMode,
	VpeMode filteringMode,
	VpeMode tuningMode,
	bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteVpeBasicControl(
		_pi->_sendErrorDetectionMode,
		toSend,
		sizeof(toSend),
		enable,
		headingMode,
		filteringMode,
		tuningMode);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}
VpeMagnetometerBasicTuningRegister VnSensor::readVpeMagnetometerBasicTuning()
{
	char toSend[17];

	size_t length = Packet::genReadVpeMagnetometerBasicTuning(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	VpeMagnetometerBasicTuningRegister reg;
	response.parseVpeMagnetometerBasicTuning(
		&reg.baseTuning,
		&reg.adaptiveTuning,
		&reg.adaptiveFiltering);

	return reg;
}

void VnSensor::writeVpeMagnetometerBasicTuning(VpeMagnetometerBasicTuningRegister &fields, bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteVpeMagnetometerBasicTuning(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), fields.baseTuning, fields.adaptiveTuning, fields.adaptiveFiltering);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

void VnSensor::writeVpeMagnetometerBasicTuning(
	const math::vec3f &baseTuning,
	const math::vec3f &adaptiveTuning,
	const math::vec3f &adaptiveFiltering,
	bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteVpeMagnetometerBasicTuning(
		_pi->_sendErrorDetectionMode,
		toSend,
		sizeof(toSend),
		baseTuning,
		adaptiveTuning,
		adaptiveFiltering);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}
VpeAccelerometerBasicTuningRegister VnSensor::readVpeAccelerometerBasicTuning()
{
	char toSend[17];

	size_t length = Packet::genReadVpeAccelerometerBasicTuning(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	VpeAccelerometerBasicTuningRegister reg;
	response.parseVpeAccelerometerBasicTuning(
		&reg.baseTuning,
		&reg.adaptiveTuning,
		&reg.adaptiveFiltering);

	return reg;
}

void VnSensor::writeVpeAccelerometerBasicTuning(VpeAccelerometerBasicTuningRegister &fields, bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteVpeAccelerometerBasicTuning(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), fields.baseTuning, fields.adaptiveTuning, fields.adaptiveFiltering);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

void VnSensor::writeVpeAccelerometerBasicTuning(
	const math::vec3f &baseTuning,
	const math::vec3f &adaptiveTuning,
	const math::vec3f &adaptiveFiltering,
	bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteVpeAccelerometerBasicTuning(
		_pi->_sendErrorDetectionMode,
		toSend,
		sizeof(toSend),
		baseTuning,
		adaptiveTuning,
		adaptiveFiltering);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}
MagnetometerCalibrationControlRegister VnSensor::readMagnetometerCalibrationControl()
{
	char toSend[17];

	size_t length = Packet::genReadMagnetometerCalibrationControl(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	MagnetometerCalibrationControlRegister reg;
	uint8_t hsiMode;
	uint8_t hsiOutput;
	response.parseMagnetometerCalibrationControl(
		&hsiMode,
		&hsiOutput,
		&reg.convergeRate);

	reg.hsiMode = static_cast<HsiMode>(hsiMode);
	reg.hsiOutput = static_cast<HsiOutput>(hsiOutput);

	return reg;
}

void VnSensor::writeMagnetometerCalibrationControl(MagnetometerCalibrationControlRegister &fields, bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteMagnetometerCalibrationControl(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), fields.hsiMode, fields.hsiOutput, fields.convergeRate);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

void VnSensor::writeMagnetometerCalibrationControl(
	HsiMode hsiMode,
	HsiOutput hsiOutput,
	const uint8_t &convergeRate,
	bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteMagnetometerCalibrationControl(
		_pi->_sendErrorDetectionMode,
		toSend,
		sizeof(toSend),
		hsiMode,
		hsiOutput,
		convergeRate);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}
CalculatedMagnetometerCalibrationRegister VnSensor::readCalculatedMagnetometerCalibration()
{
	char toSend[17];

	size_t length = Packet::genReadCalculatedMagnetometerCalibration(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	CalculatedMagnetometerCalibrationRegister reg;
	response.parseCalculatedMagnetometerCalibration(
		&reg.c,
		&reg.b);

	return reg;
}

VelocityCompensationControlRegister VnSensor::readVelocityCompensationControl()
{
	char toSend[17];

	size_t length = Packet::genReadVelocityCompensationControl(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	VelocityCompensationControlRegister reg;
	response.parseVelocityCompensationControl(
		&reg.mode,
		&reg.velocityTuning,
		&reg.rateTuning);

	return reg;
}

void VnSensor::writeVelocityCompensationControl(VelocityCompensationControlRegister &fields, bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteVelocityCompensationControl(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), fields.mode, fields.velocityTuning, fields.rateTuning);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

void VnSensor::writeVelocityCompensationControl(
	const uint8_t &mode,
	const float &velocityTuning,
	const float &rateTuning,
	bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteVelocityCompensationControl(
		_pi->_sendErrorDetectionMode,
		toSend,
		sizeof(toSend),
		mode,
		velocityTuning,
		rateTuning);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}
VelocityCompensationStatusRegister VnSensor::readVelocityCompensationStatus()
{
	char toSend[17];

	size_t length = Packet::genReadVelocityCompensationStatus(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	VelocityCompensationStatusRegister reg;
	response.parseVelocityCompensationStatus(
		&reg.x,
		&reg.xDot,
		&reg.accelOffset,
		&reg.omega);

	return reg;
}

vec3f VnSensor::readVelocityCompensationMeasurement()
{
	char toSend[17];

	size_t length = Packet::genReadVelocityCompensationMeasurement(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	vec3f reg;
	response.parseVelocityCompensationMeasurement(&reg);

	return reg;
}

void VnSensor::writeVelocityCompensationMeasurement(const vec3f &velocity, bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteVelocityCompensationMeasurement(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), velocity);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

MagneticAndGravityReferenceVectorsRegister VnSensor::readMagneticAndGravityReferenceVectors()
{
	char toSend[17];

	size_t length = Packet::genReadMagneticAndGravityReferenceVectors(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	MagneticAndGravityReferenceVectorsRegister reg;
	response.parseMagneticAndGravityReferenceVectors(
		&reg.magRef,
		&reg.accRef);

	return reg;
}

void VnSensor::writeMagneticAndGravityReferenceVectors(MagneticAndGravityReferenceVectorsRegister &fields, bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteMagneticAndGravityReferenceVectors(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), fields.magRef, fields.accRef);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

void VnSensor::writeMagneticAndGravityReferenceVectors(
	const math::vec3f &magRef,
	const math::vec3f &accRef,
	bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteMagneticAndGravityReferenceVectors(
		_pi->_sendErrorDetectionMode,
		toSend,
		sizeof(toSend),
		magRef,
		accRef);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}
ReferenceVectorConfigurationRegister VnSensor::readReferenceVectorConfiguration()
{
	char toSend[17];

	size_t length = Packet::genReadReferenceVectorConfiguration(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	ReferenceVectorConfigurationRegister reg;
	uint8_t useMagModel;
	uint8_t useGravityModel;
	response.parseReferenceVectorConfiguration(
		&useMagModel,
		&useGravityModel,
		&reg.recalcThreshold,
		&reg.year,
		&reg.position);

	reg.useMagModel = useMagModel != 0;
	reg.useGravityModel = useGravityModel != 0;

	return reg;
}

void VnSensor::writeReferenceVectorConfiguration(ReferenceVectorConfigurationRegister &fields, bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteReferenceVectorConfiguration(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), fields.useMagModel, fields.useGravityModel, 0, 0, fields.recalcThreshold, fields.year, fields.position);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

void VnSensor::writeReferenceVectorConfiguration(
	const uint8_t &useMagModel,
	const uint8_t &useGravityModel,
	const uint32_t &recalcThreshold,
	const float &year,
	const math::vec3d &position,
	bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteReferenceVectorConfiguration(
		_pi->_sendErrorDetectionMode,
		toSend,
		sizeof(toSend),
		useMagModel,
		useGravityModel,
		0,
		0,
		recalcThreshold,
		year,
		position);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}
GpsSolutionLlaRegister VnSensor::readGpsSolutionLla()
{
	char toSend[17];

	size_t length = Packet::genReadGpsSolutionLla(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	GpsSolutionLlaRegister reg;
	uint8_t gpsFix;
	response.parseGpsSolutionLla(
		&reg.time,
		&reg.week,
		&gpsFix,
		&reg.numSats,
		&reg.lla,
		&reg.nedVel,
		&reg.nedAcc,
		&reg.speedAcc,
		&reg.timeAcc);

	reg.gpsFix = static_cast<GpsFix>(gpsFix);

	return reg;
}

GpsSolutionEcefRegister VnSensor::readGpsSolutionEcef()
{
	char toSend[17];

	size_t length = Packet::genReadGpsSolutionEcef(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	GpsSolutionEcefRegister reg;
	uint8_t gpsFix;
	response.parseGpsSolutionEcef(
		&reg.tow,
		&reg.week,
		&gpsFix,
		&reg.numSats,
		&reg.position,
		&reg.velocity,
		&reg.posAcc,
		&reg.speedAcc,
		&reg.timeAcc);

	reg.gpsFix = static_cast<GpsFix>(gpsFix);

	return reg;
}

GpsConfigurationRegister VnSensor::readGpsConfiguration()
{
	char toSend[17];

	size_t length = Packet::genReadGpsConfiguration(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	GpsConfigurationRegister reg;
	uint8_t mode;
	uint8_t ppsSource;
	response.parseGpsConfiguration(
		&mode,
		&ppsSource);

	reg.mode = static_cast<GpsMode>(mode);
	reg.ppsSource = static_cast<PpsSource>(ppsSource);

	return reg;
}

void VnSensor::writeGpsConfiguration(GpsConfigurationRegister &fields, bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteGpsConfiguration(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), fields.mode, fields.ppsSource, 5, 0, 0);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

void VnSensor::writeGpsConfiguration(
	GpsMode mode,
	PpsSource ppsSource,
	bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteGpsConfiguration(
		_pi->_sendErrorDetectionMode,
		toSend,
		sizeof(toSend),
		mode,
		ppsSource,
		5,
		0,
		0);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}
vec3f VnSensor::readGpsAntennaOffset()
{
	char toSend[17];

	size_t length = Packet::genReadGpsAntennaOffset(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	vec3f reg;
	response.parseGpsAntennaOffset(&reg);

	return reg;
}

void VnSensor::writeGpsAntennaOffset(const vec3f &position, bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteGpsAntennaOffset(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), position);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

GpsCompassBaselineRegister VnSensor::readGpsCompassBaseline()
{
	char toSend[17];

	size_t length = Packet::genReadGpsCompassBaseline(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	GpsCompassBaselineRegister reg;
	response.parseGpsCompassBaseline(
		&reg.position,
		&reg.uncertainty);

	return reg;
}

void VnSensor::writeGpsCompassBaseline(GpsCompassBaselineRegister &fields, bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteGpsCompassBaseline(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), fields.position, fields.uncertainty);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

void VnSensor::writeGpsCompassBaseline(
	const math::vec3f &position,
	const math::vec3f &uncertainty,
	bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteGpsCompassBaseline(
		_pi->_sendErrorDetectionMode,
		toSend,
		sizeof(toSend),
		position,
		uncertainty);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}
GpsCompassEstimatedBaselineRegister VnSensor::readGpsCompassEstimatedBaseline()
{
	char toSend[17];

	size_t length = Packet::genReadGpsCompassEstimatedBaseline(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	GpsCompassEstimatedBaselineRegister reg;
	uint8_t estBaselineUsed;
	response.parseGpsCompassEstimatedBaseline(
		&estBaselineUsed,
		&reg.numMeas,
		&reg.position,
		&reg.uncertainty);

	reg.estBaselineUsed = estBaselineUsed != 0;

	return reg;
}

InsSolutionLlaRegister VnSensor::readInsSolutionLla()
{
	char toSend[17];

	size_t length = Packet::genReadInsSolutionLla(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	InsSolutionLlaRegister reg;
	response.parseInsSolutionLla(
		&reg.time,
		&reg.week,
		&reg.status,
		&reg.yawPitchRoll,
		&reg.position,
		&reg.nedVel,
		&reg.attUncertainty,
		&reg.posUncertainty,
		&reg.velUncertainty);

	return reg;
}

InsSolutionEcefRegister VnSensor::readInsSolutionEcef()
{
	char toSend[17];

	size_t length = Packet::genReadInsSolutionEcef(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	InsSolutionEcefRegister reg;
	response.parseInsSolutionEcef(
		&reg.time,
		&reg.week,
		&reg.status,
		&reg.yawPitchRoll,
		&reg.position,
		&reg.velocity,
		&reg.attUncertainty,
		&reg.posUncertainty,
		&reg.velUncertainty);

	return reg;
}

InsStateLlaRegister VnSensor::readInsStateLla()
{
	char toSend[17];

	size_t length = Packet::genReadInsStateLla(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	InsStateLlaRegister reg;
	response.parseInsStateLla(
		&reg.yawPitchRoll,
		&reg.position,
		&reg.velocity,
		&reg.accel,
		&reg.angularRate);

	return reg;
}

InsStateEcefRegister VnSensor::readInsStateEcef()
{
	char toSend[17];

	size_t length = Packet::genReadInsStateEcef(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	InsStateEcefRegister reg;
	response.parseInsStateEcef(
		&reg.yawPitchRoll,
		&reg.position,
		&reg.velocity,
		&reg.accel,
		&reg.angularRate);

	return reg;
}

InsBasicConfigurationRegisterVn200 VnSensor::readInsBasicConfigurationVn200()
{
	char toSend[17];

	size_t length = Packet::genReadInsBasicConfiguration(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	InsBasicConfigurationRegisterVn200 reg;
	uint8_t scenario;
	uint8_t ahrsAiding;
	response.parseInsBasicConfiguration(
		&scenario,
		&ahrsAiding);

	reg.scenario = static_cast<Scenario>(scenario);
	reg.ahrsAiding = ahrsAiding != 0;

	return reg;
}

void VnSensor::writeInsBasicConfiguration(InsBasicConfigurationRegisterVn200 &fields, bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteInsBasicConfiguration(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), fields.scenario, fields.ahrsAiding, 0, 0);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

void VnSensor::writeInsBasicConfiguration(
	Scenario scenario,
	const uint8_t &ahrsAiding,
	bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteInsBasicConfiguration(
		_pi->_sendErrorDetectionMode,
		toSend,
		sizeof(toSend),
		scenario,
		ahrsAiding,
		0,
		0);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}
InsBasicConfigurationRegisterVn300 VnSensor::readInsBasicConfigurationVn300()
{
	char toSend[17];

	size_t length = Packet::genReadInsBasicConfiguration(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	InsBasicConfigurationRegisterVn300 reg;
	uint8_t scenario;
	uint8_t ahrsAiding;
	uint8_t estBaseline;
	response.parseInsBasicConfiguration(
		&scenario,
		&ahrsAiding,
		&estBaseline);

	reg.scenario = static_cast<Scenario>(scenario);
	reg.ahrsAiding = ahrsAiding != 0;
	reg.estBaseline = estBaseline != 0;

	return reg;
}

void VnSensor::writeInsBasicConfiguration(InsBasicConfigurationRegisterVn300 &fields, bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteInsBasicConfiguration(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), fields.scenario, fields.ahrsAiding, fields.estBaseline, 0);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

void VnSensor::writeInsBasicConfiguration(
	Scenario scenario,
	const uint8_t &ahrsAiding,
	const uint8_t &estBaseline,
	bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteInsBasicConfiguration(
		_pi->_sendErrorDetectionMode,
		toSend,
		sizeof(toSend),
		scenario,
		ahrsAiding,
		estBaseline,
		0);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}
StartupFilterBiasEstimateRegister VnSensor::readStartupFilterBiasEstimate()
{
	char toSend[17];

	size_t length = Packet::genReadStartupFilterBiasEstimate(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	StartupFilterBiasEstimateRegister reg;
	response.parseStartupFilterBiasEstimate(
		&reg.gyroBias,
		&reg.accelBias,
		&reg.pressureBias);

	return reg;
}

void VnSensor::writeStartupFilterBiasEstimate(StartupFilterBiasEstimateRegister &fields, bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteStartupFilterBiasEstimate(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), fields.gyroBias, fields.accelBias, fields.pressureBias);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

void VnSensor::writeStartupFilterBiasEstimate(
	const math::vec3f &gyroBias,
	const math::vec3f &accelBias,
	const float &pressureBias,
	bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteStartupFilterBiasEstimate(
		_pi->_sendErrorDetectionMode,
		toSend,
		sizeof(toSend),
		gyroBias,
		accelBias,
		pressureBias);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

#ifdef PYTHON_WRAPPERS

/*uint32_t VnSensor::readAsyncDataOutputFrequency_boostwrapper()
{
	uint32_t adof;

	readAsyncDataOutputFrequency(&adof);

	return adof;

	//return boost::python::make_tuple(adof, 4);
}*/

#endif

}
}
