// VectorNav Programming Library v1.0.0.1
// Copyright (c) 2015 VectorNav Technologies, LLC
#ifndef _VNXPLAT_SERIALPORT_H_
#define _VNXPLAT_SERIALPORT_H_

#if defined(_MSC_VER)
	// We are using Visual Studio. Suppress warnings about unsafe library functions.
	#define _CRT_SECURE_NO_WARNINGS
#endif

#include <string>
#include <vector>

#include "vn/int.h"
#include "vn/common/isimpleport.h"
#include "vn/util/nocopy.h"

namespace vn {
namespace xplat {

/// \brief Represents a cross-platform serial port.
///
/// When the SerialPort if first created and the connection opened, the user
/// will normally have to poll the method \ref read to see if any new data is
/// available on the serial port. However, if the user code registers a
/// handler with the method \ref registerDataReceivedHandler, the SerialPort
/// ojbect will start an internal thread that monitors the serial port for new
/// data, and when new data is available, it will alert the user code through
/// the callback handler. Then the user can call \ref read to retrieve the
/// data.
class SerialPort : public common::ISimplePort, private util::NoCopy
{

	// Constructors ///////////////////////////////////////////////////////////

public:

	/// \brief Creates a new \ref SerialPort with the provided connection
	///     parameters.
	///
	/// \param[in] portName The name of the serial port.
	/// \param[in] baudrate The baudrate to open the serial port at.
	SerialPort(const std::string& portName, uint32_t baudrate);

	~SerialPort();

	// Public Methods /////////////////////////////////////////////////////////

public:

	/// \brief Returns a list of the names of all the available serial ports on
	///     the system.
	///
	/// \return The list of available serial port names.
	static std::vector<std::string> getPortNames();

	virtual void open();

	virtual void close();

	virtual bool isOpen();

	virtual void write(const char data[], size_t length);

	virtual void read(char dataBuffer[], size_t numOfBytesToRead, size_t &numOfBytesActuallyRead);

	virtual void registerDataReceivedHandler(void* userData, DataReceivedHandler handler);

	virtual void unregisterDataReceivedHandler();

	/// \brief Indicates if the platforms supports event notifications.

	/// \brief Returns the number of dropped sections of received data.
	///
	/// \return The number of sections of dropped data sections. Note this is
	///     not indicative of the total number of dropped bytes.
	size_t NumberOfReceiveDataDroppedSections();

	/// \brief With regard to optimizing COM ports provided by FTDI drivers, this
	/// method will check if the COM port has been optimized.
	///
	/// \param[in] portName The COM port name to check.
	/// \return <c>true</c> if the COM port is optimized; otherwise <c>false</c>.
	static bool determineIfPortIsOptimized(std::string portName);

	/// \brief This will perform optimization of FTDI USB serial ports.
	///
	/// If calling this method on Windows, the process must have administrator
	/// privileges to write settings to the registry. Otherwise an
	///
	/// \param[in] portName The FTDI USB Serial Port to optimize.
	static void optimizePort(std::string portName);

	// Private Members ////////////////////////////////////////////////////////

private:

	// Contains internal data, mainly stuff that is required for cross-platform
	// support.
	struct Impl;
	Impl *_pi;

};

}
}

#endif
