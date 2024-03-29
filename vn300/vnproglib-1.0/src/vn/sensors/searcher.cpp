// VectorNav Programming Library v1.0.0.1
// Copyright (c) 2015 VectorNav Technologies, LLC
#include "vn/sensors/searcher.h"

#include <list>

#include "vn/xplat/serialport.h"
#include "vn/xplat/event.h"
#include "vn/xplat/thread.h"
#include "vn/protocol/uart.h"

using namespace std;
using namespace vn::xplat;
using namespace vn::protocol::uart;

namespace vn {
namespace sensors {

struct TestHelper
{
	SerialPort *serialPort;
	PacketFinder *packetFinder;
	Event waitForCheckingOnPort;

	TestHelper(SerialPort *serialPort, PacketFinder *packetFinder) :
		serialPort(serialPort),
		packetFinder(packetFinder)
	{ }
};

struct SearchHelper
{
	Event finishedSearchingPort;
	bool sensorFound;
	string portName;
	uint32_t foundBaudrate;

	explicit SearchHelper(const string &portName) :
		sensorFound(false),
		portName(portName)
	{ }
};

void testDataReceivedHandler(void* userData);
void testValidPacketFoundHandler(void *userData, Packet &packet, size_t runningIndexOfPacketStart);
void searchThread(void* routineData);

// Collection of baudrates to test for sensors. They are listed in order of
// liklyness and fastness.
#if __cplusplus >= 201103L
vector<uint32_t> TestBaudrates { 115200, 128000, 230400, 460800, 921600, 57600, 38400, 19200, 9600 };
#else
uint32_t TestBaudratesRaw[] = { 115200, 128000, 230400, 460800, 921600, 57600, 38400, 19200, 9600 };
#endif

bool Searcher::search(const string &portName, uint32_t *foundBaudrate)
{
	#if __cplusplus < 201103L
	vector<uint32_t> TestBaudrates(TestBaudratesRaw, TestBaudratesRaw + sizeof(TestBaudratesRaw) / sizeof(TestBaudratesRaw[0]));
	#endif
	for (vector<uint32_t>::const_iterator it = TestBaudrates.begin(); it != TestBaudrates.end(); ++it)
	{
		uint32_t curBaudrate = *it;

		if (test(portName, curBaudrate))
		{
			*foundBaudrate = curBaudrate;

			return true;
		}
	}

	return false;
}

vector<pair<string, uint32_t> > Searcher::search()
{
	return search(SerialPort::getPortNames());
}

vector<pair<string, uint32_t> > Searcher::search(vector<string> portsToCheck)
{
	list<SearchHelper*> helpers;
	vector<pair<string, uint32_t> > result;

	// Start threads to test each port.
	for (vector<string>::const_iterator it = portsToCheck.begin(); it != portsToCheck.end(); ++it)
	{
		SearchHelper *sh = new SearchHelper(*it);

		helpers.push_back(sh);

		Thread::startNew(searchThread, sh);
	}

	// Wait for each thread to finish.
	for (list<SearchHelper*>::const_iterator it = helpers.begin(); it != helpers.end(); ++it)
	{
		SearchHelper* sh = (*it);

		sh->finishedSearchingPort.wait();

		if (sh->sensorFound)
		{
			result.push_back(pair<string, uint32_t>(sh->portName, sh->foundBaudrate));
		}

		delete sh;
	}

	return result;
}

bool Searcher::test(string portName, uint32_t baudrate)
{
	SerialPort sp(portName, baudrate);
	PacketFinder pf;

	try
	{
		sp.open();
	}
	catch (invalid_argument)
	{
		// Probably an unsupported baudrate for the port.
		return false;
	}

	TestHelper th(&sp, &pf);

	pf.registerPossiblePacketFoundHandler(&th, testValidPacketFoundHandler);
	sp.registerDataReceivedHandler(&th, testDataReceivedHandler);

	// Wait for a few milliseconds to see if we receive any asynchronous
	// data packets.
	if (th.waitForCheckingOnPort.waitMs(50) == Event::WAIT_SIGNALED)
	{
		sp.close();

		return true;
	}

	// We have received any asynchronous data packets so let's try sending
	// some commands.
	for (size_t i = 0; i < 9; i++)
	{
		sp.write("$VNRRG,01*XX\r\n", 14);

		if (th.waitForCheckingOnPort.waitMs(50) == Event::WAIT_SIGNALED)
		{
			sp.close();

			return true;
		}
	}

	sp.close();

	return false;
}

void searchThread(void* routineData)
{
	SearchHelper *sh = static_cast<SearchHelper*>(routineData);

	sh->sensorFound = Searcher::search(sh->portName, &sh->foundBaudrate);

	sh->finishedSearchingPort.signal();
}

void testDataReceivedHandler(void* userData)
{
	TestHelper *th = static_cast<TestHelper*>(userData);

	char buffer[0x100];
	size_t numOfBytesRead;

	th->serialPort->read(buffer, 0x100, numOfBytesRead);

	th->packetFinder->processReceivedData(buffer, numOfBytesRead);
}

void testValidPacketFoundHandler(void *userData, Packet &packet, size_t runningIndexOfPacketStart)
{
	TestHelper *th = static_cast<TestHelper*>(userData);

	th->waitForCheckingOnPort.signal();
}

}
}
