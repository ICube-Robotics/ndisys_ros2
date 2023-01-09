//----------------------------------------------------------------------------
//
//  Copyright (C) 2016, Northern Digital Inc. All rights reserved.
//
//  All Northern Digital Inc. ("NDI") Media and/or Sample Code and/or Sample Code
//  Documentation (collectively referred to as "Sample Code") is licensed and provided "as
//  is" without warranty of any kind. The licensee, by use of the Sample Code, warrants to
//  NDI that the Sample Code is fit for the use and purpose for which the licensee intends to
//  use the Sample Code. NDI makes no warranties, express or implied, that the functions
//  contained in the Sample Code will meet the licensee's requirements or that the operation
//  of the programs contained therein will be error free. This warranty as expressed herein is
//  exclusive and NDI expressly disclaims any and all express and/or implied, in fact or in
//  law, warranties, representations, and conditions of every kind pertaining in any way to
//  the Sample Code licensed and provided by NDI hereunder, including without limitation,
//  each warranty and/or condition of quality, merchantability, description, operation,
//  adequacy, suitability, fitness for particular purpose, title, interference with use or
//  enjoyment, and/or non infringement, whether express or implied by statute, common law,
//  usage of trade, course of dealing, custom, or otherwise. No NDI dealer, distributor, agent
//  or employee is authorized to make any modification or addition to this warranty.
//  In no event shall NDI nor any of its employees be liable for any direct, indirect,
//  incidental, special, exemplary, or consequential damages, sundry damages or any
//  damages whatsoever, including, but not limited to, procurement of substitute goods or
//  services, loss of use, data or profits, or business interruption, however caused. In no
//  event shall NDI's liability to the licensee exceed the amount paid by the licensee for the
//  Sample Code or any NDI products that accompany the Sample Code. The said limitations
//  and exclusions of liability shall apply whether or not any such damages are construed as
//  arising from a breach of a representation, warranty, guarantee, covenant, obligation,
//  condition or fundamental term or on any theory of liability, whether in contract, strict
//  liability, or tort (including negligence or otherwise) arising in any way out of the use of
//  the Sample Code even if advised of the possibility of such damage. In no event shall
//  NDI be liable for any claims, losses, damages, judgments, costs, awards, expenses or
//  liabilities of any kind whatsoever arising directly or indirectly from any injury to person
//  or property, arising from the Sample Code or any use thereof
//
//----------------------------------------------------------------------------

#include <iostream>

#include "ComConnection.h"

int ComConnection::read(byte_t* buffer, int length) const
{
	return read((char*)buffer, length);
}

int ComConnection::write(byte_t* buffer, int length) const
{
	return write((const char*)buffer, length);
}

ComConnection::~ComConnection()
{
	disconnect();
}

#ifdef _WIN32 // Windows serial port implementation

ComConnection::ComConnection(std::string comPort)
{
	hComm_ = INVALID_HANDLE_VALUE;
	connect(comPort.c_str());
}

bool ComConnection::connect(const char* comPort)
{
	char portName[10] = {'\0'};
    sprintf_s( portName, "\\\\.\\%s", comPort );
	hComm_ = CreateFile( portName,		// The file name
						GENERIC_READ | GENERIC_WRITE, // Read/Write mode
						0,				// exclusive-access
						0,				// no security attrs
						OPEN_EXISTING,  // comm devices must use OPEN_EXISTING
						0,				// FILE_FLAG_OVERLAPPED can be used for non-blocking I/O, but we're going to block :)
						NULL);			// hTemplate must be NULL for comm devices

	// Set read/write timeouts
	COMMTIMEOUTS timeouts = { 0, 0, TIMEOUT_MS, 0, TIMEOUT_MS };
	if (!SetCommTimeouts(hComm_, &timeouts))
	{
		disconnect();
	}

	// Begin with default parameters
	setSerialPortParams();
	return isConnected();
}

void ComConnection::disconnect()
{
	if( hComm_ != INVALID_HANDLE_VALUE )
    {
        CloseHandle( hComm_ );
        hComm_ = INVALID_HANDLE_VALUE;
    }
}

bool ComConnection::isConnected() const
{
	// The connection is open if we have a valid COM port handle
	return (hComm_ != INVALID_HANDLE_VALUE);
}

bool ComConnection::setSerialPortParams(int baudRate, int dataBits, int parity, int stopBits, int enableHandshake) const
{
	// Get the DeviceControlBlock (DCB) structure that stores the serial port settings
	DCB deviceControlBlock = {0};
	if (!GetCommState(hComm_, &deviceControlBlock))
	{
		return false;
	}

	// Set the desired parameters on the DCB structure
	deviceControlBlock.BaudRate = baudRate;
	deviceControlBlock.ByteSize = dataBits;
	deviceControlBlock.fParity = (parity > 0); // parityEnabled
	deviceControlBlock.StopBits = (stopBits == 1 ? ONESTOPBIT : TWOSTOPBITS);
	deviceControlBlock.fOutxCtsFlow = (enableHandshake > 0); // handshakeEnabled
	deviceControlBlock.fRtsControl = (enableHandshake > 0 ? RTS_CONTROL_ENABLE : RTS_CONTROL_DISABLE);

	// Send the structure back to the OS to save the settings
	if (!SetCommState(hComm_, &deviceControlBlock))
	{
		return false;
	}

	// Return true to indicate settings were saved successfully
	return true;
}

bool ComConnection::sendSerialBreak() const
{
	// Hold the transmission line in break state
	if (!SetCommBreak(hComm_))
	{
		return false;
	}

	// Keep the break active for a while
	Sleep(SERIAL_BREAK_DURATION_MS);

	// Put the transmission line back in its normal state
	if (!ClearCommBreak(hComm_))
	{
		return false;
	}
	return true;
}

int ComConnection::write(const char* buffer, int length) const
{
	DWORD lpLength = (DWORD) length;
	if (!WriteFile(hComm_, buffer, lpLength, &lpLength, NULL))
	{
		// Debugging: use GetLastError()
		// wchar_t errorMessage[256] = {0};
		// FormatMessageW(FORMAT_MESSAGE_FROM_SYSTEM, NULL, GetLastError(), MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), errorMessage, 256, NULL);
		return -1;
	}
   // A successful write sends the number of bytes requested
   return length;
}

int ComConnection::read(char* buffer, int length) const
{
	// Execute a blocking read
	DWORD lpLength = (DWORD) length;
	if (!ReadFile(hComm_, (LPVOID) buffer, lpLength, &lpLength, NULL))
	{
		// Debugging: use GetLastError()
		// wchar_t errorMessage[256] = {0};
		// FormatMessageW(FORMAT_MESSAGE_FROM_SYSTEM, NULL, GetLastError(), MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), errorMessage, 256, NULL);
		return -1;
	}

	// A successful read returned the number of bytes requested
	return length;
}

#else // Mac/Linux serial port implementation

ComConnection::ComConnection(std::string comPort)
{
	fdComm_ = 0;
	connect(comPort.c_str());
}

bool ComConnection::isConnected() const
{
	return fdComm_ > 0;
}

void ComConnection::disconnect()
{
	close(fdComm_);
}

bool ComConnection::connect(const char* comPort)
{
	// open() for blocking reads
	// Note: Serial ports have a "cu" and a "tty" port, but only the cu works with blocking mode
	fdComm_ = open(comPort, O_RDWR | O_NOCTTY );

	// std::cout << "fdComm_:" << fdComm_ << std::endl;

	// Begin with default parameters
	setSerialPortParams();
	return isConnected();
}

bool ComConnection::setSerialPortParams(int baudRate, int dataBits, int parity, int stopBits, int enableHandshake) const
{
	// The termios config structure stores the serial port settings, but setting baud rate directly is a bit of a mess
	// It's preferable to use cfget/cfset methods, see: https://en.wikibooks.org/wiki/Serial_Programming/termios#Baud-Rate_Setting
	struct termios config;
	int returnCode = tcgetattr(fdComm_, &config);
	if (returnCode != 0)
	{
		std::cout << "Failed to get serial settings!" << std::endl;
		return false;
	}

	// Set the baud rate
	if (baudRate < 1)
	{
		return false;
	}

	speed_t baudRateEnum = B9600;
	switch(baudRate)
	{
	case 1228739:
		// 19200 is aliased to 1.2M baud in the Mac and Linux serial port drivers
		// This means you cannot set the host machine to read an actual 19200 baud on Mac/Linux
		baudRateEnum = B19200;
		break;
	#ifdef B921600 // B921600 is not defined on Mac OSX 10.11 but is on Linux Ubuntu 14.04
	case 921600:
		baudRateEnum = B921600;
		break;
	#endif
	case 115200:
		baudRateEnum = B115200;
		break;
	case 57600:
		baudRateEnum = B57600;
		break;
	case 38400:
		baudRateEnum = B38400;
		break;
	case 19200:
		baudRateEnum = B19200;
		break;
	#ifdef B14400 // B14400 is defined on Mac OSX 10.11 but noto Linux Ubuntu 14.04 ... o.0
	case 14400:
		baudRateEnum = B14400;
		break;
	#endif
	case 9600:
		baudRateEnum = B9600;
		break;
	default:
		// Try any positive baud rate integer directly and hope it works. libRXTX does this in SerialImp.c around line 820
		baudRateEnum = baudRate;
		break;
	}
	std::cout << "Setting baud rate: " << baudRate << ", enum:" << baudRateEnum << std::endl;

	if (cfsetspeed(&config, baudRateEnum) != 0)
	{
		std::cout << "Failed to set serial settings! errno[" << errno << "]=" << strerror(errno) << std::endl;
		std::cout << "input-speed: " << config.c_ispeed << std::endl;
		std::cout << "output-speed: " << config.c_ospeed << std::endl;
		return false;
	}

	// Set character size
	config.c_cflag &= ~CSIZE; // delete existing info
	switch (dataBits)
	{
	case 5:
		config.c_cflag |= CS5;
		break;
	case 6:
		config.c_cflag |= CS6;
		break;
	case 7:
		config.c_cflag |= CS7;
		break;
	case 8: // fall through
	default:
		config.c_cflag |= CS8;  // default 8-bits
		break;
	};

	// Set parity { 0 = None, 1 = Odd, 2 = Even}
	config.c_cflag &= ~(PARENB); // default no parity
	config.c_cflag |= (parity == 1 ? (PARENB | PARODD)  : 0); // odd parity
	config.c_cflag |= (parity == 2 ? PARENB : 0); // even parity

	// Set stop bits
	config.c_cflag &= ~CSTOPB; // default 1 stop bit
	config.c_cflag |= (stopBits > 0 ? CSTOPB : 0); // use 2 stop bits if desired

	// Set handshake (aka flow control)
	config.c_iflag &= ~(IXON | IXOFF | IXANY); // disabled
	config.c_cflag &= ~CRTSCTS; // disabled
	config.c_cflag |= (enableHandshake > 0 ? CRTSCTS : 0);

	// turn of canonical mode.
	config.c_lflag &= ~ICANON;

	// enable receiver
	config.c_cflag |= CREAD;
//	config.c_cflag |= CLOCAL;

	config.c_cc[VMIN] = 0;
	config.c_cc[VTIME] = 10;

	// Send the structure back to the OS to save the settings
	returnCode = tcsetattr(fdComm_, TCSANOW, &config);
	if (returnCode != 0)
	{
		std::cout << "Failed to save serial settings! errno[" << errno << "]=" << strerror(errno) << std::endl;
		return false;
	}

	// Return true to indicate settings were saved successfully
	// std::cout << "Serial settings saved successfully!" << std::endl;
	return true;
}

bool ComConnection::sendSerialBreak() const
{
	// Note: Unix doesn't define the units of duration, so it's implementation dependent! :(
	// The only portable value is zero --> a break of somewhere between 250-500ms
	return tcsendbreak(fdComm_, 0) == 0;
}

int ComConnection::write(const char* buffer, int length) const
{
	int bytesReceived = ::write(fdComm_, buffer, length);
	if (bytesReceived < 0)
	{
		std::cout << "write errno[" << errno << "]=" << strerror(errno) << std::endl;
	}
	return bytesReceived;
}

int ComConnection::read(char* buffer, int length) const
{
	int bytesReceived = 0;
	do
	{
		bytesReceived = ::read(fdComm_, buffer, length);
//		if( bytesReceived == 0 )
//		{
//			std::cout << "read zero bytes" << std::endl;
//		}
	}
	while( bytesReceived == 0 );

	if (bytesReceived < 0)
	{
		std::cout << "read errno[" << errno << "]=" << strerror(errno) << std::endl;
	}
	return bytesReceived;
}
#endif
