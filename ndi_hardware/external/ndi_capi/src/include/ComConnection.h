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
#ifndef COM_CONNECTION_HPP
#define COM_CONNECTION_HPP

// Conditionally compile Windows vs. POSIX serial port communication code
#ifdef _WIN32
// Use WINAPI for connecting to Windows COM ports
// See: https://msdn.microsoft.com/en-us/library/ff802693.aspx
// #include< <windows.h> causes naming conflicts with TcpConnection's includes
#include <winsock2.h>
#else
// Use POSIX compliant interface for Mac and Linux serial communication
// See: https://www.cmrr.umn.edu/~strupp/serial.html
#include <errno.h>
#include <fcntl.h>   // for open()
#include <string.h>  // for stderr()
#include <termios.h> // for tcsendbreak()
#include <unistd.h>  // for close()

#endif

#include <string>

#include "Connection.h"

/**
 * @brief A cross platform implementation of a lightweight serial port communication class.
 */
class ComConnection : public Connection
{
public:
   /**
	* @brief Constructs a ComConnection object and connects to the given port with default settings.
	* @param comPort The port to connect to (eg. "COM10").
	*/
	ComConnection(std::string comPort);

   /**
	* @brief Closes the connection and destroys the object.
	*/
	virtual ~ComConnection();

   /**
	* @brief Returns true if the connection is open.
	*/
	bool isConnected() const;

   /**
	* @brief Connect to the given COM port.
	* @param comPort The COM port to connect to (eg. "COM10").
	*/
	bool connect(const char* comPort);

   /**
	* @brief Closes the connection.
	*/
	void disconnect();

   /**
	* @brief Reads a specified number of characters from the connection.
	* @param buffer The buffer to read into.
	* @param length The number of characters to read.
	* @returns The number of characters read, or -1 if an error occurred.
	*/
	int read(char* buffer, int length) const;

	//! Convenience method for reading into an array of byte_t
	int read(byte_t* buffer, int length) const;

   /**
	* @brief Writes a specified number of characters to the connection.
	* @param buffer The buffer to write from.
	* @param length The number of characters to write.
	* @returns The number of characters written, or -1 if an error occurred.
	*/
	int write(const char* buffer, int length) const;

	//! Convenience method for writing from an array of byte_t
	int write(byte_t* buffer, int length) const;

	// Serial port specific methods

   /**
	* @brief Sets serial port parameters governing how the host sends/receives data.
	* @param baudRate Specifies the data transmission rate
	* @param dataBits Specifies the size of a byte
	* @param parity Specifies parity: { 0 = None, 1 = Odd, 2 = Even}
	* @param stopBits Specifies the number of stop bits: { 0 = 1 bit, 1 = 2 bits}
	* @param enableHandshake Enables or disables hardware handshaking: { 0 = Off, 1 = On}
	* @returns True if the settings were saved successfully, otherwise false.
	*/
	bool setSerialPortParams(int baudRate = 9600, int dataBits = 8, int parity = 0, int stopBits = 0, int enableHandshake = 1) const;

   /**
	* @brief Sends a serial break.
	* @returns True if the break was sent successfully, otherwise false.
	*/
	bool sendSerialBreak() const;

private:
	//! The duration that the serial break is active
	static const int SERIAL_BREAK_DURATION_MS = 250;

	//! The maximum number of millisceconds the host will wait for read/write operations
	static const int TIMEOUT_MS = 100;

	#ifdef _WIN32
		// Windows specific members
		HANDLE hComm_;
	#else
		// Mac/Linux specific members
		int fdComm_;
	#endif
};

#endif // COM_CONNECTION_HPP
