//----------------------------------------------------------------------------
//
//  Copyright (C) 2017, Northern Digital Inc. All rights reserved.
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
#ifndef TCP_CONNECTION_HPP
#define TCP_CONNECTION_HPP

#ifdef _WIN32
// Include the DLL for socket programming
// Confusingly, the 64-bit library has 32 in its name as well...
// See: http://stackoverflow.com/questions/5507607/winsock-and-x64-target
#pragma comment(lib, "Ws2_32.lib")
#include <winsock2.h> // for Windows Socket API (WSA)
#include <ws2tcpip.h> // for getaddrinfo() etc...

#else
// Mac and Linux use a different API for sockets
#include <sys/socket.h>
#include <sys/types.h>
#include <netdb.h>
#include <unistd.h>
#endif

#include "Connection.h"

/**
 * @brief A cross platform socket implementation.
 */
class TcpConnection : public Connection
{
public:
	/** Constructs a socket object that is cross-platform. */
	TcpConnection();

	/**
	 * @brief Constructs a socket object and connects immediately.
	 * @param hostname The hostname or IP address of the measurement system.
	 * @param port The port to connect on. Port 8765 is default for Vega systems.
	 */
	TcpConnection(const char* hostname, const char* port = "8765");

	/**
	 * @brief Closes the socket connection and frees memory.
	 */
	virtual ~TcpConnection();

	/**
	 * @brief Closes any existing connection, and connects to the new device.
	 * @param hostname The hostname or IP address of the device.
	 * @param port The port number to connect on.
	 */
	bool connect(const char* hostname, const char* port);

	/**
	 * @brief Closes any existing connection, and connects to the new device on port 8765.
	 * @param hostname The hostname or IP address of the device.
	 */
	bool connect(const char* hostname);

	/** @brief Closes the socket connection */
	void disconnect();

	/** @brief Returns true if the socket connection succeeded */
	bool isConnected() const;

    /**
	 * @brief Reads 'length' bytes from the socket into 'buffer'
	 * @param buffer The buffer to read into.
	 * @param length The number of bytes to read.
	 */
	int read(byte_t* buffer, int length) const;

    /**
	 * @brief Reads 'length' chars  from the socket into 'buffer'
	 * @param buffer The buffer to read into.
	 * @param length The number of chars to read.
	 */
	int read(char* buffer, int length) const;

	/**
	 * @brief Writes 'length' bytes from 'buffer' to the socket
	 * @param buffer The buffer to write from.
	 * @param length The number of bytes to write.
	 */
	int write(byte_t* buffer, int length) const;

	/**
	 * @brief Writes 'length' chars from 'buffer' to the socket
	 * @param buffer The buffer to write from.
	 * @param length The number of chars to write.
	 */
	int write(const char* buffer, int length) const;

private:
	static const int NUM_CONNECTION_RETRIES = 3;

	//! Setup method common to all constructors.
	void init();

	//! Returns true if the socket is valid, otherwise false
	bool socketIsValid() const;

	/* Private Members */
	bool isConnected_;

	#ifdef _WIN32 // Windows socket implementation
	WSADATA   wsaData_;
	SOCKET    ndiSocket_;
	#else // Mac or Linux
	//! File descriptor for the socket
	int ndiSocket_;
	#endif
};

#endif // TCP_CONNECTION_HPP
