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

#include <iostream>
#include <string.h> // for memcpy

#include "TcpConnection.h"

int TcpConnection::write(const char* buffer, int length) const
{
	int result = send(ndiSocket_, buffer, length, 0);
	//std::cout << "------------socket write: " << (result > 0 ? "succeeded" : "failed") << std::endl;
	return result;
}

int TcpConnection::write(byte_t* buffer, int length) const
{
	return write((const char*)buffer, length);
}

int TcpConnection::read(char* buffer, int length) const
{
	int result = recv(ndiSocket_, buffer, length, MSG_WAITALL);
//	if (result != length )
//	{
//		std::cout << "------------failed: asked for " << length<<" read "<< result << std::endl;
//	}
	return result;
}

int TcpConnection::read(byte_t* buffer, int length) const
{
	return read((char*)buffer, length);
}

bool TcpConnection::connect(const char* hostname)
{
	return connect(hostname, "8765");
}

bool TcpConnection::connect(const char* hostname, const char* port)
{
	// Define socket options in the 'addrinfo' block
	addrinfo addressInfo;
	memset(&addressInfo, 0, sizeof(addressInfo));
	addressInfo.ai_family = AF_INET;
	addressInfo.ai_socktype = SOCK_STREAM;
	addressInfo.ai_protocol = IPPROTO_TCP;

	// Setup a TCP socket using the given hostname and port
	addrinfo* aiPointer = NULL;
	if (getaddrinfo(hostname, port, &addressInfo, &aiPointer) <= 0)
	{
		// Attempt to create a socket using the info returned by getaddrinfo()
		ndiSocket_ = socket(aiPointer->ai_family, aiPointer->ai_socktype, aiPointer->ai_protocol);

		// Technically we should try every addrinfo returned in the linked list before giving up...
		while (!socketIsValid() && aiPointer->ai_next != 0)
		{
			aiPointer = aiPointer->ai_next;
			ndiSocket_ = socket(aiPointer->ai_family, aiPointer->ai_socktype, aiPointer->ai_protocol);
		}

		// Attempt to open a connection if the socket is valid
		if (socketIsValid())
		{
			// Retry the connection a few times before giving up
			int retries = 0;
			while (!isConnected_ && retries++ <= NUM_CONNECTION_RETRIES)
			{
				// Open the socket using the connect method from the global namespace
				isConnected_ = ::connect(ndiSocket_, aiPointer->ai_addr, (int)aiPointer->ai_addrlen) >= 0;
			}
		}
	}

	return isConnected_;
}

bool TcpConnection::socketIsValid() const
{
	#ifdef _WIN32
	// On Windows, SOCKET type is unsigned, so a negative number cannot indicate invalid socket
	// so "all bits set" indicates invalid sockets:  #define INVALID_SOCKET (SOCKET)(~0)
	return ndiSocket_ != INVALID_SOCKET;
	#else
	// On Mac/Linux, sockets are simple. If the file descriptor is negative, the socket is invalid
	return ndiSocket_ >= 0;
	#endif
}

void TcpConnection::disconnect()
{
	#ifdef _WIN32
	closesocket(ndiSocket_);
	#else
	close(ndiSocket_);
	#endif
}

void TcpConnection::init()
{
	#ifdef _WIN32
	// Initializes WinSock using the v2 implementation
	// See: https://msdn.microsoft.com/en-us/library/windows/desktop/ms738545%28v=vs.85%29.aspx
	WSAStartup(MAKEWORD(2,2), &wsaData_);
	#endif
	ndiSocket_ = 0;
	isConnected_ = false;
}

bool TcpConnection::isConnected() const
{
	return isConnected_;
}

TcpConnection::TcpConnection(const char* hostname, const char* port)
{
	init();
	connect(hostname, port);
}

TcpConnection::TcpConnection()
{
	init();
}

TcpConnection::~TcpConnection()
{
	disconnect();

	#ifdef _WIN32
	// An application must call the WSACleanup function for every successful time the WSAStartup function is called.
	WSACleanup();
	#endif
}
