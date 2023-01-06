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

#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

#include "BufferedReader.h"
#include "CombinedApi.h"
#include "ComConnection.h"
#include "GbfContainer.h"
#include "GbfFrame.h"
#include "SystemCRC.h"
#include "TcpConnection.h"

CombinedApi::CombinedApi()
{
	connection_ = NULL;
	crcValidator_ = new SystemCRC();
}

CombinedApi::~CombinedApi()
{
	delete connection_;
	delete crcValidator_;
}

int CombinedApi::connect(std::string hostname)
{
	std::cout << "Connecting to " << hostname << " ..." << std::endl;

	// Delete any old connection
	if (connection_ != NULL)
	{
		delete connection_;
	}

	// Determine if the device uses serial or ethernet communication
	int errorCode = 0;
	if (hostname.substr(0,3).compare("COM") == 0 || hostname.substr(0,4).compare("/dev") == 0)
	{
		// Create a new ComConnection
		connection_ = new ComConnection(hostname);

		// Once the connection is open, the host and device need to agree on a baud rate
		if (connection_->isConnected())
		{
			// A serial break is sent to reset the device to the default 9600 baud rate
			static_cast<ComConnection*>(connection_)->sendSerialBreak();

			// Wait for the system to reply RESET
			errorCode = getErrorCodeFromResponse(readResponse());

			// Print the firmware version for debugging purposes
			std::cout << "API Revision: " << getApiRevision() << std::endl;

			// The host can now request the device go to a much faster baud rate...
			if (errorCode == 0)
			{
				std::cout << "Setting Baud921600 for compatibility. Check your API guide to see if this is optimal for your NDI device." << std::endl;
				errorCode = setCommParams(CommBaudRateEnum::Baud921600);
			}
		}
	}
	else
	{
		// Create a new TcpConnection
		connection_ = new TcpConnection(hostname.c_str());
		errorCode = connection_->isConnected() ? 0 : -1;
	}

	return errorCode;
}

int CombinedApi::setCommParams(CommBaudRateEnum::value baudRate, int dataBits, int parity, int stopBits, int enableHandshake) const
{
	// Send the COMM command
	std::string command =  std::string("COMM ").append(intToString(baudRate)).append(intToString(dataBits)).append(intToString(parity))
											   .append(intToString(stopBits)).append(intToString(enableHandshake));
	sendCommand(command);

	// Wait for the system to reply OKAY
	int errorCode = getErrorCodeFromResponse(readResponse());

	// If the device agreed to change the baud rate as desired, set the baud rate on the host to match it
	if (errorCode == 0)
	{
		// std::cout << "Debug: configuring host baud rate..." << std::endl;
		dynamic_cast<ComConnection*>(connection_)->setSerialPortParams(CommBaudRateEnum::toInt(baudRate), (dataBits > 0 ? 7 : 8), parity, stopBits, enableHandshake);
	}
	return errorCode;
}

std::string CombinedApi::getApiRevision() const
{
	// Send the APIREV command
	std::string command =  std::string("APIREV ");
	sendCommand(command);

	// Return the raw string so the client can parse the version
	return readResponse();
}

int CombinedApi::initialize() const
{
	// Send the INIT command
	std::string command =  std::string("INIT ");
	sendCommand(command);
	return getErrorCodeFromResponse(readResponse());
}

std::vector<PortHandleInfo> CombinedApi::portHandleSearchRequest(PortHandleSearchRequestOption::value option) const
{
	// Send the PHSR command
	std::stringstream stream;
	stream << "PHSR " << std::setw(2) << std::setfill('0') << (int) option;
	std::string command = stream.str();
	sendCommand(command);

	// If an error occurred, return an empty vector
	std::string response = readResponse();
	int errorCode = getErrorCodeFromResponse(response);
	std::vector<PortHandleInfo> portHandleInfoVector;
	if (errorCode != 0)
	{
		std::cout << response << " - " << errorToString(errorCode);
		return portHandleInfoVector;
	}

	// Return the response as a std::vector of easy to use objects
	int numPortHandles = stringToInt(response.substr(0,2));
	for (int i = 0; i < numPortHandles; i ++)
	{
		portHandleInfoVector.push_back(PortHandleInfo(response.substr(i * 5 + 2, 2), (uint8_t) stringToInt(response.substr(i * 5 + 4, 3))));
	}

	return portHandleInfoVector;
}

int CombinedApi::portHandleFree(std::string portHandle) const
{
	// If the port handle is invalid, print an error message and return
	if (portHandle.size() != 2)
	{
		return -1;
	}

	// Send the PHF command
	std::string command =  std::string("PHF ").append(portHandle);
	sendCommand(command);
	return getErrorCodeFromResponse(readResponse());
}

int CombinedApi::portHandleRequest(std::string hardwareDevice, std::string systemType, std::string toolType, std::string portNumber, std::string dummyTool) const
{
	// Send the PHRQ command
	std::string command =  std::string("PHRQ ").append(hardwareDevice).append(systemType).append(toolType).append(portNumber).append(dummyTool);
	sendCommand(command);

	// Return the requested port handle or an error code
	std::string response = readResponse();
	int errorCode = getErrorCodeFromResponse(response);
	if (errorCode == 0)
	{
		return stringToInt(response);
	}
	else
	{
		return errorCode;
	}
}

int CombinedApi::portHandleInitialize(std::string portHandle) const
{
	// If the port handle is invalid, print an error message and return
	if (portHandle.size() != 2)
	{
		return -1;
	}

	// Send the PINIT command
	std::string command =  std::string("PINIT ").append(portHandle);
	sendCommand(command);
	return getErrorCodeFromResponse(readResponse());
}

int CombinedApi::portHandleEnable(std::string portHandle, ToolTrackingPriority::value priority) const
{
	// If the port handle is invalid, print an error message and return
	if (portHandle.size() != 2)
	{
		return -1;
	}

	// Send the PENA command
	std::string command =  std::string("PENA ").append(portHandle);
	command += (char)priority;
	sendCommand(command);
	return getErrorCodeFromResponse(readResponse());
}

PortHandleInfo CombinedApi::portHandleInfo(std::string portHandle) const
{
	// If the port handle is invalid, print an error message and return
	if (portHandle.size() != 2)
	{
		std::cout << "Invalid port handle: " << portHandle << std::endl;
		return PortHandleInfo(portHandle);
	}

	// Send the PHINF command
	std::string command =  std::string("PHINF ").append(portHandle);
	sendCommand(command);

	// If there is no tool loaded, return an empty PortHandleInfo
	std::string response = readResponse();
	int errorCode = getErrorCodeFromResponse(response);
	if (errorCode != 0)
	{
		std::cout << response << " - " << errorToString(errorCode);
		return PortHandleInfo(portHandle);
	}
	else if (response.substr(0,10).compare("UNOCCUPIED") == 0)
	{
		std::cout << "No tool loaded at port: " << portHandle << std::endl;
		return PortHandleInfo(portHandle);
	}

	// Parse the information from the response
	std::string toolType = response.substr(0,8);
	std::string toolId = response.substr(8,12);
	toolId.erase(toolId.find_last_not_of(" \n\r\t") + 1); // .trim() whitespace
	std::string revision = response.substr(20,3);
	std::string serialNumber = response.substr(23,8);
	uint8_t status = (uint8_t) stringToInt(response.substr(31,2));
	return PortHandleInfo(portHandle, toolType, toolId, revision, serialNumber, status);
}

int CombinedApi::loadPassiveDummyTool() const
{
	return portHandleRequest("********", "*", "1", "00", "01");
}

int CombinedApi::loadActiveWirelessDummyTool() const
{
	return portHandleRequest("********", "*", "1", "00", "02");
}

int CombinedApi::loadActiveDummyTool() const
{
	return portHandleRequest("********", "*", "0", "00", "01");
}

void CombinedApi::loadSromToPort(std::string romFilePath, int portHandle) const
{
	// If the port handle is invalid, print an error message and return
	if (portHandle < 0)
	{
		std::cout << "Invalid port handle: " << portHandle << std::endl;
		return;
	}

	// If the .rom file cannot be opened, print an error message and return
	std::ifstream inputFileStream(romFilePath.c_str(), std::ios_base::binary);
	if (!inputFileStream.is_open())
	{
		std::cout << "Cannot open file: " + romFilePath << std::endl;
		return;
	}

	// Read the entire file and convert it to ASCII hex characters
	std::stringstream romStream;
	romStream << std::setfill('0') << std::hex;
	while (!inputFileStream.eof())
	{
		romStream << std::setw(2)  << inputFileStream.get();
	}
	inputFileStream.close();

	// Tool data is sent in chunks of 128 hex characters (64-bytes).
	// It must be an integer number of chunks, padded with zeroes at the end.
	const int messageSizeChars = 128;
	const int messageSizeBytes = 64;
	std::string toolDefinition = romStream.str();
	int remainder = toolDefinition.size() % messageSizeChars;
	toolDefinition.append((messageSizeChars - remainder), '0');
	const int totalIterations =  (int)toolDefinition.size() / messageSizeChars;
	std::string command = "";
	int errorCode = 0;
	std::stringstream startAddressStream;
	startAddressStream << std::setfill('0') << std::hex;
	for (int i = 0; i < totalIterations; i++)
	{
		// Pass the startAddress as a fixed width four characters of hex padded with zeroes
		startAddressStream << std::setw(4) <<  i * messageSizeBytes;

		// Send the PVWR command
		command =  std::string("PVWR ").append(intToString(portHandle, 2));
		command += startAddressStream.str();
		command += toolDefinition.substr(i * messageSizeChars, messageSizeChars);
		sendCommand(command);

		// If we run into an error, print something before exiting
		errorCode = getErrorCodeFromResponse(readResponse());
		if (errorCode != 0)
		{
			std::cout << "PVWR returned error: " << errorToString(errorCode) << std::endl;
			return;
		}

		// Reset the stringstream used to print fixed width address sizes
		startAddressStream.str("");
	}
}

int CombinedApi::startTracking() const
{
	// Send the TSTART command
	std::string command =  std::string("TSTART ");
	sendCommand(command);
	return getErrorCodeFromResponse(readResponse());
}

int CombinedApi::stopTracking() const
{
	// Send the TSTOP command
	std::string command =  std::string("TSTOP ");
	sendCommand(command);
	return getErrorCodeFromResponse(readResponse());
}

std::string CombinedApi::getTrackingDataTX(const uint16_t options) const
{
	// Send the TX command
	std::string command =  std::string("TX ").append(intToHexString(options, 4));
	sendCommand(command);
	return readResponse();
}

std::vector<ToolData> CombinedApi::getTrackingDataBX(const uint16_t options) const
{
	// Send the BX command
	std::string command =  std::string("BX ").append(intToHexString(options, 4));
	sendCommand(command);

	// Open a buffered reader on the connection to easily parse the binary reply
	BufferedReader reader(connection_);
	reader.readBytes(6);
	uint16_t startSequence = reader.get_uint16();
	uint16_t replyLengthBytes = reader.get_uint16();

	// Verify the CRC16 of the header
	unsigned int headerCRC16 = (unsigned int) reader.get_uint16();
	unsigned int calculatedCRC16 = crcValidator_->calculateCRC16(reader.getData(0, 4).c_str(), 4);
	if (calculatedCRC16 != headerCRC16)
	{
		std::cout << "CRC16 failed!" << std::endl;
		return std::vector<ToolData>();
	}

	// In the case of an unexpected binary header, return an empty vector
	if (startSequence != START_SEQUENCE)
	{
		std::cout << "Unrecognized start sequence: " << startSequence << std::endl;
		return std::vector<ToolData>();
	}

	// Get all of the data once we know how many bytes to read: replyLengthBytes + 2 bytes for trailing CRC16
	reader.readBytes(replyLengthBytes + 2);

	// Verify the CRC16 of the data
	reader.skipBytes(replyLengthBytes);
	unsigned int dataCRC16 = reader.get_uint16();
	if (crcValidator_->calculateCRC16(reader.getData(6, replyLengthBytes).c_str(), replyLengthBytes) != dataCRC16)
	{
		std::cout << "CRC16 failed!" << std::endl;
		return std::vector<ToolData>();
	}
	reader.skipBytes(-replyLengthBytes -2); // move the BufferedReader's pointer back so we can parse the data

	// Debugging: print the raw binary and/or interpreted strings
	/*std::cout << reader.toString() << std::endl;
	std::cout << "startSequence=" << intToHexString(startSequence, 4) << std::endl
		   << "replyLengthBytes=" << intToHexString(replyLengthBytes, 4) << std::endl
		   << "headerCRC16=" << intToHexString(headerCRC16, 4) << std::endl;*/

	// TODO: support all BX options. Just return if there are unexpected options, we will be binary misaligned anyway.
	if ((options & ~(TrackingReplyOption::TransformData | TrackingReplyOption::AllTransforms)) != 0x0000)
	{
		std::cout << "Reply parsing has not implemented options: " << intToHexString(options, 4) << std::endl;
		return std::vector<ToolData>();
	}

	std::vector<ToolData> toolDataVector;
	uint8_t numHandles = reader.get_byte();
	for (uint8_t i = 0; i < numHandles; i++)
	{
		// Create a new ToolData
		toolDataVector.push_back(ToolData());

		// From each two byte handle, extract the handle index and status
		toolDataVector.back().transform.toolHandle = (uint16_t) reader.get_byte();
		uint8_t handleStatus = reader.get_byte();

		// Parse BX 0001 - See API guide for protocol details
		if (options & TrackingReplyOption::TransformData)
		{
			// The transform is not transmitted at all if it is missing
			switch (handleStatus)
			{
				case 0x01: // Valid
					toolDataVector.back().transform.status = TransformStatus::Enabled;
					toolDataVector.back().transform.q0 = reader.get_double();
					toolDataVector.back().transform.qx = reader.get_double();
					toolDataVector.back().transform.qy = reader.get_double();
					toolDataVector.back().transform.qz = reader.get_double();
					toolDataVector.back().transform.tx = reader.get_double();
					toolDataVector.back().transform.ty = reader.get_double();
					toolDataVector.back().transform.tz = reader.get_double();
					toolDataVector.back().transform.error = reader.get_double();
				break;
				case 0x04: // Disabled
					// Disabled markers have no transform, status, or frame number
					toolDataVector.pop_back(); // don't return a ToolData for it, there's nothing there
					continue;
				default:
					// case 0x02: Missing or anything unexpected
					// do nothing --> blank Transform object is already initialized as missing
				break;
			};

			// Regardless of transform status, there is info about the port and frame
			toolDataVector.back().portStatus = reader.get_uint32() & 0x0000FFFF;
			toolDataVector.back().frameNumber = reader.get_uint32();
		}
	}

	// Add the systemStatus to each ToolData
	uint16_t systemStatus = reader.get_uint16();
	for (int t = 0; t < toolDataVector.size(); t++)
	{
		toolDataVector[t].systemStatus = systemStatus;
	}

	// Return the tool data
	return toolDataVector;
}

std::vector<ToolData> CombinedApi::getTrackingDataBX2(std::string options) const
{
	// Send the BX2 command
	std::string command =  std::string("BX2 ").append(options);
	sendCommand(command);

	// Open a buffered reader on the connection to easily parse the binary reply
	BufferedReader reader(connection_);

	// The BX2 reply begins with a 6 byte header:
	// (2-bytes) StartSequence: indicates how to parse the reply. A5C4 (normal)
	// (2-bytes) ReplyLength: length of the reply in bytes
	// (2-bytes) CRC16
	reader.readBytes(6);
	uint16_t startSequence = reader.get_uint16();
	uint16_t replyLengthBytes = reader.get_uint16();

	// Verify the CRC16 of the header
	unsigned int headerCRC16 = (unsigned int) reader.get_uint16();
	unsigned int calculatedCRC16 = crcValidator_->calculateCRC16(reader.getData(0, 4).c_str(), 4);
	if (calculatedCRC16 != headerCRC16)
	{
		std::cout << "CRC16 failed!" << std::endl;
		return std::vector<ToolData>();
	}

	// TODO: handle all BX2 reply types?
	if (startSequence != START_SEQUENCE)
	{
		// Return an empty vector if the binary response cannot be interpreted
		std::cout << "Unrecognized BX2 reply header: " << std::setw(4) << startSequence << " - Not implemented yet!" << std::endl;
		return std::vector<ToolData>();
	}

	// Get all of the data once we know how many bytes to read: replyLengthBytes + 2 bytes for trailing CRC16
	reader.readBytes(replyLengthBytes + 2);

	// Verify the CRC16 of the data
	reader.skipBytes(replyLengthBytes);
	unsigned int dataCRC16 = reader.get_uint16();
	if (crcValidator_->calculateCRC16(reader.getData(6, replyLengthBytes).c_str(), replyLengthBytes) != dataCRC16)
	{
		std::cout << "CRC16 failed!" << std::endl;
		return std::vector<ToolData>();
	}
	reader.skipBytes(-replyLengthBytes -2); // move the BufferedReader's pointer back so we can parse the data

	// Parse the binary into meaningful objects
	GbfContainer container(reader);

	// Debugging: print the raw binary and/or interpreted strings
	/*std::cout << reader.toString() << std::endl;
	std::cout << "-----GbfHeader" << std::endl
		      << "startSequence=" << std::setw(4) << static_cast<unsigned>(startSequence) << std::endl
		      << "replyLengthBytes=" << std::setw(4) << static_cast<unsigned>(replyLengthBytes) << std::endl
		      << "headerCRC16=" << std::setw(4) << static_cast<unsigned>(headerCRC) << std::endl
		      << container.toString()
		      << "dataCRC16=" << std::setw(4) << static_cast<unsigned>(reader.get_uint16()) << std::endl << std::endl;*/

	// Search the root GbfContainer to find the frame component
	std::vector<ToolData> retVal;
	for (int i = 0; i < container.components.size(); i++)
	{
		if (container.components[i]->componentType == GbfComponentType::Frame)
		{
			// Every GBF frame has GbfFrameDataItems for each type of tool: Passive, ActiveWireless, Active
			GbfFrame* frame = static_cast<GbfFrame*>(container.components[i]);
			retVal = frame->getToolData();
		}
	}

	// If we didn't find any, then return an empty vector
	return retVal;
}

std::string CombinedApi::intToString(int input, int width) const
{
	std::stringstream convert;
	convert << std::dec << std::setfill('0');
	convert << std::setw(width) << input;
	return convert.str();
}

std::string CombinedApi::intToHexString(int input, int width) const
{
	std::stringstream convert;
	convert << std::hex  << std::setfill('0');
	convert << std::setw(width) << input;
	return convert.str();
}

int CombinedApi::stringToInt(std::string input) const
{
	int retVal = 0;
	std::stringstream convert(input);
	convert << std::hex;
	convert >> retVal;
	return retVal;
}

int CombinedApi::getErrorCodeFromResponse(std::string response) const
{
	// Parse the error code from the response string and return it
	int errorCode = 0;
	if (response.substr(0,5).compare("ERROR") == 0)
	{
		errorCode = stringToInt(response.substr(5,2));
	}
	else if (response.substr(0,7).compare("WARNING") == 0)
	{
		errorCode = stringToInt(response.substr(7,2)) + WARNING_CODE_OFFSET;
	}

	// Use negative error codes to distinguish between port handles an errors.
	return errorCode * -1;
}

std::string CombinedApi::readResponse() const
{
	// Declare an empty string to hold the response
	std::string response = std::string("");
	char lastChar = '\0';

	// Read from the device until we encounter a terminating carriage return (CR)
	while (lastChar != CR)
	{
		connection_->read(&lastChar, 1);
		response += lastChar;
	}

	// Trim trailing CR and verify the CRC16
//	std::cout << "<<" << response;
	response.erase(response.length() - 1); // strip CR (1 char)
	unsigned int replyCRC16 = (unsigned int) stringToInt(response.substr(response.length() - 4 , 4));
	response.erase(response.length() - 4, 4); // strip CRC16 (4 chars)
	if (crcValidator_->calculateCRC16(response.c_str(), (int) response.length()) != replyCRC16)
	{
		std::cout << "CRC16 failed!" << std::endl;
	}

	// Return whatever string the device responded with
	return response;
}

int CombinedApi::sendCommand(std::string command) const
{
	// Display an error message if there is no open socket
	if (!connection_->isConnected())
	{
		std::cout << "Cannot send command: " << command << "- No open socket!" << std::endl;
		return -1;
	}

	// Add CR character to command and write the command to the socket
	command += CR;
	return connection_->write(command.c_str(), (int)command.length());
}

std::string CombinedApi::errorToString(int errorCode)
{
	errorCode *= -1; // restore the errorCode to a positive value
	if (errorCode > WARNING_CODE_OFFSET)
	{
		return getWarningString(errorCode - WARNING_CODE_OFFSET);
	}
	else
	{
		return getErrorString(errorCode);
	}
}

std::string CombinedApi::getWarningString(int warnCode)
{
	if (warnCode < 0 || warnCode >= sizeof(warningStrings))
	{
		return "Warning code not found.";
	}
	else
	{
		return warningStrings[warnCode];
	}
}

std::string CombinedApi::getErrorString(int errorCode)
{
	if (errorCode < 0 || errorCode >= sizeof(errorStrings))
	{
		return "Error code not found.";
	}
	else
	{
		return errorStrings[errorCode];
	}
}

std::string CombinedApi::getUserParameter(std::string paramName) const
{
	// Send the GET request
	std::string command = std::string("GET ").append(paramName);
	sendCommand(command);

	// Return whatever the device responded with
	return readResponse();
}

int CombinedApi::setUserParameter(std::string paramName, std::string value) const
{
	// Send the SET request
	std::string command = std::string("SET ").append(paramName).append("=").append(value);
	sendCommand(command);
	return getErrorCodeFromResponse(readResponse());
}

int CommBaudRateEnum::toInt(CommBaudRateEnum::value baudEnumValue)
{
	switch (baudEnumValue)
	{
		case Baud1228739:
			return 1228739;
		case Baud921600:
			return 921600;
		case Baud115200:
			return 115200;
		case Baud57600:
			return 57600;
		case Baud38400:
			return 38400;
		case Baud19200:
			return 19200;
		case Baud14400:
			return 14400;
		case Baud9600: // fall through
		default:
			return 9600;
	};
}
