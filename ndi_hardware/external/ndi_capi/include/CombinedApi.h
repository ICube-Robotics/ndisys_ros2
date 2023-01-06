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
#ifndef COMBINED_API_HPP
#define COMBINED_API_HPP

// Expose classes/methods as public in the library by using the tag 'CAPICOMMON_API'
#ifdef _WIN32
	#ifdef CAPICOMMON_EXPORTS
		#define CAPICOMMON_API __declspec(dllexport)
	#else
		#define CAPICOMMON_API __declspec(dllimport)
	#endif
#else
	#define CAPICOMMON_API __attribute__ ((visibility ("default")))
#endif

#include <string>
#include <vector>

#include <stdint.h> // for uint8_t etc...

#include "PortHandleInfo.h"
#include "ToolData.h"

// Forward declarations
class Connection;
class SystemCRC;

// TODO: If using C++11, replace these classic enums with enum classes ;)
namespace PortHandleSearchRequestOption
{
	//! Used by portHandleSearch() to filter the returned data.
	enum value {All = 0, PortsToFree = 1, NotInit= 2, NotEnabled = 3, Enabled = 4};
}

namespace ToolTrackingPriority
{
	//! Used by portHandleEnable() to describe the type of tool.
	enum value {Static = (int)'S', Dynamic = (int)'D', ButtonBox = (int)'B' };
}

namespace TrackingReplyOption
{
	//! The reply options used by BX and TX commands
	enum value {TransformData = 0x0001, ToolAndMarkerData = 0x0002, SingleStray3D = 0x0004, Tool3Ds = 0x0008, AllTransforms = 0x0800, PassiveStrays = 0x1000 };
}

namespace CommBaudRateEnum
{
	//! The enum used by the COMM command to set the baud rate
	enum value { Baud9600 = 0, Baud14400 = 1, Baud19200 = 2, Baud38400 = 3, Baud57600 = 4, Baud115200 = 5, Baud921600 = 6, Baud1228739 = 7 };

	//! Converts the baud rate enum value to its integer equivalent
	int toInt(CommBaudRateEnum::value baudEnumValue);
}

/**
 * @brief This class encapsulates communication with NDI devices using the Combined API (CAPI).
 * @details This class encapsulates binary and string parsing required to send/receive commands.
 *          This class does not provide an exhaustive implementation of every API call, it does
 *          not implement every option available for some commmands. It is intended as a
 *          supplement to the API guide, with working sample code that compiles and runs cross
 *          platform (Windows/Mac/Linux). Performance concerns were secondary to readability
 *          and portability. Complexities like threading were intentionally avoided.
 *          We hope you will find this sample useful. Happy coding!
 */
class CAPICOMMON_API CombinedApi
{
public:
	/**
	 * @brief Creates a new instance of the CAPIcommand object.
	 */
	CombinedApi();

	/**
	 * @brief Disconnect and cleanup.
	 */
	virtual ~CombinedApi();

	/**
	 * @brief Gets the version number of the Combined API sample
	 * @returns the version number of this Combined API sample
	 */
	std::string getVersion()
	{
		return "1.1.0";
	}

	/**
	 * @brief Connects to an ethernet device with the given hostname:port.
	 * @param hostname The hostname, IP address, or COM# of the device. Eg. "P9-B0103.local", "169.254.8.50", or "COM10"
	 * @returns This method returns zero for success, or an error code.
	 */
	int connect(std::string hostname);

	/**
	 * @brief Sets serial port settings on the device.
	 * @param baudRate Specifies the data transmission rate
	 * @param dataBits Specifies the size of a byte: { 0 = 8 bits, 1 = 7 bits}
	 * @param parity Specifies parity: { 0 = None, 1 = Odd, 2 = Even}
	 * @param stopBits Specifies the number of stop bits: { 0 = 1 bit, 1 = 2 bits}
	 * @param enableHandshake Enables or disables hardware handshaking: { 0 = Off, 1 = On}
	 * @returns Zero for success, or the error code associated with the command.
	 */
	int setCommParams(CommBaudRateEnum::value baudRate, int dataBits = 0, int parity = 0, int stopBits = 0, int enableHandshake = 1) const;

	/**
	 * @brief Prints firmware version information using APIREV.
	 * @returns The result of the APIREV command.
	 */
	std::string getApiRevision() const;

	/**
	 * @brief Gets a user parameter using the GET command.
	 * @param paramName The name of the user parameter.
	 * @returns The result of the GET command in the format "[paramName]=[value]" or an ERROR message.
	 */
	std::string getUserParameter(std::string paramName) const;

	/**
	 * @brief Sets a user parameter using the SET command.
	 * @param paramName The name of the user parameter.
	 * @param value The value to set.
	 * @returns Zero for success, or the error code associated with the command.
	 */
	int setUserParameter(std::string paramName, std::string value) const;

	/**
	 * @brief Initializes the system with INIT.
	 * @returns Zero for success, or the error code associated with the command.
	 */
	int initialize() const;

	/**
	 * @brief Search for port handles using PHSR.
	 * @param option The PortHandleSearchRequestOption that defines what data is returned.
	 * @returns A list of PortHandleInfo objects, or an empty vector if an error occurred.
	 */
	std::vector<PortHandleInfo> portHandleSearchRequest(PortHandleSearchRequestOption::value option = PortHandleSearchRequestOption::All) const;

	/**
	 * @brief Frees the specified port handle using PHF.
	 * @param portHandle The port handle to free.
	 * @returns Zero for success, -1 if portHandle is invalid, or the error code associated with the command.
	 */
	int portHandleFree(std::string portHandle) const;

	/**
	 * @brief Requests a port handle using PHRQ.
	 * @param hardwareDevice Eight character name. Use wildcards "********" or a device name returned by PHINF
	 * @param systemType One character defining the system type. Use a wildcard "*"
	 * @param toolType One character defining the tool type. Wired="0" or Wireless="1" (passive or active-wireless)
	 * @param portNumber Two characters defining the port number: [01-03] for wired tools, 00 or ** for wireless tools.
	 * @param dummyTool If you will use PVWR to load a tool definition file, use wildcards: "**"
	 *				    If ToolType=Wired, either "01" or "02" adds the active wired dummy tool.
	 *                  If ToolType=Wireless, "01" adds the passive dummy tool, "02" adds the active wireless dummy tool.
	 * @returns The requested port handle as a positive integer, or a negative integer error code.
	 */
	int portHandleRequest(std::string hardwareDevice = "********",
								  std::string systemType = "*",
								  std::string toolType = "1",
								  std::string portNumber = "00",
								  std::string dummyTool = "**" ) const;

	/**
	 * @brief Initializes a port handle using PINIT.
	 * @param portHandle The port handle to initialize.
	 * @returns Zero for success, -1 if portHandle is invalid, or the error code associated with the command.
	 */
	int portHandleInitialize(std::string portHandle) const;

	/**
	 * @brief Enables a port handle using PENA.
	 * @param portHandle The portHandle to enable.
	 * @param priority An enum value describing the type of tool.
	 * @returns Zero for success, -1 if portHandle is invalid, or the error code associated with the command.
	 */
	int portHandleEnable(std::string portHandle, ToolTrackingPriority::value priority = ToolTrackingPriority::Dynamic) const;

	/**
	 * @brief Retreieves tool information using PHINF.
	 * @param The port handle to get information about
	 * @returns A PortHandleInfo object with the information about the tool loaded in the given port.
	 *          This method returns and empty PortHandleInfo objevct if the given portHandle is invalid,
	 *          there is no tool loaded at the port, or an error occurs.
	 */
	PortHandleInfo portHandleInfo(std::string portHandle) const;

	/*
	* @brief Loads a dummy passive tool used to track stray 3Ds.
	* @details TSTART will fail if a dummy tool and regular tools of the same type are loaded and enabled.
	* @returns The requested port handle as a positive integer, or a negative integer error code.
	*/
	int loadPassiveDummyTool() const;

	/*
	* @brief Loads a dummy active wireless tool used to track stray 3Ds.
	* @details TSTART will fail if a dummy tool and regular tools of the same type are loaded and enabled.
	* @returns The requested port handle as a positive integer, or a negative integer error code.
	*/
	int loadActiveWirelessDummyTool() const;

	/*
	* @brief Loads a dummy active tool used to track stray 3Ds.
	* @details TSTART will fail if a dummy tool and regular tools of the same type are loaded and enabled.
	* @returns The requested port handle as a positive integer, or a negative integer error code.
	*/
	int loadActiveDummyTool() const;

	/**
	 * @brief Loads a tool definition file (.rom) to a port using PVWR.
	 * @param romFilePath The path to the .rom file to load.
	 * @param portHandle The port handle (2 hex chars) that was previously requested.
	 */
	void loadSromToPort(std::string romFilePath, int portHandle) const;

	/**
	 * @brief Enters tracking mode using TSTART.
	 * @returns Zero for success, or the error code associated with the command.
	 */
	int startTracking() const;

	/**
	 * @brief Exits tracking mode using TSTOP.
	 * @returns Zero for success, or the error code associated with the command.
	 */
	int stopTracking() const;

	/**
	 * @brief Retrieves tracking data as ASCII text using TX.
	 * @param options An integer concatenated from TrackingReplyOption flags described in the API guide
	 * @returns The result of the TX command: a list of handles and tracking data or an ERROR message.
	 */
	std::string getTrackingDataTX(const uint16_t options = TrackingReplyOption::TransformData | TrackingReplyOption::AllTransforms) const;

	/**
	 * @brief Retrieves binary tracking data using BX and returns its string representation.
	 * @param options An integer concatenated from TrackingReplyOption flags described in the API guide
	 * @returns ToolData for all enabled tools, or an empty vector if an error occurred.
	 */
	std::vector<ToolData> getTrackingDataBX(const uint16_t options = TrackingReplyOption::TransformData | TrackingReplyOption::AllTransforms) const;

	/**
	 * @brief Retrieves binary tracking data using BX2 and returns its string representation.
	 * @param options A string containing the BX2 options described in the Vega API guide
	 * @returns ToolData for all enabled tools that have new data since the last BX2, or an empty vector if an error occurred.
	 */
	std::vector<ToolData> getTrackingDataBX2(std::string options = "--6d=tools --3d=all --sensor=all --1d=buttons") const;

	/**
	 * @brief  Converts the input string to an integer
	 * @param input A string containing a hexadecimal number to convert to its integer equivalent.
	 * @returns The integer representation of the input.
	 */
	int stringToInt(std::string input) const;

	//! Returns the human readable string corresponding to the given error or warning code.
	static std::string errorToString(int errorCode);

private:
	/**
	 * @brief This method is used to lookup a human readable string when the device returns "ERROR[errorCode]"
	 * @returns The message associated with the error code, or "Error code not found." if the code is unrecognized.
	 */
	static std::string getErrorString(int errorCode);

	/**
	 * @brief This method is used to lookup a human readable string when the device returns "WARNING[warnCode]"
	 * @returns The message associated with the warning code, or "Warning code not found." if the code is unrecognized.
	 */
	static std::string getWarningString(int warnCode);

	/**
	 * @brief Sends a command to the device.
	 * @param command A std::string containing the ASCII command to send.
	 * @returns The number of charaters written, or -1 if an error occurred.
	 */
	int sendCommand(std::string command) const;

	/**
	 * @brief Returns the error code of the response as a negative integer.
	 */
	int getErrorCodeFromResponse(std::string response) const;

	/**
	 * @brief Reads the response from the device, and verifies the CRC.
	 * @returns The response as a std::string with trailing CR + CRC16 removed.
	 */
	std::string readResponse() const;

	/**
	 * @brief Converts the input integer to a string in decimal
	 * @param input The integer to convert
	 * @param width The width of the string, padded with zeros
	 * @returns The decimal representation of the integer as a std::string
	 */
	std::string intToString(int input, int width = 1) const;

	/**
	 * @brief Converts the input integer to a string in hexadecimal
	 * @param input The integer to convert
	 * @param width The width of the string, padded with zeros
	 * @returns The hexadecimal representation of the integer as a std::string
	 */
	std::string intToHexString(int input, int width = 1) const;

	/** Private Members */

	//! This connection can be either a TcpConnection (to a Vega) or a ComConnection (to NDI serial port devices)
	Connection* connection_;

	//! This member validates CRC16s sent along with the data
	SystemCRC* crcValidator_;

	//! The carriage return character is important for terminating ASCII replies
	static const char CR = '\r';

	//! Indicates the start of a BX or BX2 reply
	static const uint16_t START_SEQUENCE = 0xA5C4;

	//! Indicates the start of an extended reply (eg. video capture)
	static const uint16_t START_SEQUENCE_VCAP = 0xA5C8;

	//! Indicates the start of a streaming reply
	static const uint16_t START_SEQUENCE_STREAMING = 0xB5D4;

	//! To avoid confusing error code 01 with warning 01, use this offset: 1001 is a warning, and 0001 is an error.
	static const int WARNING_CODE_OFFSET = 1000;
};

//! These strings translate the warning code (the array index) to a descriptive message
static const char* warningStrings[] =
{
	"OKAY", // 0x0 not a warning
	"Possible hardware fault",
	"The tool violates unique geometry constraints",
	"The tool is incompatible with other loaded tools",
	"The tool is incompatible with other loaded tools and violate design contraints",
	"The tool does not specify a marker wavelength. The system will use the default wavelength."
};

//! These strings translate the error code (the array index) to a descriptive message
static const char* errorStrings[] =
{
	"OKAY", // 0x00 not an error
	"Invalid command.",
	"Command too long.",
	"Command too short.",
	"Invalid CRC calculated for command.",
	"Command timed out.",
	"Bad COMM settings.",
	"Incorrect number of parameters.",
	"Invalid port handle selected.",
	"Invalid priority.",
	"Invalid LED.",
	"Invalid LED state.",
	"Command is invalid while in the current mode.",
	"No tool is assigned to the selected port handle.",
	"Selected port handle not initialized.",
	"Selected port handle not enabled.",
	"System not initialized.", // 0x10
	"Unable to stop tracking.",
	"Unable to start tracking.",
	"Tool or SROM fault. Unable to initialize.",
	"Invalid Position Sensor characterization parameters.",
	"Unable to initialize the system.",
	"Unable to start Diagnostic mode.",
	"Unable to stop Diagnostic mode.",
	"Reserved",
	"Unable to read device's firmware version information.",
	"Internal system error.",
	"Reserved",
	"Invalid marker activation signature.",
	"Reserved",
	"Unable to read SROM device.",
	"Unable to write to SROM device.",
	"Reserved", // 0x20
	"Error performing current test on specified tool.",
	"Marker wavelength not supported.",
	"Command parameter is out of range.",
	"Unable to select volume.",
	"Unable to determine the system's supported features list.",
	"Reserved",
	"Reserved",
	"Too many tools are enabled.",
	"Reserved",
	"No memory is available for dynamic allocation.",
	"The requested port handle has not been allocated.",
	"The requested port handle is unoccupied.",
	"No more port handles available.",
	"Incompatible firmware versions.",
	"Invalid port description.",
	"Requested port is already assigned a port handle.", // 0x30
	"Reserved",
	"Invalid operation on the requested port handle.",
	"Feature unavailable.",
	"Parameter does not exist.",
	"Invalid value type.",
	"Parameter value is out of range.",
	"Parameter index out of range.",
	"Invalid parameter size.",
	"Permission denied.",
	"Reserved",
	"File not found.",
	"Error writing to file.",
	"Error removing file.",
	"Reserved",
	"Reserved",
	"Invalid or corrupted tool definition", // 0x40
	"Tool exceeds maximum markers, faces, or groups",
	"Required device not connected",
	"Reserved"
};

#endif // COMBINED_API_HPP
