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
#ifndef TOOL_DATA_HPP
#define TOOL_DATA_HPP

// A Note About Compiler Warning C4251 and Why It's Disabled
// ================================================
// Dynamic libraries that expose classes containing STL members like std::string may generate warning C4251.
// Exporting a class in a dynamic library requires exporting definitions for its types. The the compiler isn't
// smart enough to automatically export the implementation of STL types like std::string.
// STL classes can be complicated --> std::string is actually a template: typedef std::basic_string<char> string;
// A link error could occur if importing client-side code tried to access this member since it has no definition.
// Worse, the client code may be compiled with a STL definition different from the STL used to compile the dynamic
// library, leading to undefined behaviour. There are various solutions to this problem:
// 1. Use the PIMPL design pattern --> public d-> private implementation effectively doubles the amount of code
// 2. Heap allocate the STL types --> requires writing copying constructors for every object that does this
// 3. Disable the warning and keep STL members private --> shouldn't be a problem, recompile source if you need to
// To keep CAPIcommon code simple, option #3 is used.
#ifdef _WIN32
#pragma warning( disable: 4251 )
#endif

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

#include "MarkerData.h"
#include "Transform.h"
#include "SystemAlert.h"

/**
 * @brief This class stores tracking information related to a single tool.
 * @details Depending on how the data was gathered, some information may not be available.
 *          For example, BX returns no button information.
 */
class CAPICOMMON_API ToolData
{
public:
	//! Constructor that creates an empty ToolData
	ToolData();
	virtual ~ToolData(){};

	//! The precision used to print the transform and marker positions
	static const int PRECISION = 6;

	// Common tool tracking information //

	//! The frame number that identifies when the data was collected
	uint32_t frameNumber;

	//! The transform containing tracking information about the tool
	Transform transform;

	// Used with TX and BX //

	//! The status of the measurement device itself
	uint16_t systemStatus;

	//! The status of the tool
	uint32_t portStatus;

	// Used with BX2 //

	//! Indicates what type of frame gathered the data. See the related enum to interpret this value
	uint8_t frameType;

	//! Each frame is given an sequence number, which clients can usually ignore
	uint8_t frameSequenceIndex;

	//! Same as TransformStatus, but only codes that apply to the frame as a whole
	uint16_t frameStatus;

	//! The "seconds" part of the timestamp
	uint32_t timespec_s;

	//! The "nanoseconds" part of the timestamp
	uint32_t timespec_ns;

	//! A list of marker 3Ds, if this was requested in the BX2 options
	std::vector<MarkerData> markers;

	//! Button data assoicated with the frame
	std::vector<uint8_t> buttons;

	//! System alerts that were active during the frame
	std::vector<SystemAlert> systemAlerts;

	//! This flag indicates the data is new (useful for printing with BX2)
	bool dataIsNew;

	//! This member is useful for holding static tool information during CSV output
	std::string toolInfo;
};

namespace SystemStatus
{
	enum CAPICOMMON_API value { CommSyncError = 0x0000,
								// Reserved 0x0001, 0x0002
								ProcessingException = 0x0004,
								// Reserved 0x0008, 0x0010
								PortOccupied = 0x0020,
								PortUnoccupied = 0x0040,
								DiagnosticPending = 0x0080,
								TemperatureOutOfRange = 0x0100 };

	//! Return the systemStatus as a string of concatenated flags. Eg: DiagnosticPending|TemperatureOutOfRange
	CAPICOMMON_API std::string toString(uint16_t systemStatus);
}

namespace FrameType
{
	enum CAPICOMMON_API value { Dummy = 0x00,
								ActiveWireless = 0x01,
								Passive = 0x02,
								Active = 0x03,
								Laser = 0x04,
								Illuminated = 0x05,
								Background = 0x06,
								Magnetic = 0x07 };

	//! Converts the given error code to its std::string representation
	CAPICOMMON_API std::string toString(uint8_t frameType);
}

namespace ButtonState
{
	enum CAPICOMMON_API value {Open = 0x00, Closed = 0x01};

	//! Converts the enum to its std::string representation
	CAPICOMMON_API std::string toString(uint8_t state);
}

#endif // TOOL_DATA_HPP