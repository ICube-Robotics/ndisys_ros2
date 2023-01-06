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
#ifndef PORT_HANDLE_INFO_HPP
#define PORT_HANDLE_INFO_HPP

// A Note About Compiler Warning C4251 and Why It's Disabled
// ================================================
// Dynamic libraries that expose classes containing STL members like std::string may generate warning C4251.
// Exporting a class in a dynamic library requires exporting definitions for its types. The the compiler isn't
// smart enough to automatically export the implementation of STL types like std::string.
// STL classes can be complicated --> std::string is actually a template: typedef std::basic_string<char> string;
// A link error could occur if importing client-side code tried to access this member since it has no definition.
// It isn't clear why the compiler still complains even when the member is private and not inlined.
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

#include <stdint.h> // for uint8_t etc...

/**
 * @brief This class stores information about a tool loaded in a specific port.
 */
class CAPICOMMON_API PortHandleInfo
{
public:
	//! Constructs a PortHandleInfo with only the most basic information
	PortHandleInfo(std::string portHandle, uint8_t status = 0x00);

	//! Constructs a PortHandleInfo with all relevant information
	PortHandleInfo(std::string portHandle, std::string toolType,
		           std::string toolId, std::string revision,
				   std::string serialNumber, uint8_t status);
	virtual ~PortHandleInfo(){};

	//! Returns the port handle as a string
	std::string getPortHandle() const;

	//! Returns the tool's manufacturer ID
	std::string getToolId() const;

	//! Returns the tool's revision number
	std::string getRevision() const;

	//! Returns the serial number
	std::string getSerialNumber() const;

	//! Return the status as a string of concatenated flags. Eg: PortInitialized|PortEnabled
	std::string getStatus() const;

	//! Return the string representation of this data
	std::string toString() const;

private:
	enum portStatusFlags { ToolInPort = 0x01,
						   Switch1Closed = 0x02,
						   Switch2Closed = 0x04,
						   Switch3Closed = 0x08,
						   PortInitialized = 0x10,
						   PortEnabled = 0x20,
						   Reserved = 0x40,
						   CurrentSensed = 0x80 };

	std::string portHandle_;
	std::string toolType_;
	std::string toolId_;
	std::string revision_;
	std::string serialNumber_;
	uint8_t status_;
};

#endif // PORT_HANDLE_INFO_HPP