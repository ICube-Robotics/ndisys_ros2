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
#ifndef SYSTEM_ALERT_HPP
#define SYSTEM_ALERT_HPP

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

#include <stdint.h> // for uint8_t etc...
#include <string>

/**
 * @brief This class represents a system condition that may impact tracking performance.
 */
class CAPICOMMON_API SystemAlert
{
public:
	//! Returns the std::string representation of the system alert
	std::string toString() const;

	//! The type of SystemAlert. See the related enum for its interpretation
	uint8_t conditionType;

	//! The code which should be interpreted using the corresponding enum based on its type.
	uint16_t conditionCode;
};

namespace SystemAlertType
{
	/**
	 * @brief  Defines the type of SystemAlert.
	 * @details  A fault indicates the system is unable to function properly, and likely requires repair.
	 *           An alert is a condition that may impact measurement performance, but can be resolved without repair.
	 *           An event is a normal occurence that may impact system behaviour or performance.
	 */
	enum value { Fault = 0x00,
		         Alert = 0x01,
				 Event = 0x02 };
}

namespace SystemFaultEnum
{
	enum value { Ok = 0x0000,
		         FatalParameter = 0x0001,
				 SensorParameter = 0x0002,
				 MainVoltage = 0x0003,
				 SensorVoltage = 0x0004,
				 IlluminatorCurrent = 0x0005,
				 IlluminatorVoltage = 0x0006,
				 Sensor0Temp = 0x0007, // left temperature sensor
				 Sensor1Temp = 0x0008, // right temperature sensor
				 MainTemp = 0x0009,
				 SensorMalfunction = 0x000a };

	//! Returns the std::string representation of the fault
	std::string toString(uint16_t conditionCode);
}

namespace SystemAlertEnum
{
	enum value { Ok = 0x0000,
		         BatteryLow = 0x0001,
				 BumpDetected = 0x0002,
				 IncompatibleFirmware = 0x0003,
				 NonFatalParameter = 0x0004,
				 FlashMemoryFull = 0x0005,
				 // Reserved = 0x0006,
				 StorageTempExceeded = 0x0007,
				 TempHigh = 0x0008,
				 TempLow = 0x0009,
				 ScuDisconnected = 0x000a,
				 PtpClockSynch = 0x000e };

	//! Returns the std::string representation of the alert
	std::string toString(uint16_t conditionCode);
}

namespace SystemEventEnum
{
	enum value { Ok = 0x0000,
		         ToolPluggedIn = 0x0001,
				 ToolUnplugged = 0x0002,
				 SiuPluggedIn = 0x0003,
				 SiuUnplugged = 0x0004 };

	//! Returns the std::string representation of the event
	std::string toString(uint16_t conditionCode);
}

#endif // SYSTEM_ALERT_HPP