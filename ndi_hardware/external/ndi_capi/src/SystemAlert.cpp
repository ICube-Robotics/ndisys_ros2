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

#include <iomanip>
#include <sstream>

#include "SystemAlert.h"

std::string SystemAlert::toString() const
{
	// Lookup the appropriate toString() method based on its type
	switch (conditionType)
	{
	case SystemAlertType::Fault:
		return std::string("Fault::").append(SystemFaultEnum::toString(conditionCode));
	case SystemAlertType::Alert:
		return std::string("Alert::").append(SystemAlertEnum::toString(conditionCode));
	case SystemAlertType::Event:
		return std::string("Event::").append(SystemEventEnum::toString(conditionCode));
	default:
		break; // do nothing
	};

	// Print the unrecognized alert types in hex
	std::stringstream stream;
	stream << std::hex << std::setfill('0');
	stream << "UnrecognizedAlertType: " << std::setw(4) << conditionType;
	return stream.str();
}

std::string SystemFaultEnum::toString(uint16_t conditionCode)
{
	switch (conditionCode)
	{
	case SystemFaultEnum::Ok:
		return "Ok";
	case SystemFaultEnum::FatalParameter:
		return "FatalParameter";
	case SystemFaultEnum::SensorParameter:
		return "SensorParameter";
	case SystemFaultEnum::MainVoltage:
		return "MainVoltage";
	case SystemFaultEnum::SensorVoltage:
		return "SensorVoltage";
	case SystemFaultEnum::IlluminatorCurrent:
		return "IlluminatorCurrent";
	case SystemFaultEnum::IlluminatorVoltage:
		return "IlluminatorVoltage";
	case SystemFaultEnum::Sensor0Temp:
		return "Sensor0Temp";
	case SystemFaultEnum::Sensor1Temp:
		return "Sensor1Temp";
	case SystemFaultEnum::MainTemp:
		return "MainTemp";
	case SystemFaultEnum::SensorMalfunction:
		return "SensorMalfunction";
	default:
		break; // do nothing
	};

	// Print the unrecognized fault types in hex
	std::stringstream stream;
	stream << std::hex << std::setfill('0');
	stream << "UnrecognizedFault: " << std::setw(4) << conditionCode;
	return stream.str();
}

std::string SystemAlertEnum::toString(uint16_t conditionCode)
{
	switch (conditionCode)
	{
	case SystemAlertEnum::Ok:
		return "Ok";
	case SystemAlertEnum::BatteryLow:
		return "BatteryLow";
	case SystemAlertEnum::BumpDetected:
		return "BumpDetected";
	case SystemAlertEnum::IncompatibleFirmware:
		return "IncompatibleFirmware";
	case SystemAlertEnum::NonFatalParameter:
		return "NonFatalParameter";
	case SystemAlertEnum::FlashMemoryFull:
		return "FlashMemoryFull";
	case SystemAlertEnum::StorageTempExceeded:
		return "StorageTempExceeded";
	case SystemAlertEnum::TempHigh:
		return "TempHigh";
	case SystemAlertEnum::TempLow:
		return "TempLow";
	case SystemAlertEnum::ScuDisconnected:
		return "ScuDisconnected";
	case SystemAlertEnum::PtpClockSynch:
		return "PtpClockSynch";
	default:
		break;
	};

	// Print the unrecognized alert types in hex
	std::stringstream stream;
	stream << std::hex << std::setfill('0');
	stream << "UnrecognizedAlert: " << std::setw(4) << conditionCode;
	return stream.str();
}

std::string SystemEventEnum::toString(uint16_t conditionCode)
{
	switch (conditionCode)
	{
	case SystemEventEnum::Ok:
		return "Ok";
	case SystemEventEnum::ToolPluggedIn:
		return "ToolPluggedIn";
	case SystemEventEnum::ToolUnplugged:
		return "ToolUnplugged";
	case SystemEventEnum::SiuPluggedIn:
		return "SiuPluggedIn";
	case SystemEventEnum::SiuUnplugged:
		return "SiuUnplugged";
	default:
		return "UnrecognizedEvent";
	};

	// Print the unrecognized event types in hex
	std::stringstream stream;
	stream << std::hex << std::setfill('0');
	stream << "UnrecognizedEvent: " << std::setw(4) << conditionCode;
	return stream.str();
}