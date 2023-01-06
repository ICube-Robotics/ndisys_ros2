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

#include "Transform.h"

Transform::Transform()
{
	toolHandle = 0x0000;
	status = 0x0100; // Missing by default
	q0 = qx = qy = qz = tx = ty = tz = error = BAD_FLOAT;
}

uint8_t Transform::getFaceNumber() const
{
	// Bits 13-15 store an integer representing the index of the tool face being tracked
	return (uint8_t)((status & 0xE000) >> 12);
}

uint8_t Transform::getErrorCode() const
{
	// The error code is the least significant 8 bits of the status
	return (uint8_t)(status & 0x00FF);
}

bool Transform::isMissing() const
{
	// Bit 8 indicates if the transform is missing
	return (status & 0x0100) != 0x0000;
}

std::string TransformStatus::toString(uint8_t errorCode)
{
	switch (errorCode)
	{
		case TransformStatus::Enabled:
			return "Enabled";
		case TransformStatus::PartiallyOutOfVolume:
			return "PartiallyOutOfVolume";
		case TransformStatus::OutOfVolume:
			return "OutOfVolume";
		case TransformStatus::TooFewMarkers:
			return "TooFewMarkers";
		case TransformStatus::Inteference:
			return "Inteference";
		case TransformStatus::BadTransformFit:
			return "BadTransformFit";
		case TransformStatus::DataBufferLimit:
			return "DataBufferLimit";
		case TransformStatus::AlgorithmLimit:
			return "AlgorithmLimit";
		case TransformStatus::FellBehind:
			return "FellBehind";
		case TransformStatus::OutOfSynch:
			return "OutOfSynch";
		case TransformStatus::ProcessingError:
			return "ProcessingError";
		case TransformStatus::ToolMissing:
			return "ToolMissing";
		case TransformStatus::TrackingNotEnabled:
			return "TrackingNotEnabled";
		case TransformStatus::ToolUnplugged:
			return "ToolUnplugged";
		default:
			return "UnrecognizedEnumValue";
	}
}