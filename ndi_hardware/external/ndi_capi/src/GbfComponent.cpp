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

#include "GbfButton1D.h"
#include "GbfComponent.h"
#include "GbfContainer.h"
#include "GbfData3D.h"
#include "GbfData6D.h"
#include "GbfFrame.h"
#include "GbfSystemAlert.h"

GbfComponent* GbfComponent::buildComponent(BufferedReader& reader)
{
	// Read the component header information
	GbfComponent* component = NULL;
	uint16_t componentType = reader.get_uint16();
	uint32_t componentSize = reader.get_uint32();
	uint16_t itemOption = reader.get_uint16();
	uint32_t itemCount = reader.get_uint32();

	// Read data into the appropriate class based on the componentType
	if (componentType == GbfComponentType::Frame)
	{
		component = new GbfFrame(reader, itemCount);
	}
	else if (componentType == GbfComponentType::Data6D)
	{
		component = new GbfData6D(reader, itemCount);
	}
	else if (componentType == GbfComponentType::Data3D)
	{
		component = new GbfData3D(reader, itemCount);
	}
	else if (componentType == GbfComponentType::Button1D)
	{
		component = new GbfButton1D(reader, itemCount);
	}
	else if (componentType == GbfComponentType::SystemAlert)
	{
		component = new GbfSystemAlert(reader, itemCount);
	}
	else
	{
		// TODO: Not implement yet - GbfComponentTypes: Data2D, UV

		// Skip unrecognized items the size minus the 12 byte header we just read
		reader.skipBytes(componentSize - 12);

		// Keep track of the fact we read something odd...
		component = new GbfComponent();
	}

	// Write the header information that we read earlier
	component->componentType = componentType;
	component->componentSize = componentSize;
	component->itemOption = itemOption;
	component->itemCount = itemCount;

	return component;
}

std::string GbfComponent::toString() const
{
	std::stringstream stream;
	stream << std::hex << std::setfill('0');
	stream << "componentType=" << std::setw(4) << static_cast<unsigned>(componentType) << "(" << GbfComponentType::toString(componentType) << ")" << std::endl
		   << "componentSize=" << std::setw(8) << static_cast<unsigned>(componentSize) << std::endl
		   << "itemOption=" << std::setw(4) << static_cast<unsigned>(itemOption) << std::endl
		   << "itemCount=" << std::setw(8) << static_cast<unsigned>(itemCount) << std::endl;
	return stream.str();
}

std::string GbfComponentType::toString(uint16_t enumValue)
{
	switch (enumValue)
	{
		case GbfComponentType::Frame:
			return "ServerFrame";
		case GbfComponentType::Data6D:
			return "Data6D";
		case GbfComponentType::Data3D:
			return "Data3D";
		case GbfComponentType::Button1D:
			return "Button1D";
		case GbfComponentType::Data2D:
			return "Data2D";
		case GbfComponentType::UV:
			return "UV";
		case GbfComponentType::SystemAlert:
			return "SystemAlert";
		default:
			return "UnrecognizedEnumValue";
	}
}