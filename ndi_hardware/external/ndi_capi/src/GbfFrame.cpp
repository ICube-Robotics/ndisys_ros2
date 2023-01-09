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
#include "GbfData3D.h"
#include "GbfData6D.h"
#include "GbfFrame.h"
#include "GbfSystemAlert.h"

GbfFrame::GbfFrame(BufferedReader& reader, int dataItems)
{
	for (int i = 0; i < dataItems; i++)
	{
		data.push_back(GbfFrameDataItem(reader));
	}
}

std::vector<ToolData> GbfFrame::getToolData() const
{
	// The end goal is to flatten the data into the ToolData structures for client-side manipulation.
	std::vector<ToolData> tools;

	// System alerts are transmitted with each GbfFrameDataItem
	std::vector<SystemAlert> gbfFrameDataItemAlerts;

	// Active, Active-Wireless, and Passive tools collect data in different ways.
	// Thus, each frame of data must be divided into separate frames for each tool type.
	// BX2 transmits the data as one or more GbfFrameDataItems on each GbfFrame.
	for (int i = 0; i < data.size(); i++)
	{
		// Go through the GBF components on each GbfFrameDataItem
		for (int c = 0; c < data[i].frameData->components.size(); c++)
		{
			GbfComponent* component = data[i].frameData->components[c];
			if (component->componentType == GbfComponentType::Data6D)
			{
				// Go through each 6D on this GBF component
				GbfData6D* data6D = static_cast<GbfData6D*>(component);
				for (int j = 0; j < data6D->toolTransforms.size(); j++)
				{
					// Append the 6D if the ToolData exists
					bool toolExists = false;
					for (int t =0; t < tools.size(); t++)
					{
						if (tools[t].transform.toolHandle == data6D->toolTransforms[j].toolHandle)
						{
							toolExists = true;
							tools[t].transform = data6D->toolTransforms[j];
							break;
						}
					}
					if (!toolExists)
					{
						// Create a ToolData object with the frame and transform information
						tools.push_back(ToolData());
						tools.back().dataIsNew = true;
						tools.back().frameType = data[i].frameType;
						tools.back().frameSequenceIndex = data[i].frameSequenceIndex;
						tools.back().frameStatus = data[i].frameStatus;
						tools.back().frameNumber = data[i].frameNumber;
						tools.back().timespec_s = data[i].timespec_s;
						tools.back().timespec_ns = data[i].timespec_ns;
						tools.back().transform = data6D->toolTransforms[j];
						tools.back().systemAlerts = gbfFrameDataItemAlerts;
					}
				}
			}
			else if (component->componentType == GbfComponentType::Data3D)
			{
				// Go through each 3D on this GBF component
				GbfData3D* data3D = static_cast<GbfData3D*>(component);
				for (int j = 0; j < data3D->toolHandles.size(); j++)
				{
					// Append the 3D information if the ToolData exists
					bool toolExists = false;
					for (int t = 0; t < tools.size(); t++)
					{
						if (tools[t].transform.toolHandle == data3D->toolHandles[j])
						{
							toolExists = true;
							tools[t].markers = data3D->markers[j];
							break;
						}
					}
					if (!toolExists)
					{
						// Create a ToolData object with the frame and marker information
						tools.push_back(ToolData());
						tools.back().transform.toolHandle = data3D->toolHandles[j]; // don't forget the handle
						tools.back().dataIsNew = true;
						tools.back().frameType = data[i].frameType;
						tools.back().frameSequenceIndex = data[i].frameSequenceIndex;
						tools.back().frameStatus = data[i].frameStatus;
						tools.back().frameNumber = data[i].frameNumber;
						tools.back().timespec_s = data[i].timespec_s;
						tools.back().timespec_ns = data[i].timespec_ns;
						tools.back().markers = data3D->markers[j];
						tools.back().systemAlerts = gbfFrameDataItemAlerts;
					}
				}
			}
			else if (component->componentType == GbfComponentType::Button1D)
			{
				GbfButton1D* buttonData = static_cast<GbfButton1D*>(component);
				if (buttonData->data.size() > 0)
				{
					// Append button data if the ToolData exists
					bool toolExists = false;
					for (int t = 0; t < tools.size(); t++)
					{
						if (tools[t].transform.toolHandle == buttonData->toolHandle)
						{
							bool toolExists = false;
							tools[t].buttons = buttonData->data;
							break;
						}
					}
					if (!toolExists)
					{
						// Create a ToolData object with the button data and frame information
						tools.push_back(ToolData());
						tools.back().transform.toolHandle = buttonData->toolHandle; // don't forget the handle
						tools.back().dataIsNew = true;
						tools.back().frameType = data[i].frameType;
						tools.back().frameSequenceIndex = data[i].frameSequenceIndex;
						tools.back().frameStatus = data[i].frameStatus;
						tools.back().frameNumber = data[i].frameNumber;
						tools.back().timespec_s = data[i].timespec_s;
						tools.back().timespec_ns = data[i].timespec_ns;
						tools.back().buttons = buttonData->data;
						tools.back().systemAlerts = gbfFrameDataItemAlerts;
					}
				}
			}
			else if (component->componentType == GbfComponentType::SystemAlert)
			{
				// Store the alerts "globally" within this method so they are added to new ToolData
				GbfSystemAlert* alert = static_cast<GbfSystemAlert*>(component);
				gbfFrameDataItemAlerts = alert->data;

				// Append the alert information to all ToolData from this GbfFrameDataItem
				bool toolExists = false;
				for (int t = 0; t < tools.size(); t++)
				{
					if (tools[t].frameNumber == data[i].frameNumber)
					{
						tools[t].systemAlerts = gbfFrameDataItemAlerts;
						break;
					}
				}
			}
		} // process the next GbfComponent with more tool information
	} // process the next GbfFrameDataItem for a different tool type...
	return tools;
}

std::string GbfFrame::toString() const
{
	std::stringstream stream;
	stream << std::hex << std::setfill('0');
	stream << "-----GbfFrame " << std::endl << GbfComponent::toString();
	for (int i = 0; i < data.size(); i++)
	{
		stream << data[i].toString();
	}

	return stream.str();
}

std::string GbfFrameDataItem::toString() const
{
	std::stringstream stream;
	stream << std::hex << std::setfill('0');
	stream << "-----GbfFrameDataItem " << std::endl
		   << "frameType=" << std::setw(2) << static_cast<unsigned>(frameType) << "(" << FrameType::toString(frameType) << ")" << std::endl
		   << "frameSequenceIndex=" << std::setw(2) << static_cast<unsigned>(frameSequenceIndex) << std::endl
		   << "frameStatus=" << std::setw(4) << static_cast<unsigned>(frameStatus) << std::endl
		   << "frameNumber=" << std::setw(8) << static_cast<unsigned>(frameNumber) << std::endl
		   << "timestamp=" << std::setw(8) << static_cast<unsigned>(timespec_s) << "," << std::setw(8) << static_cast<unsigned>(timespec_ns) << std::endl
		   << frameData->toString() << std::endl;
	return stream.str();
}

GbfFrameDataItem::GbfFrameDataItem(BufferedReader& reader)
{
	frameType = reader.get_byte();
	frameSequenceIndex = reader.get_byte();
	frameStatus = reader.get_uint16();
	frameNumber = reader.get_uint32();
	timespec_s = reader.get_uint32();
	timespec_ns = reader.get_uint32();
	frameData = new GbfContainer(reader);
}