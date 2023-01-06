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

#include "GbfData3D.h"
#include "Transform.h" // for definition of BAD_FLOAT

GbfData3D::GbfData3D(BufferedReader& reader,  int numberOfTools)
{
	uint16_t toolHandle = 0x00;
	uint16_t numberOf3Ds = 0x00;
	for (int i = 0; i < numberOfTools; i++)
	{
		// Read the data
		toolHandle = reader.get_uint16();
		numberOf3Ds = reader.get_uint16();

		// Put the toolHandle into its own vector
		toolHandles.push_back(toolHandle);

		// Create a corresponding vector with the 3Ds
		std::vector<MarkerData> list3Ds;
		for ( int i = 0; i < numberOf3Ds; i++)
		{
			MarkerData pos;
			pos.status = reader.get_byte();
			reader.get_byte(); // reserved
			pos.markerIndex = reader.get_uint16();
			if (pos.status == MarkerStatus::Missing)
			{
				pos.x = BAD_FLOAT;
				pos.y = BAD_FLOAT;
				pos.z = BAD_FLOAT;
			}
			else
			{
				pos.x = reader.get_double();
				pos.y = reader.get_double();
				pos.z = reader.get_double();
			}
			list3Ds.push_back(pos);
		}
		markers.push_back(list3Ds);
	}
}

std::string GbfData3D::toString() const
{
	std::stringstream stream;
	stream << std::hex << std::setfill('0') << std::setprecision(4);
	stream << "----GbfData3D " << std::endl << GbfComponent::toString();

	// For each tool, print the tool handle and all of its associated 3Ds
	for (int toolIndex = 0; toolIndex < toolHandles.size(); toolIndex++)
	{
		stream << "toolHandleReference=" << std::setw(4) << toolHandles[toolIndex] << std::endl
		   << "numberOf3Ds=" << std::setw(4) << markers[toolIndex].size() << std::endl;

		for ( int i = 0; i < markers[toolIndex].size(); i++)
		{
			stream << "--Data3D: status=" << std::setw(2) << static_cast<unsigned>(markers[toolIndex][i].status) << " (" << MarkerStatus::toString(markers[toolIndex][i].status) << ")"
				   << ", markerIndex=" << std::setw(4) << static_cast<unsigned>(markers[toolIndex][i].markerIndex)
				   << ", [x y z] = [" << markers[toolIndex][i].x << "," << markers[toolIndex][i].y << "," << markers[toolIndex][i].z << "]" << std::endl;
		}

	}
	return stream.str();
}