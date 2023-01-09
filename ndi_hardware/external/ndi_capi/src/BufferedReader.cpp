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

#include "BufferedReader.h"

BufferedReader::BufferedReader(Connection* connection)
{
	connection_ = connection;
	currentIndex_ = 0;
}

std::string BufferedReader::toString() const
{
	std::stringstream stream;
	stream << std::hex << std::setfill('0');

	for ( int i = 0; i < buffer_.size(); i++)
	{
		stream << std::setw(2) << static_cast<unsigned>(buffer_[i]) << " ";
	}

	return stream.str();
}

std::string BufferedReader::getData(size_t start, size_t length) const
{
	std::string retVal = "";
	if ((start + length) <= buffer_.size())
	{
		for (size_t i = start; i < (start + length); i++)
		{
			retVal += (char) buffer_[i];
		}
	}
	return retVal;
}

void BufferedReader::readBytes(int numBytes)
{
	byte_t buffer = 0x00;
	while (numBytes-- > 0)
	{
		connection_->read(&buffer, 1);
		buffer_.push_back(buffer);
	}
}

void BufferedReader::skipBytes(int numBytes)
{
	if (currentIndex_ + numBytes >= buffer_.size())
	{
		// Don't wander off the end of the buffer
		currentIndex_ = (int)buffer_.size() - 1;
	}
	else if (currentIndex_ + numBytes < 0)
	{
		// Don't rewind beyond the first element
		currentIndex_ = 0;
	}
	else
	{
		// Move the index by the given number of bytes
		currentIndex_ += numBytes;
	}
}

byte_t BufferedReader::get_byte()
{
	return buffer_.at(currentIndex_++);
}

uint16_t BufferedReader::get_uint16()
{
	// The data is little-endian = least significant byte (LSB) in lowest memory address
	// Therefore, we need to construct the 16 bit integer by shifting and adding the two 8 bit bytes
	uint16_t retVal = get_byte();
	retVal += get_byte() << 8;
	return retVal;
}

uint32_t BufferedReader::get_uint32()
{
	// The data is little-endian = least significant byte (LSB) in lowest memory address
	// Therefore, we need to construct the 32 bit integer by shifting and adding the four 8 bit bytes
	uint32_t retVal = get_byte();
	retVal += get_byte() << 8;
	retVal += get_byte() << 16;
	retVal += get_byte() << 24;
	return retVal;
}

double BufferedReader::get_double()
{
	// Read the little endian float and convert it to a double
	float retVal = 0.0f;
	char* ptr = (char*)&retVal;
	ptr[0] = get_byte();
	ptr[1] = get_byte();
	ptr[2] = get_byte();
	ptr[3] = get_byte();
	return (double)retVal;
}