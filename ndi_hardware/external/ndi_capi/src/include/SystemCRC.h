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
#ifndef SYSTEM_CRC_HPP
#define SYSTEM_CRC_HPP

/**
 * @brief This class contains methods and data used to verify Cyclical Redundancy Checks (CRCs)
 */
class SystemCRC
{
public:
	/**
	 * @brief Creates the lookup table using the polynomial X^16 + X^15 + X^2 + 1
	 */
	SystemCRC();
	virtual ~SystemCRC(){};

	/**
	 * @brief Calculates the CRC16 of the ASCII reply
	 * @param reply The reply without its trailing CRC16 + CR
	 * @param replyLength The length of the reply, including its trailing CRC
	 * @returns The CRC16 of the reply
	 */
	unsigned int calculateCRC16(const char* reply, int replyLength) const;

private:
	/**
	 * @brief Calculates a CRC value using a lookup table
	 * @param crc The previous value of the running CRC
	 * @param data The data to add to the running CRC
	 * @returns The value to add to the running CRC
	 */
	unsigned int calcValue(unsigned int crc, int data) const;

	//! A lookup table storing the CRC polynomial
	static unsigned int crcTable_[256];
};

#endif // SYSTEM_CRC_HPP