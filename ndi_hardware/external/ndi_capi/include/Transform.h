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
#ifndef TRANSFORM_HPP
#define TRANSFORM_HPP

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

//! A value chosen by NDI to represent an invalid transform value or result.
static const double BAD_FLOAT = -3.697314E28;

//! The value used to compare BAD_FLOAT
static const double MAX_NEGATIVE = -3.0E28;

/**
 * @brief This class stores the 6D quaternion transformation indicating the device's orientation.
 * @details Be sure to check the transform status. When missing, the transform parameters are
 *          invalid and should be set to BAD_FLOAT.
 */
class CAPICOMMON_API Transform
{
public:
	Transform();
	virtual ~Transform(){};

	/**
	 * @brief The index of the face being tracked on a multi-face tool.
	 */
	uint8_t getFaceNumber() const;

	/**
	 * @brief Returns the error code stored in the least significant 8 bits of the status.
	 */
	uint8_t getErrorCode() const;

	/**
	 * @brief Returns true if status bit 8 is high, indicating the tool is missing.
	 */
	bool isMissing() const;

	//! The handle that uniquely identifies the tool
	uint16_t toolHandle;

	//! The TransformStatus as a two byte integer. See the related enum for its interpretation.
	uint16_t status;

	//! The quaternion parameters in camera coordinates [mm]
	double q0, qx, qy, qz;

	//! The transformation parameters in camera coordinates [mm]
	double tx,ty,tz;

	//! The RMS error in the measurement [mm]
	double error;
};

namespace TransformStatus
{
	//! The least significant eight bits of the transform status is an error code in this list
	enum CAPICOMMON_API values { Enabled = 0x00,
				  PartiallyOutOfVolume = 0x03,
				  OutOfVolume = 0x09,
				  TooFewMarkers = 0x0D,
				  Inteference = 0x0E,
				  BadTransformFit = 0x11,
				  DataBufferLimit = 0x12,
				  AlgorithmLimit = 0x13,
				  FellBehind = 0x14,
				  OutOfSynch = 0x15,
				  ProcessingError = 0x16,
				  ToolMissing = 0x1F,
				  TrackingNotEnabled = 0x20,
				  ToolUnplugged = 0x21 };

	//! Converts the given error code to its std::string representation
	CAPICOMMON_API std::string toString(uint8_t errorCode);
}

#endif // TRANSFORM_HPP