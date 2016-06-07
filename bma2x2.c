/*
 ****************************************************************************
 *
 * (C) All rights reserved by ROBERT BOSCH GMBH
 *
 ****************************************************************************/
/*  Date: 2014/04/30
 *  Revision: 1.6 $
 *
 */

/****************************************************************************
* Copyright (C) 2014 Bosch Sensortec GmbH
*
* bma2x2.c
*
* Usage:        Sensor Driver for BMA2x2 Triaxial acceleration sensor
*
****************************************************************************/
/***************************************************************************/
/*  Disclaimer
*
* Common:
* Bosch Sensortec products are developed for the consumer goods industry.
* They may only be used within the parameters of the respective valid
* product data sheet.  Bosch Sensortec products are provided with the
* express understanding that there is no warranty of fitness for a
* particular purpose.They are not fit for use in life-sustaining,
* safety or security sensitive systems or any system or device
* that may lead to bodily harm or property damage if the system
* or device malfunctions. In addition,Bosch Sensortec products are
* not fit for use in products which interact with motor vehicle systems.
* The resale and or use of products are at the purchasers own risk and
* his own responsibility. The examination of fitness for the intended use
* is the sole responsibility of the Purchaser.
*
* The purchaser shall indemnify Bosch Sensortec from all third party
* claims, including any claims for incidental, or consequential damages,
* arising from any product use not covered by the parameters of
* the respective valid product data sheet or not approved by
* Bosch Sensortec and reimburse Bosch Sensortec for all costs in
* connection with such claims.
*
* The purchaser must monitor the market for the purchased products,
* particularly with regard to product safety and inform Bosch Sensortec
* without delay of all security relevant incidents.
*
* Engineering Samples are marked with an asterisk (*) or (e).
* Samples may vary from the valid technical specifications of the product
* series. They are therefore not intended or fit for resale to third
* parties or for use in end products. Their sole purpose is internal
* client testing. The testing of an engineering sample may in no way
* replace the testing of a product series. Bosch Sensortec assumes
* no liability for the use of engineering samples.
* By accepting the engineering samples, the Purchaser agrees to indemnify
* Bosch Sensortec from all claims arising from the use of engineering
* samples.
*
* Special:
* This software module (hereinafter called "Software") and any information
* on application-sheets (hereinafter called "Information") is provided
* free of charge for the sole purpose to support your application work.
* The Software and Information is subject to the following
* terms and conditions:
*
* The Software is specifically designed for the exclusive use for
* Bosch Sensortec products by personnel who have special experience
* and training. Do not use this Software if you do not have the
* proper experience or training.
*
* This Software package is provided `` as is `` and without any expressed
* or implied warranties,including without limitation, the implied warranties
* of merchantability and fitness for a particular purpose.
*
* Bosch Sensortec and their representatives and agents deny any liability
* for the functional impairment
* of this Software in terms of fitness, performance and safety.
* Bosch Sensortec and their representatives and agents shall not be liable
* for any direct or indirect damages or injury, except as
* otherwise stipulated in mandatory applicable law.
*
* The Information provided is believed to be accurate and reliable.
* Bosch Sensortec assumes no responsibility for the consequences of use
* of such Information nor for any infringement of patents or
* other rights of third parties which may result from its use.
* No license is granted by implication or otherwise under any patent or
* patent rights of Bosch. Specifications mentioned in the Information are
* subject to change without notice.
*
* It is not allowed to deliver the source code of the Software
* to any third party without permission of
* Bosch Sensortec.
*/
/****************************************************************************/
/*! file <BMA2x2 >
    brief <Sensor driver for BMA2x2> */
#include "bma2x2.h"
/* user defined code to be added here ... */
static struct bma2x2_t *p_bma2x2;
/* Based on Bit resolution value should be modified */
u8 V_BMA2x2RESOLUTION_U8R = BMA2x2_14_RESOLUTION;
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 * Description: *//**\brief This API reads the data from
 * the given register continuously
 *
 *
 *
 *
 *  \param u8 addr, u8 *data
 *                       addr -> Address of the register
 *                       data -> address of the variable,
 *                               read value will be kept
 * \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_burst_read(u8 addr,
u8 *data, u32 len)
{
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			comres = p_bma2x2->BMA2x2_BURST_READ_FUNC
			(p_bma2x2->dev_addr, addr, data, len);
		}
	return comres;
}
/****************************************************************************
 *	Description: *//**\brief This function is used for initialize
 *	bus read and bus write functions
 *	assign the chip id and device address
 *	chip id is read in the register 0x00 bit from 0 to 7
 *
 *	 \param  bma2x2_t *bma2x2 structure pointer
 *
 *	While changing the parameter of the bma2x2_t
 *	consider the following point:
 *	Changing the reference value of the parameter
 *	will changes the local copy or local reference
 *	make sure your changes will not
 *	affect the reference value of the parameter
 *
 *
 *
 *
 *
 * \return results of bus communication function
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_init(struct bma2x2_t *bma2x2)
{
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	u8 data = C_BMA2x2_Zero_U8X;

	p_bma2x2 = bma2x2;       /* assign bma2x2 ptr */
	p_bma2x2->dev_addr = bma2x2->dev_addr;
	/* assign bma2x2 I2C_addr */
	comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
	(p_bma2x2->dev_addr,
	BMA2x2_CHIP_ID__REG, &data, 1);/* read Chip Id */
	p_bma2x2->chip_id = data;    /* get bit slice */
	return comres;
}
/****************************************************************************
 * Description: *//**\brief This API gives data to the given register and
 *                the data is written in the corresponding register address
 *
 *
 *
 *
 *  \param u8 addr, u8 data, u8 len
 *          addr -> Address of the register
 *          data -> Data to be written to the register
 *          len  -> Length of the Data
 *
 *
 *
 * \return results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_write_reg(u8 addr,
u8 *data, u8 len)
{
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		comres = p_bma2x2->BMA2x2_BUS_WRITE_FUNC
		(p_bma2x2->dev_addr, addr, data, len);
	}
	return comres;
}
/****************************************************************************
 * Description: *//**\brief This API reads the data from
 *           the given register address
 *
 *
 *
 *
 *  \param u8 addr, u8 *data, u8 len
 *         addr -> Address of the register
 *         data -> address of the variable, read value will be kept
 *         len  -> Length of the data
 *
 *
 *
 *
 * \return results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 **************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_read_reg(u8 addr,
u8 *data, u8 len)
{
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr, addr, data, len);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
 * Description: *//**\brief This API reads acceleration data X values
 *                          from location 02h and 03h
 *
 *
 *
 *
 *  \param short  *a_x   :  pointer holding the data of a_x
 *
 *
 *
 * \return results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 **************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_read_accel_x(s16 *a_x)
{
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	u8	data[2] = {0, 0};
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (V_BMA2x2RESOLUTION_U8R) {
		case BMA2x2_12_RESOLUTION:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ACC_X12_LSB__REG, data, 2);
			*a_x = (s16)((((s32)((s8)data[1]))
			<< C_BMA2x2_Eight_U8X) | (data[0] & 0xF0));
			*a_x = *a_x >> C_BMA2x2_Four_U8X;
		break;
		case BMA2x2_10_RESOLUTION:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ACC_X10_LSB__REG, data, 2);
			*a_x = (s16)((((s32)((s8)data[1]))
			<< C_BMA2x2_Eight_U8X) | (data[0] & 0xC0));
			*a_x = *a_x >> C_BMA2x2_Six_U8X;
		break;
		case BMA2x2_14_RESOLUTION:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ACC_X14_LSB__REG, data, 2);
			*a_x = (s16)((((s32)((s8)data[1]))
			<< C_BMA2x2_Eight_U8X) | (data[0] & 0xFC));
			*a_x = *a_x >> C_BMA2x2_Two_U8X;
		break;
		default:
		break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
 * Description: *//**\brief This API reads acceleration data X values of
 *                 8bit  resolution  from location 03h
 *
 *
 *
 *
 *  \param short  *a_x   :  pointer holding the data of a_x
 *
 *
 *
 * \return results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 **************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_read_accel_elight_resolution_x(s8 *a_x)
{
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	u8	data = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_X_AXIS_MSB_REG, &data, 1);
			*a_x = BMA2x2_GET_BITSLICE(data,
			BMA2x2_ACC_X_MSB);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
 * Description: *//**\brief This API reads acceleration data Y values
 *                          from location 04h and 05h
 *
 *
 *
 *
 *  \param short  *a_y   :  pointer holding the data of a_y
 *
 *
 *
 * \return results of bus communication function
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_read_accel_y(s16 *a_y)
{
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	u8 data[2] = {0, 0};
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (V_BMA2x2RESOLUTION_U8R) {
		case BMA2x2_12_RESOLUTION:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ACC_Y12_LSB__REG, data, 2);
			*a_y = (s16)((((s32)((s8)data[1]))
			<< C_BMA2x2_Eight_U8X) | (data[0] & 0xF0));
			*a_y = *a_y >> C_BMA2x2_Four_U8X;
		break;
		case BMA2x2_10_RESOLUTION:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ACC_Y10_LSB__REG, data, 2);
			*a_y = (s16)((((s32)((s8)data[1]))
			<< C_BMA2x2_Eight_U8X) | (data[0] & 0xC0));
			*a_y = *a_y >> C_BMA2x2_Six_U8X;
		break;
		case BMA2x2_14_RESOLUTION:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ACC_Y14_LSB__REG, data, 2);
			*a_y = (s16)((((s32)((s8)data[1]))
			<< C_BMA2x2_Eight_U8X) | (data[0] & 0xFC));
			*a_y = *a_y >> C_BMA2x2_Two_U8X;
		break;
		default:
		break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
 * Description: *//**\brief This API reads acceleration data Y values of
 *                 8bit  resolution  from location 05h
 *
 *
 *
 *
 *  \param short  *a_y   :  pointer holding the data of a_y
 *
 *
 *
 * \return results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 **************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_read_accel_elight_resolution_y(s8 *a_y)
{
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	u8	data = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_Y_AXIS_MSB_REG, &data, 1);
			*a_y = BMA2x2_GET_BITSLICE(data,
			BMA2x2_ACC_Y_MSB);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 * Description: *//**\brief This API reads acceleration data Z values
 *                          from location 06h and 07h
 *
 *
 *
 *
 *  \param short  *a_z   :  pointer holding the data of a_z
 *
 *
 *
 * \return results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_read_accel_z(s16 *a_z)
{
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	u8 data[2] = {0, 0};
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (V_BMA2x2RESOLUTION_U8R) {
		case BMA2x2_12_RESOLUTION:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ACC_Z12_LSB__REG, data, 2);
			*a_z = (s16)((((s32)((s8)data[1]))
			<< C_BMA2x2_Eight_U8X) | (data[0] & 0xF0));
			*a_z = *a_z >> C_BMA2x2_Four_U8X;
		break;
		case BMA2x2_10_RESOLUTION:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ACC_Z10_LSB__REG, data, 2);
			*a_z = (s16)((((s32)((s8)data[1]))
			<< C_BMA2x2_Eight_U8X) | (data[0] & 0xC0));
			*a_z = *a_z >> C_BMA2x2_Six_U8X;
		break;
		case BMA2x2_14_RESOLUTION:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ACC_Z14_LSB__REG, data, 2);
			*a_z = (s16)((((s32)((s8)data[1]))
			<< C_BMA2x2_Eight_U8X) | (data[0] & 0xFC));
			*a_z = *a_z >> C_BMA2x2_Two_U8X;
		break;
		default:
		break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
 * Description: *//**\brief This API reads acceleration data Y values of
 *                 8bit  resolution  from location 07h
 *
 *
 *
 *
 *  \param short  *a_z   :  pointer holding the data of a_z
 *
 *
 *
 * \return results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 **************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_read_accel_elight_resolution_z(s8 *a_z)
{
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	u8	data = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_Z_AXIS_MSB_REG, &data, 1);
			*a_z = BMA2x2_GET_BITSLICE(data,
			BMA2x2_ACC_Z_MSB);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 * Description: *//**\brief This API reads acceleration data X,Y,Z values
 *                          from location 02h to 07h
 *
 *
 *
 *
 *  \param bma2x2acc_t * acc : pointer holding the data of bma2x2acc_t
 *
 *
 *
 * \return results of bus communication function
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_read_accel_xyz(struct bma2x2acc_t *acc)
{
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	u8 data[6] = {0, 0, 0, 0, 0, 0};
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (V_BMA2x2RESOLUTION_U8R) {
		case BMA2x2_12_RESOLUTION:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr, BMA2x2_ACC_X12_LSB__REG, data, 6);
			/* read the x data*/
			acc->x = (s16)((((s32)((s8)data[1]))
			<< C_BMA2x2_Eight_U8X) | (data[0] & 0xF0));
			acc->x = acc->x >> C_BMA2x2_Four_U8X;

			/* read the y data*/
			acc->y = (s16)((((s32)((s8)data[3]))
			<< C_BMA2x2_Eight_U8X) | (data[2] & 0xF0));
			acc->y = acc->y >> C_BMA2x2_Four_U8X;

			/* read the z data*/
			acc->z = (s16)((((s32)((s8)data[5]))
			<< C_BMA2x2_Eight_U8X) | (data[4] & 0xF0));
			acc->z = acc->z >> C_BMA2x2_Four_U8X;

		break;
		case BMA2x2_10_RESOLUTION:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr, BMA2x2_ACC_X10_LSB__REG, data, 6);
			/* read the x data*/
			acc->x = (s16)((((s32)((s8)data[1]))
			<< C_BMA2x2_Eight_U8X) | (data[0] & 0xC0));
			acc->x = acc->x >> C_BMA2x2_Six_U8X;

			/* read the y data*/
			acc->y = (s16)((((s32)((s8)data[3]))
			<< C_BMA2x2_Eight_U8X) | (data[2] & 0xC0));
			acc->y = acc->y >> C_BMA2x2_Six_U8X;

			/* read the z data*/
			acc->z = (s16)((((s32)((s8)data[5]))
			<< C_BMA2x2_Eight_U8X) | (data[4] & 0xC0));
			acc->z = acc->z >> C_BMA2x2_Six_U8X;
		break;
		case BMA2x2_14_RESOLUTION:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr, BMA2x2_ACC_X14_LSB__REG, data, 6);

			/* read the x data*/
			acc->x = (s16)((((s32)((s8)data[1]))
			<< C_BMA2x2_Eight_U8X) | (data[0] & 0xFC));
			acc->x = acc->x >> C_BMA2x2_Two_U8X;

			/* read the y data*/
			acc->y = (s16)((((s32)((s8)data[3]))
			<< C_BMA2x2_Eight_U8X) | (data[2] & 0xFC));
			acc->y = acc->y >> C_BMA2x2_Two_U8X;

			/* read the z data*/
			acc->z = (s16)((((s32)((s8)data[5]))
			<< C_BMA2x2_Eight_U8X) | (data[4] & 0xFC));
			acc->z = acc->z >> C_BMA2x2_Two_U8X;
		break;
		default:
		break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 * Description: *//**\brief This API reads acceleration of 8 bit resolution
 *                          data of X,Y,Z values
 *                          from location 03h , 05h and 07h
 *
 *
 *
 *
 *  \param bma2x2acc_t * acc : pointer holding the data of bma2x2acc_t
 *
 *
 *
 * \return results of bus communication function
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_read_accel_eight_resolution_xyz(
struct bma2x2acc_eight_resolution *acc)
{
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	u8	data = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
		(p_bma2x2->dev_addr,
		BMA2x2_X_AXIS_MSB_REG, &data, 1);
		acc->x = BMA2x2_GET_BITSLICE(data,
		BMA2x2_ACC_X_MSB);

		comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
		(p_bma2x2->dev_addr,
		BMA2x2_Y_AXIS_MSB_REG, &data, 1);
		acc->y = BMA2x2_GET_BITSLICE(data,
		BMA2x2_ACC_Y_MSB);

		comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
		(p_bma2x2->dev_addr,
		BMA2x2_Z_AXIS_MSB_REG, &data, 1);
		acc->z = BMA2x2_GET_BITSLICE(data,
		BMA2x2_ACC_Z_MSB);
		}
	return comres;
}
/****************************************************************************
 * Description: *//**\brief This API Reads tap slope status register byte
 *                          from location 0Bh
 *
 *
 *
 *
 *   \param u8 * status_tap : Pointer holding the status_tap
 *
 *
 *
 * \return Result of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_int_tap_status(
u8 *status_tap)
{
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr, BMA2x2_STATUS_TAP_SLOPE_REG,
			status_tap, C_BMA2x2_One_U8X);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
 * Description: *//**\brief This API Reads orient status register byte
 *                          from location 0Ch
 *
 *
 *
 *
 *  \param u8 *status_orient : Pointer holding the status_orient
 *
 *
 *
 * \return Result of bus communication function
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_int_orient_status(
u8 *status_orient)
{
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr, BMA2x2_STATUS_ORIENT_HIGH_REG,
			status_orient, C_BMA2x2_One_U8X);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API Reads fifo status register byte
 *                          from location 0Eh
 *
 *
 *
 *
 *  \param u8 *status_fifo : Pointer holding the status_fifo
 *
 *
 *
 * \return Result of bus communication function
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 **************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_fifo_status(
u8 *status_fifo)
{
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC(
			p_bma2x2->dev_addr,
			BMA2x2_STATUS_FIFO_REG,
			status_fifo, C_BMA2x2_One_U8X);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
 * Description: *//**\brief This API Reads fifo frame count
 *       bits from location 0Eh
 *
 *
 *
 *
 * \param u8 *framecount : Pointer holding the fifo frame count
 *
 *
 *
 * \return Result of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_fifo_framecount(
u8 *framecount)
{
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	u8 data = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC(
			p_bma2x2->dev_addr,
			BMA2x2_FIFO_FRAME_COUNTER_S__REG,
			&data, C_BMA2x2_One_U8X);
			*framecount = BMA2x2_GET_BITSLICE(data,
			BMA2x2_FIFO_FRAME_COUNTER_S);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
 * Description: *//**\brief This API Reads fifo overrun bits
 *        from location 0Eh
 *
 *
 *
 *
 *\param u8 *overrun : Pointer holding the overrun bit
 *
 *
 *
 * \return Result of bus communication function
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 **************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_fifo_overrun(u8 *overrun)
{
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	u8 data = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC(
			p_bma2x2->dev_addr,
			BMA2x2_FIFO_OVERRUN_S__REG,
			&data, C_BMA2x2_One_U8X);
			*overrun = BMA2x2_GET_BITSLICE(data,
			BMA2x2_FIFO_OVERRUN_S);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 * Description: *//**\brief This API Reads interrupt status register byte
 *                          from location 09h
 *
 *
 *
 *
 *\param u8 * status : Pointer holding the status register
 *
 *
 *
 * \return Result of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 **************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_interrupt_status(u8 *status)
{
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC(
			p_bma2x2->dev_addr,
			BMA2x2_STATUS1_REG, status, C_BMA2x2_Four_U8X);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 * Description: *//**\brief This API is used to get
 *                 the Ranges(g values) of the sensor in the register 0x0F
 *					bit from 0 to 3
 *
 *
 *
 *
 *  \param u8 * Range : Pointer holding the Accel Range
 *                     Range -> 3 -> BMA2x2_RANGE_2G
 *                              5 -> BMA2x2_RANGE_4G
 *                              8 -> BMA2x2_RANGE_8G
 *                              12 -> BMA2x2_RANGE_16G
 *
 *
 *
 * \return communication results
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_range(u8 *range)
{
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	u8 data = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		comres = p_bma2x2->BMA2x2_BUS_READ_FUNC(p_bma2x2->dev_addr,
		BMA2x2_RANGE_SEL__REG, &data, C_BMA2x2_One_U8X);
		data = BMA2x2_GET_BITSLICE(data, BMA2x2_RANGE_SEL);
		*range = data;
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 * Description: *//**\brief This API is used to set
 *  Ranges(g value) of the sensor in the register 0x0F
 *					bit from 0 to 3
 *
 *
 *
 *
 *
 *  \param u8 Range : The value of range
 *             Range ->   3 -> 3 -> BMA2x2_RANGE_2G
 *                        5 -> BMA2x2_RANGE_4G
 *                        8 -> BMA2x2_RANGE_8G
 *                        12 -> BMA2x2_RANGE_16G
 *
 * \return communication results
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_range(u8 range)
{
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	u8 data1 = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		if ((range == C_BMA2x2_Three_U8X) ||
		(range == C_BMA2x2_Five_U8X) ||
		(range == C_BMA2x2_Eight_U8X) ||
		(range == C_BMA2x2_Twelve_U8X)) {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_RANGE_SEL_REG, &data1, C_BMA2x2_One_U8X);
			switch (range) {
			case BMA2x2_RANGE_2G:
				data1  = BMA2x2_SET_BITSLICE(data1,
				BMA2x2_RANGE_SEL, C_BMA2x2_Three_U8X);
			break;
			case BMA2x2_RANGE_4G:
				data1  = BMA2x2_SET_BITSLICE(data1,
				BMA2x2_RANGE_SEL, C_BMA2x2_Five_U8X);
			break;
			case BMA2x2_RANGE_8G:
				data1  = BMA2x2_SET_BITSLICE(data1,
				BMA2x2_RANGE_SEL, C_BMA2x2_Eight_U8X);
			break;
			case BMA2x2_RANGE_16G:
				data1  = BMA2x2_SET_BITSLICE(data1,
				BMA2x2_RANGE_SEL, C_BMA2x2_Twelve_U8X);
			break;
			default:
			break;
			}
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_RANGE_SEL_REG, &data1, C_BMA2x2_One_U8X);
		} else {
		return  E_OUT_OF_RANGE;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***********************************************************************
 * Description: *//**\brief This API is used to get
 *       the bandwidth of the sensor in the register
 *			0x10 bit from 0 to 4
 *
 *
 *
 *
 *  \param  u8 * bw : Pointer holding the bandwidth
 *                bw ->	BMA2x2_BW_7_81HZ        0x08
 *						BMA2x2_BW_15_63HZ       0x09
 *						BMA2x2_BW_31_25HZ       0x0A
 *						BMA2x2_BW_62_50HZ       0x0B
 *						BMA2x2_BW_125HZ         0x0C
 *						BMA2x2_BW_250HZ         0x0D
 *						BMA2x2_BW_500HZ         0x0E
 *						BMA2x2_BW_1000HZ        0x0F
 *
 *
 *
 * \return	results of bus communication function
 *
 *
 *************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_bandwidth(u8 *bw)
{
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	u8 data = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_BANDWIDTH__REG, &data, C_BMA2x2_One_U8X);
			data = BMA2x2_GET_BITSLICE(data, BMA2x2_BANDWIDTH);
			*bw = data;
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/************************************************************************
 * Description: *//**\brief This API is used to set
 *           Bandwidth of the sensor in the register
 *			0x10 bit from 0 to 4
 *
 *
 *
 *
 *  \param u8 bw: The value of Bandwidth
 *              bw ->	BMA2x2_BW_7_81HZ        0x08
 *						BMA2x2_BW_15_63HZ       0x09
 *						BMA2x2_BW_31_25HZ       0x0A
 *						BMA2x2_BW_62_50HZ       0x0B
 *						BMA2x2_BW_125HZ         0x0C
 *						BMA2x2_BW_250HZ         0x0D
 *						BMA2x2_BW_500HZ         0x0E
 *						BMA2x2_BW_1000HZ        0x0F
 *
 *
 *
 *
 *
 * \return results of bus communication function
 *
 *
 **************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 **************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_bandwidth(u8 bw)
{
BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
u8 data = C_BMA2x2_Zero_U8X;
u8 bandwidth = C_BMA2x2_Zero_U8X;
if (p_bma2x2 == BMA2x2_NULL) {
	return E_BMA2x2_NULL_PTR;
	} else {
	if (p_bma2x2->chip_id == 0xFB) {
		if (bw > C_BMA2x2_Seven_U8X &&
		bw < C_BMA2x2_Fifteen_U8X) {
			switch (bw) {
			case BMA2x2_BW_7_81HZ:
				bandwidth = BMA2x2_BW_7_81HZ;

				/*  7.81 Hz      64000 uS   */
			break;
			case BMA2x2_BW_15_63HZ:
				bandwidth = BMA2x2_BW_15_63HZ;

			/*  15.63 Hz     32000 uS   */
			break;
			case BMA2x2_BW_31_25HZ:
				bandwidth = BMA2x2_BW_31_25HZ;

			/*  31.25 Hz     16000 uS   */
			break;
			case BMA2x2_BW_62_50HZ:
				bandwidth = BMA2x2_BW_62_50HZ;

			/*  62.50 Hz     8000 uS   */
			break;
			case BMA2x2_BW_125HZ:
				bandwidth = BMA2x2_BW_125HZ;

			/*  125 Hz       4000 uS   */
			break;
			case BMA2x2_BW_250HZ:
				bandwidth = BMA2x2_BW_250HZ;

			/*  250 Hz       2000 uS   */
			break;
			case BMA2x2_BW_500HZ:
				bandwidth = BMA2x2_BW_500HZ;

			/*  500 Hz       1000 uS   */
			break;
			default:
			break;
			}
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_BANDWIDTH__REG, &data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE(data,
			BMA2x2_BANDWIDTH, bandwidth);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_BANDWIDTH__REG, &data, C_BMA2x2_One_U8X);
			} else {
			return  E_OUT_OF_RANGE;
			}
		} else {
		if (bw > C_BMA2x2_Seven_U8X &&
		bw < C_BMA2x2_Sixteen_U8X) {
			switch (bw) {
			case BMA2x2_BW_7_81HZ:
				bandwidth = BMA2x2_BW_7_81HZ;

			/*  7.81 Hz      64000 uS   */
			break;
			case BMA2x2_BW_15_63HZ:
				bandwidth = BMA2x2_BW_15_63HZ;

			/*  15.63 Hz     32000 uS   */
			break;
			case BMA2x2_BW_31_25HZ:
				bandwidth = BMA2x2_BW_31_25HZ;

			/*  31.25 Hz     16000 uS   */
			break;
			case BMA2x2_BW_62_50HZ:
				bandwidth = BMA2x2_BW_62_50HZ;

			/*  62.50 Hz     8000 uS   */
			break;
			case BMA2x2_BW_125HZ:
				bandwidth = BMA2x2_BW_125HZ;

			/*  125 Hz       4000 uS   */
			break;
			case BMA2x2_BW_250HZ:
			bandwidth = BMA2x2_BW_250HZ;

			/*  250 Hz       2000 uS   */
			break;
			case BMA2x2_BW_500HZ:
				bandwidth = BMA2x2_BW_500HZ;

			/*  500 Hz       1000 uS   */
			break;
			case BMA2x2_BW_1000HZ:
				bandwidth = BMA2x2_BW_1000HZ;

			/*  1000 Hz      500 uS   */
			break;
			default:
			break;
			}
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_BANDWIDTH__REG, &data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_BANDWIDTH, bandwidth);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_BANDWIDTH__REG, &data, C_BMA2x2_One_U8X);
			} else {
			return  E_OUT_OF_RANGE;
			}
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 *	Description: *//**\brief This API is used to get the operating
 *	modes of the sensor in the register 0x11 and 0x12
 *	Register 0x11 - bit from 5 to 7
 *	Register 0x12 - bit from 5 and 6
 *
 *
 *
 *
 *
 *  \param u8 * Mode : Pointer holding the Mode
 *          Mode ->      0 -> NORMAL
 *                       1 -> LOWPOWER1
 *                       2 -> SUSPEND
 *                       3 -> DEEP_SUSPEND
 *                       4 -> LOWPOWER2
 *                       5 -> STANDBY
 *
 *
 *
 * \return results of bus communication function
 *
 *
 *************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_mode(u8 *mode)
{
BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
u8 data1 = C_BMA2x2_Zero_U8X;
u8 data2 = C_BMA2x2_Zero_U8X;
if (p_bma2x2 == BMA2x2_NULL) {
	return E_BMA2x2_NULL_PTR;
	} else {
		comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
		(p_bma2x2->dev_addr, BMA2x2_MODE_CTRL_REG,
		&data1, C_BMA2x2_One_U8X);
		comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
		(p_bma2x2->dev_addr, BMA2x2_LOW_NOISE_CTRL_REG,
		&data2, C_BMA2x2_One_U8X);

		data1  = (data1 & 0xE0) >> 5;
		data2  = (data2 & 0x40) >> 6;

	if ((data1 == 0x00) &&
	(data2 == 0x00)) {
		*mode  = BMA2x2_MODE_NORMAL;
		} else {
		if ((data1 == 0x02) &&
		(data2 == 0x00)) {
			*mode  =
			BMA2x2_MODE_LOWPOWER1;
			} else {
			if ((data1 == 0x04
			|| data1 == 0x06) &&
			(data2 == 0x00)) {
				*mode  =
				BMA2x2_MODE_SUSPEND;
				} else {
				if (((data1 & 0x01) == 0x01)) {
					*mode  =
					BMA2x2_MODE_DEEP_SUSPEND;
					} else {
					if ((data1 == 0x02)
					&& (data2 == 0x01)) {
						*mode  =
						BMA2x2_MODE_LOWPOWER2;
					} else {
					if ((data1 == 0x04) &&
						(data2 == 0x01))
							*mode  =
							BMA2x2_MODE_STANDBY;
					else
						*mode =
						BMA2x2_MODE_DEEP_SUSPEND;
						}
					}
				}
			}
		}
	}
	p_bma2x2->mode = *mode;
return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 *	Description: *//**\brief This API is used to set
 *	the operating Modes of the sensor in the register 0x11 and 0x12
 *	Register 0x11 - bit from 5 to 7
 *	Register 0x12 - bit from 5 and 6
 *
 *
 *
 *
 *  \param u8 Mode: The value of mode
 *                Mode -> 0 -> NORMAL
 *                       1 -> LOWPOWER1
 *                       2 -> SUSPEND
 *                       3 -> DEEP_SUSPEND
 *                       4 -> LOWPOWER2
 *                       5 -> STANDBY
 *
 * \return results of bus communication function
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_mode(u8 mode)
{
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	u8 data1 = C_BMA2x2_Zero_U8X;
	u8 data2 = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		if (mode < C_BMA2x2_Six_U8X) {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_MODE_CTRL_REG,
			&data1, C_BMA2x2_One_U8X);
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_LOW_NOISE_CTRL_REG,
			&data2, C_BMA2x2_One_U8X);
		switch (mode) {
		case BMA2x2_MODE_NORMAL:
			data1  = BMA2x2_SET_BITSLICE
			(data1, BMA2x2_MODE_CTRL,
			C_BMA2x2_Zero_U8X);
			data2  = BMA2x2_SET_BITSLICE
			(data2, BMA2x2_LOW_POWER_MODE,
			C_BMA2x2_Zero_U8X);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr, BMA2x2_MODE_CTRL_REG,
			&data1, C_BMA2x2_One_U8X);
			p_bma2x2->delay_msec(1);
			/*A minimum delay of
			atleast 450us is required for
			the low power modes,
			as per the data sheet.*/
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_LOW_NOISE_CTRL_REG,
			&data2, C_BMA2x2_One_U8X);
		break;
		case BMA2x2_MODE_LOWPOWER1:
			data1  = BMA2x2_SET_BITSLICE
			(data1, BMA2x2_MODE_CTRL,
			C_BMA2x2_Two_U8X);
			data2  = BMA2x2_SET_BITSLICE
			(data2, BMA2x2_LOW_POWER_MODE,
			C_BMA2x2_Zero_U8X);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_MODE_CTRL_REG,
			&data1, C_BMA2x2_One_U8X);
			p_bma2x2->delay_msec(1);
			/*A minimum delay of
			atleast 450us is required
			for the low power
			modes, as per the data sheet.*/
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_LOW_NOISE_CTRL_REG,
			&data2, C_BMA2x2_One_U8X);
		break;
		case BMA2x2_MODE_SUSPEND:
			data1  = BMA2x2_SET_BITSLICE(data1,
			BMA2x2_MODE_CTRL, C_BMA2x2_Four_U8X);
			data2  = BMA2x2_SET_BITSLICE(data2,
			BMA2x2_LOW_POWER_MODE,
			C_BMA2x2_Zero_U8X);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_LOW_NOISE_CTRL_REG,
			&data2, C_BMA2x2_One_U8X);
			p_bma2x2->delay_msec(1);
			/*A minimum delay of
			atleast 450us is required for
			the low power modes,
			as per the data sheet.*/
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr, BMA2x2_MODE_CTRL_REG,
			&data1, C_BMA2x2_One_U8X);
		break;
		case BMA2x2_MODE_DEEP_SUSPEND:
			data1  = BMA2x2_SET_BITSLICE
			(data1, BMA2x2_MODE_CTRL, C_BMA2x2_One_U8X);
			data2  = BMA2x2_SET_BITSLICE
			(data2, BMA2x2_LOW_POWER_MODE, C_BMA2x2_One_U8X);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_MODE_CTRL_REG, &data1, C_BMA2x2_One_U8X);
			p_bma2x2->delay_msec(1);
			/*A minimum delay of atleast 450us is required for
			the low power modes, as per the data sheet.*/
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr, BMA2x2_LOW_NOISE_CTRL_REG,
			&data2, C_BMA2x2_One_U8X);
		break;
		case BMA2x2_MODE_LOWPOWER2:
			data1  = BMA2x2_SET_BITSLICE
			(data1, BMA2x2_MODE_CTRL, C_BMA2x2_Two_U8X);
			data2  = BMA2x2_SET_BITSLICE
			(data2, BMA2x2_LOW_POWER_MODE, C_BMA2x2_One_U8X);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr, BMA2x2_MODE_CTRL_REG,
			&data1, C_BMA2x2_One_U8X);
			p_bma2x2->delay_msec(1);
			/*A minimum delay of atleast 450us is required
			for the low power modes, as per the data sheet.*/
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr, BMA2x2_LOW_NOISE_CTRL_REG,
			&data2, C_BMA2x2_One_U8X);
		break;
		case BMA2x2_MODE_STANDBY:
			data1  = BMA2x2_SET_BITSLICE
			(data1, BMA2x2_MODE_CTRL, C_BMA2x2_Four_U8X);
			data2  = BMA2x2_SET_BITSLICE
			(data2, BMA2x2_LOW_POWER_MODE, C_BMA2x2_One_U8X);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr, BMA2x2_LOW_NOISE_CTRL_REG,
			&data2, C_BMA2x2_One_U8X);
			p_bma2x2->delay_msec(1);
			/*A minimum delay of atleast 450us is required for
			the low power modes, as per the data sheet.*/
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr, BMA2x2_MODE_CTRL_REG,
			&data1, C_BMA2x2_One_U8X);
		break;
		}
	} else {
	return  E_OUT_OF_RANGE;
	}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
 *	Description: *//**\brief This API is used to get
 *	the sleep duration status of the sensor in the register 0x11
 *	Register 0x11 - bit from 0 to 3
 *
 *
 *
 *
 *  \param  u8 *sleep_dur : Pointer holding the sleep_dur time
 *	BMA2x2_SLEEP_DUR_0_5MS        0x05
 *	BMA2x2_SLEEP_DUR_1MS          0x06
 *	BMA2x2_SLEEP_DUR_2MS          0x07
 *	BMA2x2_SLEEP_DUR_4MS          0x08
 *	BMA2x2_SLEEP_DUR_6MS          0x09
 *	BMA2x2_SLEEP_DUR_10MS         0x0A
 *	BMA2x2_SLEEP_DUR_25MS         0x0B
 *	BMA2x2_SLEEP_DUR_50MS         0x0C
 *	BMA2x2_SLEEP_DUR_100MS        0x0D
 *	BMA2x2_SLEEP_DUR_500MS        0x0E
 *	BMA2x2_SLEEP_DUR_1S           0x0F
 *
 *
 *
 * \return results of bus communication function
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 **********************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_sleep_dur(u8 *sleep_dur)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			/*SLEEP DURATION*/
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr, BMA2x2_SLEEP_DUR__REG,
			&data, C_BMA2x2_One_U8X);
			*sleep_dur = BMA2x2_GET_BITSLICE
			(data, BMA2x2_SLEEP_DUR);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
 *	Description: *//**\brief This API is used to set
 *	Sleep Duration of the sensor in the register 0x11
 *	Register 0x11 - bit from 0 to 3
 *
 *
 *
 *
 *  \param u8 sleep_dur: The value of Sleep Duration time
 *        sleep_dur ->
 *	BMA2x2_SLEEP_DUR_0_5MS        0x05
 *	BMA2x2_SLEEP_DUR_1MS          0x06
 *	BMA2x2_SLEEP_DUR_2MS          0x07
 *	BMA2x2_SLEEP_DUR_4MS          0x08
 *	BMA2x2_SLEEP_DUR_6MS          0x09
 *	BMA2x2_SLEEP_DUR_10MS         0x0A
 *	BMA2x2_SLEEP_DUR_25MS         0x0B
 *	BMA2x2_SLEEP_DUR_50MS         0x0C
 *	BMA2x2_SLEEP_DUR_100MS        0x0D
 *	BMA2x2_SLEEP_DUR_500MS        0x0E
 *	BMA2x2_SLEEP_DUR_1S           0x0F
 *
 *
 * \return results of bus communication function
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_sleep_dur(u8 sleep_dur)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	u8 sleep_duration = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		if (sleep_dur > C_BMA2x2_Four_U8X &&
		sleep_dur < C_BMA2x2_Sixteen_U8X) {
			switch (sleep_dur) {
			case BMA2x2_SLEEP_DUR_0_5MS:
				sleep_duration = BMA2x2_SLEEP_DUR_0_5MS;

				/*  0.5 MS   */
			break;
			case BMA2x2_SLEEP_DUR_1MS:
				sleep_duration = BMA2x2_SLEEP_DUR_1MS;

				/*  1 MS  */
			break;
			case BMA2x2_SLEEP_DUR_2MS:
				sleep_duration = BMA2x2_SLEEP_DUR_2MS;

				/*  2 MS  */
			break;
			case BMA2x2_SLEEP_DUR_4MS:
				sleep_duration = BMA2x2_SLEEP_DUR_4MS;

				/*  4 MS   */
			break;
			case BMA2x2_SLEEP_DUR_6MS:
				sleep_duration = BMA2x2_SLEEP_DUR_6MS;

				/*  6 MS  */
			break;
			case BMA2x2_SLEEP_DUR_10MS:
				sleep_duration = BMA2x2_SLEEP_DUR_10MS;

				/*  10 MS  */
			break;
			case BMA2x2_SLEEP_DUR_25MS:
				sleep_duration = BMA2x2_SLEEP_DUR_25MS;

				/*  25 MS  */
			break;
			case BMA2x2_SLEEP_DUR_50MS:
				sleep_duration = BMA2x2_SLEEP_DUR_50MS;

				/*  50 MS   */
			break;
			case BMA2x2_SLEEP_DUR_100MS:
				sleep_duration = BMA2x2_SLEEP_DUR_100MS;

				/*  100 MS  */
			break;
			case BMA2x2_SLEEP_DUR_500MS:
				sleep_duration = BMA2x2_SLEEP_DUR_500MS;

				/*  500 MS   */
			break;
			case BMA2x2_SLEEP_DUR_1S:
				sleep_duration = BMA2x2_SLEEP_DUR_1S;

				/*  1 SECS   */
			break;
			default:
			break;
			}
			/*SLEEP DURATION*/
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr, BMA2x2_SLEEP_DUR__REG,
			&data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_SLEEP_DUR, sleep_duration);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr, BMA2x2_SLEEP_DUR__REG,
			&data, C_BMA2x2_One_U8X);
		} else {
		return  E_OUT_OF_RANGE;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
 * Description: *//**\brief This API is used to get the sleep timer mode status
 *			in the register 0x12 bit 5
 *
 *
 *
 *
 *  \param  u8 *sleep_tmr : Pointer holding the sleep_tmr
 *                  sleep_tmr -> [0:1]
 *                  0 => enable EventDrivenSampling(EDT)
 *                  1 => enable Equidistant sampling mode(EST)
 *
 *
 *
 * \return results of bus communication function
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 **************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_sleeptmr_mode(u8 *sleep_tmr)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			/*SLEEP TIMER MODE*/
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr, BMA2x2_SLEEP_TIMER__REG,
			&data, C_BMA2x2_One_U8X);
			*sleep_tmr = BMA2x2_GET_BITSLICE
			(data, BMA2x2_SLEEP_TIMER);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/**************************************************************************
 * Description: *//**\brief This API is used to set
 *   the sleep timer mode status in the register 0x12 bit 5
 *
 *
 *
 *
 *  \param u8 sleep_tmr:	The value of sleep timer mode status
 *                  sleep_tmr -> [0:1]
 *                  0 => enable EventDrivenSampling(EDT)
 *                  1 => enable Equidistant sampling mode(EST)
 *
 *
 *
 * \return results of bus communication function
 *
 *
 **************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 **************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_sleeptmr_mode(u8 sleep_tmr)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		if (sleep_tmr < C_BMA2x2_Two_U8X) {
			/*SLEEP TIMER MODE*/
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr, BMA2x2_SLEEP_TIMER__REG,
			&data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_SLEEP_TIMER, sleep_tmr);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr, BMA2x2_SLEEP_TIMER__REG,
			&data, C_BMA2x2_One_U8X);
		} else {
		return  E_OUT_OF_RANGE;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 * Description: *//**\brief This API is used to get high bandwidth
 *		in the register 0x13 bit 7
 *
 *
 *
 *  \param u8 *high_bw : Pointer holding the high_bw
 *       high_bw ->  1 -> Unfiltered High Bandwidth
 *                   0 -> Filtered Low Bandwidth
 *
 *
 * \return results of bus communication function
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_high_bw(u8 *high_bw)
{
		BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
		u8 data = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return  E_BMA2x2_NULL_PTR;
		} else {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr, BMA2x2_EN_DATA_HIGH_BW__REG,
			&data, C_BMA2x2_One_U8X);
			*high_bw = BMA2x2_GET_BITSLICE
			(data, BMA2x2_EN_DATA_HIGH_BW);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
 * Description: *//**\brief This API is used to set high bandwidth
 *			in the register 0x13 bit 7
 *
 *
 *
 *  \param u8 high_bw: The value of high bandwidth
 *       high_bw ->   1 -> Unfiltered High Bandwidth
 *                    0 -> Filtered Low Bandwidth
 *
 *
 *
 *
 * \return results of bus communication function
 *
 *
 **************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_high_bw(u8 high_bw)
{
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	u8 data = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return  E_BMA2x2_NULL_PTR;
		}  else {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr, BMA2x2_EN_DATA_HIGH_BW__REG,
			&data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE(data,
			BMA2x2_EN_DATA_HIGH_BW, high_bw);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_DATA_HIGH_BW__REG,
			&data, C_BMA2x2_One_U8X);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 * Description: *//**\brief This API is used to get shadow dis
 *		in the register 0x13 bit 6
 *
 *
 *
 *  \param u8 *shadow_dis : Pointer holding the shadow_dis
 *           shadow_dis -> 1 -> No MSB Locking
 *                         0 -> MSB is Locked
 *
 *
 *
 * \return results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_shadow_dis(u8 *shadow_dis)
{
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	u8 data = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return  E_BMA2x2_NULL_PTR;
		} else {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_DIS_SHADOW_PROC__REG,
			&data, C_BMA2x2_One_U8X);
			*shadow_dis = BMA2x2_GET_BITSLICE
			(data, BMA2x2_DIS_SHADOW_PROC);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/**************************************************************************
 * Description: *//**\brief This API is used to set shadow dis
 *		in the register 0x13 bit 6
 *
 *
 *
 *  \param u8 shadow_dis: The value of shadow dis
 *          shadow_dis ->   1 -> No MSB Locking
 *                          0 -> MSB is Locked
 *
 *
 *
 *
 * \return results of bus communication function
 *
 *
 **************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_shadow_dis(u8 shadow_dis)
{

	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	u8 data = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return  E_BMA2x2_NULL_PTR;
		} else {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr, BMA2x2_DIS_SHADOW_PROC__REG,
			&data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_DIS_SHADOW_PROC, shadow_dis);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr, BMA2x2_DIS_SHADOW_PROC__REG,
			&data, C_BMA2x2_One_U8X);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief
 *                      This function is used for the soft reset
 *     The soft reset register will be written with 0xB6 in the register 0x14.
 *
 *
 *
 *  \param None
 *
 *
 *
 * \return results of bus communication function
 *
 *
 **************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_soft_reset(void)
{
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	u8 data = BMA2x2_EN_SOFT_RESET_VALUE;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		}  else {
			comres = p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr, BMA2x2_RESET_REG,
			&data, C_BMA2x2_One_U8X);
			/* To reset the sensor 0xB6 value will be written */
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/**************************************************************************
 * Description: *//**\brief This API is used to update the register values
 *
 *
 *
 *
 *  param:  None
 *
 *
 *
 * \return results of bus communication function
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_update_image(void)
{
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	u8 data = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return  E_BMA2x2_NULL_PTR;
		} else {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr, BMA2x2_UPDATE_IMAGE__REG,
			&data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_UPDATE_IMAGE, C_BMA2x2_One_U8X);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr, BMA2x2_UPDATE_IMAGE__REG,
			&data, C_BMA2x2_One_U8X);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 * Description: *//**\brief This API is used to get
 *  interrupt enable bits of the sensor in the registers 0x16 and 0x17
 *
 *
 *
 *
 *  \param u8 *InterruptType: The pointer holding
 *               the  interrupt type
 *                        0 -> Low_G_Interrupt
 *                        1 -> High_G_X_Interrupt
 *                        2 -> High_G_Y_Interrupt
 *                        3 -> High_G_Z_Interrupt
 *                        4 -> DATA_EN
 *                        5 -> Slope_X_Interrupt
 *                        6 -> Slope_Y_Interrupt
 *                        7 -> Slope_Z_Interrupt
 *                        8 -> Single_Tap_Interrupt
 *                        9 -> Double_Tap_Interrupt
 *                       10 -> Orient_Interrupt
 *                       11 -> Flat_Interrupt
 *       u8 value: The value of interrupts enable
 *					1 -> enable interrupt
 *					0 -> disable interrupt
 *
 *
 *
 *
 * \return results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 **************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_Int_Enable(u8 interrupttype,
u8 *value)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (interrupttype) {
		case BMA2x2_Low_G_Interrupt:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_LOWG_INT__REG,
			&data, C_BMA2x2_One_U8X);
			*value = BMA2x2_GET_BITSLICE
			(data, BMA2x2_EN_LOWG_INT);
		break;
		case BMA2x2_High_G_X_Interrupt:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_HIGHG_X_INT__REG,
			&data, C_BMA2x2_One_U8X);
			*value = BMA2x2_GET_BITSLICE
			(data, BMA2x2_EN_HIGHG_X_INT);
		break;
		case BMA2x2_High_G_Y_Interrupt:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_HIGHG_Y_INT__REG,
			&data, C_BMA2x2_One_U8X);
			*value = BMA2x2_GET_BITSLICE
			(data, BMA2x2_EN_HIGHG_Y_INT);
		break;
		case BMA2x2_High_G_Z_Interrupt:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_HIGHG_Z_INT__REG,
			&data, C_BMA2x2_One_U8X);
			*value = BMA2x2_GET_BITSLICE
			(data, BMA2x2_EN_HIGHG_Z_INT);
		break;
		case BMA2x2_DATA_EN:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_NEW_DATA_INT__REG,
			&data, C_BMA2x2_One_U8X);
			*value = BMA2x2_GET_BITSLICE
			(data, BMA2x2_EN_NEW_DATA_INT);
		break;
		case BMA2x2_Slope_X_Interrupt:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_SLOPE_X_INT__REG,
			&data, C_BMA2x2_One_U8X);
			*value = BMA2x2_GET_BITSLICE
			(data, BMA2x2_EN_SLOPE_X_INT);
		break;
		case BMA2x2_Slope_Y_Interrupt:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_SLOPE_Y_INT__REG,
			&data, C_BMA2x2_One_U8X);
			*value = BMA2x2_GET_BITSLICE
			(data, BMA2x2_EN_SLOPE_Y_INT);
		break;
		case BMA2x2_Slope_Z_Interrupt:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_SLOPE_Z_INT__REG,
			&data, C_BMA2x2_One_U8X);
			*value = BMA2x2_GET_BITSLICE
			(data, BMA2x2_EN_SLOPE_Z_INT);
		break;
		case BMA2x2_Single_Tap_Interrupt:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_SINGLE_TAP_INT__REG,
			&data, C_BMA2x2_One_U8X);
			*value = BMA2x2_GET_BITSLICE
			(data, BMA2x2_EN_SINGLE_TAP_INT);
		break;
		case BMA2x2_Double_Tap_Interrupt:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_DOUBLE_TAP_INT__REG,
			&data, C_BMA2x2_One_U8X);
			*value = BMA2x2_GET_BITSLICE
			(data, BMA2x2_EN_DOUBLE_TAP_INT);
		break;
		case BMA2x2_Orient_Interrupt:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_ORIENT_INT__REG,
			&data, C_BMA2x2_One_U8X);
			*value = BMA2x2_GET_BITSLICE
			(data, BMA2x2_EN_ORIENT_INT);
		break;
		case BMA2x2_Flat_Interrupt:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_FLAT_INT__REG,
			&data, C_BMA2x2_One_U8X);
			*value = BMA2x2_GET_BITSLICE
			(data, BMA2x2_EN_FLAT_INT);
		break;
		default:
		comres = E_OUT_OF_RANGE;
		break;
		}
	}
	return comres;
}
/****************************************************************************
 * Description: *//**\brief This API is used to set
 *  interrupt enable bits of the sensor in the registers 0x16 and 0x17
 *
 *
 *
 *
 *  \param u8 interrupttype: The value of interrupt type
 *                        0 -> Low_G_Interrupt
 *                        1 -> High_G_X_Interrupt
 *                        2 -> High_G_Y_Interrupt
 *                        3 -> High_G_Z_Interrupt
 *                        4 -> DATA_EN
 *                        5 -> Slope_X_Interrupt
 *                        6 -> Slope_Y_Interrupt
 *                        7 -> Slope_Z_Interrupt
 *                        8 -> Single_Tap_Interrupt
 *                        9 -> Double_Tap_Interrupt
 *                       10 -> Orient_Interrupt
 *                       11 -> Flat_Interrupt
 *       u8 value: The value of interrupts enable
 *				1 -> enable interrupt
 *				0 -> disable interrupt
 *
 *
 *
 * \return results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 **************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_Int_Enable(u8 interrupttype,
u8 value)
{
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	u8 data1 = C_BMA2x2_Zero_U8X;
	u8 data2 = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
		(p_bma2x2->dev_addr, BMA2x2_INT_ENABLE1_REG,
		&data1, C_BMA2x2_One_U8X);
		comres += p_bma2x2->BMA2x2_BUS_READ_FUNC
		(p_bma2x2->dev_addr, BMA2x2_INT_ENABLE2_REG,
		&data2, C_BMA2x2_One_U8X);
		value = value & C_BMA2x2_One_U8X;
		switch (interrupttype) {
		case BMA2x2_Low_G_Interrupt:
			/* Low G Interrupt  */
			data2 = BMA2x2_SET_BITSLICE(data2,
			BMA2x2_EN_LOWG_INT, value);
		break;
		case BMA2x2_High_G_X_Interrupt:
			/* High G X Interrupt */
			data2 = BMA2x2_SET_BITSLICE(data2,
			BMA2x2_EN_HIGHG_X_INT, value);
		break;
		case BMA2x2_High_G_Y_Interrupt:
			/* High G Y Interrupt */
			data2 = BMA2x2_SET_BITSLICE(data2,
			BMA2x2_EN_HIGHG_Y_INT, value);
		break;
		case BMA2x2_High_G_Z_Interrupt:
			/* High G Z Interrupt */
			data2 = BMA2x2_SET_BITSLICE(data2,
			BMA2x2_EN_HIGHG_Z_INT, value);
		break;
		case BMA2x2_DATA_EN:
			/*Data En Interrupt  */
			data2 = BMA2x2_SET_BITSLICE(data2,
			BMA2x2_EN_NEW_DATA_INT, value);
		break;
		case BMA2x2_Slope_X_Interrupt:
			/* Slope X Interrupt */
			data1 = BMA2x2_SET_BITSLICE(data1,
			BMA2x2_EN_SLOPE_X_INT, value);
		break;
		case BMA2x2_Slope_Y_Interrupt:
			/* Slope Y Interrupt */
			data1 = BMA2x2_SET_BITSLICE(data1,
			BMA2x2_EN_SLOPE_Y_INT, value);
		break;
		case BMA2x2_Slope_Z_Interrupt:
			/* Slope Z Interrupt */
			data1 = BMA2x2_SET_BITSLICE(data1,
			BMA2x2_EN_SLOPE_Z_INT, value);
		break;
		case BMA2x2_Single_Tap_Interrupt:
			/* Single Tap Interrupt */
			data1 = BMA2x2_SET_BITSLICE(data1,
				BMA2x2_EN_SINGLE_TAP_INT, value);
		break;
		case BMA2x2_Double_Tap_Interrupt:
			/* Double Tap Interrupt */
			data1 = BMA2x2_SET_BITSLICE(data1,
				BMA2x2_EN_DOUBLE_TAP_INT, value);
		break;
		case BMA2x2_Orient_Interrupt:
			/* Orient Interrupt  */
			data1 = BMA2x2_SET_BITSLICE(data1,
			BMA2x2_EN_ORIENT_INT, value);
		break;
		case BMA2x2_Flat_Interrupt:
			/* Flat Interrupt */
			data1 = BMA2x2_SET_BITSLICE(data1,
			BMA2x2_EN_FLAT_INT, value);
		break;
		default:
			comres = E_OUT_OF_RANGE;
		break;
		}
		comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
		(p_bma2x2->dev_addr, BMA2x2_INT_ENABLE1_REG,
		&data1, C_BMA2x2_One_U8X);
		comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
		(p_bma2x2->dev_addr, BMA2x2_INT_ENABLE2_REG,
		&data2, C_BMA2x2_One_U8X);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 * Description: *//**\brief This API is used to get
 * the interrupt fifo full enable status in the register 0x17 bit 5
 *
 *
 *
 *
 *  \param u8 *ffull :Pointer holding the FIFO Full status bit
 *                    FIFO Full -> [0:1]
 *                              0 --> Clear
 *                              1 --> Set
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_int_ffull(u8 *ffull)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr, BMA2x2_INT_FFULL_EN_INT__REG,
			&data, C_BMA2x2_One_U8X);
			*ffull = BMA2x2_GET_BITSLICE(data,
			BMA2x2_INT_FFULL_EN_INT);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 * Description: *//**\brief This API is used to set
 * the interrupt ffull enable status in the register 0x17 bit 5
 *
 *
 *
 *
 *  \param u8 ffull: The value of FIFO full status bit
 *            FIFO Full -> [0:1]
 *                      0 --> Clear
 *                      1 --> Set
 *
 *
 *  \return  results of bus communication function communication results
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_int_ffull(u8 ffull)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		if (ffull < C_BMA2x2_Two_U8X) {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr, BMA2x2_INT_FFULL_EN_INT__REG,
			&data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_INT_FFULL_EN_INT, ffull);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr, BMA2x2_INT_FFULL_EN_INT__REG,
			&data, C_BMA2x2_One_U8X);
		} else {
		return  E_OUT_OF_RANGE;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 * Description: *//**\brief This API is used to get
 * the interrupt fwm enable status in the register 0x17 bit 6
 *
 *
 *
 *
 *  \param u8 *fwm : Pointer holding the FIFO Water Mark
 *                   fwm -> [0:1]
 *                           0 --> Clear
 *                           1 --> Set
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_int_fwm(u8 *fwm)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr, BMA2x2_INT_FWM_EN_INT__REG,
			&data, C_BMA2x2_One_U8X);
			*fwm = BMA2x2_GET_BITSLICE
			(data, BMA2x2_INT_FWM_EN_INT);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 * Description: *//**\brief This API is used to set
 * the interrupt fwm enable status in the register 0x17 bit 6
 *
 *
 *
 *
 *  \param u8 fwm: The value of FIFO water mark
 *        FIFO Water Mark -> [0:1]
 *                0 --> Clear
 *                1 --> Set
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 **************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_int_fwm(u8 fwm)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		if (fwm < C_BMA2x2_Two_U8X) {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr, BMA2x2_INT_FWM_EN_INT__REG,
			&data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_INT_FWM_EN_INT, fwm);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr, BMA2x2_INT_FWM_EN_INT__REG,
			&data, C_BMA2x2_One_U8X);
		} else {
		return  E_OUT_OF_RANGE;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 * Description: *//**\brief This API is used to get
 * the status of slow/no motion interrupt in the register 0x18.
 * bit from 0 to 3
 *
 *
 *
 *
 *  \param u8 channel:
 *          The value of slow/no motion channel
 *           channel -->
 *           BMA2x2_ACCEL_SLO_NO_MOT_EN_X     ->     0
 *           BMA2x2_ACCEL_SLO_NO_MOT_EN_Y     ->     1
 *           BMA2x2_ACCEL_SLO_NO_MOT_EN_Z     ->     2
 *           BMA2x2_ACCEL_SLO_NO_MOT_EN_SEL   ->     3
 *      u8 *slo_data: Pointer holding the slow/no motion status
 *           slo_data
 *				enable --> 1
 *				disable --> 0
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_slo_no_mot(u8 channel,
u8 *slo_data)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (channel) {
		case BMA2x2_SLO_NO_MOT_EN_X:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_INT_SLO_NO_MOT_EN_X_INT__REG,
			&data, C_BMA2x2_One_U8X);
			*slo_data = BMA2x2_GET_BITSLICE
			(data, BMA2x2_INT_SLO_NO_MOT_EN_X_INT);
		break;
		case BMA2x2_SLO_NO_MOT_EN_Y:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_INT_SLO_NO_MOT_EN_Y_INT__REG,
			&data, C_BMA2x2_One_U8X);
			*slo_data = BMA2x2_GET_BITSLICE
			(data, BMA2x2_INT_SLO_NO_MOT_EN_Y_INT);
		break;
		case BMA2x2_SLO_NO_MOT_EN_Z:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_INT_SLO_NO_MOT_EN_Z_INT__REG,
			&data, C_BMA2x2_One_U8X);
			*slo_data = BMA2x2_GET_BITSLICE
			(data, BMA2x2_INT_SLO_NO_MOT_EN_Z_INT);
		break;
		case BMA2x2_SLO_NO_MOT_EN_SEL:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_INT_SLO_NO_MOT_EN_SEL_INT__REG,
			&data, C_BMA2x2_One_U8X);
			*slo_data = BMA2x2_GET_BITSLICE
			(data, BMA2x2_INT_SLO_NO_MOT_EN_SEL_INT);
		break;
		default:
			comres = E_OUT_OF_RANGE;
		break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
 * Description: *//**\brief This API is used to set
 * the status of slow/no motion interrupt in the register 0x18.
 * bit from 0 to 3
 *
 *
 *
 *
 *  \param  u8 channel:
 *          The value of slow/no motion channel
 *           channel -->
 *           BMA2x2_ACCEL_SLO_NO_MOT_EN_X     ->     0
 *           BMA2x2_ACCEL_SLO_NO_MOT_EN_Y     ->     1
 *           BMA2x2_ACCEL_SLO_NO_MOT_EN_Z     ->     2
 *           BMA2x2_ACCEL_SLO_NO_MOT_EN_SEL   ->     3
 *      u8 *slo_data: The value of slow/no motion status
 *           slo_data
 *				enable --> 1
 *				disable --> 0
 *
 *
 *
 *  \return  results of bus communication function communication results
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 **************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_slo_no_mot(u8 channel,
u8 slo_data)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (channel) {
		case BMA2x2_SLO_NO_MOT_EN_X:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_INT_SLO_NO_MOT_EN_X_INT__REG,
			&data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_INT_SLO_NO_MOT_EN_X_INT, slo_data);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_INT_SLO_NO_MOT_EN_X_INT__REG,
			&data, C_BMA2x2_One_U8X);
		break;
		case BMA2x2_SLO_NO_MOT_EN_Y:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_INT_SLO_NO_MOT_EN_Y_INT__REG,
			&data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_INT_SLO_NO_MOT_EN_Y_INT, slo_data);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_INT_SLO_NO_MOT_EN_Y_INT__REG,
			&data, C_BMA2x2_One_U8X);
		break;
		case BMA2x2_SLO_NO_MOT_EN_Z:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_INT_SLO_NO_MOT_EN_Z_INT__REG,
			&data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_INT_SLO_NO_MOT_EN_Z_INT, slo_data);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_INT_SLO_NO_MOT_EN_Z_INT__REG,
			&data, C_BMA2x2_One_U8X);
		break;
		case BMA2x2_SLO_NO_MOT_EN_SEL:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_INT_SLO_NO_MOT_EN_SEL_INT__REG,
			&data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_INT_SLO_NO_MOT_EN_SEL_INT, slo_data);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_INT_SLO_NO_MOT_EN_SEL_INT__REG,
			&data, C_BMA2x2_One_U8X);
		break;
		default:
			comres = E_OUT_OF_RANGE;
		break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 * Description: *//**\brief  This API is used to get
 * the status of low interrupt in the register 0x19 and 0x1B
 * INT1 -> register 0x19 bit 0
 * INT2 -> register 0x1B bit 0
 *
 *
 *
 *
 * param u8 channel : The value of low interrupt channel
 *                       channel -->
 *                       BMA2x2_ACCEL_INT1_LOWG     ->    0
 *                        BMA2x2_ACCEL_INT2_LOWG     ->    1
 *  u8 *int_low : Pointer holding the low interrupt
 *                       int_low
 *							enable --> 1
 *							disable --> 0
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 **************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_int_low(u8 channel,
u8 *int_low)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (channel) {
		case BMA2x2_INT1_LOWG:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_INT1_PAD_LOWG__REG,
			&data, C_BMA2x2_One_U8X);
			*int_low = BMA2x2_GET_BITSLICE
			(data, BMA2x2_EN_INT1_PAD_LOWG);
		break;
		case BMA2x2_INT2_LOWG:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_INT2_PAD_LOWG__REG,
			&data, C_BMA2x2_One_U8X);
			*int_low = BMA2x2_GET_BITSLICE
			(data, BMA2x2_EN_INT2_PAD_LOWG);
		break;
		default:
			comres = E_OUT_OF_RANGE;
		break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 * Description: *//**\brief This API is used to set
 * the status of low interrupt in the register 0x19 and 0x1B
 * INT1 -> register 0x19 bit 0
 * INT2 -> register 0x1B bit 0
 *
 *
 *
 *
 * param u8 channel : The value of low interrupt channel
 *                       channel -->
 *                       BMA2x2_ACCEL_INT1_LOWG     ->    0
 *                       BMA2x2_ACCEL_INT2_LOWG     ->    1
 *  u8 *int_low : The value of low interrupt
 *                       int_low
 *							enable --> 1
 *							disable --> 0
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
*****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_int_low(u8 channel,
u8 int_low)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (channel) {
		case BMA2x2_INT1_LOWG:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_INT1_PAD_LOWG__REG,
			&data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE(data,
			BMA2x2_EN_INT1_PAD_LOWG, int_low);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_INT1_PAD_LOWG__REG,
			&data, C_BMA2x2_One_U8X);
		break;
		case BMA2x2_INT2_LOWG:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_INT2_PAD_LOWG__REG,
			&data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_EN_INT2_PAD_LOWG, int_low);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_INT2_PAD_LOWG__REG,
			&data, C_BMA2x2_One_U8X);
		break;
		default:
			comres = E_OUT_OF_RANGE;
		break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 * Description: *//**\brief This API is used to get
 * the status of high interrupt in the register 0x19 and 0x1B
 * INT1 -> register 0x19 bit 1
 * INT2 -> register 0x1B bit 1
 *
 *
 *
 *
 *  \param u8 channel: The value of high interrupt channel
 *                           channel -->
 *                           BMA2x2_ACCEL_INT1_HIGHG     ->    0
 *                           BMA2x2_ACCEL_INT2_HIGHG     ->    1
 *       u8 *int_high: Pointer holding the  high interrupt
 *                           int_high
 *                           enable --> 1
 *							 disable --> 0
 *
 *
 *  \return  results of bus communication function
 *
 *
 *************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_int_high(u8 channel,
u8 *int_high)
{

	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (channel) {
		case BMA2x2_INT1_HIGHG:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_INT1_PAD_HIGHG__REG,
			&data, C_BMA2x2_One_U8X);
			*int_high = BMA2x2_GET_BITSLICE
			(data, BMA2x2_EN_INT1_PAD_HIGHG);
		break;
		case BMA2x2_INT2_HIGHG:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_INT2_PAD_HIGHG__REG,
			&data, C_BMA2x2_One_U8X);
			*int_high = BMA2x2_GET_BITSLICE
			(data, BMA2x2_EN_INT2_PAD_HIGHG);
		break;
		default:
		comres = E_OUT_OF_RANGE;
		break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
 * Description: *//**\brief This API is used to set
 * the status of high interrupt in the register 0x19 and 0x1B
 * INT1 -> register 0x19 bit 1
 * INT2 -> register 0x1B bit 1
 *
 *
 *
 *
 *  \param u8 channel: The value of high interrupt channel
 *                           channel -->
 *                           BMA2x2_ACCEL_INT1_HIGHG     ->    0
 *                           BMA2x2_ACCEL_INT2_HIGHG     ->    1
 *       u8 *int_high: The value of high interrupt
 *                           int_high
 *                           enable --> 1
 *							 disable --> 0
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 **************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 **************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_int_high(u8 channel,
u8 int_high)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (channel) {
		case BMA2x2_INT1_HIGHG:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_INT1_PAD_HIGHG__REG,
			&data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_EN_INT1_PAD_HIGHG, int_high);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr, BMA2x2_EN_INT1_PAD_HIGHG__REG,
			&data, C_BMA2x2_One_U8X);
		break;
		case BMA2x2_INT2_HIGHG:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_INT2_PAD_HIGHG__REG,
			&data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_EN_INT2_PAD_HIGHG, int_high);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_INT2_PAD_HIGHG__REG,
			&data, C_BMA2x2_One_U8X);
		break;
		default:
		comres = E_OUT_OF_RANGE;
		break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 * Description: *//**\brief This API is used to get
 * the status of slope interrupt in the register 0x19 and 0x1B
 * INT1 -> register 0x19 bit 2
 * INT2 -> register 0x1B bit 2
 *
 *
 *
 ** \param u8 channel: Pointer holding the slope channel number
 *	channel -->BMA2x2_ACCEL_INT1_SLOPE ->    0
 *	BMA2x2_ACCEL_INT2_SLOPE ->    1
 *
 *	u8 *int_slope: Pointer holding the slope value
 *	int_slope
 *	enable --> 1
 *	disable --> 0
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 **************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_int_slope(u8 channel,
u8 *int_slope)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (channel) {
		case BMA2x2_INT1_SLOPE:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_INT1_PAD_SLOPE__REG,
			&data, C_BMA2x2_One_U8X);
			*int_slope = BMA2x2_GET_BITSLICE
			(data, BMA2x2_EN_INT1_PAD_SLOPE);
		break;
		case BMA2x2_INT2_SLOPE:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_INT2_PAD_SLOPE__REG,
			&data, C_BMA2x2_One_U8X);
			*int_slope = BMA2x2_GET_BITSLICE
			(data, BMA2x2_EN_INT2_PAD_SLOPE);
		break;
		default:
		comres = E_OUT_OF_RANGE;
		break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*************************************************************************
 * Description: *//**\brief This API is used to set
 * the status of slope interrupt in the register 0x19 and 0x1B
 * INT1 -> register 0x19 bit 2
 * INT2 -> register 0x1B bit 2
 *
 *
 *
 *
 * \param u8 channel:The value of slope channel number
 *	channel -->BMA2x2_ACCEL_INT1_SLOPE     ->    0
 *	BMA2x2_ACCEL_INT2_SLOPE     ->    1
 *
 *  u8  int_slope: The slope status value
 *	int_slope
 *	enable --> 1
 *	disable --> 0
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 **************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***********************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_int_slope(u8 channel,
u8 int_slope)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (channel) {
		case BMA2x2_INT1_SLOPE:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_INT1_PAD_SLOPE__REG,
			&data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_EN_INT1_PAD_SLOPE, int_slope);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_INT1_PAD_SLOPE__REG,
			&data, C_BMA2x2_One_U8X);
		break;
		case BMA2x2_INT2_SLOPE:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_INT2_PAD_SLOPE__REG,
			&data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_EN_INT2_PAD_SLOPE, int_slope);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_INT2_PAD_SLOPE__REG,
			&data, C_BMA2x2_One_U8X);
		break;
		default:
			comres = E_OUT_OF_RANGE;
		break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
 * Description: *//**\brief This API is used to get
 * the status of slow/no motion interrupt in the register 0x19 and 0x1B
 * INT1 -> register 0x19 bit 3
 * INT2 -> register 0x1B bit 3
 *
 *
 *
 *
 * \param u8 channel:
 *	The value of slow/no motion channel number
 *	channel -->BMA2x2_ACCEL_INT1_SLO_NO_MOT     ->    0
 *	BMA2x2_ACCEL_INT2_SLO_NO_MOT    ->    1
 *
 *	u8 *int_slope: Pointer holding the slow/no value
 *	int_slo_no_mot
 *	enable --> 1
 *	disable --> 0
 *  \return  results of bus communication function
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 **************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_int_slo_no_mot(u8 channel,
u8 *int_slo_no_mot)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (channel) {
		case BMA2x2_INT1_SLO_NO_MOT:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_INT1_PAD_SLO_NO_MOT__REG,
			&data, C_BMA2x2_One_U8X);
			*int_slo_no_mot = BMA2x2_GET_BITSLICE
			(data, BMA2x2_EN_INT1_PAD_SLO_NO_MOT);
		break;
		case BMA2x2_INT2_SLO_NO_MOT:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_INT2_PAD_SLO_NO_MOT__REG,
			&data, C_BMA2x2_One_U8X);
			*int_slo_no_mot = BMA2x2_GET_BITSLICE
			(data, BMA2x2_EN_INT2_PAD_SLO_NO_MOT);
		break;
		default:
			comres = E_OUT_OF_RANGE;
		break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
 * Description: *//**\brief This API is used to set
 * the status of slow/no motion interrupt in the register 0x19 and 0x1B
 * INT1 -> register 0x19 bit 3
 * INT2 -> register 0x1B bit 3
 *
 *
 *
 *
 *	\param u8 channel:The value of slow/no motion channel number
 *	channel -->BMA2x2_ACCEL_INT1_SLO_NO_MOT     ->    0
 *	BMA2x2_ACCEL_INT2_SLO_NO_MOT     ->    1
 *
 *	u8 int_slo_no_mot:The slow/no motion status value
 *	int_slo_no_mot
 *	enable --> 1
 *	disable --> 0
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 **************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_int_slo_no_mot(u8 channel,
u8 int_slo_no_mot)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (channel) {
		case BMA2x2_INT1_SLO_NO_MOT:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_INT1_PAD_SLO_NO_MOT__REG,
			&data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_EN_INT1_PAD_SLO_NO_MOT, int_slo_no_mot);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_INT1_PAD_SLO_NO_MOT__REG,
			&data, C_BMA2x2_One_U8X);
		break;
		case BMA2x2_INT2_SLO_NO_MOT:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_INT2_PAD_SLO_NO_MOT__REG,
			&data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_EN_INT2_PAD_SLO_NO_MOT, int_slo_no_mot);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_INT2_PAD_SLO_NO_MOT__REG,
			&data, C_BMA2x2_One_U8X);
		break;
		default:
			comres = E_OUT_OF_RANGE;
		break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
 * Description: *//**\brief This API is used to get
 *  the status of double tap interrupt in the register 0x19 and 0x1B
 * INT1 -> register 0x19 bit 4
 * INT2 -> register 0x1B bit 4
 *
 *
 *
 *
 * \param u8 channel:
 *	The value of double tap channel number
 *	channel -->BMA2x2_ACCEL_INT1_DTAP     ->    0
 *	BMA2x2_ACCEL_INT2_DTAP     ->    1
 *
 *	u8 *int_d_tape:
 *	Pointer holding the double tap interrupt value
 *	int_d_tap
 *	enable --> 1
 *	disable --> 0
 *
 *
 *  \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 **************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_int_d_tap(u8 channel,
u8 *int_d_tap)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (channel) {
		case BMA2x2_INT1_DTAP:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_INT1_PAD_DB_TAP__REG,
			&data, C_BMA2x2_One_U8X);
			*int_d_tap = BMA2x2_GET_BITSLICE
			(data, BMA2x2_EN_INT1_PAD_DB_TAP);
		break;
		case BMA2x2_INT2_DTAP:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_INT2_PAD_DB_TAP__REG,
			&data, C_BMA2x2_One_U8X);
			*int_d_tap = BMA2x2_GET_BITSLICE
			(data, BMA2x2_EN_INT2_PAD_DB_TAP);
		break;
		default:
			comres = E_OUT_OF_RANGE;
		break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
 * Description: *//**\brief This API is used to set
 * the status of double tap interrupt in the register 0x19 and 0x1B
 * INT1 -> register 0x19 bit 4
 * INT2 -> register 0x1B bit 4
 *
 *
 *
 *
 * \param u8 channel:
 *	The value of double tap interrupt channel number
 *  channel --> BMA2x2_ACCEL_INT1_DTAP     ->    0
 *              BMA2x2_ACCEL_INT2_DTAP     ->    1
 *
 *	u8  int_d_tap:
 *	The double tap interrupt status value
 *	int_d_tap
 *	enable --> 1
 *	disable --> 0
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 **************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_int_d_tap(u8 channel,
u8 int_d_tap)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (channel) {
		case BMA2x2_INT1_DTAP:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_INT1_PAD_DB_TAP__REG,
			&data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_EN_INT1_PAD_DB_TAP, int_d_tap);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_INT1_PAD_DB_TAP__REG,
			&data, C_BMA2x2_One_U8X);
		break;
		case BMA2x2_INT2_DTAP:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_INT2_PAD_DB_TAP__REG,
			&data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_EN_INT2_PAD_DB_TAP, int_d_tap);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_INT2_PAD_DB_TAP__REG,
			&data, C_BMA2x2_One_U8X);
		break;
		default:
		comres = E_OUT_OF_RANGE;
		break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
 * Description: *//**\brief This API is used to get
 * the status of single tap interrupt in the register 0x19 and 0x1B
 * INT1 -> register 0x19 bit 5
 * INT2 -> register 0x1B bit 5
 *
 *
 *
 *
 *	\param u8 channel:
 *	The value of single tap interrupt channel number
 *	channel -->	BMA2x2_ACCEL_INT1_STAP ->    0
 *				BMA2x2_ACCEL_INT2_STAP ->    1
 *
 *  u8 *int_s_tap:
 *	Pointer holding the single tap interrupt value
 *  int_s_tap
 *	enable --> 1
 *	disable --> 0
 *
 *
 *  \return  results of bus communication function
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_int_s_tap(u8 channel,
u8 *int_s_tap)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (channel) {
		case BMA2x2_INT1_STAP:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_INT1_PAD_SNG_TAP__REG,
			&data, C_BMA2x2_One_U8X);
			*int_s_tap = BMA2x2_GET_BITSLICE
			(data, BMA2x2_EN_INT1_PAD_SNG_TAP);
		break;
		case BMA2x2_INT2_STAP:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_INT2_PAD_SNG_TAP__REG,
			&data, C_BMA2x2_One_U8X);
			*int_s_tap = BMA2x2_GET_BITSLICE
			(data, BMA2x2_EN_INT2_PAD_SNG_TAP);
		break;
		default:
			comres = E_OUT_OF_RANGE;
		break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 * Description: *//**\brief This API is used to set
 * the status of single tap interrupt in the register 0x19 and 0x1B
 * INT1 -> register 0x19 bit 5
 * INT2 -> register 0x1B bit 5
 *
 *
 *
 *
 * \param u8 channel:
 *	The value of single tap interrupt channel number
 *	channel -->	BMA2x2_ACCEL_INT1_STAP     ->    0
 *				BMA2x2_ACCEL_INT2_STAP     ->    1
 *
 *  u8 int_s_tap:
 *	The single tap interrupt status value
 *  int_s_tap
 *	enable --> 1
 *	disable --> 0
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_int_s_tap(u8 channel,
u8 int_s_tap)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (channel) {
		case BMA2x2_INT1_STAP:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_INT1_PAD_SNG_TAP__REG,
			&data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE(data,
			BMA2x2_EN_INT1_PAD_SNG_TAP, int_s_tap);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_INT1_PAD_SNG_TAP__REG,
			&data, C_BMA2x2_One_U8X);
		break;
		case BMA2x2_INT2_STAP:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_INT2_PAD_SNG_TAP__REG,
			&data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_EN_INT2_PAD_SNG_TAP, int_s_tap);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_INT2_PAD_SNG_TAP__REG,
			&data, C_BMA2x2_One_U8X);
		break;
		default:
			comres = E_OUT_OF_RANGE;
		break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 * Description: *//**\brief This API is used to get
 * the status of orient interrupt in the register 0x19 and 0x1B
 * INT1 -> register 0x19 bit 6
 * INT2 -> register 0x1B bit 6
 *
 *
 *
 *
 ** \param u8 channel:
 *	The value of orient interrupt channel number
 *	channel -->	BMA2x2_ACCEL_INT1_ORIENT     ->    0
 *				BMA2x2_ACCEL_INT2_ORIENT     ->    1
 *
 *  u8 *int_orient: Pointer holding the orient interrupt value
 *	int_orient
 *	enable --> 1
 *	disable --> 0
 *
 *
 *  \return  results of bus communication function
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_int_orient(u8 channel,
u8 *int_orient)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (channel) {
		case BMA2x2_INT1_ORIENT:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_INT1_PAD_ORIENT__REG,
			&data, C_BMA2x2_One_U8X);
			*int_orient = BMA2x2_GET_BITSLICE
			(data, BMA2x2_EN_INT1_PAD_ORIENT);
		break;
		case BMA2x2_INT2_ORIENT:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_INT2_PAD_ORIENT__REG,
			&data, C_BMA2x2_One_U8X);
			*int_orient = BMA2x2_GET_BITSLICE
			(data, BMA2x2_EN_INT2_PAD_ORIENT);
		break;
		default:
			comres = E_OUT_OF_RANGE;
		break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
 * Description: *//**\brief This API is used to set
 * the status of orient interrupt in the register 0x19 and 0x1B
 * INT1 -> register 0x19 bit 6
 * INT2 -> register 0x1B bit 6
 *
 *
 *
 *
 *	\param u8 channel:
 *	The value of orient interrupt channel number
 *	channel -->	BMA2x2_ACCEL_INT1_ORIENT     ->    0
 *				BMA2x2_ACCEL_INT2_ORIENT     ->    1
 *
 *  u8  int_orient:
 *	The orient interrupt status value
 *	int_orient
 *	enable --> 1
 *	disable --> 0
 *
 *
 *  \return  results of bus communication function
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_int_orient(u8 channel,
u8 int_orient)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (channel) {
		case BMA2x2_INT1_ORIENT:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_INT1_PAD_ORIENT__REG,
			&data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_EN_INT1_PAD_ORIENT, int_orient);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_INT1_PAD_ORIENT__REG,
			&data, C_BMA2x2_One_U8X);
		break;
		case BMA2x2_INT2_ORIENT:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_INT2_PAD_ORIENT__REG,
			&data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_EN_INT2_PAD_ORIENT, int_orient);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_INT2_PAD_ORIENT__REG,
			&data, C_BMA2x2_One_U8X);
		break;
		default:
			comres = E_OUT_OF_RANGE;
		break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 * Description: *//**\brief This API is used to get
 * the status of flat interrupt in the register 0x19 and 0x1B
 * INT1 -> register 0x19 bit 7
 * INT2 -> register 0x1B bit 7
 *
 *
 *
 *
 *\param u8 channel:
 *	The value of flat interrupt channel number
 *	channel -->	BMA2x2_ACCEL_INT1_FLAT     ->    0
 *				BMA2x2_ACCEL_INT2_FLAT     ->    1
 *
 *  u8 *int_flat: Pointer holding the flat interrupt value
 *	int_flat
 *	enable --> 1
 *	disable --> 0
 *
 *
 *  \return  results of bus communication function
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 **************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_int_flat(u8 channel,
u8 *int_flat)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (channel) {
		case BMA2x2_INT1_FLAT:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_INT1_PAD_FLAT__REG,
			&data, C_BMA2x2_One_U8X);
			*int_flat = BMA2x2_GET_BITSLICE
			(data, BMA2x2_EN_INT1_PAD_FLAT);
		break;
		case BMA2x2_INT2_FLAT:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_INT2_PAD_FLAT__REG,
			&data, C_BMA2x2_One_U8X);
			*int_flat = BMA2x2_GET_BITSLICE
			(data, BMA2x2_EN_INT2_PAD_FLAT);
		break;
		default:
			comres = E_OUT_OF_RANGE;
		break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/**************************************************************************
 * Description: *//**\brief This API is used to set
 * the status of flat interrupt in the register 0x19 and 0x1B
 * INT1 -> register 0x19 bit 7
 * INT2 -> register 0x1B bit 7
 *
 *
 *
 *
 * \param u8 channel:
 *	The value of flat interrupt channel number
 *	channel --> BMA2x2_ACCEL_INT1_FLAT     ->    0
 *				BMA2x2_ACCEL_INT2_FLAT     ->    1
 *
 *	u8 int_flat:
 *	The flat interrupt status value
 *	int_flat
 *	enable --> 1
 *	disable --> 0
 *
 *
 *  \return  results of bus communication function
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_int_flat(u8 channel,
u8 int_flat)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (channel) {
		case BMA2x2_INT1_FLAT:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_INT1_PAD_FLAT__REG,
			&data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_EN_INT1_PAD_FLAT, int_flat);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_INT1_PAD_FLAT__REG,
			&data, C_BMA2x2_One_U8X);
		break;
		case BMA2x2_INT2_FLAT:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_INT2_PAD_FLAT__REG,
			&data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_EN_INT2_PAD_FLAT, int_flat);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_INT2_PAD_FLAT__REG,
			&data, C_BMA2x2_One_U8X);
		break;
		default:
			comres = E_OUT_OF_RANGE;
		break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/**************************************************************************
 * Description: *//**\brief This API is used to get
 * the status of new data in the register 0x19
 * INT1 -> register 0x19 bit 0
 * INT2 -> register 0x19 bit 7
 *
 *
 *
 *	\param u8 channel:
 *		The value of new data interrupt channel number
 *		channel -->	BMA2x2_ACCEL_INT1_NDATA     ->    0
 *					BMA2x2_ACCEL_INT2_NDATA     ->    1
 *
 *	u8 *int_newdata: Pointer holding the new data interrupt value
 *	int_newdata
 *	enable --> 1
 *	disable --> 0
 *
 *
 *  \return  results of bus communication function
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_newdata(u8 channel,
u8 *int_newdata)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (channel) {
		case BMA2x2_INT1_NDATA:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_INT1_PAD_NEWDATA__REG,
			&data, C_BMA2x2_One_U8X);
			*int_newdata = BMA2x2_GET_BITSLICE
			(data, BMA2x2_EN_INT1_PAD_NEWDATA);
		break;
		case BMA2x2_INT2_NDATA:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_INT2_PAD_NEWDATA__REG,
			&data, C_BMA2x2_One_U8X);
			*int_newdata = BMA2x2_GET_BITSLICE
			(data, BMA2x2_EN_INT2_PAD_NEWDATA);
		break;
		default:
			comres = E_OUT_OF_RANGE;
		break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 * Description: *//**\brief This API is used to set
 * the status of new data in the register 0x19
 * INT1 -> register 0x19 bit 0
 * INT2 -> register 0x19 bit 7
 *
 *
 *
 * \param u8 channel:
 *	The value of new data interrupt channel number
 *	channel -->	BMA2x2_ACCEL_INT1_NDATA     ->    0
 *				BMA2x2_ACCEL_INT2_NDATA     ->    1
 *
 *	u8 int_newdata:
 *	The new data interrupt status value
 *	int_newdata
 *	enable --> 1
 *	disable --> 0
 *
 *
 *  \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_newdata(u8 channel,
u8 int_newdata)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (channel) {
		case BMA2x2_INT1_NDATA:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_INT1_PAD_NEWDATA__REG,
			&data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_EN_INT1_PAD_NEWDATA, int_newdata);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_INT1_PAD_NEWDATA__REG,
			&data, C_BMA2x2_One_U8X);
		break;
		case BMA2x2_INT2_NDATA:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_INT2_PAD_NEWDATA__REG,
			&data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_EN_INT2_PAD_NEWDATA, int_newdata);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_INT2_PAD_NEWDATA__REG,
			&data, C_BMA2x2_One_U8X);
		break;
		default:
			comres = E_OUT_OF_RANGE;
		break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
 * Description: *//**\brief This API is used to get the fwm interrupt1 data
 * in the register 0x1A
 * INT1 -> register 0x1A bit 1
 *
 *  \param  u8 *int1_fwm :
 *	Pointer holding the interrupt1 FIFO watermark
 *	int1_fwm --> [0:1]
 *	0 --> disable
 *	1 --> enable
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_int1_fwm(u8 *int1_fwm)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_INT1_PAD_FWM__REG,
			&data, C_BMA2x2_One_U8X);
			*int1_fwm = BMA2x2_GET_BITSLICE
			(data, BMA2x2_EN_INT1_PAD_FWM);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
 * Description: *//**\brief This API is used to set the fwm interrupt1 data
 *in the register 0x1A
 * INT1 -> register 0x1A bit 1
 *
 *
 *	u8 int1_fwm:
 *	The fwm interrupt1 status value
 *	int1_fwm
 *	0 --> disable
 *	1 --> enable
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_int1_fwm(u8 int1_fwm)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		if (int1_fwm < C_BMA2x2_Two_U8X) {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr, BMA2x2_EN_INT1_PAD_FWM__REG,
			&data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_EN_INT1_PAD_FWM, int1_fwm);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_INT1_PAD_FWM__REG,
			&data, C_BMA2x2_One_U8X);
		} else {
		return  E_OUT_OF_RANGE;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 * Description: *//**\brief This API is used to get
 * the fwm interrupt1 data in the register 0x1A
 * INT2 -> register 0x1A bit 6
 *
 *  \param  u8 *int2_fwm :
 *	Pointer holding the interrupt1 FIFO watermark
 *	int2_fwm --> [0:1]
 *	0 --> disable
 *	1 --> enable
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_int2_fwm(u8 *int2_fwm)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_INT2_PAD_FWM__REG,
			&data, C_BMA2x2_One_U8X);
			*int2_fwm = BMA2x2_GET_BITSLICE
			(data, BMA2x2_EN_INT2_PAD_FWM);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 *	Description: *//**\brief This API is used to set
 *	the fwm interrupt1 data in the register 0x1A
 *	INT2 -> register 0x1A bit 6
 *
 *
 *	u8 int2_fwm:
 *	The fwm interrupt1 status value
 *	int2_fwm
 *	0 --> disable
 *	1 --> enable
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_int2_fwm(u8 int2_fwm)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		if (int2_fwm < C_BMA2x2_Two_U8X) {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_INT2_PAD_FWM__REG,
			&data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_EN_INT2_PAD_FWM, int2_fwm);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_INT2_PAD_FWM__REG,
			&data, C_BMA2x2_One_U8X);
		} else {
		return  E_OUT_OF_RANGE;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 *	Description: *//**\brief This API is used to get
 *	the ffull interrupt1 data in the register 0x1A
 *	INT1 -> register 0x1A bit 2
 *
 *
 *
 *  \param u8 *int1_ffull : Pointer holding the int1_ffull
 *	int1_ffull
 *	0 --> enable
 *	1 --> disable
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_int1_ffull(u8 *int1_ffull)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_INT1_PAD_FFULL__REG,
			&data, C_BMA2x2_One_U8X);
			*int1_ffull = BMA2x2_GET_BITSLICE
			(data, BMA2x2_EN_INT1_PAD_FFULL);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 *	Description: *//**\brief This API is used to set
 *	the ffull interrupt1 data in the register 0x1A
 *	INT1 -> register 0x1A bit 2
 *
 *
 *
 *	u8 int1_ffull:
 *	The ffull interrupt1 status value
 *	int1_ffull
 *	0 --> enable
 *	1 --> disable
 *
 *
 *  \return  results of bus communication function
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_int1_ffull(u8 int1_ffull)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		if (int1_ffull < C_BMA2x2_Two_U8X) {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_INT1_PAD_FFULL__REG,
			&data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_EN_INT1_PAD_FFULL, int1_ffull);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_INT1_PAD_FFULL__REG,
			&data, C_BMA2x2_One_U8X);
			} else {
			return  E_OUT_OF_RANGE;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 *	Description: *//**\brief This API is used to get
 *	the ffull interrupt1 data in the register 0x1A
 *	INT2 -> register 0x1A bit 5
 *
 *
 *
 *  \param u8 *int1_ffull : Pointer holding the int2_ffull
 *	int2_ffull
 *	0 --> enable
 *	1 --> disable
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_int2_ffull(u8 *int2_ffull)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_INT2_PAD_FFULL__REG,
			&data, C_BMA2x2_One_U8X);
			*int2_ffull = BMA2x2_GET_BITSLICE
			(data, BMA2x2_EN_INT2_PAD_FFULL);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 *	Description: *//**\brief This API is used to set
 *	the ffull interrupt1 data in the register 0x1A
 *	INT2 -> register 0x1A bit 5
 *
 *
 *
 *	u8 int2_ffull:
 *	The ffull interrupt1 status value
 *	int2_ffull
 *	0 --> enable
 *	1 --> disable
 *
 *
 *  \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_int2_ffull(u8 int2_ffull)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		if (int2_ffull < C_BMA2x2_Two_U8X) {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_INT2_PAD_FFULL__REG,
			&data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_EN_INT2_PAD_FFULL, int2_ffull);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_INT2_PAD_FFULL__REG,
			&data, C_BMA2x2_One_U8X);
			} else {
			return  E_OUT_OF_RANGE;
			}
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 *	Description: *//**\brief This API is used to get
 *	the source status data in the register 0x1E bit from 0 to 5
 *
 *
 *
 *	\param u8 channel:
 *	The value of source status channel number
 *	channel -->	BMA2x2_ACCEL_SRC_LOWG         0
 *				BMA2x2_ACCEL_SRC_HIGHG        1
 *				BMA2x2_ACCEL_SRC_SLOPE        2
 *				BMA2x2_ACCEL_SRC_SLO_NO_MOT   3
 *				BMA2x2_ACCEL_SRC_TAP          4
 *				BMA2x2_ACCEL_SRC_DATA         5
 *
 *	u8 *int_source: Pointer holding the source status value
 *	int_source
 *	0 --> enable
 *	1 --> disable
 *
 *	\return  results of bus communication function
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_source(u8 channel,
u8 *int_source)
{
		u8 data = C_BMA2x2_Zero_U8X;
		BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return  E_BMA2x2_NULL_PTR;
		} else {
		switch (channel) {
		case BMA2x2_SRC_LOWG:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_UNFILT_INT_SRC_LOWG__REG,
			&data, C_BMA2x2_One_U8X);
			*int_source = BMA2x2_GET_BITSLICE
			(data, BMA2x2_UNFILT_INT_SRC_LOWG);
		break;
		case BMA2x2_SRC_HIGHG:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_UNFILT_INT_SRC_HIGHG__REG,
			&data, C_BMA2x2_One_U8X);
			*int_source = BMA2x2_GET_BITSLICE
			(data, BMA2x2_UNFILT_INT_SRC_HIGHG);
		break;
		case BMA2x2_SRC_SLOPE:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_UNFILT_INT_SRC_SLOPE__REG,
			&data, C_BMA2x2_One_U8X);
			*int_source = BMA2x2_GET_BITSLICE
			(data, BMA2x2_UNFILT_INT_SRC_SLOPE);
		break;
		case BMA2x2_SRC_SLO_NO_MOT:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_UNFILT_INT_SRC_SLO_NO_MOT__REG,
			&data, C_BMA2x2_One_U8X);
			*int_source = BMA2x2_GET_BITSLICE
			(data, BMA2x2_UNFILT_INT_SRC_SLO_NO_MOT);
		break;
		case BMA2x2_SRC_TAP:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_UNFILT_INT_SRC_TAP__REG,
			&data, C_BMA2x2_One_U8X);
			*int_source = BMA2x2_GET_BITSLICE
			(data, BMA2x2_UNFILT_INT_SRC_TAP);
		break;
		case BMA2x2_SRC_DATA:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_UNFILT_INT_SRC_DATA__REG,
			&data, C_BMA2x2_One_U8X);
			*int_source = BMA2x2_GET_BITSLICE
			(data, BMA2x2_UNFILT_INT_SRC_DATA);
		break;
		default:
			comres = E_OUT_OF_RANGE;
		break;
			}
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 * Description: *//**\brief  This API is used to set
 *	the source status data in the register 0x1E bit from 0 to 5
 *
 *
 *
 *	\param u8 channel:
 *	The value of source status channel number
 *	channel -->	BMA2x2_ACCEL_SRC_LOWG         0
 *				BMA2x2_ACCEL_SRC_HIGHG        1
 *				BMA2x2_ACCEL_SRC_SLOPE        2
 *				BMA2x2_ACCEL_SRC_SLO_NO_MOT   3
 *				BMA2x2_ACCEL_SRC_TAP          4
 *				BMA2x2_ACCEL_SRC_DATA         5
 *
 *  u8 int_source:
 *	The source status value
 *	int_source
 *	0 --> enable
 *	1 --> disable
 *
 *  \return  results of bus communication
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_source(u8 channel,
u8 int_source)
{
		u8 data = C_BMA2x2_Zero_U8X;
		BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
		if (p_bma2x2 == BMA2x2_NULL) {
			return  E_BMA2x2_NULL_PTR;
		} else {
		switch (channel) {
		case BMA2x2_SRC_LOWG:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_UNFILT_INT_SRC_LOWG__REG,
			&data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_UNFILT_INT_SRC_LOWG, int_source);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_UNFILT_INT_SRC_LOWG__REG,
			&data, C_BMA2x2_One_U8X);
		break;
		case BMA2x2_SRC_HIGHG:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_UNFILT_INT_SRC_HIGHG__REG,
			&data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_UNFILT_INT_SRC_HIGHG, int_source);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_UNFILT_INT_SRC_HIGHG__REG,
			&data, C_BMA2x2_One_U8X);
		break;
		case BMA2x2_SRC_SLOPE:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_UNFILT_INT_SRC_SLOPE__REG,
			&data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_UNFILT_INT_SRC_SLOPE, int_source);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_UNFILT_INT_SRC_SLOPE__REG,
			&data, C_BMA2x2_One_U8X);
		break;
		case BMA2x2_SRC_SLO_NO_MOT:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_UNFILT_INT_SRC_SLO_NO_MOT__REG,
			&data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_UNFILT_INT_SRC_SLO_NO_MOT, int_source);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_UNFILT_INT_SRC_SLO_NO_MOT__REG,
			&data, C_BMA2x2_One_U8X);
		break;
		case BMA2x2_SRC_TAP:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_UNFILT_INT_SRC_TAP__REG,
			&data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_UNFILT_INT_SRC_TAP, int_source);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_UNFILT_INT_SRC_TAP__REG,
			&data, C_BMA2x2_One_U8X);
		break;
		case BMA2x2_SRC_DATA:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_UNFILT_INT_SRC_DATA__REG,
			&data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_UNFILT_INT_SRC_DATA, int_source);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_UNFILT_INT_SRC_DATA__REG,
			&data, C_BMA2x2_One_U8X);
		break;
		default:
			comres = E_OUT_OF_RANGE;
		break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 *	Description: *//**\brief This API is used to get
 *	the output type status in the register 0x20.
 *	INT1 -> bit 1
 *	INT2 -> bit 3
 *
 *	\param u8 channel:
 *   The value of output type channel number
 *	channel --> BMA2x2_ACCEL_INT1_OUTPUT    ->   0
 *				BMA2x2_ACCEL_INT2_OUTPUT    ->   1
 *
 *	u8 *int_od: Pointer holding the output type value
 *	open drain   ->   1
 *	push pull    ->   0
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_int_od(u8 channel,
u8 *int_od)
{
		u8 data = C_BMA2x2_Zero_U8X;
		BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
		if (p_bma2x2 == BMA2x2_NULL) {
			return  E_BMA2x2_NULL_PTR;
		} else {
		switch (channel) {
		case BMA2x2_INT1_OUTPUT:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_INT1_PAD_OUTPUT_TYPE__REG,
			&data, C_BMA2x2_One_U8X);
			*int_od = BMA2x2_GET_BITSLICE
			(data, BMA2x2_INT1_PAD_OUTPUT_TYPE);
		break;
		case BMA2x2_INT2_OUTPUT:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_INT2_PAD_OUTPUT_TYPE__REG,
			&data, C_BMA2x2_One_U8X);
			*int_od = BMA2x2_GET_BITSLICE
			(data, BMA2x2_INT2_PAD_OUTPUT_TYPE);
		break;
		default:
			comres = E_OUT_OF_RANGE;
		break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 * Description: *//**\brief This API is used to set
 *	the output type status in the register 0x20.
 *	INT1 -> bit 1
 *	INT2 -> bit 3
 *
 *	\param u8 channel:
 *	The value of output type status channel number
 *	channel -->	BMA2x2_ACCEL_INT1_OUTPUT    ->   0
 *				BMA2x2_ACCEL_INT2_OUTPUT    ->   1
 *
 *  u8  int_od: The output type status value
 *	open drain   ->   1
 *	push pull    ->   0
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_int_od(u8 channel,
u8 int_od)
{
		u8 data = C_BMA2x2_Zero_U8X;
		BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
		if (p_bma2x2 == BMA2x2_NULL) {
			return  E_BMA2x2_NULL_PTR;
		}  else {
		switch (channel) {
		case BMA2x2_INT1_OUTPUT:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_INT1_PAD_OUTPUT_TYPE__REG,
			&data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_INT1_PAD_OUTPUT_TYPE, int_od);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_INT1_PAD_OUTPUT_TYPE__REG,
			&data, C_BMA2x2_One_U8X);
		break;
		case BMA2x2_INT2_OUTPUT:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_INT2_PAD_OUTPUT_TYPE__REG,
			&data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_INT2_PAD_OUTPUT_TYPE, int_od);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_INT2_PAD_OUTPUT_TYPE__REG,
			&data, C_BMA2x2_One_U8X);
		break;
		default:
			comres = E_OUT_OF_RANGE;
		break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 *	Description: *//**\brief This API is used to get
 *	Active Level status in the register 0x20
 *	INT1 -> bit 0
 *	INT2 -> bit 2
 *
 *	\param u8 channel:
 *	The value of Active Level channel number
 *	channel -->	BMA2x2_ACCEL_INT1_LEVEL    ->    0
 *				BMA2x2_ACCEL_INT2_LEVEL    ->    1
 *
 *  u8 *int_lvl: Pointer holding the Active Level status value
 *	Active HIGH   ->   1
 *	Active LOW    ->   0
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_int_lvl(u8 channel,
u8 *int_lvl)
{
		u8 data = C_BMA2x2_Zero_U8X;
		BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
		if (p_bma2x2 == BMA2x2_NULL) {
			return  E_BMA2x2_NULL_PTR;
		} else {
		switch (channel) {
		case BMA2x2_INT1_LEVEL:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_INT1_PAD_ACTIVE_LEVEL__REG,
			&data, C_BMA2x2_One_U8X);
			*int_lvl = BMA2x2_GET_BITSLICE
			(data, BMA2x2_INT1_PAD_ACTIVE_LEVEL);
		break;
		case BMA2x2_INT2_LEVEL:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_INT2_PAD_ACTIVE_LEVEL__REG,
			&data, C_BMA2x2_One_U8X);
			*int_lvl = BMA2x2_GET_BITSLICE
			(data, BMA2x2_INT2_PAD_ACTIVE_LEVEL);
		break;
		default:
			comres = E_OUT_OF_RANGE;
		break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 *	Description: *//**\brief This API is used to set
 *	Active Level status in the register 0x20
 *	INT1 -> bit 0
 *	INT2 -> bit 2
 *
 *
 *
 *	\param u8 channel:
 *                  The value of Active Level channel number
 *                     channel -->BMA2x2_ACCEL_INT1_LEVEL    ->    0
 *                                BMA2x2_ACCEL_INT2_LEVEL    ->    1
 *
 *  u8 int_lvl:
 *	The value of Active Level channel number
 *	channel -->	BMA2x2_ACCEL_INT1_LEVEL    ->    0
 *				BMA2x2_ACCEL_INT2_LEVEL    ->    1
 *	Active HIGH   ->   1
 *	Active LOW    ->   0
 *
 *
 *  \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_int_lvl(u8 channel,
u8 int_lvl)
{
		u8 data = C_BMA2x2_Zero_U8X;
		BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
		if (p_bma2x2 == BMA2x2_NULL) {
			return  E_BMA2x2_NULL_PTR;
		} else {
		switch (channel) {
		case BMA2x2_INT1_LEVEL:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_INT1_PAD_ACTIVE_LEVEL__REG,
			&data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_INT1_PAD_ACTIVE_LEVEL, int_lvl);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_INT1_PAD_ACTIVE_LEVEL__REG,
			&data, C_BMA2x2_One_U8X);
		break;
		case BMA2x2_INT2_LEVEL:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_INT2_PAD_ACTIVE_LEVEL__REG,
			&data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_INT2_PAD_ACTIVE_LEVEL, int_lvl);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_INT2_PAD_ACTIVE_LEVEL__REG,
			&data, C_BMA2x2_One_U8X);
		break;
		default:
			comres = E_OUT_OF_RANGE;
		break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 *	Description: *//**\brief This API is used to set
 *	the reset interrupt in the register 0x21 bit 7
 *
 *
 *
 *  \param u8 reset_int: Reset the interrupt
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_reset_interrupt(u8 reset_int)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr, BMA2x2_RESET_INT__REG,
			&data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_RESET_INT, reset_int);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr, BMA2x2_RESET_INT__REG,
			&data, C_BMA2x2_One_U8X);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 *	Description: *//**\brief This API is used to get
 *	the latch duration in the register 0x21 bit from 0 to 3
 *
 *	\param u8 *latch_int:
 *    Pointer holding the latch duration value
 *      latch_int -->
 *			BMA2x2_LATCH_DUR_NON_LATCH    0x00
 *			BMA2x2_LATCH_DUR_250MS        0x01
 *			BMA2x2_LATCH_DUR_500MS        0x02
 *			BMA2x2_LATCH_DUR_1S           0x03
 *			BMA2x2_LATCH_DUR_2S           0x04
 *			BMA2x2_LATCH_DUR_4S           0x05
 *			BMA2x2_LATCH_DUR_8S           0x06
 *			BMA2x2_LATCH_DUR_LATCH        0x07
 *			BMA2x2_LATCH_DUR_NON_LATCH1   0x08
 *			BMA2x2_LATCH_DUR_250US        0x09
 *			BMA2x2_LATCH_DUR_500US        0x0A
 *			BMA2x2_LATCH_DUR_1MS          0x0B
 *			BMA2x2_LATCH_DUR_12_5MS       0x0C
 *			BMA2x2_LATCH_DUR_25MS         0x0D
 *			BMA2x2_LATCH_DUR_50MS         0x0E
 *			BMA2x2_LATCH_DUR_LATCH1       0x0F
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_latch_int(u8 *latch_int)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr, BMA2x2_LATCH_INT__REG,
			&data, C_BMA2x2_One_U8X);
			*latch_int = BMA2x2_GET_BITSLICE
			(data, BMA2x2_LATCH_INT);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 *	Description: *//**\brief This API is used to set
 *	the latch duration in the register 0x21 bit from 0 to 3
 *
 *
 *
 *	u8 latch_int:
 *	The latch duration value
 *          BMA2x2_LATCH_DUR_NON_LATCH    0x00
 *			BMA2x2_LATCH_DUR_250MS        0x01
 *			BMA2x2_LATCH_DUR_500MS        0x02
 *			BMA2x2_LATCH_DUR_1S           0x03
 *			BMA2x2_LATCH_DUR_2S           0x04
 *			BMA2x2_LATCH_DUR_4S           0x05
 *			BMA2x2_LATCH_DUR_8S           0x06
 *			BMA2x2_LATCH_DUR_LATCH        0x07
 *			BMA2x2_LATCH_DUR_NON_LATCH1   0x08
 *			BMA2x2_LATCH_DUR_250US        0x09
 *			BMA2x2_LATCH_DUR_500US        0x0A
 *			BMA2x2_LATCH_DUR_1MS          0x0B
 *			BMA2x2_LATCH_DUR_12_5MS       0x0C
 *			BMA2x2_LATCH_DUR_25MS         0x0D
 *			BMA2x2_LATCH_DUR_50MS         0x0E
 *			BMA2x2_LATCH_DUR_LATCH1       0x0F
 *
 *  \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_latch_int(u8 latch_int)
{
u8 data = C_BMA2x2_Zero_U8X;
BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
u8 latch_duration = C_BMA2x2_Zero_U8X;
if (p_bma2x2 == BMA2x2_NULL)  {
		return E_BMA2x2_NULL_PTR;
		} else  {
		if (latch_int < C_BMA2x2_Sixteen_U8X) {
			switch (latch_int) {
			case BMA2x2_LATCH_DUR_NON_LATCH:
				latch_duration = BMA2x2_LATCH_DUR_NON_LATCH;

				/*  NON LATCH   */
			break;
			case BMA2x2_LATCH_DUR_250MS:
				latch_duration = BMA2x2_LATCH_DUR_250MS;

				/*  250 MS  */
			break;
			case BMA2x2_LATCH_DUR_500MS:
				latch_duration = BMA2x2_LATCH_DUR_500MS;

				/*  500 MS  */
			break;
			case BMA2x2_LATCH_DUR_1S:
				latch_duration = BMA2x2_LATCH_DUR_1S;

				/*  1 S   */
			break;
			case BMA2x2_LATCH_DUR_2S:
				latch_duration = BMA2x2_LATCH_DUR_2S;

				/*  2 S  */
			break;
			case BMA2x2_LATCH_DUR_4S:
				latch_duration = BMA2x2_LATCH_DUR_4S;

				/*  4 S  */
			break;
			case BMA2x2_LATCH_DUR_8S:
				latch_duration = BMA2x2_LATCH_DUR_8S;

				/*  8 S  */
			break;
			case BMA2x2_LATCH_DUR_LATCH:
				latch_duration = BMA2x2_LATCH_DUR_LATCH;

				/*  LATCH  */
			break;
			case BMA2x2_LATCH_DUR_NON_LATCH1:
				latch_duration = BMA2x2_LATCH_DUR_NON_LATCH1;

				/*  NON LATCH1  */
			break;
			case BMA2x2_LATCH_DUR_250US:
				latch_duration = BMA2x2_LATCH_DUR_250US;

				/*  250 US   */
			break;
			case BMA2x2_LATCH_DUR_500US:
				latch_duration = BMA2x2_LATCH_DUR_500US;

				/*  500 US   */
			break;
			case BMA2x2_LATCH_DUR_1MS:
				latch_duration = BMA2x2_LATCH_DUR_1MS;

				/*  1 MS   */
			break;
			case BMA2x2_LATCH_DUR_12_5MS:
				latch_duration = BMA2x2_LATCH_DUR_12_5MS;

				/*  12.5 MS   */
			break;
			case BMA2x2_LATCH_DUR_25MS:
				latch_duration = BMA2x2_LATCH_DUR_25MS;

				/*  25 MS   */
			break;
			case BMA2x2_LATCH_DUR_50MS:
				latch_duration = BMA2x2_LATCH_DUR_50MS;

				/*  50 MS   */
			break;
			case BMA2x2_LATCH_DUR_LATCH1:
				latch_duration = BMA2x2_LATCH_DUR_LATCH1;

				/*  LATCH1   */
			break;
			default:
			break;
			}
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr, BMA2x2_LATCH_INT__REG,
			&data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_LATCH_INT, latch_duration);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr, BMA2x2_LATCH_INT__REG,
			&data, C_BMA2x2_One_U8X);
		} else {
		return  E_OUT_OF_RANGE;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 *	Description: *//**\brief This API is used to get the duration of
 *	Low, High, Slope and slow no motion interrupts in the registers
 *	LOW_DURATION		-> register 0x22 bit form 0 to 7
 *	HIGH_DURATION		-> register 0x25 bit form 0 to 7
 *	SLOPE_DURATION		-> register 0x27 bit form 0 to 1
 *	SLO_NO_MOT_DURATION -> register 0x27 bit form 2 to 7
 *
 *	\param u8 channel:
 *	The value of duration channel number
 *	channel --> BMA2x2_ACCEL_LOW_DURATION            0
 *				BMA2x2_ACCEL_HIGH_DURATION           1
 *				BMA2x2_ACCEL_SLOPE_DURATION          2
 *				BMA2x2_ACCEL_SLO_NO_MOT_DURATION     3
 *
 *	u8 *dur: Pointer holding the duration value
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 **************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_dur(u8 channel,
u8 *dur)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (channel) {
		case BMA2x2_LOW_DURATION:
			/*LOW DURATION*/
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr, BMA2x2_LOW_DURN_REG,
			&data, C_BMA2x2_One_U8X);
			*dur = data;
		break;
		case BMA2x2_HIGH_DURATION:
			/*HIGH DURATION*/
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr, BMA2x2_HIGH_DURN_REG,
			&data, C_BMA2x2_One_U8X);
			*dur = data;
		break;
		case BMA2x2_SLOPE_DURATION:
			/*SLOPE DURATION*/
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr, BMA2x2_SLOPE_DUR__REG,
			&data, C_BMA2x2_One_U8X);
			*dur = BMA2x2_GET_BITSLICE
			(data, BMA2x2_SLOPE_DUR);
		break;
		case BMA2x2_SLO_NO_MOT_DURATION:
			/*SLO NO MOT DURATION*/
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr, BMA2x2_SLO_NO_MOT_DUR__REG,
			&data, C_BMA2x2_One_U8X);
			*dur = BMA2x2_GET_BITSLICE
			(data, BMA2x2_SLO_NO_MOT_DUR);
		break;
		default:
			comres = E_OUT_OF_RANGE;
		break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
 *	Description: *//**\brief This API is used to set the duration of
 *	Low, High, Slope and slow no motion interrupts in the registers
 *	LOW_DURATION		-> register 0x22 bit form 0 to 7
 *	HIGH_DURATION		-> register 0x25 bit form 0 to 7
 *	SLOPE_DURATION		-> register 0x27 bit form 0 to 1
 *	SLO_NO_MOT_DURATION -> register 0x27 bit form 2 to 7
 *
 *	\param u8 channel:
 *	The value of duration channel number
 *	channel --> BMA2x2_ACCEL_LOW_DURATION            0
 *				BMA2x2_ACCEL_HIGH_DURATION           1
 *				BMA2x2_ACCEL_SLOPE_DURATION          2
 *				BMA2x2_ACCEL_SLO_NO_MOT_DURATION     3
 *
 *
 *	u8 dur: duration value
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_dur(u8 channel,
u8 dur)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL)  {
		return E_BMA2x2_NULL_PTR;
		}  else  {
		switch (channel)   {
		case BMA2x2_LOW_DURATION:
			/*LOW DURATION*/
			data = dur;
			comres = p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr, BMA2x2_LOW_DURN_REG,
			&data, C_BMA2x2_One_U8X);
		break;
		case BMA2x2_HIGH_DURATION:
			/*HIGH DURATION*/
			data = dur;
			comres = p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_HIGH_DURN_REG,
			&data, C_BMA2x2_One_U8X);
		break;
		case BMA2x2_SLOPE_DURATION:
			/*SLOPE DURATION*/
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_SLOPE_DUR__REG,
			&data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_SLOPE_DUR, dur);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_SLOPE_DUR__REG,
			&data, C_BMA2x2_One_U8X);
		break;
		case BMA2x2_SLO_NO_MOT_DURATION:
			/*SLO NO MOT DURATION*/
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_SLO_NO_MOT_DUR__REG,
			&data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_SLO_NO_MOT_DUR, dur);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_SLO_NO_MOT_DUR__REG,
			&data, C_BMA2x2_One_U8X);
		break;
		default:
			comres = E_OUT_OF_RANGE;
		break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
 * Description: *//**\brief This API is used to get the threshold of
 *	Low, High, Slope and slow no motion interrupts in the registers
 *	LOW_THRESHOLD		-> register 0x23 bit form 0 to 7
 *	HIGH_THRESHOLD		-> register 0x26 bit form 0 to 7
 *	SLOPE_THRESHOLD		-> register 0x28 bit form 0 to 7
 *	SLO_NO_MOT_THRESHOLD -> register 0x29 bit form 0 to 7
 *
 *	\param u8 channel:
 *    The value of threshold channel number
 *    channel -->BMA2x2_ACCEL_LOW_THRESHOLD            0
 *               BMA2x2_ACCEL_HIGH_THRESHOLD           1
 *               BMA2x2_ACCEL_SLOPE_THRESHOLD          2
 *               BMA2x2_ACCEL_SLO_NO_MOT_THRESHOLD     3
 *
 *  u8 *thr: Pointer holding the threshold value of threshold
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_thr(u8 channel,
u8 *thr)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (channel) {
		case BMA2x2_LOW_THRESHOLD:
			/*LOW THRESHOLD*/
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr, BMA2x2_LOW_THRES_REG,
			&data, C_BMA2x2_One_U8X);
			*thr = data;
		break;
		case BMA2x2_HIGH_THRESHOLD:
			/*HIGH THRESHOLD*/
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_HIGH_THRES_REG,
			&data, C_BMA2x2_One_U8X);
			*thr = data;
		break;
		case BMA2x2_SLOPE_THRESHOLD:
			/*SLOPE THRESHOLD*/
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_SLOPE_THRES_REG,
			&data, C_BMA2x2_One_U8X);
			*thr = data;
		break;
		case BMA2x2_SLO_NO_MOT_THRESHOLD:
			/*SLO NO MOT THRESHOLD*/
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_SLO_NO_MOT_THRES_REG,
			&data, C_BMA2x2_One_U8X);
			*thr = data;
		break;
		default:
			comres = E_OUT_OF_RANGE;
		break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 * Description: *//**\brief This API is used to set the threshold of
 *	Low, High, Slope and slow no motion interrupts in the registers
 *	LOW_THRESHOLD		-> register 0x23 bit form 0 to 7
 *	HIGH_THRESHOLD		-> register 0x26 bit form 0 to 7
 *	SLOPE_THRESHOLD		-> register 0x28 bit form 0 to 7
 *	SLO_NO_MOT_THRESHOLD -> register 0x29 bit form 0 to 7
 *
 *
 *
 *
 *	\param u8 channel:
 *	The value of threshold channel number
 *	channel -->	BMA2x2_ACCEL_LOW_THRESHOLD            0
 *				BMA2x2_ACCEL_HIGH_THRESHOLD           1
 *				BMA2x2_ACCEL_SLOPE_THRESHOLD          2
 *				BMA2x2_ACCEL_SLO_NO_MOT_THRESHOLD     3
 *
 *  u8  thr: The threshold status value
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_thr(u8 channel,
u8 thr)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (channel) {
		case BMA2x2_LOW_THRESHOLD:
			/*LOW THRESHOLD*/
			data = thr;
			comres = p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_LOW_THRES_REG, &data, C_BMA2x2_One_U8X);
		break;
		case BMA2x2_HIGH_THRESHOLD:
			/*HIGH THRESHOLD*/
			data = thr;
			comres = p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_HIGH_THRES_REG, &data, C_BMA2x2_One_U8X);
		break;
		case BMA2x2_SLOPE_THRESHOLD:
			/*SLOPE THRESHOLD*/
			data = thr;
			comres = p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_SLOPE_THRES_REG, &data, C_BMA2x2_One_U8X);
		break;
		case BMA2x2_SLO_NO_MOT_THRESHOLD:
			/*SLO NO MOT THRESHOLD*/
			data = thr;
			comres = p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_SLO_NO_MOT_THRES_REG, &data, C_BMA2x2_One_U8X);
		break;
		default:
			comres = E_OUT_OF_RANGE;
		break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 *	Description: *//**\brief This API is used to get
 *	the low high hysteresis in the registers 0x24
 *	LOWG_HYST  -> bit form 0 to 1
 *	HIGHG_HYST  -> bit from 6 to 7(it depends on the range selection)
 *
 * \param u8 channel: The value of selection of hysteresis channel number
 *	channel -->	BMA2x2_ACCEL_LOWG_HYST		0
 *				BMA2x2_ACCEL_HIGHG_HYST		1
 *
 *  u8 *hyst: Pointer holding the hysteresis value
 *
 *  \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_low_high_hyst(u8 channel,
u8 *hyst)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (channel) {
		case BMA2x2_LOWG_HYST:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_LOWG_HYST__REG,
			&data, C_BMA2x2_One_U8X);
			*hyst = BMA2x2_GET_BITSLICE
			(data, BMA2x2_LOWG_HYST);
		break;
		case BMA2x2_HIGHG_HYST:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_HIGHG_HYST__REG, &data, C_BMA2x2_One_U8X);
			*hyst = BMA2x2_GET_BITSLICE
			(data, BMA2x2_HIGHG_HYST);
		break;
		default:
			comres = E_OUT_OF_RANGE;
		break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 *	Description: *//**\brief This API is used to set
 *	the low high hysteresis in the registers 0x24
 *	LOWG_HYST  -> bit form 0 to 1
 *	HIGHG_HYST  -> bit from 6 to 7(it depends on the range selection)
 *
 *
 *
 *	\param u8 channel: The value of hysteresis channel number
 *	channel -->	BMA2x2_ACCEL_LOWG_HYST  ->    0
 *				BMA2x2_ACCEL_HIGHG_HYST ->    1
 *
 *	u8 hyst: The hysteresis value
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_low_high_hyst(u8 channel,
u8 hyst)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (channel) {
		case BMA2x2_LOWG_HYST:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr, BMA2x2_LOWG_HYST__REG,
			&data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_LOWG_HYST, hyst);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_LOWG_HYST__REG, &data, C_BMA2x2_One_U8X);
		break;
		case BMA2x2_HIGHG_HYST:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_HIGHG_HYST__REG, &data,  C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_HIGHG_HYST, hyst);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_HIGHG_HYST__REG,
			&data,  C_BMA2x2_One_U8X);
		break;
		default:
			comres = E_OUT_OF_RANGE;
		break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
 *	Description: *//**\brief This API is used to get
 *	low_g  mode in the registers 0x24 bit 2
 *
 *
 *	\param u8 *mode: Pointer holding the Low_G mode value
 *	mode:
 *	0 -> single mode
 *	1 -> sum mode
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
*****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 ******************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_lowg_mode(u8 *mode)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr, BMA2x2_LOWG_INT_MODE__REG,
			&data, C_BMA2x2_One_U8X);
			*mode = BMA2x2_GET_BITSLICE(data, BMA2x2_LOWG_INT_MODE);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 *	Description: *//**\brief This API is used to set
 *	low_g  mode in the registers 0x24 bit 2
 *
 *
 *
 *	\param u8 mode: The Low g mode status value
 *	mode:
 *	0 -> single mode
 *	1 -> sum mode
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_lowg_mode(u8 mode)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_LOWG_INT_MODE__REG, &data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_LOWG_INT_MODE, mode);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_LOWG_INT_MODE__REG,
			&data, C_BMA2x2_One_U8X);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 *	Description: *//**\brief This API is used to get
 *	the tap duration in the register 0x2A bit form 0 to 2
 *
 *
 *
 *	\param u8 *int_orient: Pointer holding the tap duration value
 *	tap_dur -->  0 -> 50ms
 *               1 -> 100ms
 *               2 -> 150ms
 *               3 -> 200ms
 *               4 -> 250ms
 *               5 -> 375ms
 *               6 -> 500ms
 *               7 -> 700ms
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_tap_dur(u8 *tap_dur)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_TAP_DUR__REG, &data, C_BMA2x2_One_U8X);
			*tap_dur = BMA2x2_GET_BITSLICE
			(data, BMA2x2_TAP_DUR);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 *	Description: *//**\brief This API is used to set
 *	the tap duration in the register 0x2A bit form 0 to 2
 *
 *
 *
 *	\param u8  tap_dur: The tap duration value
 *	tap_dur	->	0 -> 50ms
 *				1 -> 100ms
 *				2 -> 150ms
 *				3 -> 200ms
 *				4 -> 250ms
 *				5 -> 375ms
 *				6 -> 500ms
 *				7 -> 700ms
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_tap_dur(u8 tap_dur)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC(
			p_bma2x2->dev_addr,
			BMA2x2_TAP_DUR__REG, &data,
			C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_TAP_DUR, tap_dur);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_TAP_DUR__REG, &data, C_BMA2x2_One_U8X);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 *	Description: *//**\brief This API is used to get
 *	the tap shock form the register 0x2A bit 6
 *
 *
 *
 *	\param u8 *tap_shock: Pointer holding the tap shock value
 *	tap_shock
 *	1 -> 75ms
 *	0 -> 50ms
 *
 *
 *  \return  results of bus communication function
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_tap_shock(u8 *tap_shock)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_TAP_SHOCK_DURN__REG,
			&data, C_BMA2x2_One_U8X);
			*tap_shock = BMA2x2_GET_BITSLICE(data,
			BMA2x2_TAP_SHOCK_DURN);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 *	Description: *//**\brief This API is used to set
 *	the tap shock form the register 0x2A bit 6
 *
 *
 *
 *	\param u8 u8 tap_shock: The tap shock value
 *     tap_shock --> 0 -> 50ms
 *                   1 -> 75ms
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_tap_shock(u8 tap_shock)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_TAP_SHOCK_DURN__REG, &data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE(data,
			BMA2x2_TAP_SHOCK_DURN, tap_shock);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_TAP_SHOCK_DURN__REG, &data, C_BMA2x2_One_U8X);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 *	Description: *//**\brief This API is used to get
 *	the tap quiet in the register 0x2A bit 7
 *
 *
 *
 *  \param  u8 *tap_quiet :Pointer holding the tap quiet value
 *          tap_quiet ->  0 -> 30ms
 *                        1 -> 20ms
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_tap_quiet(u8 *tap_quiet)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_TAP_QUIET_DURN__REG, &data,
			C_BMA2x2_One_U8X);
			*tap_quiet = BMA2x2_GET_BITSLICE
			(data, BMA2x2_TAP_QUIET_DURN);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 *	Description: *//**\brief This API is used to set
 *	the tap quiet in the register 0x2A bit 7
 *
 *
 *
 *  \param u8 tap_quiet : The tap quiet value
 *            tap_quiet ->    0 -> 30ms
 *                            1 -> 20ms
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_tap_quiet(u8 tap_quiet)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_TAP_QUIET_DURN__REG,
			&data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE(data,
			BMA2x2_TAP_QUIET_DURN, tap_quiet);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_TAP_QUIET_DURN__REG,
			&data, C_BMA2x2_One_U8X);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 *	Description: *//**\brief This API is used to get
 *	the tap threshold in the register 0x2B bit from 0 to 4
 *
 *
 *
 *  \param u8 *tap_thr : Pointer holding the tap threshold
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_tap_thr(u8 *tap_thr)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_TAP_THRES__REG,
			&data, C_BMA2x2_One_U8X);
			*tap_thr = BMA2x2_GET_BITSLICE
			(data, BMA2x2_TAP_THRES);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 *	Description: *//**\brief This API is used to set
 *	the tap threshold in the register 0x2B bit from 0 to 4
 *
 *
 *
 *  \param u8 tap_thr: The tap threshold value
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_tap_thr(u8 tap_thr)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_TAP_THRES__REG, &data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_TAP_THRES, tap_thr);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_TAP_THRES__REG, &data, C_BMA2x2_One_U8X);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 *	Description: *//**\brief This API is used to get
 *	the tap sample in the register 0x2B bit 6 and 7
 *
 *
 *
 *  \param u8  *tap_sample : Pointer holding the tap sample
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_tap_sample(u8 *tap_sample)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_TAP_SAMPLES__REG, &data, C_BMA2x2_One_U8X);
			*tap_sample = BMA2x2_GET_BITSLICE
			(data, BMA2x2_TAP_SAMPLES);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 *	Description: *//**\brief This API is used to set
 *	the tap sample in the register 0x2B bit 6 and 7
 *
 *
 *
 *  \param u8 tap_sample: The tap sample value
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_tap_sample(u8 tap_sample)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr, BMA2x2_TAP_SAMPLES__REG,
			&data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_TAP_SAMPLES, tap_sample);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_TAP_SAMPLES__REG, &data, C_BMA2x2_One_U8X);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 *	Description: *//**\brief This API is used to get
 *	the orient mode in the register 0x2C bit 0 and 1
 *
 *
 *
 *  \param u8 *orient_mode : Pointer holding the orient mode value
 *	orient_mode  ->
 *	00 -> 45' symmetrical
 *	01 -> 63' high asymmetrical
 *	10 -> 27' low asymmetrical
 *	11 -> reserved
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 **************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_orient_mode(u8 *orient_mode)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC(
			p_bma2x2->dev_addr,
			BMA2x2_ORIENT_MODE__REG, &data, C_BMA2x2_One_U8X);
			*orient_mode = BMA2x2_GET_BITSLICE(
			data, BMA2x2_ORIENT_MODE);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 *	Description: *//**\brief This API is used to set
 *	the orient mode in the register 0x2C bit 0 and 1
 *
 *
 *
 *  \param u8 orient_mode: The orient mode value
 *	orient_mode ->
 *	00	->	45' symmetrical
 *	01	->	63' high asymmetrical
 *	10	->	27' low asymmetrical
 *	11	->	reserved
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_orient_mode(u8 orient_mode)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ORIENT_MODE__REG, &data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE(data,
			BMA2x2_ORIENT_MODE, orient_mode);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ORIENT_MODE__REG, &data, C_BMA2x2_One_U8X);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 *	Description: *//**\brief This API is used to get
 *	the orient block in the register 0x2C bit 2 and 3
 *
 *
 *
 *	\param u8 *orient_block : Pointer holding the orient block value
 *	orient_block ->
 *	00 -> disabled
 *	01 -> horizontal position or accel >1.75g
 *	10 -> horizontal position or accel >1.75g or slope > 0.2g
 *	11 -> horizontal position or accel >1.75g or slope > 0.4g or wait 100ms
 *
 *
 *  \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_orient_block(
u8 *orient_block)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ORIENT_BLOCK__REG, &data, C_BMA2x2_One_U8X);
			*orient_block = BMA2x2_GET_BITSLICE
			(data, BMA2x2_ORIENT_BLOCK);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 *	Description: *//**\brief This API is used to set
 *	the orient block in the register 0x2C bit 2 and 3
 *
 *
 *
 *  \param u8 orient_block: The orient block value
 *       orient_block ->
 *	00 -> disabled
 *	01 -> horizontal position or accel >1.75g
 *	10 -> horizontal position or accel >1.75g or slope > 0.2g
 *	11 -> horizontal position or accel >1.75g or slope > 0.4g or wait 100ms
 *
 *
 *  \return  results of bus communication function
 *
 *
 *************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_orient_block(u8 orient_block)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ORIENT_BLOCK__REG, &data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_ORIENT_BLOCK, orient_block);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ORIENT_BLOCK__REG, &data, C_BMA2x2_One_U8X);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 *	Description: *//**\brief This API is used to get
 *	the orient hysteresis in the register 0x2C bit 4 to 6
 *
 *
 *
 *  \param u8 *orient_hyst : Pointer holding the orient_hyst
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_orient_hyst(u8 *orient_hyst)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ORIENT_HYST__REG, &data, C_BMA2x2_One_U8X);
			*orient_hyst = BMA2x2_GET_BITSLICE
			(data, BMA2x2_ORIENT_HYST);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 *	Description: *//**\brief This API is used to set
 *	the orient hysteresis in the register 0x2C bit 4 to 6
 *
 *
 *
 *  \param u8 orient_hyst: The orient hysteresis value
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_orient_hyst(u8 orient_hyst)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ORIENT_HYST__REG, &data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE(data,
			BMA2x2_ORIENT_HYST, orient_hyst);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC(
			p_bma2x2->dev_addr,
			BMA2x2_ORIENT_HYST__REG,
			&data, C_BMA2x2_One_U8X);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
 *	Description: *//**\brief  This API is used to get
 *	the theta value of orient and flat interrupts
 *	ORIENT_THETA -> register 0x2D bit 0 to 5
 *	FLAT_THETA -> register 0x2E bit 0 to 5
 *
 *	\param u8 channel: The value of theta channel number
 *	channel -->	BMA2x2_ACCEL_ORIENT_THETA	0
 *				BMA2x2_ACCEL_FLAT_THETA		1
 *
 *  u8 *theta: Pointer holding the theta value
 *
 *  \return  results of bus communication function
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_theta(u8 channel,
u8 *theta)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (channel) {
		case BMA2x2_ORIENT_THETA:
			/*ORIENT THETA*/
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_THETA_BLOCK__REG, &data, C_BMA2x2_One_U8X);
			*theta = BMA2x2_GET_BITSLICE
			(data, BMA2x2_THETA_BLOCK);
		break;
		case BMA2x2_FLAT_THETA:
			/*FLAT THETA*/
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_THETA_FLAT__REG,
			&data, C_BMA2x2_One_U8X);
			*theta = data;
		break;
		default:
			comres = E_OUT_OF_RANGE;
		break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 * Description: *//**\brief This API is used to set
 *	the theta value of orient and flat interrupts
 *	ORIENT_THETA -> register 0x2D bit 0 to 5
 *	FLAT_THETA -> register 0x2E bit 0 to 5
 *
 *	\param u8 channel: The value of theta channel number
 *	channel -->	BMA2x2_ACCEL_ORIENT_THETA	0
 *				BMA2x2_ACCEL_FLAT_THETA		1
 *
 *	u8 theta: The theta value
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_theta(u8 channel,
u8 theta)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (channel) {
		case BMA2x2_ORIENT_THETA:
			/*ORIENT THETA*/
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_THETA_BLOCK__REG, &data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE(data,
			BMA2x2_THETA_BLOCK, theta);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_THETA_BLOCK__REG,
			&data, C_BMA2x2_One_U8X);
		break;
		case BMA2x2_FLAT_THETA:
			/*FLAT THETA*/
			data = theta;
			comres = p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_THETA_FLAT__REG,
			&data, C_BMA2x2_One_U8X);
		break;
		default:
			comres = E_OUT_OF_RANGE;
		break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
 *	Description: *//**\brief This API is used to get
 *  the status of "orient_ud_en" enable in the register 0x2D bit 6
 *
 *
 *
 *
 *  \param u8 *orient_en : Pointer holding the orient_en
 *	orient_en ->	1 -> Generates Interrupt
 *					0 -> Do not generate interrupt
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_orient_en(u8 *orient_en)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ORIENT_UD_EN__REG, &data, C_BMA2x2_One_U8X);
			*orient_en = BMA2x2_GET_BITSLICE
			(data, BMA2x2_ORIENT_UD_EN);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 *	Description: *//**\brief This API is used to set
 *  the "orient_ud_en" enable in the register 0x2D bit 6
 *
 *
 *
 *
 *  \param u8 orient_en: The orient enable value
 *	orient_en ->
 *	1	->	Generates Interrupt
 *	0	->	Do not generate interrupt
 *
 *
 *  \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_orient_en(u8 orient_en)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ORIENT_UD_EN__REG, &data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE(data,
			BMA2x2_ORIENT_UD_EN, orient_en);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ORIENT_UD_EN__REG,
			&data, C_BMA2x2_One_U8X);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 *	Description: *//**\brief This API is used to get
 *	the status of flat hysteresis("flat_hy) in the register 0x2F bit 0 to 2
 *
 *
 *
 *
 *  \param u8 *flat_hyst : Pointer holding the flat_hyst
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_flat_hyst(u8 *flat_hyst)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_FLAT_HYS__REG, &data, C_BMA2x2_One_U8X);
			*flat_hyst = BMA2x2_GET_BITSLICE
			(data, BMA2x2_FLAT_HYS);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 *	Description: *//**\brief This API is used to set
 *	the value flat hysteresis("flat_hy) in the register 0x2F bit 0 to 2
 *
 *
 *
 *  \param u8 flat_hyst: The flat hysteresis value
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_flat_hyst(u8 flat_hyst)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC(
			p_bma2x2->dev_addr,
			BMA2x2_FLAT_HYS__REG,
			&data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_FLAT_HYS, flat_hyst);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_FLAT_HYS__REG, &data, C_BMA2x2_One_U8X);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 *	Description: *//**\brief This API is used to get
 *  the status of flat hold time(flat_hold_time)
 *	in the register 0x2F bit 4 and 5
 *
 *
 *
 *
 *  \param  u8 *flat_hold_time : Pointer holding the flat_hold_time
 *             flat_hold_time ->  00 -> disabled
 *                                01 -> 512ms
 *                                10 -> 1024ms
 *                                11 -> 2048ms
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_flat_hold_time(
u8 *flat_hold_time)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_FLAT_HOLD_TIME__REG,
			&data, C_BMA2x2_One_U8X);
			*flat_hold_time = BMA2x2_GET_BITSLICE
			(data, BMA2x2_FLAT_HOLD_TIME);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/**************************************************************************
 *	Description: *//**\brief This API is used to set
 *	the value of flat hold time(flat_hold_time)
 *	in the register 0x2F bit 4 and 5
 *
 *
 *
 *
 *  \param u8 flat_hold_time: The flat hold time value
 *              flat_hold_time -> 00 -> disabled
 *                                01 -> 512ms
 *                                10 -> 1024ms
 *                                11 -> 2048ms
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_flat_hold_time(
u8 flat_hold_time)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_FLAT_HOLD_TIME__REG, &data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_FLAT_HOLD_TIME, flat_hold_time);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_FLAT_HOLD_TIME__REG, &data, C_BMA2x2_One_U8X);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
 *	Description: *//**\brief This API is used to get
 *	the fifo water mark level trigger(fifo_water_mark_level_trigger_retain)
 *	status in the register 0x30 bit from 0 to 5
 *
 *
 *
 *
 *  \param u8 *fifo_wml_trig: Pointer holding the fifo_wml_trig
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_fifo_wml_trig(
u8 *fifo_wml_trig)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_FIFO_WML_TRIG_RETAIN__REG,
			&data, C_BMA2x2_One_U8X);
			*fifo_wml_trig = BMA2x2_GET_BITSLICE
			(data, BMA2x2_FIFO_WML_TRIG_RETAIN);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
 *	Description: *//**\brief This API is used to set
 *	the fifo water mark level trigger(fifo_water_mark_level_trigger_retain)
 *	value in the register 0x30 bit from 0 to 5
 *
 *
 *
 *
 *  \param u8 fifo_wml_trig: The fifo water mark level trigger value
 *
 *  \return  results of bus communication function
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_fifo_wml_trig(
u8 fifo_wml_trig)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		if (fifo_wml_trig < C_BMA2x2_ThirtyTwo_U8X) {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_FIFO_WML_TRIG_RETAIN__REG,
			&data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_FIFO_WML_TRIG_RETAIN,
			fifo_wml_trig);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_FIFO_WML_TRIG_RETAIN__REG,
			&data, C_BMA2x2_One_U8X);
		} else {
		return  E_OUT_OF_RANGE;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 *	Description: *//**\brief This API is for to get
 *	the self test axis(self_test_axis) in the register ox32 bit 0 to 2
 *
 *
 *
 *  \param u8 *self_test_axis : Pointer holding the self_test_axis
 *
 *	0x00	-> self test disabled
 *	0x01	-> x-axis
 *	0x02	-> y-axis
 *	0x03	-> z-axis
 *
 *  \return  results of bus communication function
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_self_test_axis(
u8 *self_test_axis)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_SELF_TEST__REG,
			&data, C_BMA2x2_One_U8X);
			*self_test_axis = BMA2x2_GET_BITSLICE
			(data, BMA2x2_EN_SELF_TEST);
		}
	return comres;
}
/***************************************************************************
 *	Description: *//**\brief This API is for to Set the value of
 *	the self test axis(self_test_axis) in the register ox32 bit 0 to 2
 *
 *
 *
 *  \param u8 self_test_axis: The self test axis value
 *
 *	0x00	-> self test disabled
 *	0x01	-> x-axis
 *	0x02	-> y-axis
 *	0x03	-> z-axis
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 **************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_self_test_axis(
u8 self_test_axis)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		if (self_test_axis < C_BMA2x2_Four_U8X) {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_SELF_TEST__REG,
			&data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_EN_SELF_TEST, self_test_axis);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_SELF_TEST__REG,
			&data, C_BMA2x2_One_U8X);
		 } else {
		return  E_OUT_OF_RANGE;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
 *	Description: *//**\brief This API is for to get
 *	the Self Test sign(self_test_sign) in the register 0x32 bit 2
 *
 *
 *
 *  \param u8 *self_test_sign : Pointer holding the self_test_sign
 *	self_test_sign ->
 *	0 -> negative sign
 *	1 -> positive sign
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 **************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_self_test_sign(
u8 *self_test_sign)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_NEG_SELF_TEST__REG,
			&data, C_BMA2x2_One_U8X);
			*self_test_sign = BMA2x2_GET_BITSLICE
			(data, BMA2x2_NEG_SELF_TEST);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 *	Description: *//**\brief This API is for to Set the value of
 *	the self test axis(self_test_axis) in the register ox32 bit 0 to 2
 *
 *
 *
 *  \param u8 self_test_axis: The self test axis value
 *
 *	0x00	-> self test disabled
 *	0x01	-> x-axis
 *	0x02	-> y-axis
 *	0x03	-> z-axis
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_self_test_sign(
u8 self_test_sign)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		if (self_test_sign < C_BMA2x2_Two_U8X) {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_NEG_SELF_TEST__REG,
			&data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_NEG_SELF_TEST, self_test_sign);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_NEG_SELF_TEST__REG,
			&data, C_BMA2x2_One_U8X);
		} else {
		return  E_OUT_OF_RANGE;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
 * Description: *//**\brief This API is used to get
 * the status of nvm program mode(nvm_prog_mode)in the register 0x33 bit 0
 *
 *
 *
 *
 *  \param  u8 *nvmprog_mode : Pointer holding the nvmprog_mode
 *	nvmprog_mode
 *	1 -> Enable program mode
 *	0 -> Disable program mode
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 **************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 **************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_nvmprog_mode(
u8 *nvmprog_mode)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return  E_BMA2x2_NULL_PTR;
	} else {
		comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
		(p_bma2x2->dev_addr,
		BMA2x2_UNLOCK_EE_PROG_MODE__REG,
		&data, C_BMA2x2_One_U8X);
		*nvmprog_mode = BMA2x2_GET_BITSLICE
		(data, BMA2x2_UNLOCK_EE_PROG_MODE);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
 * Description: *//**\brief This API is used to set
 * the value of nvm program mode(nvm_prog_mode) in the register 0x33 bit 0
 *
 *
 *
 *  \param u8 prgmode : The nvw program mode value
 *                prgmode ->   1 -> Enable program mode
 *                             0 -> Disable program mode
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_nvmprog_mode(u8 prgmode)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
	} else {
		comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
		(p_bma2x2->dev_addr,
		BMA2x2_UNLOCK_EE_PROG_MODE__REG,
		&data, C_BMA2x2_One_U8X);
		data = BMA2x2_SET_BITSLICE
		(data, BMA2x2_UNLOCK_EE_PROG_MODE, prgmode);
		comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
		(p_bma2x2->dev_addr,
		BMA2x2_UNLOCK_EE_PROG_MODE__REG,
		&data, C_BMA2x2_One_U8X);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
 *	Description: *//**\brief This API is used to set
 *	the value of nvm program trig(nvm_prog_trig) in the register 0x33 bit 1
 *
 *
 *
 *
 *	\param u8 trig: The nvm program trig value
 *	trig
 *	1 -> trig program seq (wo)
 *	0 -> No Action
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_nvprog_trig(u8 trig)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
	} else {
		comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
		(p_bma2x2->dev_addr,
		BMA2x2_START_EE_PROG_TRIG__REG,
		&data, C_BMA2x2_One_U8X);
		data = BMA2x2_SET_BITSLICE
		(data, BMA2x2_START_EE_PROG_TRIG, trig);
		comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
		(p_bma2x2->dev_addr,
		BMA2x2_START_EE_PROG_TRIG__REG,
		&data, C_BMA2x2_One_U8X);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
 * Description: *//**\brief This API is used to get
 * the status of nvmprogram ready(nvm_rdy) in the register bit 2
 *
 *
 *
 *
 *  \param u8 *ready: The pointer holding the nvmprogram ready value
 *	ready
 *	1 -> program seq finished
 *	0 -> program seq in progress
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_nvmprog_ready(u8 *ready)
{
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	u8 data = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
	} else {
		comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
		(p_bma2x2->dev_addr,
		BMA2x2_EE_PROG_READY__REG,
		&data, C_BMA2x2_One_U8X);
		*ready = BMA2x2_GET_BITSLICE
		(data, BMA2x2_EE_PROG_READY);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
 *	Description: *//**\brief This API is used to get
 *  the status of nvm program remain(nvm_remain) in the register 0x33
 *	bit 4 to 7
 *
 *
 *
 *  \param u8 *remain:
 *        The pointer holding the nvm program remain value
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 **************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_nvmprog_remain(u8 *remain)
{
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	u8 data = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
	} else {
		comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
		(p_bma2x2->dev_addr,
		BMA2x2_EE_REMAIN__REG, &data, C_BMA2x2_One_U8X);
		*remain = BMA2x2_GET_BITSLICE
		(data, BMA2x2_EE_REMAIN);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
 *	Description: *//**\brief This API is used to get the status of spi3
 *	in the register 0x34 bit 0
 *
 *
 *
 *  \param  u8 *spi3 : Pointer holding the spi3 value
 *          spi3 ->    0 -> spi3
 *                     1 -> spi4(default)
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 **************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_spi3(u8 *spi3)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_SPI_MODE_3__REG,
			&data, C_BMA2x2_One_U8X);
			*spi3 = BMA2x2_GET_BITSLICE
			(data, BMA2x2_EN_SPI_MODE_3);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
 *	Description: *//**\brief This API is used to set the status of spi3
 *	in the register 0x34 bit 0
 *
 *
 *
 *  \param u8 spi3: The spi3 value
 *        spi3 -> 0 -> spi3
 *                1 -> spi4(default)
 *
 *
 *  \return  results of bus communication function
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 **************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_spi3(u8 spi3)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_SPI_MODE_3__REG,
			&data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_EN_SPI_MODE_3, spi3);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_SPI_MODE_3__REG,
			&data, C_BMA2x2_One_U8X);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
 *	Description: *//**\brief This API is used to get the status of i2c
 *	wdt selection(i2c_wdt_sel) and wdt enable(i2c_wdt_en) in the register
 *	0x36 bit 1 and 2
 *
 *
 *	\param u8 channel: The value of i2c wdt channel number
 *	channel --> BMA2x2_ACCEL_I2C_SELECT          0
 *				BMA2x2_ACCEL_I2C_EN              1
 *
 *  u8 *prog_mode: Pointer holding the prog_mode value
 *       prog_mode --> x,0 ->OFF
 *                     0,1 ->1 ms
 *                     1,1 ->50ms
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 **************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_i2c_wdt(u8 channel,
u8 *prog_mode)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (channel) {
		case BMA2x2_I2C_SELECT:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_I2C_WATCHDOG_PERIOD__REG,
			&data, C_BMA2x2_One_U8X);
			*prog_mode = BMA2x2_GET_BITSLICE(data,
			BMA2x2_I2C_WATCHDOG_PERIOD);
		break;
		case BMA2x2_I2C_EN:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_I2C_WATCHDOG__REG,
			&data, C_BMA2x2_One_U8X);
			*prog_mode = BMA2x2_GET_BITSLICE
			(data, BMA2x2_EN_I2C_WATCHDOG);
		break;
		default:
		comres = E_OUT_OF_RANGE;
		break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
 * Description: *//**\brief This API is used to set the value of i2c
 *	wdt selection(i2c_wdt_sel) and wdt enable(i2c_wdt_en) in the register
 *	0x36 bit 1 and 2
 *
 *
 *	\param u8 channel: The value of i2c wdt channel number
 *	channel --> BMA2x2_ACCEL_I2C_SELECT          0
 *				BMA2x2_ACCEL_I2C_EN              1
 *
 *  u8 u8 prog_mode: The orient prog_mode status value
 *	prog_mode ->
 *	x,0 ->OFF
 *	0,1 ->1 ms
 *	1,1 ->50ms
 *
 *  \return  results of bus communication function
 *
 *
 **************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 **************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_i2c_wdt(u8 channel,
u8 prog_mode)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (channel) {
		case BMA2x2_I2C_SELECT:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_I2C_WATCHDOG_PERIOD__REG,
			&data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_I2C_WATCHDOG_PERIOD, prog_mode);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_I2C_WATCHDOG_PERIOD__REG,
			&data, C_BMA2x2_One_U8X);
		break;
		case BMA2x2_I2C_EN:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_I2C_WATCHDOG__REG,
			&data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_EN_I2C_WATCHDOG, prog_mode);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_I2C_WATCHDOG__REG,
			&data, C_BMA2x2_One_U8X);
		break;
		default:
			comres = E_OUT_OF_RANGE;
		break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/**************************************************************************
 *	Description: *//**\brief This API is used to get
 *	the status slow compensation(hp_x_en, hp_y_en and hp_z_en)
 *	in the register 0x36 bit 0 to 2
 *	SLOW_COMP_X -> bit 0
 *	SLOW_COMP_Y -> bit 1
 *	SLOW_COMP_Z -> bit 2
 *
 *
 *	\param u8 channel:
 *	The value of slow compensation channel number
 *	channel --> BMA2x2_ACCEL_SLOW_COMP_X              0
 *				BMA2x2_ACCEL_SLOW_COMP_Y              1
 *				BMA2x2_ACCEL_SLOW_COMP_Z              2
 *
 *  u8 *slow_comp: Pointer holding the slow compensation value
 *	slow_comp : 1 -> enable
 *				0 -> disable slow offset
 *
 *
 *  \return  results of bus communication function
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_slow_comp(u8 channel,
u8 *slow_comp)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (channel) {
		case BMA2x2_SLOW_COMP_X:
			/*SLOW COMP X*/
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_SLOW_COMP_X__REG,
			&data, C_BMA2x2_One_U8X);
			*slow_comp = BMA2x2_GET_BITSLICE
			(data, BMA2x2_EN_SLOW_COMP_X);
		break;
		case BMA2x2_SLOW_COMP_Y:
			/*SLOW COMP Y*/
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_SLOW_COMP_Y__REG,
			&data, C_BMA2x2_One_U8X);
			*slow_comp = BMA2x2_GET_BITSLICE
			(data, BMA2x2_EN_SLOW_COMP_Y);
		break;
		case BMA2x2_SLOW_COMP_Z:
			/*SLOW COMP Z*/
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_SLOW_COMP_Z__REG,
			&data, C_BMA2x2_One_U8X);
			*slow_comp = BMA2x2_GET_BITSLICE
			(data, BMA2x2_EN_SLOW_COMP_Z);
		break;
		default:
			comres = E_OUT_OF_RANGE;
		break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*************************************************************************
 *	Description: *//**\brief This API is used to set
 *	the status slow compensation(hp_x_en, hp_y_en and hp_z_en)
 *	in the register 0x36 bit 0 to 2
 *	SLOW_COMP_X -> bit 0
 *	SLOW_COMP_Y -> bit 1
 *	SLOW_COMP_Z -> bit 2
 *
 *
 *	\param u8 channel:
 *	The value of slow compensation channel number
 *	channel --> BMA2x2_ACCEL_SLOW_COMP_X              0
 *				BMA2x2_ACCEL_SLOW_COMP_Y              1
 *				BMA2x2_ACCEL_SLOW_COMP_Z              2
 *
 *
 *	u8 slow_comp: The slow_comp status value
 *	slow_comp : 1 -> enable
 *				0 -> disable slow offset
 *
 *  \return  results of bus communication function
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_slow_comp(u8 channel,
u8 slow_comp)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (channel) {
		case BMA2x2_SLOW_COMP_X:
			/*SLOW COMP X*/
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_SLOW_COMP_X__REG,
			&data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_EN_SLOW_COMP_X, slow_comp);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_SLOW_COMP_X__REG,
			&data, C_BMA2x2_One_U8X);
		break;
		case BMA2x2_SLOW_COMP_Y:
			/*SLOW COMP Y*/
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_SLOW_COMP_Y__REG,
			&data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_EN_SLOW_COMP_Y, slow_comp);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_SLOW_COMP_Y__REG,
			&data, C_BMA2x2_One_U8X);
		break;
		case BMA2x2_SLOW_COMP_Z:
			/*SLOW COMP Z*/
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_SLOW_COMP_Z__REG,
			&data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_EN_SLOW_COMP_Z, slow_comp);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_EN_SLOW_COMP_Z__REG,
			&data, C_BMA2x2_One_U8X);
		break;
		default:
			comres = E_OUT_OF_RANGE;
		break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***********************************************************************
 *	Description: *//**\brief This API is used to get
 *	the status of fast offset compensation(cal_rdy) in the register 0x36
 *	bit 4(Read Only Possible)
 *
 *
 *
 *  \param u8: Pointer holding the cal rdy
 *
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 **************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_cal_rdy(u8 *rdy)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
		(p_bma2x2->dev_addr,
		BMA2x2_FAST_CAL_RDY_S__REG,
		&data, C_BMA2x2_One_U8X);
		*rdy = BMA2x2_GET_BITSLICE(data,
		BMA2x2_FAST_CAL_RDY_S);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
 *	Description: *//**\brief This API is used to set
 *	the status of fast offset calculation(cal_trig) in the register 0x36
 *	bit 6(Write only possible)
 *
 *
 *
 *  \param u8 cal_trig: The value of call trig
 *
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_cal_trig(u8 cal_trig)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_CAL_TRIGGER__REG, &data,
			C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE(data,
			BMA2x2_CAL_TRIGGER, cal_trig);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_CAL_TRIGGER__REG,
			&data, C_BMA2x2_One_U8X);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*********************************************************************
 *	Description: *//**\brief This API is used to set
 *	the status of offset reset(offset_reset) in the register 0x36
 *	bit 7(Write only possible)
 *
 *
 *
 *  \param u8 offset_reset: The offset reset value
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 **************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_offset_reset(u8 offset_reset)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_RESET_OFFSET_REGS__REG,
			&data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_RESET_OFFSET_REGS,
			offset_reset);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_RESET_OFFSET_REGS__REG,
			&data, C_BMA2x2_One_U8X);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
 *	Description: *//**\brief This API is used to get
 *	the status of offset target axis(offset_target_x, offset_target_y and
 *	offset_target_z) and cut_off in the register 0x37
 *	CUT_OFF -> bit 0
 *	OFFSET_TRIGGER_X -> bit 1 and 2
 *	OFFSET_TRIGGER_Y -> bit 3 and 4
 *	OFFSET_TRIGGER_Z -> bit 5 and 6
 *
 *
 *	\param u8 channel:
 *	The value of offset axis selection channel number
 *	channel --> BMA2x2_ACCEL_CUT_OFF              ->	0
 *				BMA2x2_ACCEL_OFFSET_TRIGGER_X     ->    1
 *				BMA2x2_ACCEL_OFFSET_TRIGGER_Y     ->    2
 *				BMA2x2_ACCEL_OFFSET_TRIGGER_Z     ->    3
 *
 * u8 offset: The offset target value
 *	CUT_OFF -> 0 or 1
 *	offset -->	BMA2x2_ACCEL_OFFSET_TRIGGER_X	-> 0,1,2,3
 *				BMA2x2_ACCEL_OFFSET_TRIGGER_Y	-> 0,1,2,3
 *				BMA2x2_ACCEL_OFFSET_TRIGGER_Z	-> 0,1,2,3
 *
 *  \return  results of bus communication function
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_offset_target(u8 channel,
u8 *offset)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (channel) {
		case BMA2x2_CUT_OFF:
			/*CUT-OFF*/
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_COMP_CUTOFF__REG, &data,
			C_BMA2x2_One_U8X);
			*offset = BMA2x2_GET_BITSLICE(data,
			BMA2x2_COMP_CUTOFF);
		break;
		case BMA2x2_OFFSET_TRIGGER_X:
			/*OFFSET TRIGGER X*/
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_COMP_TARGET_OFFSET_X__REG,
			&data, C_BMA2x2_One_U8X);
			*offset = BMA2x2_GET_BITSLICE(data,
			BMA2x2_COMP_TARGET_OFFSET_X);
		break;
		case BMA2x2_OFFSET_TRIGGER_Y:
			/*OFFSET TRIGGER Y*/
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_COMP_TARGET_OFFSET_Y__REG,
			&data, C_BMA2x2_One_U8X);
			*offset = BMA2x2_GET_BITSLICE(data,
			BMA2x2_COMP_TARGET_OFFSET_Y);
		break;
		case BMA2x2_OFFSET_TRIGGER_Z:
			/*OFFSET TRIGGER Z*/
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_COMP_TARGET_OFFSET_Z__REG,
			&data, C_BMA2x2_One_U8X);
			*offset = BMA2x2_GET_BITSLICE
			(data, BMA2x2_COMP_TARGET_OFFSET_Z);
		break;
		default:
			comres = E_OUT_OF_RANGE;
		break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
 *	Description: *//**\brief This API is used to set
 *	the value of offset target axis(offset_target_x, offset_target_y and
 *	offset_target_z) and cut_off in the register 0x37
 *	CUT_OFF -> bit 0
 *	OFFSET_TRIGGER_X -> bit 1 and 2
 *	OFFSET_TRIGGER_Y -> bit 3 and 4
 *	OFFSET_TRIGGER_Z -> bit 5 and 6
 *
 *
 *	\param u8 channel:
 *	The value of offset axis selection channel number
 *	channel --> BMA2x2_ACCEL_CUT_OFF              ->	0
 *				BMA2x2_ACCEL_OFFSET_TRIGGER_X     ->    1
 *				BMA2x2_ACCEL_OFFSET_TRIGGER_Y     ->    2
 *				BMA2x2_ACCEL_OFFSET_TRIGGER_Z     ->    3
 *
 * u8 offset: The offset target value
 *	CUT_OFF -> 0 or 1
 *	offset -->	BMA2x2_ACCEL_OFFSET_TRIGGER_X	-> 0,1,2,3
 *				BMA2x2_ACCEL_OFFSET_TRIGGER_Y	-> 0,1,2,3
 *				BMA2x2_ACCEL_OFFSET_TRIGGER_Z	-> 0,1,2,3
 *
 *  \return  results of bus communication function
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_offset_target(u8 channel,
u8 offset)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (channel) {
		case BMA2x2_CUT_OFF:
			/*CUT-OFF*/
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_COMP_CUTOFF__REG,
			&data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_COMP_CUTOFF, offset);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_COMP_CUTOFF__REG,
			&data, C_BMA2x2_One_U8X);
		break;
		case BMA2x2_OFFSET_TRIGGER_X:
			/*OFFSET TARGET X*/
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_COMP_TARGET_OFFSET_X__REG,
			&data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_COMP_TARGET_OFFSET_X, offset);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_COMP_TARGET_OFFSET_X__REG,
			&data, C_BMA2x2_One_U8X);
		break;
		case BMA2x2_OFFSET_TRIGGER_Y:
			/*OFFSET TARGET Y*/
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_COMP_TARGET_OFFSET_Y__REG,
			&data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_COMP_TARGET_OFFSET_Y, offset);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_COMP_TARGET_OFFSET_Y__REG,
			&data, C_BMA2x2_One_U8X);
		break;
		case BMA2x2_OFFSET_TRIGGER_Z:
			/*OFFSET TARGET Z*/
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_COMP_TARGET_OFFSET_Z__REG,
			&data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_COMP_TARGET_OFFSET_Z, offset);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_COMP_TARGET_OFFSET_Z__REG,
			&data, C_BMA2x2_One_U8X);
		break;
		default:
			comres = E_OUT_OF_RANGE;
		break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/**************************************************************************
 *	Description: *//**\brief This API is used to get the status of offset
 *	(offset_x, offset_y and offset_z) in the registers 0x38,0x39 and 0x3A
 *	offset_x -> register 0x38 bit 0 to 7
 *	offset_y -> register 0x39 bit 0 to 7
 *	offset_z -> register 0x3A bit 0 to 7
 *
 *
 *	\param u8 channel: The value of offset channel number
 *	Channel ->
 *		BMA2x2_ACCEL_X_AXIS     ->      0
 *		BMA2x2_ACCEL_Y_AXIS     ->      1
 *		BMA2x2_ACCEL_Z_AXIS     ->      2
 *
 *  u8 *offset: Pointer holding the offset value
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_offset(u8 channel,
s8 *offset)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (channel) {
		case BMA2x2_X_AXIS:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_OFFSET_X_AXIS_REG, &data, C_BMA2x2_One_U8X);
			*offset = (s8)data;
		break;
		case BMA2x2_Y_AXIS:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_OFFSET_Y_AXIS_REG, &data, C_BMA2x2_One_U8X);
			*offset = (s8)data;
		break;
		case BMA2x2_Z_AXIS:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_OFFSET_Z_AXIS_REG, &data, C_BMA2x2_One_U8X);
			*offset = (s8)data;
		break;
		default:
			comres = E_OUT_OF_RANGE;
		break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
 * Description: *//**\brief This API is used to set the value of offset
 *	(offset_x, offset_y and offset_z) in the registers 0x38,0x39 and 0x3A
 *	offset_x -> register 0x38 bit 0 to 7
 *	offset_y -> register 0x39 bit 0 to 7
 *	offset_z -> register 0x3A bit 0 to 7
 *
 *
 *	\param u8 channel: The value of offset channel number
 *	Channel ->
 *		BMA2x2_ACCEL_X_AXIS     ->      0
 *		BMA2x2_ACCEL_Y_AXIS     ->      1
 *		BMA2x2_ACCEL_Z_AXIS     ->      2
 *
 *	u8 offset: The offset value
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_offset(u8 channel,
s8 offset)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (channel) {
		case BMA2x2_X_AXIS:
			data = offset;
			comres = p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_OFFSET_X_AXIS_REG, &data, C_BMA2x2_One_U8X);
		break;
		case BMA2x2_Y_AXIS:
			data = offset;
			comres = p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_OFFSET_Y_AXIS_REG, &data, C_BMA2x2_One_U8X);
		break;
		case BMA2x2_Z_AXIS:
			data = offset;
			comres = p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_OFFSET_Z_AXIS_REG, &data, C_BMA2x2_One_U8X);
		break;
		default:
			comres = E_OUT_OF_RANGE;
		break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
 *	Description: *//**\brief This API is used to get
 *	the status of fifo mode(fifo_mode) in the register 0x3E bit 6 and 7
 *
 *
 *
 *
 *  \param u8 *fifo_mode : Pointer holding the fifo_mode
 *  \param u8 fifo_mode: The fifo mode value
 *	fifo_mode	0 --> Bypass
 *				1 --> FIFO
 *				2 --> Stream
 *				3 --> Reserved
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_fifo_mode(u8 *fifo_mode)
{
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	u8 data = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC(
			p_bma2x2->dev_addr,
			BMA2x2_FIFO_MODE__REG, &data,
			C_BMA2x2_One_U8X);
			*fifo_mode = BMA2x2_GET_BITSLICE(data,
			BMA2x2_FIFO_MODE);
		}
	return comres;
}
/*****************************************************************************
 *	Description: *//**\brief This API is used set to FIFO mode
 *	the value of fifo mode(fifo_mode) in the register 0x3E bit 6 and 7
 *
 *
 *
 *  \param u8 fifo_mode: The fifo mode value
 *	fifo_mode	0 --> Bypass
 *				1 --> FIFO
 *				2 --> Stream
 *				3 --> Reserved
 *
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_fifo_mode(u8 fifo_mode)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		if (fifo_mode < C_BMA2x2_Four_U8X) {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_FIFO_MODE__REG, &data, C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE(data,
			BMA2x2_FIFO_MODE, fifo_mode);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_FIFO_MODE__REG,
			&data, C_BMA2x2_One_U8X);
		} else {
		return  E_OUT_OF_RANGE;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 * Description: *//**\brief This API is used to get
 * the status of fifo data select in the register 0x3E bit 0 and 1
 *
 *
 *
 *
 *  \param u8 *data_sel : Pointer holding the data_sel
 *         data_sel --> [0:3]
 *         0 --> X,Y and Z (DEFAULT)
 *         1 --> Y only
 *         2 --> X only
 *         3 --> Z only
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_fifo_data_sel(u8 *data_sel)
{
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	u8 data = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC(
			p_bma2x2->dev_addr,
			BMA2x2_FIFO_DATA_SELECT__REG,
			&data, C_BMA2x2_One_U8X);
			*data_sel = BMA2x2_GET_BITSLICE(data,
			BMA2x2_FIFO_DATA_SELECT);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 *	Description: *//**\brief This API is used to set
 *	the value of fifo data select in the register 0x3E bit 0 and 1
 *
 *
 *
 *  \param u8 data_sel: The data sel value
 *         data_sel --> [0:3]
 *         0 --> X,Y and Z (DEFAULT)
 *         1 --> Y only
 *         2 --> X only
 *         3 --> Z only
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_fifo_data_sel(u8 data_sel)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		if (data_sel < C_BMA2x2_Four_U8X) {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_FIFO_DATA_SELECT__REG, &data,
			C_BMA2x2_One_U8X);
			data = BMA2x2_SET_BITSLICE
			(data, BMA2x2_FIFO_DATA_SELECT, data_sel);
			comres += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_FIFO_DATA_SELECT__REG,
			&data, C_BMA2x2_One_U8X);
		} else {
		return  E_OUT_OF_RANGE;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 *	Description: *//**\brief This API is used to get
 *	the fifo data(fifo_data_output_register) in the register 0x3F
 *
 *
 *
 *
 *
 *  \param  u8 *out_reg : Pointer holding the out_reg
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_fifo_data_out_reg(
u8 *out_reg)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			/*GET FIFO DATA OUTPUT REGISTER*/
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC(
			p_bma2x2->dev_addr,
			BMA2x2_FIFO_DATA_OUTPUT_REG,
			&data, C_BMA2x2_One_U8X);
			*out_reg = data;
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 * Description: *//**\brief This API is used to read the temperature
 *
 *
 *
 *
 *  \param s8 *temperature:
 *                 Pointer holding the temperature value
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_read_temperature(s8 *temperature)
{
	u8 data = C_BMA2x2_Zero_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC(
			p_bma2x2->dev_addr,
			BMA2x2_TEMPERATURE_REG,
			&data, C_BMA2x2_One_U8X);
			*temperature = (s8)data;
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 * Description: *//**\brief This API reads acceleration data X,Y,Z values and
 *                          Temperature data from location 02h to 08h
 *
 *
 *
 *
 *  \param bma2x2acc_data * acc : Pointer holding the bma2x2acc_data
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_read_accel_xyzt(struct bma2x2acc_data *acc)
{
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	u8 data[7] = {0, 0, 0, 0, 0, 0, 0};
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (V_BMA2x2RESOLUTION_U8R) {
		case BMA2x2_12_RESOLUTION:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr, BMA2x2_ACC_X12_LSB__REG, data, 7);

			/* read x data*/
			acc->x = (s16)((((s32)((s8)data[1]))
			<< C_BMA2x2_Eight_U8X) | (data[0] & 0xF0));
			acc->x = acc->x >> C_BMA2x2_Four_U8X;

			/* read y data*/
			acc->y = (s16)((((s32)((s8)data[3]))
			<< C_BMA2x2_Eight_U8X) | (data[2] & 0xF0));
			acc->y = acc->y >> C_BMA2x2_Four_U8X;

			/* read z data*/
			acc->z = (s16)((((s32)((s8)data[5]))
			<< C_BMA2x2_Eight_U8X) | (data[4] & 0xF0));
			acc->z = acc->z >> C_BMA2x2_Four_U8X;

			acc->temperature = (s8)data[6];
		break;
		case BMA2x2_10_RESOLUTION:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr, BMA2x2_ACC_X10_LSB__REG, data, 7);

			/* read x data*/
			acc->x = (s16)((((s32)((s8)data[1]))
			<< C_BMA2x2_Eight_U8X) | (data[0] & 0xC0));
			acc->x = acc->x >> C_BMA2x2_Six_U8X;

			/* read y data*/
			acc->y = (s16)((((s32)((s8)data[3]))
			<< C_BMA2x2_Eight_U8X) | (data[2] & 0xC0));
			acc->y = acc->y >> C_BMA2x2_Six_U8X;

			/* read z data*/
			acc->z = (s16)((((s32)((s8)data[5]))
			<< C_BMA2x2_Eight_U8X) | (data[4] & 0xC0));
			acc->z = acc->z >> C_BMA2x2_Six_U8X;

			/* read temperature data*/
			acc->temperature = (s8)data[6];
		break;
		case BMA2x2_14_RESOLUTION:
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr, BMA2x2_ACC_X14_LSB__REG, data, 7);

			/* read x data*/
			acc->x = (s16)((((s32)((s8)data[1]))
			<< C_BMA2x2_Eight_U8X) | (data[0] & 0xFC));
			acc->x = acc->x >> C_BMA2x2_Two_U8X;

			/* read y data*/
			acc->y = (s16)((((s32)((s8)data[3]))
			<< C_BMA2x2_Eight_U8X) | (data[2] & 0xFC));
			acc->y = acc->y >> C_BMA2x2_Two_U8X;

			/* read z data*/
			acc->z = (s16)((((s32)((s8)data[5]))
			<< C_BMA2x2_Eight_U8X) | (data[4] & 0xFC));
			acc->z = acc->z >> C_BMA2x2_Two_U8X;

			/* read temperature data*/
			acc->temperature = (s8)data[6];
		break;
		default:
		break;
		}
	}
	return comres;
}
/****************************************************************************
 * Description: *//**\brief This API reads acceleration of 8 bit resolution
 *                          data of X,Y,Z values
 *                          from location 03h , 05h and 07h
 *
 *
 *
 *
 *  \param bma2x2acc_t * acc : pointer holding the data of bma2x2acc_t
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_read_accel_eight_resolution_xyzt(
struct bma2x2acc_eight_resolution_t *acc)
{
	BMA2x2_RETURN_FUNCTION_TYPE comres = C_BMA2x2_Zero_U8X;
	u8	data = C_BMA2x2_Zero_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_X_AXIS_MSB_REG, &data, 1);
			acc->x = BMA2x2_GET_BITSLICE(data,
			BMA2x2_ACC_X_MSB);

			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_Y_AXIS_MSB_REG, &data, 1);
			acc->y = BMA2x2_GET_BITSLICE(data,
			BMA2x2_ACC_Y_MSB);

			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_Z_AXIS_MSB_REG, &data, 1);
			acc->z = BMA2x2_GET_BITSLICE(data,
			BMA2x2_ACC_Z_MSB);

			comres = p_bma2x2->BMA2x2_BUS_READ_FUNC(
			p_bma2x2->dev_addr,
			BMA2x2_TEMPERATURE_REG, &data,
			C_BMA2x2_One_U8X);
			acc->temperature = (s8)data;
		}
	return comres;
}
