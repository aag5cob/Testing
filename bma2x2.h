/*
 ***************************************************************************
 *
 * (C) All rights reserved by ROBERT BOSCH GMBH
 *
 **************************************************************************/
/*  Date: 2014/04/30
 *  Revision: 1.6 $
 *
 */

/**************************************************************************
* Copyright (C) 2014 Bosch Sensortec GmbH
*
* bma2x2.h
*
* Usage:        BMA2x2 Sensor Driver Support Header File
*
**************************************************************************/
/************************************************************************/
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
/******************************************************************************/
/*! \file bma2x2.h
    \brief BMA2x2 Sensor Driver Support Header File */
/* user defined code to be added here ... */
#ifndef __BMA2x2_H__
#define __BMA2x2_H__

/*******************************************************
* These definition uses for define the data types
********************************************************
*While porting the API please consider the following
*Please check the version of C standard
*Are you using Linux platform
*******************************************************/

/*********************************************************
* This definition uses for the Linux platform support
* Please use the types.h for your data types definitions
*********************************************************/
#ifdef	__KERNEL__

#include <linux/types.h>

#else /* ! __KERNEL__ */
/**********************************************************
* These definition uses for define the C
* standard version data types
***********************************************************/
# if !defined(__STDC_VERSION__)

/************************************************
 * compiler is C11 C standard
************************************************/
#if (__STDC_VERSION__ == 201112L)

/************************************************/
#include <stdint.h>
/************************************************/

/*unsigned integer types*/
#define	u8	uint8_t
#define	u16	uint16_t
#define	u32	uint32_t
#define	u64	uint64_t

/*signed integer types*/
#define	s8	int8_t
#define	s16	int16_t
#define	s32	int32_t
#define	s64	int64_t

/************************************************
 * compiler is C99 C standard
************************************************/

#elif (__STDC_VERSION__ == 199901L)

/* stdint.h is a C99 supported c library.
which is used to fixed the integer size*/
/************************************************/
#include <stdint.h>
/************************************************/

/*unsigned integer types*/
#define	u8	uint8_t
#define	u16	uint16_t
#define	u32	uint32_t
#define	u64	uint64_t

/*signed integer types*/
#define	s8	int8_t
#define	s16	int16_t
#define	s32	int32_t
#define	s64	int64_t

/************************************************
 * compiler is C89 or other C standard
************************************************/

#else /*  !defined(__STDC_VERSION__) */

/*signed integer types*/
#define	s8	signed char
#define	s16	signed short
#define	s32	signed int
#define	s64	signed long long

/*unsigned integer types*/
#define	u8  unsigned char
#define	u16	unsigned short
#define	u32	unsigned int
#define	u64	unsigned long long

#endif

#else

/*signed integer types*/
#define	s8	signed char
#define	s16	signed short
#define	s32	signed int
#define	s64	signed long long

/*unsigned integer types*/
#define	u8  unsigned char
#define	u16	unsigned short
#define	u32	unsigned int
#define	u64	unsigned long long
#endif
#endif

/*Example....
* #define YOUR_H_DEFINE  < <Doxy Comment for YOUR_H_DEFINE>
* Define the calling convention of YOUR bus communication routine.
* \note This includes types of parameters.
* This example shows the configuration for an SPI bus link.

* If your communication function looks like this:

* write_my_bus_xy(u8 device_addr,
* u8 register_addr, u8 * data, u8 length);

* The BMA2x2_WR_FUNC_PTR would equal:

* #define     BMA2x2_WR_FUNC_PTR char
* (* bus_write)(u8, u8, u8 *, u8)

* Parameters can be mixed as needed refer to
* the \ref BMA2x2_BUS_WRITE_FUNC  macro.
*/
#define BMA2x2_WR_FUNC_PTR s8(*bus_write)\
(u8, u8, u8 *, u8)



/** link macro between API function calls and bus write function
*  \note The bus write function can change since
* this is a system dependant issue.

* If the bus_write parameter calling order is like:
* reg_addr, reg_data, wr_len it would be as it is here.

* If the parameters are differently ordered or your communication function
* like I2C need to know the device address,
* you can change this macro accordingly.


* define BMA2x2_BUS_WRITE_FUNC(dev_addr, reg_addr, reg_data, wr_len)\
* bus_write(dev_addr, reg_addr, reg_data, wr_len)

* This macro lets all API functions call YOUR communication routine in
* a way that equals your definition in the
* \ref BMA2x2_WR_FUNC_PTR definition.

*/




#define BMA2x2_BUS_WRITE_FUNC(dev_addr, reg_addr, reg_data, wr_len)\
bus_write(dev_addr, reg_addr, reg_data, wr_len)


/** Define the calling convention of YOUR bus communication routine.
*\note This includes types of parameters. This example
*shows the configuration for an SPI bus link.

*If your communication function looks like this:

*read_my_bus_xy(u8 device_addr,
*u8 register_addr, u8* data, u8 length);

*The BMA2x2_RD_FUNC_PTR would equal:

*#define     BMA2x2_RD_FUNC_PTR s8
*(* bus_read)(u8, u8, u8*, u8)

*Parameters can be mixed as needed refer to the
\ref BMA2x2_BUS_READ_FUNC  macro.
*/

#define BMA2x2_SPI_RD_MASK 0x80
/* for spi read transactions on SPI the MSB has to be set */
#define BMA2x2_RD_FUNC_PTR s8(*bus_read)\
(u8, u8, u8 *, u8)
#define BMA2x2_BRD_FUNC_PTR s8(*burst_read)\
(u8, u8, u8 *, u32)


/** link macro between API function calls and bus read function
* \note The bus write function can change since
* this is a system dependant issue.

* If the bus_read parameter calling order is like:
* reg_addr, reg_data, wr_len it would be as it is here.

*  If the parameters are differently ordered or your
*  communication function like I2C need to know the device address,
*  you can change this macro accordingly.


*  define BMA2x2_BUS_READ_FUNC(dev_addr, reg_addr, reg_data, wr_len)\
* bus_read(dev_addr, reg_addr, reg_data, wr_len)

* This macro lets all API functions call YOUR
* communication routine in a way that equals your definition in the
* \ref BMA2x2_WR_FUNC_PTR definition.

* \note: this macro also includes the "MSB='1'" for reading BMA2x2 addresses.
*/


#define BMA2x2_BUS_READ_FUNC(dev_addr, reg_addr, reg_data, r_len)\
bus_read(dev_addr, reg_addr, reg_data, r_len)
#define BMA2x2_BURST_READ_FUNC(device_addr,\
register_addr, register_data, rd_len)\
burst_read(device_addr, register_addr, register_data, rd_len)

/* The following definition of I2C address is used for the following sensors
* BMA255
* BMA355
* BMA280
* BMA282
* BMA223
* BMA254
* BMA284
* BMA250E
* BMA222E
*/
#define BMA2x2_I2C_ADDR1                0x18
#define BMA2x2_I2C_ADDR2                0x19

/* The following definition of I2C address is used for the following sensors
* BMC150
* BMC056
* BMC156
*/
#define BMA2x2_I2C_ADDR3                0x10
#define BMA2x2_I2C_ADDR4                0x11

#define         C_BMA2x2_Zero_U8X                       ((u8)0)
#define         C_BMA2x2_One_U8X                        ((u8)1)
#define         C_BMA2x2_Two_U8X                        ((u8)2)
#define         C_BMA2x2_Three_U8X                      ((u8)3)
#define         C_BMA2x2_Four_U8X                       ((u8)4)
#define         C_BMA2x2_Five_U8X                       ((u8)5)
#define         C_BMA2x2_Six_U8X                        ((u8)6)
#define         C_BMA2x2_Seven_U8X                      ((u8)7)
#define         C_BMA2x2_Eight_U8X                      ((u8)8)
#define         C_BMA2x2_Nine_U8X                       ((u8)9)
#define         C_BMA2x2_Twelve_U8X                     ((u8)12)
#define         C_BMA2x2_Fifteen_U8X                    ((u8)15)
#define         C_BMA2x2_Sixteen_U8X                    ((u8)16)
#define         C_BMA2x2_ThirtyTwo_U8X                  ((u8)32)



/*
API error codes
*/
#define E_OUT_OF_RANGE          ((s8)-2)
#define E_BMA2x2_NULL_PTR       ((s8)-127)
#define BMA2x2_NULL             ((u8)0)

#define	BMA2x2_RETURN_FUNCTION_TYPE        s8
/**< This refers BMA2x2 return type as char */



 /*  register definitions */
#define BMA2x2_EEP_OFFSET                       0x16
#define BMA2x2_IMAGE_BASE                       0x38
#define BMA2x2_IMAGE_LEN                        22


#define BMA2x2_CHIP_ID_REG                      0x00
#define BMA2x2_X_AXIS_LSB_REG                   0x02
#define BMA2x2_X_AXIS_MSB_REG                   0x03
#define BMA2x2_Y_AXIS_LSB_REG                   0x04
#define BMA2x2_Y_AXIS_MSB_REG                   0x05
#define BMA2x2_Z_AXIS_LSB_REG                   0x06
#define BMA2x2_Z_AXIS_MSB_REG                   0x07
#define BMA2x2_TEMPERATURE_REG                  0x08
#define BMA2x2_STATUS1_REG                      0x09
#define BMA2x2_STATUS2_REG                      0x0A
#define BMA2x2_STATUS_TAP_SLOPE_REG             0x0B
#define BMA2x2_STATUS_ORIENT_HIGH_REG           0x0C
#define BMA2x2_STATUS_FIFO_REG                  0x0E
#define BMA2x2_RANGE_SEL_REG                    0x0F
#define BMA2x2_BW_SEL_REG                       0x10
#define BMA2x2_MODE_CTRL_REG                    0x11
#define BMA2x2_LOW_NOISE_CTRL_REG               0x12
#define BMA2x2_DATA_CTRL_REG                    0x13
#define BMA2x2_RESET_REG                        0x14
#define BMA2x2_INT_ENABLE1_REG                  0x16
#define BMA2x2_INT_ENABLE2_REG                  0x17
#define BMA2x2_INT_SLO_NO_MOT_REG               0x18
#define BMA2x2_INT1_PAD_SEL_REG                 0x19
#define BMA2x2_INT_DATA_SEL_REG                 0x1A
#define BMA2x2_INT2_PAD_SEL_REG                 0x1B
#define BMA2x2_INT_SRC_REG                      0x1E
#define BMA2x2_INT_SET_REG                      0x20
#define BMA2x2_INT_CTRL_REG                     0x21
#define BMA2x2_LOW_DURN_REG                     0x22
#define BMA2x2_LOW_THRES_REG                    0x23
#define BMA2x2_LOW_HIGH_HYST_REG                0x24
#define BMA2x2_HIGH_DURN_REG                    0x25
#define BMA2x2_HIGH_THRES_REG                   0x26
#define BMA2x2_SLOPE_DURN_REG                   0x27
#define BMA2x2_SLOPE_THRES_REG                  0x28
#define BMA2x2_SLO_NO_MOT_THRES_REG             0x29
#define BMA2x2_TAP_PARAM_REG                    0x2A
#define BMA2x2_TAP_THRES_REG                    0x2B
#define BMA2x2_ORIENT_PARAM_REG                 0x2C
#define BMA2x2_THETA_BLOCK_REG                  0x2D
#define BMA2x2_THETA_FLAT_REG                   0x2E
#define BMA2x2_FLAT_HOLD_TIME_REG               0x2F
#define BMA2x2_FIFO_WML_TRIG                    0x30
#define BMA2x2_SELF_TEST_REG                    0x32
#define BMA2x2_EEPROM_CTRL_REG                  0x33
#define BMA2x2_SERIAL_CTRL_REG                  0x34
#define BMA2x2_OFFSET_CTRL_REG                  0x36
#define BMA2x2_OFFSET_PARAMS_REG                0x37
#define BMA2x2_OFFSET_X_AXIS_REG                0x38
#define BMA2x2_OFFSET_Y_AXIS_REG                0x39
#define BMA2x2_OFFSET_Z_AXIS_REG                0x3A
#define BMA2x2_GP0_REG                          0x3B
#define BMA2x2_GP1_REG                          0x3C
#define BMA2x2_FIFO_MODE_REG                    0x3E
#define BMA2x2_FIFO_DATA_OUTPUT_REG             0x3F

#define BMA2x2_12_RESOLUTION                    0
#define BMA2x2_10_RESOLUTION                    1
#define BMA2x2_14_RESOLUTION                    2

/* register write and read delays */

#define BMA2x2_MDELAY_DATA_TYPE                 u32
#define BMA2x2_EE_W_DELAY                       28

/* delay after EEP write is 28 msec
* bma2x2 acceleration data
* brief Structure containing acceleration values
* for x,y and z-axis in S16
*/

struct  bma2x2acc_t {
s16 x,
y,
z;
};

struct bma2x2acc_data {
s16 x,
y,
z;
s8 temperature;
};


struct  bma2x2acc_eight_resolution {
s8 x,
y,
z;
};

struct bma2x2acc_eight_resolution_t {
s8 x,
y,
z;
s8 temperature;
};

/****************************************************************************
 *	struct bma2x2_t is used for assigning the following parameters.
 *
 *	Bus write function pointer: BMA2x2_WR_FUNC_PTR
 *	Bus read function pointer: BMA2x2_RD_FUNC_PTR
 *	Burst read function pointer: BMA2x2_BRD_FUNC_PTR
 *	Delay function pointer: delay_msec
 *
 *	I2C address: dev_addr
 *	Chip id of the sensor: chip_id
****************************************************************************/

struct bma2x2_t {
u8 mode;
/**< save current bma2x2 operation mode */
u8 chip_id;
/**< save bma2x2's chip id which has to be*/
u8 dev_addr;
/**< initializes bma2x2's I2C device address*/
BMA2x2_WR_FUNC_PTR;
/**< function pointer to the SPI/I2C write function */
BMA2x2_RD_FUNC_PTR;
/**< function pointer to the SPI/I2C read function */
BMA2x2_BRD_FUNC_PTR;

void(*delay_msec)(BMA2x2_MDELAY_DATA_TYPE);
/**< function pointer to a pause in mili seconds function */
};


#define BMA2x2_CHIP_ID__POS             0
#define BMA2x2_CHIP_ID__MSK             0xFF
#define BMA2x2_CHIP_ID__LEN             8
#define BMA2x2_CHIP_ID__REG             BMA2x2_CHIP_ID_REG

/* DATA REGISTERS */
#define BMA2x2_NEW_DATA_X__POS          0
#define BMA2x2_NEW_DATA_X__LEN          1
#define BMA2x2_NEW_DATA_X__MSK          0x01
#define BMA2x2_NEW_DATA_X__REG          BMA2x2_X_AXIS_LSB_REG

#define BMA2x2_ACC_X14_LSB__POS           2
#define BMA2x2_ACC_X14_LSB__LEN           6
#define BMA2x2_ACC_X14_LSB__MSK           0xFC
#define BMA2x2_ACC_X14_LSB__REG           BMA2x2_X_AXIS_LSB_REG

#define BMA2x2_ACC_X12_LSB__POS           4
#define BMA2x2_ACC_X12_LSB__LEN           4
#define BMA2x2_ACC_X12_LSB__MSK           0xF0
#define BMA2x2_ACC_X12_LSB__REG           BMA2x2_X_AXIS_LSB_REG

#define BMA2x2_ACC_X10_LSB__POS           6
#define BMA2x2_ACC_X10_LSB__LEN           2
#define BMA2x2_ACC_X10_LSB__MSK           0xC0
#define BMA2x2_ACC_X10_LSB__REG           BMA2x2_X_AXIS_LSB_REG

#define BMA2x2_ACC_X8_LSB__POS           0
#define BMA2x2_ACC_X8_LSB__LEN           0
#define BMA2x2_ACC_X8_LSB__MSK           0x00
#define BMA2x2_ACC_X8_LSB__REG           BMA2x2_X_AXIS_LSB_REG

#define BMA2x2_ACC_X_MSB__POS           0
#define BMA2x2_ACC_X_MSB__LEN           8
#define BMA2x2_ACC_X_MSB__MSK           0xFF
#define BMA2x2_ACC_X_MSB__REG           BMA2x2_X_AXIS_MSB_REG

#define BMA2x2_NEW_DATA_Y__POS          0
#define BMA2x2_NEW_DATA_Y__LEN          1
#define BMA2x2_NEW_DATA_Y__MSK          0x01
#define BMA2x2_NEW_DATA_Y__REG          BMA2x2_Y_AXIS_LSB_REG

#define BMA2x2_ACC_Y14_LSB__POS           2
#define BMA2x2_ACC_Y14_LSB__LEN           6
#define BMA2x2_ACC_Y14_LSB__MSK           0xFC
#define BMA2x2_ACC_Y14_LSB__REG           BMA2x2_Y_AXIS_LSB_REG

#define BMA2x2_ACC_Y12_LSB__POS           4
#define BMA2x2_ACC_Y12_LSB__LEN           4
#define BMA2x2_ACC_Y12_LSB__MSK           0xF0
#define BMA2x2_ACC_Y12_LSB__REG           BMA2x2_Y_AXIS_LSB_REG

#define BMA2x2_ACC_Y10_LSB__POS           6
#define BMA2x2_ACC_Y10_LSB__LEN           2
#define BMA2x2_ACC_Y10_LSB__MSK           0xC0
#define BMA2x2_ACC_Y10_LSB__REG           BMA2x2_Y_AXIS_LSB_REG

#define BMA2x2_ACC_Y8_LSB__POS           0
#define BMA2x2_ACC_Y8_LSB__LEN           0
#define BMA2x2_ACC_Y8_LSB__MSK           0x00
#define BMA2x2_ACC_Y8_LSB__REG           BMA2x2_Y_AXIS_LSB_REG

#define BMA2x2_ACC_Y_MSB__POS           0
#define BMA2x2_ACC_Y_MSB__LEN           8
#define BMA2x2_ACC_Y_MSB__MSK           0xFF
#define BMA2x2_ACC_Y_MSB__REG           BMA2x2_Y_AXIS_MSB_REG

#define BMA2x2_NEW_DATA_Z__POS          0
#define BMA2x2_NEW_DATA_Z__LEN          1
#define BMA2x2_NEW_DATA_Z__MSK          0x01
#define BMA2x2_NEW_DATA_Z__REG          BMA2x2_Z_AXIS_LSB_REG

#define BMA2x2_ACC_Z14_LSB__POS           2
#define BMA2x2_ACC_Z14_LSB__LEN           6
#define BMA2x2_ACC_Z14_LSB__MSK           0xFC
#define BMA2x2_ACC_Z14_LSB__REG           BMA2x2_Z_AXIS_LSB_REG

#define BMA2x2_ACC_Z12_LSB__POS           4
#define BMA2x2_ACC_Z12_LSB__LEN           4
#define BMA2x2_ACC_Z12_LSB__MSK           0xF0
#define BMA2x2_ACC_Z12_LSB__REG           BMA2x2_Z_AXIS_LSB_REG

#define BMA2x2_ACC_Z10_LSB__POS           6
#define BMA2x2_ACC_Z10_LSB__LEN           2
#define BMA2x2_ACC_Z10_LSB__MSK           0xC0
#define BMA2x2_ACC_Z10_LSB__REG           BMA2x2_Z_AXIS_LSB_REG

#define BMA2x2_ACC_Z8_LSB__POS           0
#define BMA2x2_ACC_Z8_LSB__LEN           0
#define BMA2x2_ACC_Z8_LSB__MSK           0x00
#define BMA2x2_ACC_Z8_LSB__REG           BMA2x2_Z_AXIS_LSB_REG

#define BMA2x2_ACC_Z_MSB__POS           0
#define BMA2x2_ACC_Z_MSB__LEN           8
#define BMA2x2_ACC_Z_MSB__MSK           0xFF
#define BMA2x2_ACC_Z_MSB__REG           BMA2x2_Z_AXIS_MSB_REG

/* Temperature */
#define BMA2x2_ACC_TEMP_MSB__POS           0
#define BMA2x2_ACC_TEMP_MSB__LEN           8
#define BMA2x2_ACC_TEMP_MSB__MSK           0xFF
#define BMA2x2_ACC_TEMP_MSB__REG           BMA2x2_TEMPERATURE_REG

/*  INTERRUPT STATUS BITS  */
#define BMA2x2_LOWG_INT_S__POS          0
#define BMA2x2_LOWG_INT_S__LEN          1
#define BMA2x2_LOWG_INT_S__MSK          0x01
#define BMA2x2_LOWG_INT_S__REG          BMA2x2_STATUS1_REG

#define BMA2x2_HIGHG_INT_S__POS          1
#define BMA2x2_HIGHG_INT_S__LEN          1
#define BMA2x2_HIGHG_INT_S__MSK          0x02
#define BMA2x2_HIGHG_INT_S__REG          BMA2x2_STATUS1_REG

#define BMA2x2_SLOPE_INT_S__POS          2
#define BMA2x2_SLOPE_INT_S__LEN          1
#define BMA2x2_SLOPE_INT_S__MSK          0x04
#define BMA2x2_SLOPE_INT_S__REG          BMA2x2_STATUS1_REG


#define BMA2x2_SLO_NO_MOT_INT_S__POS          3
#define BMA2x2_SLO_NO_MOT_INT_S__LEN          1
#define BMA2x2_SLO_NO_MOT_INT_S__MSK          0x08
#define BMA2x2_SLO_NO_MOT_INT_S__REG          BMA2x2_STATUS1_REG

#define BMA2x2_DOUBLE_TAP_INT_S__POS     4
#define BMA2x2_DOUBLE_TAP_INT_S__LEN     1
#define BMA2x2_DOUBLE_TAP_INT_S__MSK     0x10
#define BMA2x2_DOUBLE_TAP_INT_S__REG     BMA2x2_STATUS1_REG

#define BMA2x2_SINGLE_TAP_INT_S__POS     5
#define BMA2x2_SINGLE_TAP_INT_S__LEN     1
#define BMA2x2_SINGLE_TAP_INT_S__MSK     0x20
#define BMA2x2_SINGLE_TAP_INT_S__REG     BMA2x2_STATUS1_REG

#define BMA2x2_ORIENT_INT_S__POS         6
#define BMA2x2_ORIENT_INT_S__LEN         1
#define BMA2x2_ORIENT_INT_S__MSK         0x40
#define BMA2x2_ORIENT_INT_S__REG         BMA2x2_STATUS1_REG

#define BMA2x2_FLAT_INT_S__POS           7
#define BMA2x2_FLAT_INT_S__LEN           1
#define BMA2x2_FLAT_INT_S__MSK           0x80
#define BMA2x2_FLAT_INT_S__REG           BMA2x2_STATUS1_REG

#define BMA2x2_FIFO_FULL_INT_S__POS           5
#define BMA2x2_FIFO_FULL_INT_S__LEN           1
#define BMA2x2_FIFO_FULL_INT_S__MSK           0x20
#define BMA2x2_FIFO_FULL_INT_S__REG           BMA2x2_STATUS2_REG

#define BMA2x2_FIFO_WM_INT_S__POS           6
#define BMA2x2_FIFO_WM_INT_S__LEN           1
#define BMA2x2_FIFO_WM_INT_S__MSK           0x40
#define BMA2x2_FIFO_WM_INT_S__REG           BMA2x2_STATUS2_REG

#define BMA2x2_DATA_INT_S__POS           7
#define BMA2x2_DATA_INT_S__LEN           1
#define BMA2x2_DATA_INT_S__MSK           0x80
#define BMA2x2_DATA_INT_S__REG           BMA2x2_STATUS2_REG

#define BMA2x2_SLOPE_FIRST_X__POS        0
#define BMA2x2_SLOPE_FIRST_X__LEN        1
#define BMA2x2_SLOPE_FIRST_X__MSK        0x01
#define BMA2x2_SLOPE_FIRST_X__REG        BMA2x2_STATUS_TAP_SLOPE_REG

#define BMA2x2_SLOPE_FIRST_Y__POS        1
#define BMA2x2_SLOPE_FIRST_Y__LEN        1
#define BMA2x2_SLOPE_FIRST_Y__MSK        0x02
#define BMA2x2_SLOPE_FIRST_Y__REG        BMA2x2_STATUS_TAP_SLOPE_REG

#define BMA2x2_SLOPE_FIRST_Z__POS        2
#define BMA2x2_SLOPE_FIRST_Z__LEN        1
#define BMA2x2_SLOPE_FIRST_Z__MSK        0x04
#define BMA2x2_SLOPE_FIRST_Z__REG        BMA2x2_STATUS_TAP_SLOPE_REG

#define BMA2x2_SLOPE_SIGN_S__POS         3
#define BMA2x2_SLOPE_SIGN_S__LEN         1
#define BMA2x2_SLOPE_SIGN_S__MSK         0x08
#define BMA2x2_SLOPE_SIGN_S__REG         BMA2x2_STATUS_TAP_SLOPE_REG

#define BMA2x2_TAP_FIRST_X__POS        4
#define BMA2x2_TAP_FIRST_X__LEN        1
#define BMA2x2_TAP_FIRST_X__MSK        0x10
#define BMA2x2_TAP_FIRST_X__REG        BMA2x2_STATUS_TAP_SLOPE_REG

#define BMA2x2_TAP_FIRST_Y__POS        5
#define BMA2x2_TAP_FIRST_Y__LEN        1
#define BMA2x2_TAP_FIRST_Y__MSK        0x20
#define BMA2x2_TAP_FIRST_Y__REG        BMA2x2_STATUS_TAP_SLOPE_REG

#define BMA2x2_TAP_FIRST_Z__POS        6
#define BMA2x2_TAP_FIRST_Z__LEN        1
#define BMA2x2_TAP_FIRST_Z__MSK        0x40
#define BMA2x2_TAP_FIRST_Z__REG        BMA2x2_STATUS_TAP_SLOPE_REG

#define BMA2x2_TAP_SIGN_S__POS         7
#define BMA2x2_TAP_SIGN_S__LEN         1
#define BMA2x2_TAP_SIGN_S__MSK         0x80
#define BMA2x2_TAP_SIGN_S__REG         BMA2x2_STATUS_TAP_SLOPE_REG

#define BMA2x2_HIGHG_FIRST_X__POS        0
#define BMA2x2_HIGHG_FIRST_X__LEN        1
#define BMA2x2_HIGHG_FIRST_X__MSK        0x01
#define BMA2x2_HIGHG_FIRST_X__REG        BMA2x2_STATUS_ORIENT_HIGH_REG

#define BMA2x2_HIGHG_FIRST_Y__POS        1
#define BMA2x2_HIGHG_FIRST_Y__LEN        1
#define BMA2x2_HIGHG_FIRST_Y__MSK        0x02
#define BMA2x2_HIGHG_FIRST_Y__REG        BMA2x2_STATUS_ORIENT_HIGH_REG

#define BMA2x2_HIGHG_FIRST_Z__POS        2
#define BMA2x2_HIGHG_FIRST_Z__LEN        1
#define BMA2x2_HIGHG_FIRST_Z__MSK        0x04
#define BMA2x2_HIGHG_FIRST_Z__REG        BMA2x2_STATUS_ORIENT_HIGH_REG

#define BMA2x2_HIGHG_SIGN_S__POS         3
#define BMA2x2_HIGHG_SIGN_S__LEN         1
#define BMA2x2_HIGHG_SIGN_S__MSK         0x08
#define BMA2x2_HIGHG_SIGN_S__REG         BMA2x2_STATUS_ORIENT_HIGH_REG

#define BMA2x2_ORIENT_S__POS             4
#define BMA2x2_ORIENT_S__LEN             3
#define BMA2x2_ORIENT_S__MSK             0x70
#define BMA2x2_ORIENT_S__REG             BMA2x2_STATUS_ORIENT_HIGH_REG

#define BMA2x2_FLAT_S__POS               7
#define BMA2x2_FLAT_S__LEN               1
#define BMA2x2_FLAT_S__MSK               0x80
#define BMA2x2_FLAT_S__REG               BMA2x2_STATUS_ORIENT_HIGH_REG

/*FIFO_STATUS*/
#define BMA2x2_FIFO_FRAME_COUNTER_S__POS             0
#define BMA2x2_FIFO_FRAME_COUNTER_S__LEN             7
#define BMA2x2_FIFO_FRAME_COUNTER_S__MSK             0x7F
#define BMA2x2_FIFO_FRAME_COUNTER_S__REG             BMA2x2_STATUS_FIFO_REG

#define BMA2x2_FIFO_OVERRUN_S__POS             7
#define BMA2x2_FIFO_OVERRUN_S__LEN             1
#define BMA2x2_FIFO_OVERRUN_S__MSK             0x80
#define BMA2x2_FIFO_OVERRUN_S__REG             BMA2x2_STATUS_FIFO_REG

#define BMA2x2_RANGE_SEL__POS             0
#define BMA2x2_RANGE_SEL__LEN             4
#define BMA2x2_RANGE_SEL__MSK             0x0F
#define BMA2x2_RANGE_SEL__REG             BMA2x2_RANGE_SEL_REG

#define BMA2x2_BANDWIDTH__POS             0
#define BMA2x2_BANDWIDTH__LEN             5
#define BMA2x2_BANDWIDTH__MSK             0x1F
#define BMA2x2_BANDWIDTH__REG             BMA2x2_BW_SEL_REG

#define BMA2x2_SLEEP_DUR__POS             1
#define BMA2x2_SLEEP_DUR__LEN             4
#define BMA2x2_SLEEP_DUR__MSK             0x1E
#define BMA2x2_SLEEP_DUR__REG             BMA2x2_MODE_CTRL_REG

#define BMA2x2_MODE_CTRL__POS             5
#define BMA2x2_MODE_CTRL__LEN             3
#define BMA2x2_MODE_CTRL__MSK             0xE0
#define BMA2x2_MODE_CTRL__REG             BMA2x2_MODE_CTRL_REG


#define BMA2x2_EN_LOW_POWER__POS          6
#define BMA2x2_EN_LOW_POWER__LEN          1
#define BMA2x2_EN_LOW_POWER__MSK          0x40
#define BMA2x2_EN_LOW_POWER__REG          BMA2x2_MODE_CTRL_REG



#define BMA2x2_SLEEP_TIMER__POS          5
#define BMA2x2_SLEEP_TIMER__LEN          1
#define BMA2x2_SLEEP_TIMER__MSK          0x20
#define BMA2x2_SLEEP_TIMER__REG          BMA2x2_LOW_NOISE_CTRL_REG


#define BMA2x2_LOW_POWER_MODE__POS          6
#define BMA2x2_LOW_POWER_MODE__LEN          1
#define BMA2x2_LOW_POWER_MODE__MSK          0x40
#define BMA2x2_LOW_POWER_MODE__REG          BMA2x2_LOW_NOISE_CTRL_REG
/**     DISABLE MSB SHADOWING PROCEDURE          **/
#define BMA2x2_DIS_SHADOW_PROC__POS       6
#define BMA2x2_DIS_SHADOW_PROC__LEN       1
#define BMA2x2_DIS_SHADOW_PROC__MSK       0x40
#define BMA2x2_DIS_SHADOW_PROC__REG       BMA2x2_DATA_CTRL_REG

/**     FILTERED OR UNFILTERED ACCELERATION DATA  **/
#define BMA2x2_EN_DATA_HIGH_BW__POS         7
#define BMA2x2_EN_DATA_HIGH_BW__LEN         1
#define BMA2x2_EN_DATA_HIGH_BW__MSK         0x80
#define BMA2x2_EN_DATA_HIGH_BW__REG         BMA2x2_DATA_CTRL_REG

#define BMA2x2_EN_SOFT_RESET_VALUE        0xB6

/**     INTERRUPT ENABLE REGISTER              **/
#define BMA2x2_EN_SLOPE_X_INT__POS         0
#define BMA2x2_EN_SLOPE_X_INT__LEN         1
#define BMA2x2_EN_SLOPE_X_INT__MSK         0x01
#define BMA2x2_EN_SLOPE_X_INT__REG         BMA2x2_INT_ENABLE1_REG

#define BMA2x2_EN_SLOPE_Y_INT__POS         1
#define BMA2x2_EN_SLOPE_Y_INT__LEN         1
#define BMA2x2_EN_SLOPE_Y_INT__MSK         0x02
#define BMA2x2_EN_SLOPE_Y_INT__REG         BMA2x2_INT_ENABLE1_REG

#define BMA2x2_EN_SLOPE_Z_INT__POS         2
#define BMA2x2_EN_SLOPE_Z_INT__LEN         1
#define BMA2x2_EN_SLOPE_Z_INT__MSK         0x04
#define BMA2x2_EN_SLOPE_Z_INT__REG         BMA2x2_INT_ENABLE1_REG

#define BMA2x2_EN_DOUBLE_TAP_INT__POS      4
#define BMA2x2_EN_DOUBLE_TAP_INT__LEN      1
#define BMA2x2_EN_DOUBLE_TAP_INT__MSK      0x10
#define BMA2x2_EN_DOUBLE_TAP_INT__REG      BMA2x2_INT_ENABLE1_REG

#define BMA2x2_EN_SINGLE_TAP_INT__POS      5
#define BMA2x2_EN_SINGLE_TAP_INT__LEN      1
#define BMA2x2_EN_SINGLE_TAP_INT__MSK      0x20
#define BMA2x2_EN_SINGLE_TAP_INT__REG      BMA2x2_INT_ENABLE1_REG

#define BMA2x2_EN_ORIENT_INT__POS          6
#define BMA2x2_EN_ORIENT_INT__LEN          1
#define BMA2x2_EN_ORIENT_INT__MSK          0x40
#define BMA2x2_EN_ORIENT_INT__REG          BMA2x2_INT_ENABLE1_REG

#define BMA2x2_EN_FLAT_INT__POS            7
#define BMA2x2_EN_FLAT_INT__LEN            1
#define BMA2x2_EN_FLAT_INT__MSK            0x80
#define BMA2x2_EN_FLAT_INT__REG            BMA2x2_INT_ENABLE1_REG

/**     INTERRUPT ENABLE REGISTER              **/
#define BMA2x2_EN_HIGHG_X_INT__POS         0
#define BMA2x2_EN_HIGHG_X_INT__LEN         1
#define BMA2x2_EN_HIGHG_X_INT__MSK         0x01
#define BMA2x2_EN_HIGHG_X_INT__REG         BMA2x2_INT_ENABLE2_REG

#define BMA2x2_EN_HIGHG_Y_INT__POS         1
#define BMA2x2_EN_HIGHG_Y_INT__LEN         1
#define BMA2x2_EN_HIGHG_Y_INT__MSK         0x02
#define BMA2x2_EN_HIGHG_Y_INT__REG         BMA2x2_INT_ENABLE2_REG

#define BMA2x2_EN_HIGHG_Z_INT__POS         2
#define BMA2x2_EN_HIGHG_Z_INT__LEN         1
#define BMA2x2_EN_HIGHG_Z_INT__MSK         0x04
#define BMA2x2_EN_HIGHG_Z_INT__REG         BMA2x2_INT_ENABLE2_REG

#define BMA2x2_EN_LOWG_INT__POS            3
#define BMA2x2_EN_LOWG_INT__LEN            1
#define BMA2x2_EN_LOWG_INT__MSK            0x08
#define BMA2x2_EN_LOWG_INT__REG            BMA2x2_INT_ENABLE2_REG

#define BMA2x2_EN_NEW_DATA_INT__POS        4
#define BMA2x2_EN_NEW_DATA_INT__LEN        1
#define BMA2x2_EN_NEW_DATA_INT__MSK        0x10
#define BMA2x2_EN_NEW_DATA_INT__REG        BMA2x2_INT_ENABLE2_REG


#define BMA2x2_INT_FFULL_EN_INT__POS        5
#define BMA2x2_INT_FFULL_EN_INT__LEN        1
#define BMA2x2_INT_FFULL_EN_INT__MSK        0x20
#define BMA2x2_INT_FFULL_EN_INT__REG        BMA2x2_INT_ENABLE2_REG

#define BMA2x2_INT_FWM_EN_INT__POS        6
#define BMA2x2_INT_FWM_EN_INT__LEN        1
#define BMA2x2_INT_FWM_EN_INT__MSK        0x40
#define BMA2x2_INT_FWM_EN_INT__REG        BMA2x2_INT_ENABLE2_REG

/*INT SLO NO MOT*/
#define BMA2x2_INT_SLO_NO_MOT_EN_X_INT__POS        0
#define BMA2x2_INT_SLO_NO_MOT_EN_X_INT__LEN        1
#define BMA2x2_INT_SLO_NO_MOT_EN_X_INT__MSK        0x01
#define BMA2x2_INT_SLO_NO_MOT_EN_X_INT__REG        BMA2x2_INT_SLO_NO_MOT_REG

#define BMA2x2_INT_SLO_NO_MOT_EN_Y_INT__POS        1
#define BMA2x2_INT_SLO_NO_MOT_EN_Y_INT__LEN        1
#define BMA2x2_INT_SLO_NO_MOT_EN_Y_INT__MSK        0x02
#define BMA2x2_INT_SLO_NO_MOT_EN_Y_INT__REG        BMA2x2_INT_SLO_NO_MOT_REG

#define BMA2x2_INT_SLO_NO_MOT_EN_Z_INT__POS        2
#define BMA2x2_INT_SLO_NO_MOT_EN_Z_INT__LEN        1
#define BMA2x2_INT_SLO_NO_MOT_EN_Z_INT__MSK        0x04
#define BMA2x2_INT_SLO_NO_MOT_EN_Z_INT__REG        BMA2x2_INT_SLO_NO_MOT_REG

#define BMA2x2_INT_SLO_NO_MOT_EN_SEL_INT__POS        3
#define BMA2x2_INT_SLO_NO_MOT_EN_SEL_INT__LEN        1
#define BMA2x2_INT_SLO_NO_MOT_EN_SEL_INT__MSK        0x08
#define BMA2x2_INT_SLO_NO_MOT_EN_SEL_INT__REG        BMA2x2_INT_SLO_NO_MOT_REG

#define BMA2x2_EN_INT1_PAD_LOWG__POS        0
#define BMA2x2_EN_INT1_PAD_LOWG__LEN        1
#define BMA2x2_EN_INT1_PAD_LOWG__MSK        0x01
#define BMA2x2_EN_INT1_PAD_LOWG__REG        BMA2x2_INT1_PAD_SEL_REG

#define BMA2x2_EN_INT1_PAD_HIGHG__POS       1
#define BMA2x2_EN_INT1_PAD_HIGHG__LEN       1
#define BMA2x2_EN_INT1_PAD_HIGHG__MSK       0x02
#define BMA2x2_EN_INT1_PAD_HIGHG__REG       BMA2x2_INT1_PAD_SEL_REG

#define BMA2x2_EN_INT1_PAD_SLOPE__POS       2
#define BMA2x2_EN_INT1_PAD_SLOPE__LEN       1
#define BMA2x2_EN_INT1_PAD_SLOPE__MSK       0x04
#define BMA2x2_EN_INT1_PAD_SLOPE__REG       BMA2x2_INT1_PAD_SEL_REG


#define BMA2x2_EN_INT1_PAD_SLO_NO_MOT__POS        3
#define BMA2x2_EN_INT1_PAD_SLO_NO_MOT__LEN        1
#define BMA2x2_EN_INT1_PAD_SLO_NO_MOT__MSK        0x08
#define BMA2x2_EN_INT1_PAD_SLO_NO_MOT__REG        BMA2x2_INT1_PAD_SEL_REG

#define BMA2x2_EN_INT1_PAD_DB_TAP__POS      4
#define BMA2x2_EN_INT1_PAD_DB_TAP__LEN      1
#define BMA2x2_EN_INT1_PAD_DB_TAP__MSK      0x10
#define BMA2x2_EN_INT1_PAD_DB_TAP__REG      BMA2x2_INT1_PAD_SEL_REG

#define BMA2x2_EN_INT1_PAD_SNG_TAP__POS     5
#define BMA2x2_EN_INT1_PAD_SNG_TAP__LEN     1
#define BMA2x2_EN_INT1_PAD_SNG_TAP__MSK     0x20
#define BMA2x2_EN_INT1_PAD_SNG_TAP__REG     BMA2x2_INT1_PAD_SEL_REG

#define BMA2x2_EN_INT1_PAD_ORIENT__POS      6
#define BMA2x2_EN_INT1_PAD_ORIENT__LEN      1
#define BMA2x2_EN_INT1_PAD_ORIENT__MSK      0x40
#define BMA2x2_EN_INT1_PAD_ORIENT__REG      BMA2x2_INT1_PAD_SEL_REG

#define BMA2x2_EN_INT1_PAD_FLAT__POS        7
#define BMA2x2_EN_INT1_PAD_FLAT__LEN        1
#define BMA2x2_EN_INT1_PAD_FLAT__MSK        0x80
#define BMA2x2_EN_INT1_PAD_FLAT__REG        BMA2x2_INT1_PAD_SEL_REG

#define BMA2x2_EN_INT2_PAD_LOWG__POS        0
#define BMA2x2_EN_INT2_PAD_LOWG__LEN        1
#define BMA2x2_EN_INT2_PAD_LOWG__MSK        0x01
#define BMA2x2_EN_INT2_PAD_LOWG__REG        BMA2x2_INT2_PAD_SEL_REG

#define BMA2x2_EN_INT2_PAD_HIGHG__POS       1
#define BMA2x2_EN_INT2_PAD_HIGHG__LEN       1
#define BMA2x2_EN_INT2_PAD_HIGHG__MSK       0x02
#define BMA2x2_EN_INT2_PAD_HIGHG__REG       BMA2x2_INT2_PAD_SEL_REG

#define BMA2x2_EN_INT2_PAD_SLOPE__POS       2
#define BMA2x2_EN_INT2_PAD_SLOPE__LEN       1
#define BMA2x2_EN_INT2_PAD_SLOPE__MSK       0x04
#define BMA2x2_EN_INT2_PAD_SLOPE__REG       BMA2x2_INT2_PAD_SEL_REG



#define BMA2x2_EN_INT2_PAD_SLO_NO_MOT__POS        3
#define BMA2x2_EN_INT2_PAD_SLO_NO_MOT__LEN        1
#define BMA2x2_EN_INT2_PAD_SLO_NO_MOT__MSK        0x08
#define BMA2x2_EN_INT2_PAD_SLO_NO_MOT__REG        BMA2x2_INT2_PAD_SEL_REG

#define BMA2x2_EN_INT2_PAD_DB_TAP__POS      4
#define BMA2x2_EN_INT2_PAD_DB_TAP__LEN      1
#define BMA2x2_EN_INT2_PAD_DB_TAP__MSK      0x10
#define BMA2x2_EN_INT2_PAD_DB_TAP__REG      BMA2x2_INT2_PAD_SEL_REG

#define BMA2x2_EN_INT2_PAD_SNG_TAP__POS     5
#define BMA2x2_EN_INT2_PAD_SNG_TAP__LEN     1
#define BMA2x2_EN_INT2_PAD_SNG_TAP__MSK     0x20
#define BMA2x2_EN_INT2_PAD_SNG_TAP__REG     BMA2x2_INT2_PAD_SEL_REG

#define BMA2x2_EN_INT2_PAD_ORIENT__POS      6
#define BMA2x2_EN_INT2_PAD_ORIENT__LEN      1
#define BMA2x2_EN_INT2_PAD_ORIENT__MSK      0x40
#define BMA2x2_EN_INT2_PAD_ORIENT__REG      BMA2x2_INT2_PAD_SEL_REG

#define BMA2x2_EN_INT2_PAD_FLAT__POS        7
#define BMA2x2_EN_INT2_PAD_FLAT__LEN        1
#define BMA2x2_EN_INT2_PAD_FLAT__MSK        0x80
#define BMA2x2_EN_INT2_PAD_FLAT__REG        BMA2x2_INT2_PAD_SEL_REG

#define BMA2x2_EN_INT1_PAD_NEWDATA__POS     0
#define BMA2x2_EN_INT1_PAD_NEWDATA__LEN     1
#define BMA2x2_EN_INT1_PAD_NEWDATA__MSK     0x01
#define BMA2x2_EN_INT1_PAD_NEWDATA__REG     BMA2x2_INT_DATA_SEL_REG


#define BMA2x2_EN_INT1_PAD_FWM__POS     1
#define BMA2x2_EN_INT1_PAD_FWM__LEN     1
#define BMA2x2_EN_INT1_PAD_FWM__MSK     0x02
#define BMA2x2_EN_INT1_PAD_FWM__REG     BMA2x2_INT_DATA_SEL_REG

#define BMA2x2_EN_INT1_PAD_FFULL__POS     2
#define BMA2x2_EN_INT1_PAD_FFULL__LEN     1
#define BMA2x2_EN_INT1_PAD_FFULL__MSK     0x04
#define BMA2x2_EN_INT1_PAD_FFULL__REG     BMA2x2_INT_DATA_SEL_REG

#define BMA2x2_EN_INT2_PAD_FFULL__POS     5
#define BMA2x2_EN_INT2_PAD_FFULL__LEN     1
#define BMA2x2_EN_INT2_PAD_FFULL__MSK     0x20
#define BMA2x2_EN_INT2_PAD_FFULL__REG     BMA2x2_INT_DATA_SEL_REG

#define BMA2x2_EN_INT2_PAD_FWM__POS     6
#define BMA2x2_EN_INT2_PAD_FWM__LEN     1
#define BMA2x2_EN_INT2_PAD_FWM__MSK     0x40
#define BMA2x2_EN_INT2_PAD_FWM__REG     BMA2x2_INT_DATA_SEL_REG

#define BMA2x2_EN_INT2_PAD_NEWDATA__POS     7
#define BMA2x2_EN_INT2_PAD_NEWDATA__LEN     1
#define BMA2x2_EN_INT2_PAD_NEWDATA__MSK     0x80
#define BMA2x2_EN_INT2_PAD_NEWDATA__REG     BMA2x2_INT_DATA_SEL_REG

/*****          INTERRUPT SOURCE SELECTION                      *****/
#define BMA2x2_UNFILT_INT_SRC_LOWG__POS        0
#define BMA2x2_UNFILT_INT_SRC_LOWG__LEN        1
#define BMA2x2_UNFILT_INT_SRC_LOWG__MSK        0x01
#define BMA2x2_UNFILT_INT_SRC_LOWG__REG        BMA2x2_INT_SRC_REG

#define BMA2x2_UNFILT_INT_SRC_HIGHG__POS       1
#define BMA2x2_UNFILT_INT_SRC_HIGHG__LEN       1
#define BMA2x2_UNFILT_INT_SRC_HIGHG__MSK       0x02
#define BMA2x2_UNFILT_INT_SRC_HIGHG__REG       BMA2x2_INT_SRC_REG

#define BMA2x2_UNFILT_INT_SRC_SLOPE__POS       2
#define BMA2x2_UNFILT_INT_SRC_SLOPE__LEN       1
#define BMA2x2_UNFILT_INT_SRC_SLOPE__MSK       0x04
#define BMA2x2_UNFILT_INT_SRC_SLOPE__REG       BMA2x2_INT_SRC_REG


#define BMA2x2_UNFILT_INT_SRC_SLO_NO_MOT__POS        3
#define BMA2x2_UNFILT_INT_SRC_SLO_NO_MOT__LEN        1
#define BMA2x2_UNFILT_INT_SRC_SLO_NO_MOT__MSK        0x08
#define BMA2x2_UNFILT_INT_SRC_SLO_NO_MOT__REG        BMA2x2_INT_SRC_REG

#define BMA2x2_UNFILT_INT_SRC_TAP__POS         4
#define BMA2x2_UNFILT_INT_SRC_TAP__LEN         1
#define BMA2x2_UNFILT_INT_SRC_TAP__MSK         0x10
#define BMA2x2_UNFILT_INT_SRC_TAP__REG         BMA2x2_INT_SRC_REG

#define BMA2x2_UNFILT_INT_SRC_DATA__POS        5
#define BMA2x2_UNFILT_INT_SRC_DATA__LEN        1
#define BMA2x2_UNFILT_INT_SRC_DATA__MSK        0x20
#define BMA2x2_UNFILT_INT_SRC_DATA__REG        BMA2x2_INT_SRC_REG

/*****  INTERRUPT PAD ACTIVE LEVEL AND OUTPUT TYPE       *****/
#define BMA2x2_INT1_PAD_ACTIVE_LEVEL__POS       0
#define BMA2x2_INT1_PAD_ACTIVE_LEVEL__LEN       1
#define BMA2x2_INT1_PAD_ACTIVE_LEVEL__MSK       0x01
#define BMA2x2_INT1_PAD_ACTIVE_LEVEL__REG       BMA2x2_INT_SET_REG

#define BMA2x2_INT2_PAD_ACTIVE_LEVEL__POS       2
#define BMA2x2_INT2_PAD_ACTIVE_LEVEL__LEN       1
#define BMA2x2_INT2_PAD_ACTIVE_LEVEL__MSK       0x04
#define BMA2x2_INT2_PAD_ACTIVE_LEVEL__REG       BMA2x2_INT_SET_REG


/*****  OUTPUT TYPE IF SET TO 1 IS : OPEN DRIVE , IF NOT SET
	IT IS PUSH-PULL                                  *****/
#define BMA2x2_INT1_PAD_OUTPUT_TYPE__POS        1
#define BMA2x2_INT1_PAD_OUTPUT_TYPE__LEN        1
#define BMA2x2_INT1_PAD_OUTPUT_TYPE__MSK        0x02
#define BMA2x2_INT1_PAD_OUTPUT_TYPE__REG        BMA2x2_INT_SET_REG

#define BMA2x2_INT2_PAD_OUTPUT_TYPE__POS        3
#define BMA2x2_INT2_PAD_OUTPUT_TYPE__LEN        1
#define BMA2x2_INT2_PAD_OUTPUT_TYPE__MSK        0x08
#define BMA2x2_INT2_PAD_OUTPUT_TYPE__REG        BMA2x2_INT_SET_REG
/*****               INTERRUPT MODE SELECTION              ******/
#define BMA2x2_LATCH_INT__POS                0
#define BMA2x2_LATCH_INT__LEN                4
#define BMA2x2_LATCH_INT__MSK                0x0F
#define BMA2x2_LATCH_INT__REG                BMA2x2_INT_CTRL_REG

/*****               LATCHED INTERRUPT RESET               ******/
#define BMA2x2_RESET_INT__POS           7
#define BMA2x2_RESET_INT__LEN           1
#define BMA2x2_RESET_INT__MSK           0x80
#define BMA2x2_RESET_INT__REG           BMA2x2_INT_CTRL_REG

/*****               LOW-G HYSTERESIS                       ******/
#define BMA2x2_LOWG_HYST__POS                   0
#define BMA2x2_LOWG_HYST__LEN                   2
#define BMA2x2_LOWG_HYST__MSK                   0x03
#define BMA2x2_LOWG_HYST__REG                   BMA2x2_LOW_HIGH_HYST_REG

/*****               LOW-G INTERRUPT MODE                   ******/
/*****       IF 1 -- SUM MODE , 0 -- SINGLE MODE            ******/
#define BMA2x2_LOWG_INT_MODE__POS               2
#define BMA2x2_LOWG_INT_MODE__LEN               1
#define BMA2x2_LOWG_INT_MODE__MSK               0x04
#define BMA2x2_LOWG_INT_MODE__REG               BMA2x2_LOW_HIGH_HYST_REG




/*****               HIGH-G HYSTERESIS                       ******/
#define BMA2x2_HIGHG_HYST__POS                  6
#define BMA2x2_HIGHG_HYST__LEN                  2
#define BMA2x2_HIGHG_HYST__MSK                  0xC0
#define BMA2x2_HIGHG_HYST__REG                  BMA2x2_LOW_HIGH_HYST_REG

/*****               SLOPE DURATION                        ******/
#define BMA2x2_SLOPE_DUR__POS                    0
#define BMA2x2_SLOPE_DUR__LEN                    2
#define BMA2x2_SLOPE_DUR__MSK                    0x03
#define BMA2x2_SLOPE_DUR__REG                    BMA2x2_SLOPE_DURN_REG

/*SLO_NO_MOT_DUR ADDED*/
#define BMA2x2_SLO_NO_MOT_DUR__POS                    2
#define BMA2x2_SLO_NO_MOT_DUR__LEN                    6
#define BMA2x2_SLO_NO_MOT_DUR__MSK                    0xFC
#define BMA2x2_SLO_NO_MOT_DUR__REG                    BMA2x2_SLOPE_DURN_REG

/*****               TAP DURATION                        ******/
#define BMA2x2_TAP_DUR__POS                    0
#define BMA2x2_TAP_DUR__LEN                    3
#define BMA2x2_TAP_DUR__MSK                    0x07
#define BMA2x2_TAP_DUR__REG                    BMA2x2_TAP_PARAM_REG


/*****               TAP SHOCK DURATION                 ******/
#define BMA2x2_TAP_SHOCK_DURN__POS             6
#define BMA2x2_TAP_SHOCK_DURN__LEN             1
#define BMA2x2_TAP_SHOCK_DURN__MSK             0x40
#define BMA2x2_TAP_SHOCK_DURN__REG             BMA2x2_TAP_PARAM_REG

/* This advance tap interrupt only uses for the chip id 0xFB */
/*****               ADV TAP INT                        ******/
#define BMA2x2_ADV_TAP_INT__POS                5
#define BMA2x2_ADV_TAP_INT__LEN                1
#define BMA2x2_ADV_TAP_INT__MSK                0x20
#define BMA2x2_ADV_TAP_INT__REG                BMA2x2_TAP_PARAM_REG

/*****               TAP QUIET DURATION                 ******/
#define BMA2x2_TAP_QUIET_DURN__POS             7
#define BMA2x2_TAP_QUIET_DURN__LEN             1
#define BMA2x2_TAP_QUIET_DURN__MSK             0x80
#define BMA2x2_TAP_QUIET_DURN__REG             BMA2x2_TAP_PARAM_REG

/*****               TAP THRESHOLD                       ******/
#define BMA2x2_TAP_THRES__POS                  0
#define BMA2x2_TAP_THRES__LEN                  5
#define BMA2x2_TAP_THRES__MSK                  0x1F
#define BMA2x2_TAP_THRES__REG                  BMA2x2_TAP_THRES_REG

/*****               TAP SAMPLES                         ******/
#define BMA2x2_TAP_SAMPLES__POS                6
#define BMA2x2_TAP_SAMPLES__LEN                2
#define BMA2x2_TAP_SAMPLES__MSK                0xC0
#define BMA2x2_TAP_SAMPLES__REG                BMA2x2_TAP_THRES_REG

/*****       ORIENTATION MODE                        ******/
#define BMA2x2_ORIENT_MODE__POS                  0
#define BMA2x2_ORIENT_MODE__LEN                  2
#define BMA2x2_ORIENT_MODE__MSK                  0x03
#define BMA2x2_ORIENT_MODE__REG                  BMA2x2_ORIENT_PARAM_REG

/*****       ORIENTATION BLOCKING                    ******/
#define BMA2x2_ORIENT_BLOCK__POS                 2
#define BMA2x2_ORIENT_BLOCK__LEN                 2
#define BMA2x2_ORIENT_BLOCK__MSK                 0x0C
#define BMA2x2_ORIENT_BLOCK__REG                 BMA2x2_ORIENT_PARAM_REG

/*****       ORIENTATION HYSTERESIS                  ******/
#define BMA2x2_ORIENT_HYST__POS                  4
#define BMA2x2_ORIENT_HYST__LEN                  3
#define BMA2x2_ORIENT_HYST__MSK                  0x70
#define BMA2x2_ORIENT_HYST__REG                  BMA2x2_ORIENT_PARAM_REG

/*****       ORIENTATION AXIS SELECTION              ******/
/***** IF SET TO 1 -- X AND Z ARE SWAPPED , Y IS INVERTED */
#define BMA2x2_ORIENT_UD_EN__POS                  6
#define BMA2x2_ORIENT_UD_EN__LEN                  1
#define BMA2x2_ORIENT_UD_EN__MSK                  0x40
#define BMA2x2_ORIENT_UD_EN__REG                  BMA2x2_THETA_BLOCK_REG


/*****       THETA BLOCKING                    ******/
#define BMA2x2_THETA_BLOCK__POS                  0
#define BMA2x2_THETA_BLOCK__LEN                  6
#define BMA2x2_THETA_BLOCK__MSK                  0x3F
#define BMA2x2_THETA_BLOCK__REG                  BMA2x2_THETA_BLOCK_REG

/*****       THETA FLAT                        ******/
#define BMA2x2_THETA_FLAT__POS                  0
#define BMA2x2_THETA_FLAT__LEN                  6
#define BMA2x2_THETA_FLAT__MSK                  0x3F
#define BMA2x2_THETA_FLAT__REG                  BMA2x2_THETA_FLAT_REG

/*****      FLAT HOLD TIME                     ******/
#define BMA2x2_FLAT_HOLD_TIME__POS              4
#define BMA2x2_FLAT_HOLD_TIME__LEN              2
#define BMA2x2_FLAT_HOLD_TIME__MSK              0x30
#define BMA2x2_FLAT_HOLD_TIME__REG              BMA2x2_FLAT_HOLD_TIME_REG

/*****      FLAT HYS                           ******/
#define BMA2x2_FLAT_HYS__POS                   0
#define BMA2x2_FLAT_HYS__LEN                   3
#define BMA2x2_FLAT_HYS__MSK                   0x07
#define BMA2x2_FLAT_HYS__REG                   BMA2x2_FLAT_HOLD_TIME_REG

/*****      FIFO WATER MARK LEVEL TRIGGER RETAIN                        ******/
#define BMA2x2_FIFO_WML_TRIG_RETAIN__POS                   0
#define BMA2x2_FIFO_WML_TRIG_RETAIN__LEN                   6
#define BMA2x2_FIFO_WML_TRIG_RETAIN__MSK                   0x3F
#define BMA2x2_FIFO_WML_TRIG_RETAIN__REG                   BMA2x2_FIFO_WML_TRIG

/*****      ACTIVATE SELF TEST                 ******/
#define BMA2x2_EN_SELF_TEST__POS                0
#define BMA2x2_EN_SELF_TEST__LEN                2
#define BMA2x2_EN_SELF_TEST__MSK                0x03
#define BMA2x2_EN_SELF_TEST__REG                BMA2x2_SELF_TEST_REG

/*****     SELF TEST -- NEGATIVE               ******/
#define BMA2x2_NEG_SELF_TEST__POS               2
#define BMA2x2_NEG_SELF_TEST__LEN               1
#define BMA2x2_NEG_SELF_TEST__MSK               0x04
#define BMA2x2_NEG_SELF_TEST__REG               BMA2x2_SELF_TEST_REG

/*****     EEPROM CONTROL                      ******/
/* SETTING THIS BIT  UNLOCK'S WRITING SETTING REGISTERS TO EEPROM */
#define BMA2x2_UNLOCK_EE_PROG_MODE__POS     0
#define BMA2x2_UNLOCK_EE_PROG_MODE__LEN     1
#define BMA2x2_UNLOCK_EE_PROG_MODE__MSK     0x01
#define BMA2x2_UNLOCK_EE_PROG_MODE__REG     BMA2x2_EEPROM_CTRL_REG


/* SETTING THIS BIT STARTS WRITING SETTING REGISTERS TO EEPROM */
#define BMA2x2_START_EE_PROG_TRIG__POS      1
#define BMA2x2_START_EE_PROG_TRIG__LEN      1
#define BMA2x2_START_EE_PROG_TRIG__MSK      0x02
#define BMA2x2_START_EE_PROG_TRIG__REG      BMA2x2_EEPROM_CTRL_REG


/* STATUS OF WRITING TO EEPROM */
#define BMA2x2_EE_PROG_READY__POS          2
#define BMA2x2_EE_PROG_READY__LEN          1
#define BMA2x2_EE_PROG_READY__MSK          0x04
#define BMA2x2_EE_PROG_READY__REG          BMA2x2_EEPROM_CTRL_REG


/* UPDATE IMAGE REGISTERS WRITING TO EEPROM */
#define BMA2x2_UPDATE_IMAGE__POS                3
#define BMA2x2_UPDATE_IMAGE__LEN                1
#define BMA2x2_UPDATE_IMAGE__MSK                0x08
#define BMA2x2_UPDATE_IMAGE__REG                BMA2x2_EEPROM_CTRL_REG

#define BMA2x2_EE_REMAIN__POS                4
#define BMA2x2_EE_REMAIN__LEN                4
#define BMA2x2_EE_REMAIN__MSK                0xF0
#define BMA2x2_EE_REMAIN__REG                BMA2x2_EEPROM_CTRL_REG
/* SPI INTERFACE MODE SELECTION */
#define BMA2x2_EN_SPI_MODE_3__POS              0
#define BMA2x2_EN_SPI_MODE_3__LEN              1
#define BMA2x2_EN_SPI_MODE_3__MSK              0x01
#define BMA2x2_EN_SPI_MODE_3__REG              BMA2x2_SERIAL_CTRL_REG

/* I2C WATCHDOG PERIOD SELECTION */
#define BMA2x2_I2C_WATCHDOG_PERIOD__POS        1
#define BMA2x2_I2C_WATCHDOG_PERIOD__LEN        1
#define BMA2x2_I2C_WATCHDOG_PERIOD__MSK        0x02
#define BMA2x2_I2C_WATCHDOG_PERIOD__REG        BMA2x2_SERIAL_CTRL_REG

/* I2C WATCHDOG ENABLE */
#define BMA2x2_EN_I2C_WATCHDOG__POS            2
#define BMA2x2_EN_I2C_WATCHDOG__LEN            1
#define BMA2x2_EN_I2C_WATCHDOG__MSK            0x04
#define BMA2x2_EN_I2C_WATCHDOG__REG            BMA2x2_SERIAL_CTRL_REG

/* SPI INTERFACE MODE SELECTION */
/* SETTING THIS BIT  UNLOCK'S WRITING TRIMMING REGISTERS TO EEPROM */
#define BMA2x2_UNLOCK_EE_WRITE_TRIM__POS        4
#define BMA2x2_UNLOCK_EE_WRITE_TRIM__LEN        4
#define BMA2x2_UNLOCK_EE_WRITE_TRIM__MSK        0xF0
#define BMA2x2_UNLOCK_EE_WRITE_TRIM__REG        BMA2x2_CTRL_UNLOCK_REG

/**    OFFSET  COMPENSATION     **/
/**    SLOW COMPENSATION FOR X,Y,Z AXIS      **/
#define BMA2x2_EN_SLOW_COMP_X__POS              0
#define BMA2x2_EN_SLOW_COMP_X__LEN              1
#define BMA2x2_EN_SLOW_COMP_X__MSK              0x01
#define BMA2x2_EN_SLOW_COMP_X__REG              BMA2x2_OFFSET_CTRL_REG

#define BMA2x2_EN_SLOW_COMP_Y__POS              1
#define BMA2x2_EN_SLOW_COMP_Y__LEN              1
#define BMA2x2_EN_SLOW_COMP_Y__MSK              0x02
#define BMA2x2_EN_SLOW_COMP_Y__REG              BMA2x2_OFFSET_CTRL_REG

#define BMA2x2_EN_SLOW_COMP_Z__POS              2
#define BMA2x2_EN_SLOW_COMP_Z__LEN              1
#define BMA2x2_EN_SLOW_COMP_Z__MSK              0x04
#define BMA2x2_EN_SLOW_COMP_Z__REG              BMA2x2_OFFSET_CTRL_REG

/**    FAST COMPENSATION READY FLAG          **/
#define BMA2x2_FAST_CAL_RDY_S__POS             4
#define BMA2x2_FAST_CAL_RDY_S__LEN             1
#define BMA2x2_FAST_CAL_RDY_S__MSK             0x10
#define BMA2x2_FAST_CAL_RDY_S__REG             BMA2x2_OFFSET_CTRL_REG

/**    FAST COMPENSATION FOR X,Y,Z AXIS      **/
#define BMA2x2_CAL_TRIGGER__POS                5
#define BMA2x2_CAL_TRIGGER__LEN                2
#define BMA2x2_CAL_TRIGGER__MSK                0x60
#define BMA2x2_CAL_TRIGGER__REG                BMA2x2_OFFSET_CTRL_REG

/**    RESET OFFSET REGISTERS                **/
#define BMA2x2_RESET_OFFSET_REGS__POS           7
#define BMA2x2_RESET_OFFSET_REGS__LEN           1
#define BMA2x2_RESET_OFFSET_REGS__MSK           0x80
#define BMA2x2_RESET_OFFSET_REGS__REG           BMA2x2_OFFSET_CTRL_REG

/**     SLOW COMPENSATION  CUTOFF               **/
#define BMA2x2_COMP_CUTOFF__POS                 0
#define BMA2x2_COMP_CUTOFF__LEN                 1
#define BMA2x2_COMP_CUTOFF__MSK                 0x01
#define BMA2x2_COMP_CUTOFF__REG                 BMA2x2_OFFSET_PARAMS_REG

/**     COMPENSATION TARGET                  **/
#define BMA2x2_COMP_TARGET_OFFSET_X__POS        1
#define BMA2x2_COMP_TARGET_OFFSET_X__LEN        2
#define BMA2x2_COMP_TARGET_OFFSET_X__MSK        0x06
#define BMA2x2_COMP_TARGET_OFFSET_X__REG        BMA2x2_OFFSET_PARAMS_REG

#define BMA2x2_COMP_TARGET_OFFSET_Y__POS        3
#define BMA2x2_COMP_TARGET_OFFSET_Y__LEN        2
#define BMA2x2_COMP_TARGET_OFFSET_Y__MSK        0x18
#define BMA2x2_COMP_TARGET_OFFSET_Y__REG        BMA2x2_OFFSET_PARAMS_REG

#define BMA2x2_COMP_TARGET_OFFSET_Z__POS        5
#define BMA2x2_COMP_TARGET_OFFSET_Z__LEN        2
#define BMA2x2_COMP_TARGET_OFFSET_Z__MSK        0x60
#define BMA2x2_COMP_TARGET_OFFSET_Z__REG        BMA2x2_OFFSET_PARAMS_REG

/**     FIFO DATA SELECT              **/
#define BMA2x2_FIFO_DATA_SELECT__POS                 0
#define BMA2x2_FIFO_DATA_SELECT__LEN                 2
#define BMA2x2_FIFO_DATA_SELECT__MSK                 0x03
#define BMA2x2_FIFO_DATA_SELECT__REG                 BMA2x2_FIFO_MODE_REG



/*FIFO MODE*/

#define BMA2x2_FIFO_MODE__POS                 6
#define BMA2x2_FIFO_MODE__LEN                 2
#define BMA2x2_FIFO_MODE__MSK                 0xC0
#define BMA2x2_FIFO_MODE__REG                 BMA2x2_FIFO_MODE_REG


#define BMA2x2_GET_BITSLICE(regvar, bitname)\
((regvar & bitname##__MSK) >> bitname##__POS)


#define BMA2x2_SET_BITSLICE(regvar, bitname, val)\
((regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK))


/** \endcond */


/* CONSTANTS */

#define BMA2x2_STATUS1                             0
/**< It refers BMA2x2 STATUS_INT1 */
#define BMA2x2_STATUS2                             1
/**< It refers BMA2x2 STATUS_INT2 */
#define BMA2x2_STATUS3                             2
/**< It refers BMA2x2 STATUS_INT_TAP */
#define BMA2x2_STATUS4                             3
/**< It refers BMA2x2 STATUS_INT_ORIENT */
#define BMA2x2_STATUS5                             4
/**< It refers BMA2x2 STATUS_INT_FIFO */

/* range and bandwidth */

#define BMA2x2_RANGE_2G                 3
/**< sets range to +/- 2G mode \see BMA2x2_set_range() */
#define BMA2x2_RANGE_4G                 5
/**< sets range to +/- 4G mode \see BMA2x2_set_range() */
#define BMA2x2_RANGE_8G                 8
/**< sets range to +/- 8G mode \see BMA2x2_set_range() */
#define BMA2x2_RANGE_16G                12
/**< sets range to +/- 16G mode \see BMA2x2_set_range() */


#define BMA2x2_BW_7_81HZ        0x08
 /**< sets bandwidth to LowPass 7.81  HZ \see BMA2x2_set_bandwidth() */
#define BMA2x2_BW_15_63HZ       0x09
/**< sets bandwidth to LowPass 15.63 HZ \see BMA2x2_set_bandwidth() */
#define BMA2x2_BW_31_25HZ       0x0A
/**< sets bandwidth to LowPass 31.25 HZ \see BMA2x2_set_bandwidth() */
#define BMA2x2_BW_62_50HZ       0x0B
 /**< sets bandwidth to LowPass 62.50 HZ \see BMA2x2_set_bandwidth() */
#define BMA2x2_BW_125HZ         0x0C
 /**< sets bandwidth to LowPass 125HZ \see BMA2x2_set_bandwidth() */
#define BMA2x2_BW_250HZ         0x0D
/**< sets bandwidth to LowPass 250HZ \see BMA2x2_set_bandwidth() */
#define BMA2x2_BW_500HZ         0x0E
/**< sets bandwidth to LowPass 500HZ \see BMA2x2_set_bandwidth() */
#define BMA2x2_BW_1000HZ        0x0F
 /**< sets bandwidth to LowPass 1000HZ \see BMA2x2_set_bandwidth() */

/*        SLEEP DURATION              */
#define BMA2x2_SLEEP_DUR_0_5MS        0x05
/* sets sleep duration to 0.5 ms  */
#define BMA2x2_SLEEP_DUR_1MS          0x06
 /* sets sleep duration to 1 ms */
#define BMA2x2_SLEEP_DUR_2MS          0x07
/* sets sleep duration to 2 ms */
#define BMA2x2_SLEEP_DUR_4MS          0x08
/* sets sleep duration to 4 ms */
#define BMA2x2_SLEEP_DUR_6MS          0x09
/* sets sleep duration to 6 ms*/
#define BMA2x2_SLEEP_DUR_10MS         0x0A
/* sets sleep duration to 10 ms */
#define BMA2x2_SLEEP_DUR_25MS         0x0B
/* sets sleep duration to 25 ms */
#define BMA2x2_SLEEP_DUR_50MS         0x0C
 /* sets sleep duration to 50 ms */
#define BMA2x2_SLEEP_DUR_100MS        0x0D
 /* sets sleep duration to 100 ms */
#define BMA2x2_SLEEP_DUR_500MS        0x0E
 /* sets sleep duration to 500 ms */
#define BMA2x2_SLEEP_DUR_1S           0x0F
  /* sets sleep duration to 1 s */

/*        LATCH DURATION              */
#define BMA2x2_LATCH_DUR_NON_LATCH    0x00
/* sets LATCH duration to NON LATCH  */
#define BMA2x2_LATCH_DUR_250MS        0x01
/* sets LATCH duration to 250 ms */
#define BMA2x2_LATCH_DUR_500MS        0x02
/* sets LATCH duration to 500 ms */
#define BMA2x2_LATCH_DUR_1S           0x03
 /* sets LATCH duration to 1 s */
#define BMA2x2_LATCH_DUR_2S           0x04
 /* sets LATCH duration to 2 s*/
#define BMA2x2_LATCH_DUR_4S           0x05
 /* sets LATCH duration to 4 s */
#define BMA2x2_LATCH_DUR_8S           0x06
 /* sets LATCH duration to 8 s */
#define BMA2x2_LATCH_DUR_LATCH        0x07
 /* sets LATCH duration to LATCH */
#define BMA2x2_LATCH_DUR_NON_LATCH1   0x08
 /* sets LATCH duration to NON LATCH1 */
#define BMA2x2_LATCH_DUR_250US        0x09
 /* sets LATCH duration to 250 Us */
#define BMA2x2_LATCH_DUR_500US        0x0A
 /* sets LATCH duration to 500 Us */
#define BMA2x2_LATCH_DUR_1MS          0x0B
 /* sets LATCH duration to 1 Ms */
#define BMA2x2_LATCH_DUR_12_5MS       0x0C
/* sets LATCH duration to 12.5 Ms */
#define BMA2x2_LATCH_DUR_25MS         0x0D
/* sets LATCH duration to 25 Ms */
#define BMA2x2_LATCH_DUR_50MS         0x0E
 /* sets LATCH duration to 50 Ms */
#define BMA2x2_LATCH_DUR_LATCH1       0x0F
/* sets LATCH duration to LATCH*/

/* mode settings */

#define BMA2x2_MODE_NORMAL             0
#define BMA2x2_MODE_LOWPOWER1          1
#define BMA2x2_MODE_SUSPEND            2
#define BMA2x2_MODE_DEEP_SUSPEND       3
#define BMA2x2_MODE_LOWPOWER2          4
#define BMA2x2_MODE_STANDBY            5

/* BMA2x2 AXIS      */

#define BMA2x2_X_AXIS           0
/**< It refers BMA2x2 X-axis */
#define BMA2x2_Y_AXIS           1
/**< It refers BMA2x2 Y-axis */
#define BMA2x2_Z_AXIS           2
/**< It refers BMA2x2 Z-axis */

/*  INTERRUPT TYPES    */

#define BMA2x2_Low_G_Interrupt       0
#define BMA2x2_High_G_X_Interrupt    1
#define BMA2x2_High_G_Y_Interrupt    2
#define BMA2x2_High_G_Z_Interrupt    3
#define BMA2x2_DATA_EN               4
#define BMA2x2_Slope_X_Interrupt     5
#define BMA2x2_Slope_Y_Interrupt     6
#define BMA2x2_Slope_Z_Interrupt     7
#define BMA2x2_Single_Tap_Interrupt  8
#define BMA2x2_Double_Tap_Interrupt  9
#define BMA2x2_Orient_Interrupt      10
#define BMA2x2_Flat_Interrupt        11
#define BMA2x2_FFULL_INTERRUPT       12
#define BMA2x2_FWM_INTERRUPT         13

/*  INTERRUPTS PADS  */

#define BMA2x2_INT1_LOWG         0
#define BMA2x2_INT2_LOWG         1
#define BMA2x2_INT1_HIGHG        0
#define BMA2x2_INT2_HIGHG        1
#define BMA2x2_INT1_SLOPE        0
#define BMA2x2_INT2_SLOPE        1
#define BMA2x2_INT1_SLO_NO_MOT   0
#define BMA2x2_INT2_SLO_NO_MOT   1
#define BMA2x2_INT1_DTAP         0
#define BMA2x2_INT2_DTAP         1
#define BMA2x2_INT1_STAP         0
#define BMA2x2_INT2_STAP         1
#define BMA2x2_INT1_ORIENT       0
#define BMA2x2_INT2_ORIENT       1
#define BMA2x2_INT1_FLAT         0
#define BMA2x2_INT2_FLAT         1
#define BMA2x2_INT1_NDATA        0
#define BMA2x2_INT2_NDATA        1
#define BMA2x2_INT1_FWM          0
#define BMA2x2_INT2_FWM          1
#define BMA2x2_INT1_FFULL        0
#define BMA2x2_INT2_FFULL        1

/*       SOURCE REGISTER        */

#define BMA2x2_SRC_LOWG         0
#define BMA2x2_SRC_HIGHG        1
#define BMA2x2_SRC_SLOPE        2
#define BMA2x2_SRC_SLO_NO_MOT   3
#define BMA2x2_SRC_TAP          4
#define BMA2x2_SRC_DATA         5

#define BMA2x2_INT1_OUTPUT      0
#define BMA2x2_INT2_OUTPUT      1
#define BMA2x2_INT1_LEVEL       0
#define BMA2x2_INT2_LEVEL       1

/*    DURATION         */

#define BMA2x2_LOW_DURATION            0
#define BMA2x2_HIGH_DURATION           1
#define BMA2x2_SLOPE_DURATION          2
#define BMA2x2_SLO_NO_MOT_DURATION     3

/*      THRESHOLD        */

#define BMA2x2_LOW_THRESHOLD            0
#define BMA2x2_HIGH_THRESHOLD           1
#define BMA2x2_SLOPE_THRESHOLD          2
#define BMA2x2_SLO_NO_MOT_THRESHOLD     3


#define BMA2x2_LOWG_HYST                0
#define BMA2x2_HIGHG_HYST               1

#define BMA2x2_ORIENT_THETA             0
#define BMA2x2_FLAT_THETA               1

#define BMA2x2_I2C_SELECT               0
#define BMA2x2_I2C_EN                   1

/*    COMPENSATION           */

#define BMA2x2_SLOW_COMP_X              0
#define BMA2x2_SLOW_COMP_Y              1
#define BMA2x2_SLOW_COMP_Z              2

/*       OFFSET TRIGGER          */

#define BMA2x2_CUT_OFF                  0
#define BMA2x2_OFFSET_TRIGGER_X         1
#define BMA2x2_OFFSET_TRIGGER_Y         2
#define BMA2x2_OFFSET_TRIGGER_Z         3

/*       GP REGISTERS           */

#define BMA2x2_GP0                      0
#define BMA2x2_GP1                      1

/*    SLO NO MOT REGISTER          */

#define BMA2x2_SLO_NO_MOT_EN_X          0
#define BMA2x2_SLO_NO_MOT_EN_Y          1
#define BMA2x2_SLO_NO_MOT_EN_Z          2
#define BMA2x2_SLO_NO_MOT_EN_SEL        3


/* wake up */

#define BMA2x2_WAKE_UP_DUR_20MS         0
#define BMA2x2_WAKE_UP_DUR_80MS         1
#define BMA2x2_WAKE_UP_DUR_320MS                2
#define BMA2x2_WAKE_UP_DUR_2560MS               3


/* LG/HG thresholds are in LSB and depend on RANGE setting */
/* no range check on threshold calculation */

#define BMA2x2_SELF_TEST0_ON            1
#define BMA2x2_SELF_TEST1_ON            2

#define BMA2x2_EE_W_OFF                 0
#define BMA2x2_EE_W_ON                  1

/* Resolution Settings */
#define BMA2x2_RESOLUTION_12_BIT        0
#define BMA2x2_RESOLUTION_10_BIT        1
#define BMA2x2_RESOLUTION_14_BIT        3

#define BMA2x2           0x16
#define BMA280           0x17
#define BMA222E          0x18
#define BMA250E          0x19
/** Macro to convert floating point
low-g-thresholds in G to 8-bit register values.<br>
  * Example: BMA2x2_LOW_TH_IN_G( 0.3, 2.0) generates
  * the register value for 0.3G threshold in 2G mode.
  * \brief convert g-values to 8-bit value
 */
#define BMA2x2_LOW_TH_IN_G(gthres, range)  ((256 * gthres) / range)

/** Macro to convert floating point high-g-thresholds
    in G to 8-bit register values.<br>
  * Example: BMA2x2_HIGH_TH_IN_G( 1.4, 2.0)
  * generates the register value for 1.4G threshold in 2G mode.
  * \brief convert g-values to 8-bit value
 */
#define BMA2x2_HIGH_TH_IN_G(gthres, range)   ((256 * gthres) / range)

/** Macro to convert floating point low-g-hysteresis
in G to 8-bit register values.<br>
  * Example: BMA2x2_LOW_HY_IN_G( 0.2, 2.0)
  *generates the register value for 0.2G threshold in 2G mode.
  * \brief convert g-values to 8-bit value
 */
#define BMA2x2_LOW_HY_IN_G(ghyst, range)   ((32 * ghyst) / range)

/** Macro to convert floating point high-g-hysteresis
   in G to 8-bit register values.<br>
  * Example: BMA2x2_HIGH_HY_IN_G( 0.2, 2.0) generates
  *the register value for 0.2G threshold in 2G mode.
  * \brief convert g-values to 8-bit value
 */
#define BMA2x2_HIGH_HY_IN_G(ghyst, range)    ((32 * ghyst) / range)


/** Macro to convert floating point G-thresholds
    to 8-bit register values<br>
  * Example: BMA2x2_SLOPE_TH_IN_G( 1.2, 2.0)
  * generates the register value for 1.2G threshold in 2G mode.
  * \brief convert g-values to 8-bit value
 */

#define BMA2x2_SLOPE_TH_IN_G(gthres, range)    ((128 * gthres) / range)
/** user defined Enums
* Example..
* enum {
* E_YOURDATA1, < <DOXY Comment for E_YOURDATA1>
* E_YOURDATA2  < <DOXY Comment for E_YOURDATA2>
* };
*/
/** Example...
* struct DUMMY_STRUCT {
* data1, < <DOXY Comment for data1>
* data2  < <DOXY Comment for data1>
* };*/
/*****************************************************************************
 * Description: *//**\brief This API reads the data from the given register
 *
 *
 *
 *
 *  \param u8 addr, u8 *data
 *                       addr -> Address of the register
 *                       data -> address of the variable,
 *                              read value will be kept
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

BMA2x2_RETURN_FUNCTION_TYPE bma2x2_burst_read(u8 addr,
u8 *data, u32 len);
/*****************************************************************************
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
 ******************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_init(struct bma2x2_t *bma2x2);

/*****************************************************************************
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_write_reg(u8 addr,
u8 *data, u8 len);

/*****************************************************************************
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_read_reg(u8 addr,
u8 *data, u8 len);

/*****************************************************************************
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_read_accel_x(s16 *a_x);

/*****************************************************************************
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_read_accel_elight_resolution_x(s8 *a_x);

/*****************************************************************************
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
 ******************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_read_accel_y(s16 *a_y);

/*****************************************************************************
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_read_accel_elight_resolution_y(s8 *a_y);

/*****************************************************************************
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_read_accel_z(s16 *a_z);

/*****************************************************************************
 * Description: *//**\brief This API reads acceleration data Y values of
 *                 8bit  resolution  from location 07h
 *
 *
 *
 *
 *  \param short  *a_z   :  pointer holding the data of of a_z
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
 ******************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_read_accel_elight_resolution_z(s8 *a_z);

/*****************************************************************************
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
 ******************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_read_accel_xyz(struct bma2x2acc_t *acc);

/*****************************************************************************
 * Description: *//**\brief This API reads acceleration of 8 bit resolution
 *                          data of X,Y,Z values
 *                          from location 03h , 05h and 07h
 *
 *
 *
 *
 *  \param bma2x2acc_t * acc : pointer holding the data of bma2x2acc_t
 *	 * \return results of bus communication function
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_read_accel_eight_resolution_xyz(
struct bma2x2acc_eight_resolution *acc);

/*****************************************************************************
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_int_tap_status(
u8 *status_tap);

/*****************************************************************************
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
 ******************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_int_orient_status(
u8 *status_orient);

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
 ******************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_fifo_status(
u8 *status_fifo);

/*****************************************************************************
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_fifo_framecount(
u8 *framecount);

/*****************************************************************************
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
 ******************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_fifo_overrun(
u8 *overrun);

/*****************************************************************************
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_interrupt_status(
u8 *status);

/*****************************************************************************
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_range(
u8 *range);

/*****************************************************************************
 * Description: *//**\brief This API is used to set
 *  Ranges(g value) of the sensor in the register 0x0F
 *					bit from 0 to 3
 *
 *
 *
 *
 *
 *  \param u8 Range : The value of range
 *             Range ->   3 -> BMA2x2_RANGE_2G
 *                        5 -> BMA2x2_RANGE_4G
 *                        8 -> BMA2x2_RANGE_8G
 *                        12 -> BMA2x2_RANGE_16G
 *
 * \return communication results
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_range(
u8 range);

/*****************************************************************************
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_bandwidth(
u8 *bw);

/*****************************************************************************
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_bandwidth(
u8 bw);

/*****************************************************************************
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_mode(
u8 *mode);

/*****************************************************************************
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_mode(u8 mode);

/*****************************************************************************
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
 ******************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_sleep_dur(
u8 *sleep_dur);

/*****************************************************************************
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_sleep_dur(
u8 sleep_dur);

/*****************************************************************************
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
 ******************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_sleeptmr_mode(
u8 *sleep_tmr);

/*****************************************************************************
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_sleeptmr_mode(
u8 sleep_tmr);

/*****************************************************************************
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
 ******************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_high_bw(u8 *high_bw);

/*****************************************************************************
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_high_bw(u8 high_bw);

/*****************************************************************************
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_shadow_dis(u8 *shadow_dis);

/*****************************************************************************
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_shadow_dis(u8 shadow_dis);

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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_soft_reset(void);

/*****************************************************************************
 * Description: *//**\brief This API is used to update the register values
 *
 *
 *
 *
 *  \param  None
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
 ******************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_update_image(void);

/*****************************************************************************
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_Int_Enable(u8 interrupttype,
u8 *value);

/*****************************************************************************
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_Int_Enable(u8 interrupttype,
u8 value);

/*****************************************************************************
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_int_ffull(u8 *ffull);

/*****************************************************************************
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_int_ffull(u8 ffull);

/*****************************************************************************
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_int_fwm(u8 *fwm);

/*****************************************************************************
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_int_fwm(u8 fwm);

/*****************************************************************************
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_slo_no_mot(u8 channel,
u8 *slo_data);

/*****************************************************************************
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_slo_no_mot(u8 channel,
u8 slo_data);

/*****************************************************************************
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_int_low(u8 channel,
u8 *int_low);

/*****************************************************************************
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
 *
 ******************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_int_low(u8 channel,
u8 int_low);

/*****************************************************************************
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_int_high(u8 channel,
u8 *int_high);

/*****************************************************************************
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_int_high(u8 channel,
u8 int_high);

/*****************************************************************************
 * Description: *//**\brief This API is used to get
 * the status of slope interrupt in the register 0x19 and 0x1B
 * INT1 -> register 0x19 bit 2
 * INT2 -> register 0x1B bit 2
 *
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_int_slope(u8 channel,
u8 *int_slope);

/*****************************************************************************
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_int_slope(u8 channel,
u8 int_slope);

/*****************************************************************************
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
 ******************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_int_slo_no_mot(u8 channel,
u8 *int_slo_no_mot);

/*****************************************************************************
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
 ******************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_int_slo_no_mot(u8 channel,
u8 int_slo_no_mot);

/*****************************************************************************
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_int_d_tap(u8 channel,
u8 *int_d_tap);

/*****************************************************************************
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
 ******************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_int_d_tap(u8 channel,
u8 int_d_tap);

/*****************************************************************************
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
u8 *int_s_tap);

/*****************************************************************************
 * Description: *//**\brief This API is used to set
 * the status of single tap interrupt
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_int_s_tap(u8 channel,
u8 int_s_tap);

/*****************************************************************************
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
 ******************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_int_orient(u8 channel,
u8 *int_orient);

/*****************************************************************************
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_int_orient(u8 channel,
u8 int_orient);

/*****************************************************************************
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_int_flat(u8 channel,
u8 *int_flat);

/*****************************************************************************
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
 ******************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_int_flat(u8 channel,
u8 int_flat);

/*****************************************************************************
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
 ******************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_newdata(u8 channel,
u8 *int_newdata);

/*****************************************************************************
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_newdata(u8 channel,
u8 int_newdata);

/*****************************************************************************
 * Description: *//**\brief This API is used to get the fwm interrupt1 data
 * in the register 0x1A
 * INT1 -> register 0x1A bit 1
 *
 *
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_int1_fwm(u8 *int1_fwm);

/*****************************************************************************
 *	Description: *//**\brief This API is used to set the fwm interrupt1 data
 *	in the register 0x1A
 *	INT1 -> register 0x1A bit 1
 *
 *
 *
 *
 *	u8 u8 int1_fwm:
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_int1_fwm(u8 int1_fwm);

/*****************************************************************************
 *	Description: *//**\brief This API is used to get
 *	the fwm interrupt1 data in the register 0x1A
 *	INT2 -> register 0x1A bit 6
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_int2_fwm(u8 *int2_fwm);

/*****************************************************************************
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_int2_fwm(u8 int2_fwm);

/*****************************************************************************
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_int1_ffull(u8 *int1_ffull);

/*****************************************************************************
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
 ******************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_int1_ffull(u8 int1_ffull);

/*****************************************************************************
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_int2_ffull(u8 *int2_ffull);

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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_int2_ffull(u8 int2_ffull);

/*****************************************************************************
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_source(u8 channel,
u8 *int_source);

/*****************************************************************************
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_source(u8 channel,
u8 int_source);

/*****************************************************************************
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_int_od(u8 channel,
u8 *int_od);

/*****************************************************************************
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
 *  u8  int_orient: The output type status value
 *	open drain   ->   1
 *	push pull    ->   0
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_int_od(u8 channel,
u8 int_od);

/*****************************************************************************
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_int_lvl(u8 channel,
u8 *int_lvl);

/*****************************************************************************
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_int_lvl(u8 channel,
u8 int_lvl);

/*****************************************************************************
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
 ******************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_reset_interrupt(u8 reset_int);

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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_latch_int(u8 *latch_int);

/*****************************************************************************
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_latch_int(u8 latch_int);

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
 ******************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_dur(u8 channel,
u8 *dur);

/*****************************************************************************
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
 ******************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_dur(u8 channel,
u8 dur);

/*****************************************************************************
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
u8 *thr);

/*****************************************************************************
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
 ******************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_thr(u8 channel,
u8 thr);

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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_low_high_hyst(u8 channel,
u8 *hyst);

/*****************************************************************************
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_low_high_hyst(u8 channel,
u8 hyst);

/*****************************************************************************
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_lowg_mode(
u8 *mode);

/*****************************************************************************
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_lowg_mode(
u8 mode);

/*****************************************************************************
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
 ******************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_tap_dur(u8 *tap_dur);

/*****************************************************************************
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
 ******************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_tap_dur(u8 tap_dur);

/*****************************************************************************
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
 ******************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_tap_shock(u8 *tap_shock);

/*****************************************************************************
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_tap_shock(u8 tap_shock);

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
 ******************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_tap_quiet(u8 *tap_quiet);

/*****************************************************************************
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
 ******************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_tap_quiet(u8 tap_quiet);

/*****************************************************************************
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_tap_thr(u8 *tap_thr);

/*****************************************************************************
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
 ******************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_tap_thr(
u8 tap_thr);

/*****************************************************************************
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
 ******************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_tap_sample(
u8 *tap_sample);

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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_tap_sample(
u8 tap_sample);

/*****************************************************************************
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_orient_mode(
u8 *orient_mode);

/*****************************************************************************
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
 ******************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_orient_mode(
u8 orient_mode);

/*****************************************************************************
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_orient_block(
u8 *orient_block);

/*****************************************************************************
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_orient_block(
u8 orient_block);

/*****************************************************************************
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_orient_hyst(
u8 *orient_hyst);

/*****************************************************************************
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
 ******************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_orient_hyst(
u8 orient_hyst);

/*****************************************************************************
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
 ******************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_theta(u8 channel,
u8 *theta);

/*****************************************************************************
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_theta(u8 channel,
u8 theta);

/*****************************************************************************
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
 ******************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_orient_en(u8 *orient_en);

/*****************************************************************************
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_orient_en(u8 orient_en);

/*****************************************************************************
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_flat_hyst(u8 *flat_hyst);

/*****************************************************************************
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
 ******************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_flat_hyst(u8 flat_hyst);

/*****************************************************************************
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
 ******************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_flat_hold_time(
u8 *flat_hold_time);

/*****************************************************************************
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
 ******************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_flat_hold_time(
u8 flat_hold_time);

/*****************************************************************************
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_fifo_wml_trig(
u8 *fifo_wml_trig);

/*****************************************************************************
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_fifo_wml_trig(
u8 fifo_wml_trig);

/*****************************************************************************
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
 ******************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_self_test_axis(
u8 *self_test_axis);

/*****************************************************************************
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
 ******************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_self_test_axis(
u8 self_test_axis);

/*****************************************************************************
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
 ******************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_self_test_sign(
u8 *self_test_sign);

/*****************************************************************************
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
 ******************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_self_test_sign(
u8 self_test_sign);
/*****************************************************************************
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_nvmprog_mode(
u8 *nvmprog_mode);

/*****************************************************************************
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
 ******************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_nvmprog_mode(u8 prgmode);
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_nvprog_trig(u8 trig);
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_nvmprog_ready(u8 *ready);
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_nvmprog_remain(u8 *remain);
/*****************************************************************************
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
 ******************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_spi3(u8 *spi3);

/*****************************************************************************
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
 ******************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_spi3(u8 spi3);

/*****************************************************************************
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_i2c_wdt(u8 channel,
u8 *prog_mode);

/*****************************************************************************
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_i2c_wdt(u8 channel,
u8 prog_mode);

/*****************************************************************************
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
 ******************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_slow_comp(u8 channel,
u8 *slow_comp);

/*****************************************************************************
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
 ******************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_slow_comp(u8 channel,
u8 slow_comp);

/*****************************************************************************
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_cal_rdy(u8 *rdy);
/*****************************************************************************
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_cal_trig(u8 cal_trig);

/*****************************************************************************
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_offset_reset(
u8 offset_reset);
/*****************************************************************************
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
 *	channel --> BMA2x2_ACCEL_CUT_OFF              ->    0
 *				BMA2x2_ACCEL_OFFSET_TRIGGER_X     ->    1
 *				BMA2x2_ACCEL_OFFSET_TRIGGER_Y     ->    2
 *				BMA2x2_ACCEL_OFFSET_TRIGGER_Z     ->    3
 *
 *  u8 *offset: Pointer holding the offset target value
 *	CUT_OFF -> 0 or 1
 *	offset -->
 *	BMA2x2_ACCEL_OFFSET_TRIGGER_X   -> 0,1,2,3
 *	BMA2x2_ACCEL_OFFSET_TRIGGER_Y   -> 0,1,2,3
 *	BMA2x2_ACCEL_OFFSET_TRIGGER_Z   -> 0,1,2,3
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_offset_target(u8 channel,
u8 *offset);

/*****************************************************************************
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
 ******************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_offset_target(u8 channel,
u8 offset);

/*****************************************************************************
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
 ******************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_offset(u8 channel,
s8 *offset);

/*****************************************************************************
 *	Description: *//**\brief This API is used to set the value of offset
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
 ******************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_offset(u8 channel,
s8 offset);

/*****************************************************************************
 *	Description: *//**\brief This API is used to get
 *	the status of fifo mode(fifo_mode) in the register 0x3E bit 6 and 7
 *
 *
 *
 *
 *  \param u8 *fifo_mode : Pointer holding the fifo_mode
 *	fifo_mode
 *	0 --> Bypass
 *	1 --> FIFO
 *	2 --> Stream
 *	3 --> Reserved
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_fifo_mode(
u8 *fifo_mode);

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
 ******************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_fifo_mode(
u8 fifo_mode);

/*****************************************************************************
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
 ******************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_fifo_data_sel(
u8 *data_sel);

/*****************************************************************************
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
 ******************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_fifo_data_sel(
u8 data_sel);

/*****************************************************************************
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
 ******************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_fifo_data_out_reg(
u8 *out_reg);
/*****************************************************************************
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
 ******************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_read_temperature(
s8 *temperature);

/*****************************************************************************
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
 ******************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_read_accel_xyzt(
struct bma2x2acc_data *acc);

/*****************************************************************************
 * Description: *//**\brief This API reads acceleration of 8 bit resolution
 *                          data of X,Y,Z values
 *                          from location 03h , 05h and 07h
 *
 *
 *
 *
 *  \param bma2x2acc_t * acc : Pointer holding the bma2x2acc_t
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_read_accel_eight_resolution_xyzt(
struct bma2x2acc_eight_resolution_t *acc);
#endif
