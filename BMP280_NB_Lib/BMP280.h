/*
 * BMP280.h
 *
 *  Created on: Jan 16, 2025
 *      Author: Amir Hossein Chiani
 *      Non_blocking library for (Bosch BMP280) barometric sensor
 *      Library options:
 *      				1:Pressure(Hectopascal)
 *      				2:Temperature(Celsius)
 *      				3:Altitude(Centimeter)
 */

#ifndef BMP280_H_
#define BMP280_H_

#include "i2c.h"
#include "stm32f1xx_hal.h"
#include "main.h"


/****************************************************************************************************/
// Device I2C addresses for write in hal functions.
#define BMP280_I2C_address_write 		0xEC // if SDO pin conneted to GND = 0x76
//#define BMP280_I2C_address_write 		0xEE // if SDO pin conneted to VDDIO = 0x77
/****************************************************************************************************/
// BMP280 I2C port.
#define BMP280_I2C_port                 &hi2c1 // I2C port where the chip connected
/****************************************************************************************************/


// Constants input of BMP280_Set_Config function.
typedef enum
{
	BMP280_FILTER_OFF = 0x00,
	BMP280_FILTER_2   = 0x04,
	BMP280_FILTER_4   = 0x08,
	BMP280_FILTER_8   = 0x0C,
	BMP280_FILTER_16  = 0x10
} 	BMP280_filter_params;
typedef enum
{
	BMP280_STANDBY_0p5ms  = 0x00,
	BMP280_STANDBY_62p5ms = 0x20,
	BMP280_STANDBY_125ms  = 0x40,
	BMP280_STANDBY_250ms  = 0x60,
	BMP280_STANDBY_500ms  = 0x80,
	BMP280_STANDBY_1000ms = 0xA0,
	BMP280_STANDBY_2000ms = 0xC0,
	BMP280_STANDBY_4000ms = 0xE0
}	BMP280_standbyt_params;
/****************************************************************************************************/
// Constants input of BMP280_Set_Ctrl_meas function.
typedef enum
{
	BMP280_Sleep_mode 	= 0x00,
	BMP280_Forced_mode	= 0x01,
	BMP280_Normal_mode	= 0X03
}	BMP280_mode_params;
typedef enum
{
	BMP280_osrs_p_skip 	= 0x00,
	BMP280_osrs_p_x1 	= 0x04,
	BMP280_osrs_p_x2	= 0x08,
	BMP280_osrs_p_x4	= 0x0C,
	BMP280_osrs_p_x8	= 0x10,
	BMP280_osrs_p_x16	= 0x14
}	BMP280_osrs_p_params;
typedef enum
{
	BMP280_osrs_t_skip 	= 0x00,
	BMP280_osrs_t_x1 	= 0x20,
	BMP280_osrs_t_x2	= 0x40,
	BMP280_osrs_t_x4	= 0x60,
	BMP280_osrs_t_x8	= 0x80,
	BMP280_osrs_t_x16	= 0xA0
}	BMP280_osrs_t_params;
/****************************************************************************************************/
// Register addresses.
#define BMP280_chip_id_ad 		0xD0  // This is the chip id register
#define BMP280_reset_ad   		0xE0  // Software reset control register
#define BMP280_status_ad  		0xF3  // Device status register
#define BMP280_ctrl_meas_ad  	0xF4  // Pressure and temperature measure control register
#define BMP280_config_ad  		0xF5  // Configuration register
#define BMP280_press_msb_ad  	0xF7  // Pressure readings MSB
#define BMP280_press_lsb_ad  	0xF8  // Pressure readings LSB
#define BMP280_press_xlsb_ad  	0xF9  // Pressure readings XLSB
#define BMP280_temp_msb_ad  	0xFA  // Temperature data MSB
#define BMP280_temp_lsb_ad  	0xFB  // Temperature data LSB
#define BMP280_temp_xlsb_ad  	0xFC  // Temperature data XLSB
#define BMP280_calibration_ad	0x88 // Calibration first address
/****************************************************************************************************/
// Register masks.
#define BMP280_status_measuring_mask  	0x08  // Bit masking for measuring bit in status register.
#define BMP280_status_im_update_mask  	0x01  // Bit masking for im_update bit in status register.
/****************************************************************************************************/
// Soft reset word.
#define BMP280_soft_reset       0xB6
/****************************************************************************************************/
// Chip id.
#define BMP280_chip_id          0x58
/****************************************************************************************************/
// Device I2C raw addresses.
#define BMP280_I2C_address 		0x76 // if SDO pin conneted to GND = 0x76
//#define BMP280_I2C_address 		0x77 // if SDO pin conneted to VDDIO = 0x77
/****************************************************************************************************/
// Function return values.
#define BMP280_busy  0
#define BMP280_ok    1
#define BMP280_error 2

/****************************************************************************************************/
/*
 * This function must be use in
 * void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) function
 * for library state machines.
 */
void BMP280_Rx_it(void);
/****************************************************************************************************/
/*
 * This function must be use in
 * void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c) function
 * for library state machines.
 */
void BMP280_Tx_it(void);
/****************************************************************************************************/
/* This function reset the chip
 * then get chip id from chip for confidence the chip is ok.
 *
 * Return values:
 * 1:BMP280_busy = Function is working.
 * 2:BMP280_error = Chip id is invalid.
 * 3:BMP280_ok = Chip id is valid.
 * NOTE:This function must be use in state machine!!!
 */
uint8_t BMP280_Reset(void);
/****************************************************************************************************/
/*
 * This function write filter and standby time settings
 * in config register.
 *
 * Return values:
 * 1:BMP280_busy = Function is working.
 * 2:BMP280_ok = Write done.
 * NOTE:This function must be use in state machine!!!
 */
uint8_t BMP280_Set_Config(uint8_t filter,uint8_t standby);
/****************************************************************************************************/
/*
 * This function write mode,osrs_p and osrs_t settings
 * in ctrl_meas register.
 *
 * Return values:
 * 1:BMP280_busy = Function is working.
 * 2:BMP280_ok = Write done.
 * NOTE:This function must be use in state machine!!!
 */
uint8_t BMP280_Set_Ctrl_meas(uint8_t mode,uint8_t osrs_p,uint8_t osrs_t);
/****************************************************************************************************/
/*
 * This function check the measuring bit of status register.
 *
 * Return values:
 * 1:BMP280_busy = Function is working.
 * 2:BMP280_error = Measuring (measure bit is 1).
 * 3:BMP280_ok = Measurement done (measure bit is 0).
 * NOTE:This function must be use in state machine!!!
 */
uint8_t BMP280_Get_Status_measuring(void);
/****************************************************************************************************/
/*
 * This function check the im_update bit of status register.
 *
 * Return values:
 * 1:BMP280_busy = Function is working.
 * 2:BMP280_error = NVM data are copying to image registers (im_update bit is 1).
 * 3:BMP280_ok = Copying is done (im_update bit is 0).
 * NOTE:This function must be use in state machine!!!
 */
uint8_t BMP280_Get_Status_im_update(void);
/****************************************************************************************************/
/*
 * This function get unique calibration value from calibration register.
 *
 * 1:BMP280_busy = Function is working.
 * 2:BMP280_ok = Getting is done and  data stored in BMP280_calib_params structure.
 * NOTE:This function must be use in state machine!!!
 */
uint8_t BMP280_Get_Calibration(void);
/****************************************************************************************************/
/*
 * This function get raw temperature value and store it
 * in function input argument variable.
 *
 * Function argument: variable address
 *
 * 1:BMP280_busy = Function is working.
 * 2:BMP280_ok = Getting is done and  data stored in raw_temp input argument.
 * NOTE:This function must be use in state machine!!!
 */
uint8_t BMP280_Get_Raw_temp(int32_t *raw_temp);
/****************************************************************************************************/
/*
 * This function get raw pressure value and store it
 * in function input argument variable.
 *
 * Function argument: variable address
 *
 * 1:BMP280_busy = Function is working.
 * 2:BMP280_ok = Getting is done and  data stored in raw_press input argument.
 * NOTE:This function must be use in state machine!!!
 */
uint8_t BMP280_Get_Raw_press(uint32_t *raw_press);
/****************************************************************************************************/
/*
 * API
 *
 * This function get five argument from user and initialize the chip.
 *
 * Function arguments: filter, standby, mode, osrs_p, osrs_t values.
 *
 * 1:BMP280_busy = Function is working.
 * 2:BMP280_ok = Initialization is done.
 * NOTE:This function must be use in state machine!!!
 */
uint8_t BMP280_Init(uint8_t filter,uint8_t standby,uint8_t mode,uint8_t osrs_p,uint8_t osrs_t);
/****************************************************************************************************/
/*
 * API
 *
 * This function give temperature with celsius unit.
 * and store it in function input argument variable
 * Argument value: Temperature (C).
 * Argument type: Double.
 */
uint8_t BMP280_Temperature_C(double *c_temp);
/****************************************************************************************************/
/*
 * API
 *
 * This function give pressure with hectopascal unit
 * and store it in function input argument variable.
 * Argument value: Pressure (Hpa).
 * Argument type: Double.
 */
uint8_t BMP280_Pressure_Hpa(double *c_Press);
/****************************************************************************************************/
/*
 * API
 *
 * This function calculate altitude from pressure.
 * Argument value: Altitude (cm).
 * Argument type: Double.
 */
uint8_t BMP280_Altitude_cm(double *c_Altitude);
#endif /* BMP280_H_ */
