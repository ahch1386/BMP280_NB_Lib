/*
 * BMP280.c
 *
 *  Created on: Jan 16, 2025
 *      Author: Amir Hossein Chiani
 */
#include "BMP280.h"
#include "stdbool.h"
#include <stdint.h>
#include <string.h>
#include <math.h>
/****************************************************************************************************/
int32_t t_fine = 0; // This is the t_fine variable for temp and press measurement.
/****************************************************************************************************/
// Function state machine variables.
uint8_t BMP280_Reset_state = 0; // BMP280_Reset function state machine.
uint8_t BMP280_Set_Config_state = 0; // BMP280_set_config function state machine.
uint8_t BMP280_Set_Ctrl_meas_state = 0; // BMP280_Set_Ctrl_meas function state machine.
uint8_t BMP280_Get_Status_measuring_state = 0; // BMP280_Get_Status_measuring function state machine.
uint8_t BMP280_Get_Status_im_update_state = 0; // BMP280_Get_Status_im_update function state machine.
uint8_t BMP280_Get_Calibration_state = 0; // BMP280_Get_Calibration function state machine.
uint8_t BMP280_Get_Raw_temp_state = 0; // BMP280_Get_Raw_temp function state machine.
uint8_t BMP280_Get_Raw_press_state = 0; // BMP280_Get_Raw_press function state machine.
uint8_t BMP280_Init_state = 0; // BMP280_Init function state machine.
/****************************************************************************************************/
//Calibration parameters.
typedef struct
{
	uint16_t dig_T1;
	int16_t  dig_T2;
	int16_t  dig_T3;
	uint16_t dig_P1;
	int16_t  dig_P2;
	int16_t  dig_P3;
	int16_t  dig_P4;
	int16_t  dig_P5;
	int16_t  dig_P6;
	int16_t  dig_P7;
	int16_t  dig_P8;
	int16_t  dig_P9;
}	BMP280_calib_params;
BMP280_calib_params calibparams;

/****************************************************************************************************/
/*
 * This function must be use in
 * void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) function
 * for library state machines.
 */
void BMP280_Rx_it(void)
{
	if(BMP280_Reset_state == 3){BMP280_Reset_state = 4;}
	if(BMP280_Get_Status_measuring_state == 1){BMP280_Get_Status_measuring_state = 2;}
	if(BMP280_Get_Status_im_update_state == 1){BMP280_Get_Status_im_update_state = 2;}
	if(BMP280_Get_Calibration_state == 1){BMP280_Get_Calibration_state = 2;}
	if(BMP280_Get_Raw_temp_state == 1){BMP280_Get_Raw_temp_state = 2;}
	if(BMP280_Get_Raw_press_state == 1){BMP280_Get_Raw_press_state = 2;}
}
/****************************************************************************************************/
/*
 * This function must be use in
 * void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c) function
 * for library state machines.
 */
void BMP280_Tx_it(void)
{
	if(BMP280_Reset_state == 1){BMP280_Reset_state = 2;}
	if(BMP280_Set_Config_state == 1){BMP280_Set_Config_state = 2;}
	if(BMP280_Set_Ctrl_meas_state == 1){BMP280_Set_Ctrl_meas_state = 2;}

}
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
uint8_t BMP280_Reset(void)
{

	static uint8_t chip_id = 0;
	if(BMP280_Reset_state == 0)
	{
		HAL_I2C_Mem_Write_IT(BMP280_I2C_port,BMP280_I2C_address_write,BMP280_reset_ad,1,BMP280_soft_reset,1);
		BMP280_Reset_state = 1;
	}
	if(BMP280_Reset_state == 2)
	{
		HAL_I2C_Mem_Read_IT(BMP280_I2C_port,BMP280_I2C_address_write,BMP280_chip_id_ad,1,&chip_id,1);
		BMP280_Reset_state = 3;
	}
	if(BMP280_Reset_state != 4)
	{
		return BMP280_busy;
	}
	if(BMP280_Reset_state == 4 && BMP280_chip_id != chip_id)
	{
		BMP280_Reset_state = 0;
		return BMP280_error;
	}
	if(BMP280_Reset_state == 4 && BMP280_chip_id == chip_id)
	{
		BMP280_Reset_state = 0;
		return BMP280_ok;
	}

}
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
uint8_t BMP280_Set_Config(uint8_t filter,uint8_t standby)
{
	static uint8_t config_data = 0;
	if(BMP280_Set_Config_state == 0)
	{
		config_data = (filter | standby);
		HAL_I2C_Mem_Write_IT(BMP280_I2C_port,BMP280_I2C_address_write,BMP280_config_ad,1,&config_data,1);
		BMP280_Set_Config_state = 1;
	}
	if(BMP280_Set_Config_state != 2)
	{
		return BMP280_busy;
	}
	if(BMP280_Set_Config_state == 2)
	{
		BMP280_Set_Config_state = 0;
		return BMP280_ok;
	}

}
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
uint8_t BMP280_Set_Ctrl_meas(uint8_t mode,uint8_t osrs_p,uint8_t osrs_t)
{
	static uint8_t ctrl_meas_data = 0;
	if(BMP280_Set_Ctrl_meas_state == 0)
	{
		ctrl_meas_data = (mode | osrs_p | osrs_t);
		HAL_I2C_Mem_Write_IT(BMP280_I2C_port,BMP280_I2C_address_write,BMP280_ctrl_meas_ad,1,&ctrl_meas_data,1);
		BMP280_Set_Ctrl_meas_state = 1;
	}
	if(BMP280_Set_Ctrl_meas_state != 2)
	{
		return BMP280_busy;
	}
	if(BMP280_Set_Ctrl_meas_state == 2)
	{
		BMP280_Set_Ctrl_meas_state = 0;
		return BMP280_ok;
	}

}
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
uint8_t BMP280_Get_Status_measuring(void)
{
	static uint8_t status_measuring_data = 0;
	if(BMP280_Get_Status_measuring_state == 0)
	{
		HAL_I2C_Mem_Read_IT(BMP280_I2C_port,BMP280_I2C_address_write,BMP280_status_ad,1,&status_measuring_data,1);
		BMP280_Get_Status_measuring_state = 1;
	}
	if(BMP280_Get_Status_measuring_state != 2)
	{
		return BMP280_busy;
	}
	if(BMP280_Get_Status_measuring_state == 2 && status_measuring_data == BMP280_status_measuring_mask)
	{
		BMP280_Get_Status_measuring_state = 0;
		return BMP280_error;
	}
	if(BMP280_Get_Status_measuring_state == 2 && status_measuring_data != BMP280_status_measuring_mask)
	{
		BMP280_Get_Status_measuring_state = 0;
		return BMP280_ok;
	}

}
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
uint8_t BMP280_Get_Status_im_update()

{
	static uint8_t status_im_update_data = 0;
	if(BMP280_Get_Status_im_update_state == 0)
	{
		HAL_I2C_Mem_Read_IT(BMP280_I2C_port,BMP280_I2C_address_write,BMP280_status_ad,1,&status_im_update_data,1);
		BMP280_Get_Status_im_update_state = 1;
	}
	if(BMP280_Get_Status_im_update_state != 2)
	{
		return BMP280_busy;
	}
	if(BMP280_Get_Status_im_update_state == 2 && status_im_update_data == BMP280_status_im_update_mask)
	{
		BMP280_Get_Status_im_update_state = 0;
		return BMP280_error;
	}
	if(BMP280_Get_Status_im_update_state == 2 && status_im_update_data != BMP280_status_im_update_mask)
	{
		BMP280_Get_Status_im_update_state = 0;
		return BMP280_ok;
	}
}
/****************************************************************************************************/
/*
 * This function get unique calibration value from calibration register.
 *
 * 1:BMP280_busy = Function is working.
 * 2:BMP280_ok = Getting is done and  data stored in BMP280_calib_params structure.
 * NOTE:This function must be use in state machine!!!
 */
uint8_t BMP280_Get_Calibration()

{
	static uint8_t calib_array [24];
	if(BMP280_Get_Calibration_state == 0 && BMP280_Get_Status_im_update() == BMP280_ok)
	{
		HAL_I2C_Mem_Read_IT(BMP280_I2C_port,BMP280_I2C_address_write,BMP280_calibration_ad,1,&calib_array,24);
		BMP280_Get_Calibration_state = 1;
	}
	if(BMP280_Get_Calibration_state == 1)
	{
		return BMP280_busy;
	}
	if(BMP280_Get_Calibration_state == 2)
	{
		memcpy(&calibparams,calib_array,sizeof(BMP280_calib_params));
		BMP280_Get_Calibration_state = 0;
		return BMP280_ok;
	}
}
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
uint8_t BMP280_Get_Raw_temp(int32_t *raw_temp)
{
	static uint8_t raw_temp_array [3];
	if(BMP280_Get_Raw_temp_state == 0)
	{
		HAL_I2C_Mem_Read_IT(BMP280_I2C_port,BMP280_I2C_address_write,BMP280_temp_msb_ad,1,&raw_temp_array,3);
		BMP280_Get_Raw_temp_state = 1;

	}
	if(BMP280_Get_Raw_temp_state != 2)
	{
		return BMP280_busy;
	}
	if(BMP280_Get_Raw_temp_state == 2)
	{

		*raw_temp = ((raw_temp_array[0] << 12) | (raw_temp_array[1] << 4) | (raw_temp_array[2] >> 4));
		BMP280_Get_Raw_temp_state = 0;
		return BMP280_ok;
	}
}
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
uint8_t BMP280_Get_Raw_press(uint32_t *raw_press)
{
	static uint8_t raw_press_array [3];
	if(BMP280_Get_Raw_press_state == 0)
	{
		HAL_I2C_Mem_Read_IT(BMP280_I2C_port,BMP280_I2C_address_write,BMP280_press_msb_ad,1,&raw_press_array,3);
		BMP280_Get_Raw_press_state = 1;
	}
	if(BMP280_Get_Raw_press_state != 2)
	{
		return BMP280_busy;
	}
	if(BMP280_Get_Raw_press_state == 2)
	{
		*raw_press = ((raw_press_array[0] << 12) | (raw_press_array[1] << 4) | (raw_press_array[2] >> 4));
		BMP280_Get_Raw_press_state = 0;
		return BMP280_ok;
	}
}
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
uint8_t BMP280_Init(uint8_t filter,uint8_t standby,uint8_t mode,uint8_t osrs_p,uint8_t osrs_t)
{
	if(BMP280_Init_state == 0)
	{
		if(BMP280_Reset() == BMP280_ok){BMP280_Init_state = 1;}
	}
	if(BMP280_Init_state == 1)
	{
		if(BMP280_Get_Calibration() == BMP280_ok){BMP280_Init_state = 2;}
	}
	if(BMP280_Init_state == 2)
	{
		if(BMP280_Set_Config(filter, standby) == BMP280_ok){BMP280_Init_state = 3;}
	}
	if(BMP280_Init_state == 3)
	{
		if(BMP280_Set_Ctrl_meas(mode, osrs_p, osrs_t) == BMP280_ok)
		{
			BMP280_Init_state = 0;
			return BMP280_ok;
		}
	}
	if(BMP280_Init_state != 3){return BMP280_busy;}
	return BMP280_error;
}
/****************************************************************************************************/
/*
 * API
 *
 * This function give temperature with celsius unit.
 * and store it in function input argument variable
 * Argument value: Temperature (C).
 * Argument type: Double.
 */
uint8_t BMP280_Temperature_C(double *c_temp)
{
	static uint8_t BMP280_Temperature_C_state = 0;
	static double var1,var2;
	static double temp;
	static uint32_t raw_temp;
	if(BMP280_Temperature_C_state == 0 && BMP280_Get_Status_measuring() == BMP280_ok)
	{
		BMP280_Temperature_C_state = 1;
	}
	if(BMP280_Temperature_C_state == 1 && BMP280_Get_Raw_temp(&raw_temp) == BMP280_ok)
	{
		var1 = (((double)raw_temp)/16384.0 - ((double)calibparams.dig_T1)/1024.0) * ((double)calibparams.dig_T2);
		var2 = ((((double)raw_temp)/131072.0 - ((double)calibparams.dig_T1)/8192.0) * (((double)raw_temp)/131072.0 - ((double)calibparams.dig_T1)/8192.0)) * ((double)calibparams.dig_T3);
		t_fine = (int32_t)(var1 + var2);
		temp = (var1 + var2) / 5120.0;
		*c_temp = temp;
		BMP280_Temperature_C_state = 0;
		return BMP280_ok;
	}
	return BMP280_error;
}
/****************************************************************************************************/
/*
 * API
 *
 * This function give pressure with hectopascal unit
 * and store it in function input argument variable.
 * Argument value: Pressure (Hpa).
 * Argument type: Double.
 */
uint8_t BMP280_Pressure_Hpa(double *c_Press)
{
	static uint8_t BMP280_Pressure_Hpa_state = 0;
	static double var1,var2;
	static double press;
	static double temp;
	static uint32_t raw_press;
	if(BMP280_Pressure_Hpa_state == 0 && BMP280_Temperature_C(&temp) == BMP280_ok)
	{
		BMP280_Pressure_Hpa_state = 1;
	}
	if(BMP280_Pressure_Hpa_state == 1 && BMP280_Get_Status_measuring() == BMP280_ok)
	{
		BMP280_Pressure_Hpa_state = 2;
	}
	if(BMP280_Pressure_Hpa_state == 2 && BMP280_Get_Raw_press(&raw_press) == BMP280_ok)
	{
		var1 = ((double)t_fine/2.0) - 64000.0;
		var2 = var1 * var1 * ((double)calibparams.dig_P6) / 32768.0;
		var2 = var2 + var1 * ((double)calibparams.dig_P5) * 2.0;
		var2 = (var2/4.0)+(((double)calibparams.dig_P4) * 65536.0);
		var1 = (((double)calibparams.dig_P3) * var1 * var1 / 524288.0 + ((double)calibparams.dig_P2) * var1) / 524288.0;
		var1 = (1.0 + var1 / 32768.0)*((double)calibparams.dig_P1);
		if (var1 == 0.0)
		{
			return BMP280_error; // avoid exception caused by division by zero
		}
		press = 1048576.0 - (double)raw_press;
		press = (press - (var2 / 4096.0)) * 6250.0 / var1;
		var1 = ((double)calibparams.dig_P9) * press * press / 2147483648.0;
		var2 = press * ((double)calibparams.dig_P8) / 32768.0;
		press = press + (var1 + var2 + ((double)calibparams.dig_P7)) / 16.0;
		press /= 100;
		*c_Press = press;
		BMP280_Pressure_Hpa_state = 0;
		return BMP280_ok;
	}
	return BMP280_error;
}
/****************************************************************************************************/
/*
 * API
 *
 * This function calculate altitude from pressure.
 * Argument value: Altitude (cm).
 * Argument type: Double.
 */
uint8_t BMP280_Altitude_cm(double *c_Altitude)
{
	static double altitude;
	static double sea_level_press = 1013.25;
	static double press;
	if(BMP280_Pressure_Hpa(&press) == BMP280_ok)
	{
		altitude = 44330 * (1.0 - pow(press / sea_level_press, 0.1903));
		altitude *= 100;
		*c_Altitude = altitude;
		return BMP280_ok;
	}
	return BMP280_error;
}
