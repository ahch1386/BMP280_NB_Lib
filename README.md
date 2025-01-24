# BMP280_NB_Lib
## BMP280_NB_Lib is the first Non_blocking library for Bosch BMP280 barometric sensor and STM32 microcontrollers I2C communication.  
  
This library is developed with HAL functions for STM32F1 series by default, but you can   
use it in each STM32 microcontrollers easily.So don't worry!  
  
This library is very user friendly and easy to use for every one.  
  
This library has following facilities:  
1. Temperature measurement.  
2. Pressure measurement.  
3. Altitude measurement.

This library has four API functions:  
1. BMP280_Init();  
2. BMP280_Temperature_C();  
3. BMP280_Pressure_Hpa();
4. BMP280_Altitude_cm();  

# Usage
### You can see how to use the library in example code folder
### and .h file comments
### and following below description:

1. Make a project in your IDE.
2. Add BMP280_NB_Lib folder in your project.
3. #include "BMP280.h" header file in your project.
4. You must #include "stm32xxxx_hal.h" file for your STM32 microcontroller in BMP280.h file.
5. Go to BMP280.h file and select your sensor I2C address and I2C port:  
BMP280.h settings:  
  // Device I2C addresses for write in hal functions.  
  #define BMP280_I2C_address_write 		  0xEC     // if SDO pin conneted to GND = 0x76  
  //#define BMP280_I2C_address_write 		0xEE     // if SDO pin conneted to VDDIO = 0x77  
  /****************************************************************************************************/  
  // BMP280 I2C port.  
  #define BMP280_I2C_port               &hi2c1   // I2C port where the chip connected
6. 

   
