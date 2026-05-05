#ifndef BMP280_H
#define BMP280_H

#include "main.h"
#include <stdint.h>

// Try both addresses - 0x76 is most common for 4-pin modules
#define BMP280_I2C_ADDR_76     (0x76 << 1)
#define BMP280_I2C_ADDR_77     (0x77 << 1)

// Use this address (change if needed)
#define BMP280_I2C_ADDR        BMP280_I2C_ADDR_76

#define BMP280_CHIP_ID        0x58

// BMP280 Register addresses
#define BMP280_REG_ID         0xD0
#define BMP280_REG_RESET      0xE0
#define BMP280_REG_STATUS     0xF3
#define BMP280_REG_CTRL_MEAS  0xF4
#define BMP280_REG_CONFIG     0xF5
#define BMP280_REG_PRESS_MSB  0xF7
#define BMP280_REG_PRESS_LSB  0xF8
#define BMP280_REG_PRESS_XLSB 0xF9
#define BMP280_REG_TEMP_MSB   0xFA
#define BMP280_REG_TEMP_LSB   0xFB
#define BMP280_REG_TEMP_XLSB  0xFC

#define BMP280_REG_CALIB00    0x88

typedef struct {
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
    int32_t  t_fine;
} BMP280_CalibData;

HAL_StatusTypeDef BMP280_Init(I2C_HandleTypeDef *hi2c);
float BMP280_ReadPressure(I2C_HandleTypeDef *hi2c);
float BMP280_ReadTemperature(I2C_HandleTypeDef *hi2c);

#endif