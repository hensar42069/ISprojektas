#include "bmp280.h"

static BMP280_CalibData calib_data;
static I2C_HandleTypeDef *bmp280_i2c;
static uint8_t device_address = BMP280_I2C_ADDR;

static HAL_StatusTypeDef BMP280_ReadRegisters(uint8_t reg, uint8_t *data, uint8_t len) {
    return HAL_I2C_Mem_Read(bmp280_i2c, device_address, reg, I2C_MEMADD_SIZE_8BIT, data, len, 100);
}

static HAL_StatusTypeDef BMP280_WriteRegister(uint8_t reg, uint8_t value) {
    return HAL_I2C_Mem_Write(bmp280_i2c, device_address, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 100);
}

HAL_StatusTypeDef BMP280_Init(I2C_HandleTypeDef *hi2c) {
    bmp280_i2c = hi2c;
    uint8_t chip_id;
    uint8_t calib[24];
    HAL_StatusTypeDef status;
    
    // Try device address 0x76 first
    device_address = BMP280_I2C_ADDR_76;
    status = HAL_I2C_IsDeviceReady(bmp280_i2c, device_address, 3, 100);
    
    if (status != HAL_OK) {
        // Try 0x77
        device_address = BMP280_I2C_ADDR_77;
        status = HAL_I2C_IsDeviceReady(bmp280_i2c, device_address, 3, 100);
        
        if (status != HAL_OK) {
            return HAL_ERROR;  // No device found at either address
        }
    }
    
    // Read chip ID
    status = BMP280_ReadRegisters(BMP280_REG_ID, &chip_id, 1);
    if (status != HAL_OK || (chip_id != BMP280_CHIP_ID && chip_id != 0x60)) {
        // BME280 returns 0x60, BMP280 returns 0x58
        // If neither, might still work - continue cautiously
    }
    
    // Reset sensor
    BMP280_WriteRegister(BMP280_REG_RESET, 0xB6);
    HAL_Delay(10);
    
    // Read calibration data
    if (BMP280_ReadRegisters(BMP280_REG_CALIB00, calib, 24) != HAL_OK) {
        return HAL_ERROR;
    }
    
    calib_data.dig_T1 = (calib[1] << 8) | calib[0];
    calib_data.dig_T2 = (calib[3] << 8) | calib[2];
    calib_data.dig_T3 = (calib[5] << 8) | calib[4];
    calib_data.dig_P1 = (calib[7] << 8) | calib[6];
    calib_data.dig_P2 = (calib[9] << 8) | calib[8];
    calib_data.dig_P3 = (calib[11] << 8) | calib[10];
    calib_data.dig_P4 = (calib[13] << 8) | calib[12];
    calib_data.dig_P5 = (calib[15] << 8) | calib[14];
    calib_data.dig_P6 = (calib[17] << 8) | calib[16];
    calib_data.dig_P7 = (calib[19] << 8) | calib[18];
    calib_data.dig_P8 = (calib[21] << 8) | calib[20];
    calib_data.dig_P9 = (calib[23] << 8) | calib[22];
    
    // Configure: Normal mode, temp osr x1, pressure osr x1, normal mode
    // This sets: 001 001 11 = 0x27
    BMP280_WriteRegister(BMP280_REG_CTRL_MEAS, 0x27);
    
    // Optional: Configure filter and standby
    BMP280_WriteRegister(BMP280_REG_CONFIG, 0x00);
    
    HAL_Delay(50);  // Wait for first conversion
    
    return HAL_OK;
}

static int32_t BMP280_CompensateTemperature(int32_t adc_T) {
    int32_t var1, var2;
    
    var1 = ((((adc_T >> 3) - ((int32_t)calib_data.dig_T1 << 1))) * ((int32_t)calib_data.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)calib_data.dig_T1)) * ((adc_T >> 4) - ((int32_t)calib_data.dig_T1))) >> 12) * ((int32_t)calib_data.dig_T3)) >> 14;
    calib_data.t_fine = var1 + var2;
    
    return (calib_data.t_fine * 5 + 128) >> 8;
}

static float BMP280_CompensatePressure(int32_t adc_P) {
    int64_t var1, var2, p;
    
    var1 = ((int64_t)calib_data.t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)calib_data.dig_P6;
    var2 = var2 + ((var1 * (int64_t)calib_data.dig_P5) << 17);
    var2 = var2 + (((int64_t)calib_data.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)calib_data.dig_P3) >> 8) + ((var1 * (int64_t)calib_data.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)calib_data.dig_P1) >> 33;
    
    if (var1 == 0) {
        return 0.0f;
    }
    
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)calib_data.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)calib_data.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)calib_data.dig_P7) << 4);
    
    return (float)p / 25600.0f; // Convert to kPa
}

float BMP280_ReadTemperature(I2C_HandleTypeDef *hi2c) {
    uint8_t data[3];
    int32_t adc_T;
    
    if (BMP280_ReadRegisters(BMP280_REG_TEMP_MSB, data, 3) != HAL_OK) {
        return -273.0f;  // Error value
    }
    
    adc_T = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | (data[2] >> 4);
    
    return BMP280_CompensateTemperature(adc_T) / 100.0f;
}

float BMP280_ReadPressure(I2C_HandleTypeDef *hi2c) {
    uint8_t data[3];
    int32_t adc_P;
    
    // Read temperature first to update t_fine
    BMP280_ReadTemperature(hi2c);
    
    // Read pressure
    if (BMP280_ReadRegisters(BMP280_REG_PRESS_MSB, data, 3) != HAL_OK) {
        return 0.0f;
    }
    
    adc_P = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | (data[2] >> 4);
    
    return BMP280_CompensatePressure(adc_P);
}