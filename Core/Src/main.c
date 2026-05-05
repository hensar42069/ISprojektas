/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// Pressure averaging buffer
float pressure_buffer[10] = {0};
uint8_t buffer_index = 0;
uint8_t buffer_full = 0;
float pressure_avg = 0;

// Timing
uint32_t last_lcd_update = 0;
uint32_t last_uart_update = 0;
uint32_t last_sample = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define BMP280_ADDR (0x76 << 1)

uint16_t dig_T1, dig_P1;
int16_t dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
int32_t t_fine;
uint8_t bmp_ok = 0;

uint8_t BMP280_Read8(uint8_t reg) {
    uint8_t val;
    HAL_I2C_Mem_Read(&hi2c1, BMP280_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
    return val;
}

uint16_t BMP280_Read16(uint8_t reg) {
    uint8_t buf[2];
    HAL_I2C_Mem_Read(&hi2c1, BMP280_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, 2, 100);
    return (buf[1] << 8) | buf[0];
}

uint32_t BMP280_Read24(uint8_t reg) {
    uint8_t buf[3];
    HAL_I2C_Mem_Read(&hi2c1, BMP280_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, 3, 100);
    return ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
}

void BMP280_Write8(uint8_t reg, uint8_t val) {
    HAL_I2C_Mem_Write(&hi2c1, BMP280_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
}

void BMP280_Init(void) {
    uint8_t id = BMP280_Read8(0xD0);
    
   
    if (id != 0x58 && id != 0x60) {
        bmp_ok = 0;
        return;
    }
    
   
    dig_T1 = BMP280_Read16(0x88);
    dig_T2 = (int16_t)BMP280_Read16(0x8A);
    dig_T3 = (int16_t)BMP280_Read16(0x8C);
    dig_P1 = BMP280_Read16(0x8E);
    dig_P2 = (int16_t)BMP280_Read16(0x90);
    dig_P3 = (int16_t)BMP280_Read16(0x92);
    dig_P4 = (int16_t)BMP280_Read16(0x94);
    dig_P5 = (int16_t)BMP280_Read16(0x96);
    dig_P6 = (int16_t)BMP280_Read16(0x98);
    dig_P7 = (int16_t)BMP280_Read16(0x9A);
    dig_P8 = (int16_t)BMP280_Read16(0x9C);
    dig_P9 = (int16_t)BMP280_Read16(0x9E);
    
    
    BMP280_Write8(0xF4, 0x57);
    BMP280_Write8(0xF5, 0x10);
    
    bmp_ok = 1;
}

float Sensor_ReadPressure(void) {
    
    int32_t adc_T = BMP280_Read24(0xFA);
    if (adc_T == 0x800000) adc_T = 0;  
    
    
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
    t_fine = var1 + var2;
    
    
    int32_t adc_P = BMP280_Read24(0xF7);
    if (adc_P == 0x800000) adc_P = 0;
    

    var1 = (((int64_t)t_fine) >> 1) - (int64_t)64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * (int64_t)dig_P6;
    var2 = var2 + ((var1 * (int64_t)dig_P5) << 1);
    var2 = (var2 >> 2) + ((int64_t)dig_P4 << 16);
    var1 = (((dig_P3 * ((var1 >> 2) * (var1 >> 2)) >> 13) >> 3) + (((int64_t)dig_P2 * var1) >> 1)) >> 18;
    var1 = ((((32768 + var1)) * (int64_t)dig_P1) >> 15);
    
    if (var1 == 0) return 0;
    
    uint32_t p = (((uint32_t)(((int64_t)1048576) - adc_P) - (var2 >> 12))) * 3125;
    if (p < 0x80000000) p = (p << 1) / ((uint32_t)var1);
    else p = (p / (uint32_t)var1) * 2;
    
    var1 = (((int64_t)dig_P9) * ((int64_t)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
    var2 = (((int64_t)(p >> 2)) * (int64_t)dig_P8) >> 13;
    p = (uint32_t)((int64_t)p + ((var1 + var2 + dig_P7) >> 4));
    
    return (((float)p / 100.0f) - 220.0f) / 10;  //Sukalibruota su tikro jutiklio rodmenimis
}



void LCD_Pulse(void) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_Delay(1);
}

void LCD_Nibble(uint8_t data) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,  (data & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1,  (data & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2,  (data & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, (data & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    LCD_Pulse();
}

void LCD_Send(uint8_t data, uint8_t rs) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, rs ? GPIO_PIN_SET : GPIO_PIN_RESET);
    LCD_Nibble(data >> 4);
    LCD_Nibble(data & 0x0F);
}

void LCD_Cmd(uint8_t cmd)  { LCD_Send(cmd, 0); }
void LCD_Chr(char c)       { LCD_Send(c, 1); }
void LCD_Str(const char *s){ while(*s) LCD_Chr(*s++); }

void LCD_Init(void) {

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_Delay(50);
    

    LCD_Nibble(0x03); HAL_Delay(5);
    LCD_Nibble(0x03); HAL_Delay(5);
    LCD_Nibble(0x03); HAL_Delay(5);
    LCD_Nibble(0x02); HAL_Delay(5);
    
    LCD_Cmd(0x28); HAL_Delay(1);  
    LCD_Cmd(0x0C); HAL_Delay(1);  
    LCD_Cmd(0x01); HAL_Delay(5);  
    LCD_Cmd(0x06); HAL_Delay(1);  
}

void LCD_SetCursor(uint8_t row, uint8_t col) {
    LCD_Cmd(0x80 | ((row ? 0x40 : 0x00) + col));
    HAL_Delay(1);
}

void LCD_Clear(void) {
    LCD_Cmd(0x01);
    HAL_Delay(5);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	//Nenaudojama
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
     
  char msg[50];
  
  HAL_UART_Transmit(&huart2, (uint8_t*)"\r\nPrisijungiama...\r\n", 22, 100);
  
  LCD_Init();
  
 
  if (HAL_I2C_IsDeviceReady(&hi2c1, (0x76 << 1), 3, 100) == HAL_OK) {
      HAL_UART_Transmit(&huart2, (uint8_t*)"Yra itaisas 0x76\r\n", 22, 100);
      
    
      uint8_t id;
      HAL_I2C_Mem_Read(&hi2c1, (0x76 << 1), 0xD0, I2C_MEMADD_SIZE_8BIT, &id, 1, 100);
      sprintf(msg, "ID: 0x%02X\r\n", id);
      HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
      
      LCD_Clear();
      LCD_SetCursor(0, 0);
      sprintf(msg, "BMP280: 0x%02X", id);
      LCD_Str(msg);
      
      
      BMP280_Init();
      if (bmp_ok) {
          LCD_SetCursor(1, 0);
          LCD_Str("Prisijungta      ");
          HAL_UART_Transmit(&huart2, (uint8_t*)"BMP280 OK\r\n", 16, 100);
      } else {
          LCD_SetCursor(1, 0);
          LCD_Str("Neprisijungta    ");
          HAL_UART_Transmit(&huart2, (uint8_t*)"Neprisijungta\r\n", 13, 100);
      }
  } else {
      HAL_UART_Transmit(&huart2, (uint8_t*)"Nieko nera 0x76\r\n", 18, 100);
      LCD_Clear();
      LCD_Str("Neprisijungta");
  }
  
  HAL_Delay(3000);
  LCD_Clear();
  /* USER CODE END 2 */
  


  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
			  

    /* USER CODE BEGIN 3 */
    
     
    if(HAL_GetTick() - last_sample >= 100) {
        last_sample = HAL_GetTick();
        float p = Sensor_ReadPressure();
        
        
        pressure_buffer[buffer_index] = p;
        buffer_index++;
        if(buffer_index >= 10) {
            buffer_index = 0;
            buffer_full = 1;
        }
        
        
        float sum = 0;
        uint8_t count = buffer_full ? 10 : buffer_index;
        if(count > 0) {
            for(uint8_t i = 0; i < count; i++) sum += pressure_buffer[i];
            pressure_avg = sum / count;
        }
    }
    
    
     
    if(HAL_GetTick() - last_lcd_update >= 5000) {
        last_lcd_update = HAL_GetTick();
        
        
        char line1[17];  
        char line2[17];
        
        snprintf(line1, 17, "P: %-9.1f kPa", pressure_avg);
        line1[16] = 0;  
        
      
        LCD_SetCursor(0, 0);
        LCD_Str(line1);
        
        
    }
    
    if(HAL_GetTick() - last_uart_update >= 1000) {
        last_uart_update = HAL_GetTick();
        sprintf(msg, "Vidurkis: %.1f kPa\r\n", pressure_avg);
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
    }
    
  

    
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00100E15;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIMEx_RemapConfig(&htim3, TIM3_TI1_GPIO) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : BTN_MODE_Pin */
  GPIO_InitStruct.Pin = BTN_MODE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BTN_MODE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10
                           PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//Nenaudojama
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */