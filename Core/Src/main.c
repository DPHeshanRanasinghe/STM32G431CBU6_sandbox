/* USER CODE BEGIN Header */
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
#include "i2c.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/


/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "usbd_cdc_if.h" // Required for USB-C serial transmission
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

/* USER CODE BEGIN PV */
uint8_t mpu6050_address = 0;
uint8_t mpu6050_ready = 0;

// IMU Raw Data
int16_t raw_acc_x = 0, raw_acc_y = 0, raw_acc_z = 0;
int16_t raw_gyro_x = 0, raw_gyro_y = 0, raw_gyro_z = 0;

// Calibration Offsets
int32_t gyro_x_offset = 0, gyro_y_offset = 0, gyro_z_offset = 0;

// USB Buffer
char usb_tx_buffer[192];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USB_Device_Init();
  MX_I2C1_Init();
/* USER CODE BEGIN 2 */
  HAL_Delay(800); 
  
  // 1. Scan for MPU6050
  for (uint8_t i = 1; i < 128; i++)
  {
      if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i << 1), 2, 10) == HAL_OK)
      {
          if ((i == 0x68) || (i == 0x69))
          {
              mpu6050_address = i;
              break;
          }
      }
  }

  // 2. Wake Up Sensor
  if (mpu6050_address != 0)
  {
      uint8_t wake_data = 0x00;
      if (HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(mpu6050_address << 1), 0x6B, 1, &wake_data, 1, 10) == HAL_OK)
      {
          mpu6050_ready = 1;
      }
  }

  // 3. Calibrate Gyroscopes (Keep it still for 1 second!)
  if (mpu6050_ready == 1)
  {
      int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;
      uint8_t calib_buf[6]; 

      for (uint16_t i = 0; i < 500; i++)
      {
          if (HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(mpu6050_address << 1), 0x43, 1, calib_buf, 6, 10) == HAL_OK)
          {
              sum_gx += (int16_t)((calib_buf[0] << 8) | calib_buf[1]);
              sum_gy += (int16_t)((calib_buf[2] << 8) | calib_buf[3]);
              sum_gz += (int16_t)((calib_buf[4] << 8) | calib_buf[5]);
          }
          HAL_Delay(2); 
      }
      
      gyro_x_offset = sum_gx / 500;
      gyro_y_offset = sum_gy / 500;
      gyro_z_offset = sum_gz / 500;
  }
/* USER CODE END 2 */

  /* Infinite loop */
/* Infinite loop */
/* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      uint8_t i2c_buf[14]; 
      uint8_t read_ok = 0;
      
      // Read all 6 axes
      if ((mpu6050_ready != 0) && (HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(mpu6050_address << 1), 0x3B, 1, i2c_buf, 14, 10) == HAL_OK))
      {
          read_ok = 1;
          raw_acc_x = (int16_t)((i2c_buf[0] << 8) | i2c_buf[1]);
          raw_acc_y = (int16_t)((i2c_buf[2] << 8) | i2c_buf[3]);
          raw_acc_z = (int16_t)((i2c_buf[4] << 8) | i2c_buf[5]);
          
          raw_gyro_x = (int16_t)((i2c_buf[8] << 8)  | i2c_buf[9]);
          raw_gyro_y = (int16_t)((i2c_buf[10] << 8) | i2c_buf[11]);
          raw_gyro_z = (int16_t)((i2c_buf[12] << 8) | i2c_buf[13]);
      }

      int msg_len;

      if (read_ok != 0)
      {
          // Scaled integer output avoids needing float printf support.
          int32_t gx_mdps = ((int32_t)raw_gyro_x - gyro_x_offset) * 1000L / 131L;
          int32_t gy_mdps = ((int32_t)raw_gyro_y - gyro_y_offset) * 1000L / 131L;
          int32_t gz_mdps = ((int32_t)raw_gyro_z - gyro_z_offset) * 1000L / 131L;

          int32_t ax_mmps2 = (int32_t)raw_acc_x * 9810L / 16384L;
          int32_t ay_mmps2 = (int32_t)raw_acc_y * 9810L / 16384L;
          int32_t az_mmps2 = (int32_t)raw_acc_z * 9810L / 16384L;

          msg_len = snprintf(usb_tx_buffer, sizeof(usb_tx_buffer),
                             "Gyro_mdps [X:%ld Y:%ld Z:%ld] Accel_mmps2 [X:%ld Y:%ld Z:%ld] RawAcc [X:%d Y:%d Z:%d]\r\n",
                             (long)gx_mdps, (long)gy_mdps, (long)gz_mdps,
                             (long)ax_mmps2, (long)ay_mmps2, (long)az_mmps2,
                             raw_acc_x, raw_acc_y, raw_acc_z);
      }
      else
      {
          msg_len = snprintf(usb_tx_buffer, sizeof(usb_tx_buffer),
                             "MPU6050 not ready/read failed. addr=0x%02X ready=%u\r\n",
                             mpu6050_address, mpu6050_ready);
      }

      if (msg_len > 0)
      {
          CDC_Transmit_FS((uint8_t*)usb_tx_buffer, (uint16_t)msg_len);
      }

      HAL_Delay(500);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
