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
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MOTOR_PWM_MAX_DUTY       (__HAL_TIM_GET_AUTORELOAD(&htim2) + 1U)
#define PID_SAMPLE_TIME_MS       50U
#define PID_SAMPLE_TIME_SEC      0.05f
#define ENCODER_TICKS_PER_REV    7392.0f
#define PI                       3.14159265f
#define WHEEL_RADIUS_M           0.033f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile int16_t encoder_ticks = 0;
int16_t prev_ticks = 0;

/* PID Variables */
volatile float target_rpm = 0.0f; // Updated by USB CDC command input.
volatile float target_velocity_mps = 0.0f;
extern volatile uint8_t target_rpm_updated;
float current_rpm = 0.0f;
float error = 0.0f;
float previous_error = 0.0f;
float integral = 0.0f;

/* Tuning Gains (Start with P and I only) */
float Kp = 10.0f; // The Muscle
float Ki = 0.5f;  // The Memory
float Kd = 0.0f;  // The Brakes

float control_output = 0.0f; // The calculated adjustment
float current_pwm = 0.0f;    // The actual PWM applied to the motor
char usb_msg[96];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void Motor_Forward(uint32_t speed)
{
  if (speed > MOTOR_PWM_MAX_DUTY)
  {
    speed = MOTOR_PWM_MAX_DUTY;
  }

  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);     // LPWM (Reverse OFF)
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, speed); // RPWM (Forward)
}

static void Motor_Reverse(uint32_t speed)
{
  if (speed > MOTOR_PWM_MAX_DUTY)
  {
    speed = MOTOR_PWM_MAX_DUTY;
  }

  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);     // RPWM (Forward OFF)
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, speed); // LPWM (Reverse)
}
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
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  __HAL_TIM_SET_COUNTER(&htim3, 0);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  Motor_Forward(0);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* 1. Read Current Ticks */
    encoder_ticks = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);

    /* 2. Calculate the difference (This naturally handles the 16-bit rollover!) */
    int16_t tick_diff = encoder_ticks - prev_ticks;
    prev_ticks = encoder_ticks;

    /* Make forward rotation positive based on the current encoder wiring. */
    tick_diff = -tick_diff;

    /* 3. Convert tick difference to RPM (50ms loop) */
    current_rpm = ((float)tick_diff * 60.0f) / (ENCODER_TICKS_PER_REV * PID_SAMPLE_TIME_SEC);

    /* 4. The PID Math */
    if (target_rpm_updated != 0U)
    {
      integral = 0.0f;
      previous_error = 0.0f;
      target_rpm_updated = 0U;
    }

    error = target_rpm - current_rpm;
    integral += error * PID_SAMPLE_TIME_SEC;
    
    // Anti-Windup: Prevent the memory from getting ridiculously large
    if (integral > 100.0f) integral = 100.0f;
    if (integral < -100.0f) integral = -100.0f;

    float derivative = (error - previous_error) / PID_SAMPLE_TIME_SEC;
    previous_error = error;

    // Calculate the total adjustment needed
    control_output = (Kp * error) + (Ki * integral) + (Kd * derivative);

    /* 5. Apply the adjustment to the Motor PWM */
    current_pwm += control_output;

    // Safety Clamp: allow full reverse to full forward
    if (current_pwm > (float)MOTOR_PWM_MAX_DUTY) current_pwm = (float)MOTOR_PWM_MAX_DUTY;
    if (current_pwm < -(float)MOTOR_PWM_MAX_DUTY) current_pwm = -(float)MOTOR_PWM_MAX_DUTY;

    if (current_pwm >= 0.0f)
    {
      Motor_Forward((uint32_t)current_pwm);
    }
    else
    {
      Motor_Reverse((uint32_t)(-current_pwm));
    }

    /* 6. Telemetry: Send data to USB */
    int msg_len = snprintf(usb_msg, sizeof(usb_msg), "Target: %d rpm | Cmd: %d mm/s | RPM: %d | PWM: %d\r\n",
                           (int)target_rpm, (int)(target_velocity_mps * 1000.0f),
                           (int)current_rpm, (int)current_pwm);
    if (msg_len > 0)
    {
      CDC_Transmit_FS((uint8_t*)usb_msg, (uint16_t)msg_len);
    }

    /* Toggle the onboard LED */
    HAL_GPIO_TogglePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin);
    
    /* 7. Timing Control: 50ms (20Hz Update Rate) */
    HAL_Delay(PID_SAMPLE_TIME_MS);
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
