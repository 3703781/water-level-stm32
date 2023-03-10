/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "lcd.h"
#include "key.h"
#include "flash.h"
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
typedef struct
{
  float boot_delay;
  float before_acq;
  float after_acq;
  // the system will not need to apply params, when -0.1 < need_start < 0.1,
  float need_start;
} delay_param_struct_typedef;

typedef struct
{
  float boot_delay_counter;
  float before_acq_counter;
  float after_acq_counter;
  float is_running;
} current_status_struct_typedef;

static delay_param_struct_typedef __attribute__((aligned(4))) delay_param_struct = {
    .boot_delay = 5.0f,
    .before_acq = 2.0f,
    .after_acq = 30.0f,
    .need_start = 0.0f};
float *delay_param_float_ptr = (float *)(&delay_param_struct); // the paramemter settings

static current_status_struct_typedef __attribute__((aligned(4))) current_status_struct = {
    .after_acq_counter = 5.0f,
    .before_acq_counter = 2.0f,
    .after_acq_counter = 30.0f,
    // the current system status
    .is_running = 0.0f};

static float *current_status_float_ptr = (float *)(&current_status_struct); // the current status

int32_t water_level_ad_value = -1;
int32_t water_level_ad_value_display = -1;

uint8_t is_modifyed = 0;
uint32_t magic_num = 0x0badc0de;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

#ifdef __GNUC__
int _write(int fd, char *ptr, int len)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, 0xFFFF);
  return len;
}
#endif

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int is_running(void)
{
  /*return 1 means running return 0 means not running*/
  if (current_status_struct.is_running > 0.5f)
    return 1;
  else
    return 0;
}

int key_dispacher(int key_code, float *delay_param_ptr, float *current_status, uint32_t *which)
{
  /*dispach the key code and modify the params. */

  if (is_running())
    // block the button when the system is running
    return -1;

  if (key_code == WKUP_PRES)
  {
    (*which)++;
    (*which) %= 4;
    HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
  }
  else if (key_code == KEY0_PRES)
  {
    delay_param_ptr[*which] += 1.0f;
    if (*which != 3)
      is_modifyed = 1;
    HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
  }
  else if (key_code == KEY1_PRES)
  {
    delay_param_ptr[*which] -= 1.0f;
    if (*which != 3)
      is_modifyed = 1;
    HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
  }

  /*write to the rom*/
  return 0;
};

void update_screen(float *delay_param_ptr, current_status_struct_typedef *current_status, uint32_t *which)
{
  /*Update the screen based on the current status*/
  char *param_string[] = {"Boot Delay(s)",
                          "Before Acq(s)",
                          "After Acq(s)",
                          "     [   Wait   ]",
                          "     [   Start  ]"};

  BACK_COLOR = BLACK;
  // display current paramemters in delay_param_ptr
  for (int i = 0; i < 3; i++)
  {
    POINT_COLOR = WHITE;
    // LCD_ShowString(30, 10 + 40 * i, 200, 24, 24, param_string[i]);             // display the param string
    LCD_ShowString(250, 10 + 40 * i, 50, 24, 24, "%4.1f", delay_param_ptr[i]); // display the param
    // show the counter if it is running
    if (is_running())
    {
      POINT_COLOR = PINK;
      LCD_ShowString(185, 10 + 40 * i, 50, 24, 24, "%4.1f", ((float *)current_status)[i]);
    }
    else
    {
      LCD_ShowString(185, 10 + 40 * i, 50, 24, 24, "    ");
    }
  }

  POINT_COLOR = WHITE;
  // instruct the current selected item >
  for (int i = 0; i < 4; i++)
  {
    if (*which == i)
      LCD_ShowString(5, 10 + 40 * i, 5, 24, 24, ">"); // instruct the current param
    else
      LCD_ShowString(5, 10 + 40 * i, 5, 24, 24, " "); // instruct the current param
  }

  // display the start stop button
  if (is_running())
    // the system is running, try to stop it
    LCD_ShowString(20, 130, 300, 24, 24, param_string[3]);
  else
    LCD_ShowString(20, 130, 300, 24, 24, param_string[4]);

  POINT_COLOR = LIGHTBLUE;
  // get the water levele value from the ad
  if (water_level_ad_value > 0)
    water_level_ad_value_display = water_level_ad_value;
  // show the ui
  LCD_ShowString(30, 190, 300, 24, 24, "A/D Value");
  // show the waterlevel
  if (water_level_ad_value_display > 0)
    LCD_ShowString(200, 190, 100, 24, 24, "%6.4f V", (float)water_level_ad_value_display * 3.3f / 4096.0f);
  else
    LCD_ShowString(200, 190, 100, 24, 24, "       ");
};

int perform_operation(delay_param_struct_typedef *delay_param_struct, current_status_struct_typedef *current_status)
{
  // if the status is not change, just return
  if (-0.25f < delay_param_struct->need_start && delay_param_struct->need_start < 0.25f)
    return 0;
  if (is_modifyed)
  {
    FLASH_Write(FLASH_SAVE_ADDR, (uint16_t *)&magic_num, sizeof(magic_num) / 2);
    FLASH_Write(FLASH_SAVE_ADDR + 16, (uint16_t *)delay_param_struct, sizeof(*delay_param_struct) / 2);
    is_modifyed = 0;
  }

  if (!is_running())
  {
    // Turn on the relay
    HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_SET);
    // start the timer
    HAL_TIM_Base_Start_IT(&htim1);
    // show the counters
    current_status->before_acq_counter = delay_param_struct->before_acq;
    current_status->after_acq_counter = delay_param_struct->after_acq;
    current_status->boot_delay_counter = delay_param_struct->boot_delay;
    current_status->is_running = 1.0f;
    water_level_ad_value_display = -1;
  }
  // else
  // {
  //   // TODO: CANCLE THE PROCESS
  //   printf("cancle\r\n");
  // }
  // restore the button status
  // delay_param_struct->need_start = 0.0f;
  return 0;
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  /* USER CODE BEGIN 1 */
  uint32_t which_param = 3; // the selected item
  uint32_t magic_tmp = 0;
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
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  LCD_Init();
  KEY_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  FLASH_Read(FLASH_SAVE_ADDR, (uint16_t *)&magic_tmp, sizeof(magic_tmp) / 2);
  if (magic_tmp == magic_num)
  {
    FLASH_Read(FLASH_SAVE_ADDR + 16, (uint16_t *)&delay_param_struct, sizeof(delay_param_struct) / 2);
    delay_param_struct.need_start = 0;
  }

  POINT_COLOR = DARKBLUE;
  BACK_COLOR = WHITE;
  LCD_ShowString(25, 100, 300, 24, 24, "Loading...");

  HAL_Delay((int)(delay_param_struct.boot_delay * 1000));
  POINT_COLOR = WHITE;
  BACK_COLOR = BLACK;
  LCD_Clear(BLACK);
  LCD_ShowString(30, 10, 200, 24, 24, "Boot Delay(s)");
  LCD_ShowString(30, 50, 200, 24, 24, "Before Acq(s)");
  LCD_ShowString(30, 90, 200, 24, 24, "After Acq(s)");
  POINT_COLOR = 0xad55; // rgb 565
  LCD_DrawLine(20, 45, 300, 45);
  LCD_DrawLine(20, 85, 300, 85);
  LCD_DrawLine(20, 125, 300, 125);
  LCD_DrawLine(20, 180, 300, 180);
  while (1)
  { // scan the keys to get the user operation
    uint8_t key_code = KEY_Scan(0);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // ===================== key dispach ==========================
    key_dispacher(key_code, delay_param_float_ptr, current_status_float_ptr, &which_param);
    // ======================= display ===========================
    update_screen(delay_param_float_ptr, &current_status_struct, &which_param);
    // ================== perform the operation ===================
    perform_operation(&delay_param_struct, &current_status_struct);
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

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1)
  {
    // show running status
    HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);

    // counter before the acquisition
    if (current_status_struct.before_acq_counter > 0.0f)
      current_status_struct.before_acq_counter -= 0.1f;
    else if (water_level_ad_value < 0)
    {
      // printf("acquire the water \r\n");
      current_status_struct.before_acq_counter = 0.0f;
      // acquire the water level
      water_level_ad_value = ADC_Read(&hadc1, 200);
      HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_RESET);
    }

    // counter after the acquisition
    if (current_status_struct.after_acq_counter > 0.0f && water_level_ad_value >= 0)
      current_status_struct.after_acq_counter -= 0.1f;

    // stop if all the things are done
    if (current_status_struct.after_acq_counter <= 0.0f && water_level_ad_value >= 0)
    {
      HAL_TIM_Base_Stop_IT(htim);
      current_status_struct.is_running = 0.0f;
      water_level_ad_value = -1;
    }
  }
}
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
    printf("error\r\n");
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
