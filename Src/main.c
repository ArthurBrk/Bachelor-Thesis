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
#define MASS		   221	  // Handy
#define THRUST_SIGN	   1.0f
#define THRUST_LIMIT   1800.0f
#define ALPHA		   0.3f   // GLAETTUNG
#define ESC_MIN        1000   // Stillstand / Arming
#define ESC_MAX        2000   // Vollgas
#define MAX_THROTTLE   100
#define STEPS          2      // Schrittweite pro Tastendruck
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;
__IO uint32_t BspButtonState = BUTTON_RELEASED;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
static volatile int thr1 = 0, thr2 = 0;
extern UART_HandleTypeDef hcom_uart[];
static int32_t hx_offset = 0;
static int32_t hx_raw = 0;
static float hx_scale = 1.0f;
static float thrust_g = 0.0f;
static uint8_t hx_calibrated = 0;
static uint8_t stream = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void hx_delay(void)
{
	for(volatile int i = 0; i < 20; i++)
		__NOP();
}

static void hx711_init(void)
{
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    g.Pin   = SCK_Pin;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(SCK_GPIO_Port, &g);
    HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_Pin, GPIO_PIN_RESET);  // SCK low = HX711 aktiv

    g.Pin  = DT_Pin;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(DT_GPIO_Port, &g);
}

static int hx711_ready(void)   // DT = low bedeutet: neuer Messwert liegt an
{
    return HAL_GPIO_ReadPin(DT_GPIO_Port, DT_Pin) == GPIO_PIN_RESET;
}

static int32_t hx711_read_raw(void)
{
    uint32_t v = 0;
    __disable_irq();   // SCK darf nie >60 us high bleiben, sonst schaltet der HX711 ab
    for (int i = 0; i < 24; i++)
    {
        HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_Pin, GPIO_PIN_SET);
        hx_delay();
        v = (v << 1) | (HAL_GPIO_ReadPin(DT_GPIO_Port, DT_Pin) == GPIO_PIN_SET);
        HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_Pin, GPIO_PIN_RESET);
        hx_delay();
    }
    // 25. Puls: naechste Messung auf Kanal A mit Verstaerkung 128
    HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_Pin, GPIO_PIN_SET);
    hx_delay();
    HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_Pin, GPIO_PIN_RESET);
    hx_delay();
    __enable_irq();

    if (v & 0x800000) v |= 0xFF000000;   // 24 Bit Vorzeichen erweitern
    return (int32_t)v;
}

static int hx711_avg(int n, int32_t *out)
{
    int64_t sum = 0;
    for (int i = 0; i < n; i++)
    {
        uint32_t t0 = HAL_GetTick();
        while (!hx711_ready())
        {
            if (HAL_GetTick() - t0 > 500) return -1;   // HX711 antwortet nicht
        }
        sum += hx711_read_raw();
    }
    *out = (int32_t)(sum / n);
    return 0;
}

static void do_tare(void)
{
    int32_t v;
    if (thr1 != 0 || thr2 != 0) { printf("Tara nur bei gestoppten Motoren!\r\n"); return; }
    if (hx711_avg(10, &v) == 0) { hx_offset = v; thrust_g = 0.0f; printf("Tara OK (roh %ld)\r\n", (long)v); }
    else printf("HX711 antwortet nicht - Verdrahtung pruefen!\r\n");
}

static void do_calibrate(void)
{
    int32_t v;
    if (thr1 != 0 || thr2 != 0) { printf("Kalibrieren nur bei gestoppten Motoren!\r\n"); return; }
    if (hx711_avg(10, &v) != 0) { printf("HX711 antwortet nicht!\r\n"); return; }
    int32_t d = v - hx_offset;
    if (d == 0) { printf("Kein Unterschied zur Tara - Gewicht angehaengt?\r\n"); return; }
    hx_scale = (float)d / MASS;
    hx_calibrated = 1;
    thrust_g = THRUST_SIGN * MASS;
    printf("Kalibriert mit %d g (%ld Zaehlwerte)\r\n", (int)MASS, (long)d);
}

static void print_status(void)
{
    int32_t t10 = (int32_t)(thrust_g * 10.0f);   // 0,1-g-Schritte, ohne printf-Float
    char sign = (t10 < 0) ? '-' : ' ';
    if (t10 < 0) t10 = -t10;
    printf("M1: %2d %%  M2: %2d %%  Schub:%c%ld.%ld g%s  (roh %ld)\r\n",
           thr1, thr2, sign, (long)(t10 / 10), (long)(t10 % 10),
           hx_calibrated ? "" : " [unkalibriert]", (long)hx_raw);
}

static int limit(int x)
{
      if (x < 0)
    	  return 0;
      if (x > MAX_THROTTLE)
    	  return MAX_THROTTLE;
      return x;
}

static void throttle(void)
{
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ESC_MIN + thr1 * (ESC_MAX - ESC_MIN) / 100);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, ESC_MIN + thr2 * (ESC_MAX - ESC_MIN) / 100);
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
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  throttle();
  hx711_init();
  /* USER CODE END 2 */

  /* Initialize led */
  BSP_LED_Init(LED_GREEN);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN BSP */
  setvbuf(stdin,  NULL, _IONBF, 0);
  setvbuf(stdout, NULL, _IONBF, 0);
  BSP_LED_On(LED_GREEN);
  /* -- Sample board code to send message over COM1 port ---- */
  printf("Code laeuft!\n");
  printf("\r\nq/a: M1 +/-  w/s: M2 +/-  e/d: beide +/-  Leertaste: STOP\r\n");

  printf("Bereit. q/a: M1  w/s: M2  e/d: beide  Leertaste: STOP\r\n");
  printf("t: Tara  c: Kalibrieren (%d g anhaengen)  m: Ausgabe an/aus\r\n", (int)MASS);

  /* -- Sample board code to switch on led ---- */


  /* USER CODE END BSP */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* -- Sample board code for User push-button in interrupt mode ---- */
	  static uint32_t t_print = 0;
	  uint8_t c;

	  if (HAL_UART_Receive(&hcom_uart[COM1], &c, 1, 10) == HAL_OK)
	  {
		  uint8_t motor_key = 1;
		  switch (c)
		  	  {
				  case 'q': thr1 = limit(thr1 + STEPS); break;  // Motor 1 schneller
				  case 'a': thr1 = limit(thr1 - STEPS); break;  // Motor 1 langsamer
				  case 'w': thr2 = limit(thr2 + STEPS); break;  // Motor 2 schneller
				  case 's': thr2 = limit(thr2 - STEPS); break;  // Motor 2 langsamer
				  case 'e': thr1 = limit(thr1 + STEPS);         // beide schneller
						  	thr2 = limit(thr2 + STEPS); break;
				  case 'd': thr1 = limit(thr1 - STEPS);         // beide langsamer
						    thr2 = limit(thr2 - STEPS); break;
				  case ' ': thr1 = 0; thr2 = 0; break;             // NOT-AUS
				  case 't': motor_key = 0; do_tare(); break;
				  case 'c': motor_key = 0; do_calibrate(); break;
				  case 'm': motor_key = 0; stream = !stream; break;
				  default:  motor_key = 0; break;
		  	  }
			  if (motor_key)
			  {
				  throttle();
				  print_status();
			  }
	  }
	  else
	  {
		  __HAL_UART_CLEAR_OREFLAG(&hcom_uart[COM1]);
		  __HAL_UART_CLEAR_FEFLAG(&hcom_uart[COM1]);
	  }

	  if (hx711_ready())
	  {
		  hx_raw = hx711_read_raw();
		  float g = THRUST_SIGN * (float)(hx_raw - hx_offset) / hx_scale;
		  thrust_g = thrust_g + ALPHA * (g - thrust_g);

		  if (hx_calibrated && (thrust_g > THRUST_LIMIT || thrust_g < -THRUST_LIMIT))
		  {
			  thr1 = 0; thr2 = 0;
	          throttle();
	          printf("\r\n!!! UEBERLAST - NOT-AUS !!!\r\n");
		  }
	  }

	  // laufende Ausgabe zweimal pro Sekunde
	  if (stream && (HAL_GetTick() - t_print >= 500))
	  {
		  t_print = HAL_GetTick();
		  print_status();
	  }
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 169;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DT_Pin */
  GPIO_InitStruct.Pin = DT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SCK_Pin */
  GPIO_InitStruct.Pin = SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SCK_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief BSP Push Button callback
  * @param Button Specifies the pressed button
  * @retval None
  */
void BSP_PB_Callback(Button_TypeDef Button)
{
  if (Button == BUTTON_USER)
  {
    BspButtonState = BUTTON_PRESSED;
  }
}

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
