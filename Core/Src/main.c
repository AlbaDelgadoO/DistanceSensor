/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define INTERRUPTOR_Pin GPIO_PIN_4
#define INTERRUPTOR_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_3
#define LED_GPIO_Port GPIOB

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MOTOR_PIN GPIO_PIN_6 // Pin conectado al motor vibrador
#define MOTOR_PORT GPIOA     // Puerto GPIO
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim15;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static uint32_t distance_cm = 0;
uint8_t contador = 0;
volatile uint8_t flag = 0;
volatile uint32_t delay_ms = 0, rebotes = 0;
uint8_t cadena_tiempos_pulsacion[5];

uint8_t posicion_cadena = 0;
uint8_t lleno = 0;
uint8_t tiempo_pulsacion = 0;
uint8_t num_total_tiempo = 0;
uint8_t posicion_real = 0;
uint8_t valor_anterior = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM15_Init(void);
/* USER CODE BEGIN PFP */
void ReiniciarSistema(uint8_t *cadena_tiempos_pulsacion, uint8_t *num_total_tiempo, uint8_t *posicion_cadena, uint8_t *posicion_real, uint8_t *lleno, uint8_t *contador);
void NotificarUrgencia(void);
void NotificarNoUrgencia(void);
static void Ultrasonic_Trigger(void);
static uint32_t Ultrasonic_ReadEcho(void);
//static uint32_t LeerDistancia(void);
static void ControlarVibrador(uint32_t distancia);
static void ProcesarBoton(void);
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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM15_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  static uint32_t last_update = 0;
  while (1)
  {
	  uint32_t current_tick = HAL_GetTick(); // Tiempo actual en ms desde el arranque
	  // Medir distancia y controlar vibrador cada 500ms (ejemplo)
	  if ((current_tick - last_update) >= 500) {
		  last_update = current_tick;
		  Ultrasonic_Trigger();
		  distance_cm = Ultrasonic_ReadEcho();
		  // Aquí podés también imprimir la distancia por UART si quieres, por ejemplo:
		  // char buffer[50];
		  // snprintf(buffer, sizeof(buffer), "Distancia: %ld cm\r\n", distance_cm);
		  // HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
		  ControlarVibrador(distance_cm);
	  }
	  // Lógica del boton
      ProcesarBoton();
      // Pequeño retardo para no saturar el bucle
      HAL_Delay(5);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 0;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 65535;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */
  HAL_TIM_MspPostInit(&htim15);

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
  huart2.Init.BaudRate = 38400;
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : INTERRUPTOR_Pin */
  GPIO_InitStruct.Pin = INTERRUPTOR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INTERRUPTOR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// Función para enviar el pulso TRIG
void Ultrasonic_Trigger(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Configurar PA7 como salida para TRIG
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Generar un pulso de 10 µs en TRIG
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
    HAL_Delay(0); // Breve retardo para asegurar el pulso de 10 µs
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
}
uint32_t Ultrasonic_ReadEcho(void)
{
    uint32_t start_time, stop_time;

    // Configurar PA7 como entrada para ECHO
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Esperar a que ECHO se eleve a HIGH
    while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == GPIO_PIN_RESET);

    // Capturar el tiempo de inicio con el temporizador
    start_time = __HAL_TIM_GET_COUNTER(&htim2);

    // Esperar a que ECHO baje a LOW
    while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == GPIO_PIN_SET);

    // Capturar el tiempo de finalización con el temporizador
    stop_time = __HAL_TIM_GET_COUNTER(&htim2);

    // Calcular el tiempo transcurrido
	uint32_t time_in_microseconds = stop_time - start_time;

	// Si el valor es demasiado grande, posiblemente hay un overflow
	if (time_in_microseconds > 0xFFFFFF)
	{
		time_in_microseconds = 0;
	}

	// Convertir el tiempo en distancia (cm), usando la fórmula
	// Velocidad del sonido en cm/µs: 0.0343 cm/µs
	uint32_t distance_in_cm = ((time_in_microseconds * 0.0343) / 2)*10;

	return distance_in_cm;
}

/*static uint32_t LeerDistancia(void)
{
    // 1. Enviar el pulso TRIG
    Ultrasonic_Trigger();
    // 2. Leer el tiempo de respuesta del pulso ECHO
    uint32_t dist = Ultrasonic_ReadEcho();

   // 3. Imprimir la distancia a través del UART
    char buffer[50];
    snprintf(buffer, sizeof(buffer), "Distancia: %ld cm\r\n", dist);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

    return dist;
}*/

static void ControlarVibrador(uint32_t distancia)
{
    uint32_t vibration_on_time = 0;
    uint32_t vibration_off_time = 0;

    if (distancia <= 200 && distancia > 180) {
        vibration_on_time = 30;
        vibration_off_time = 150;
    } else if (distancia <= 180 && distancia > 165) {
        vibration_on_time = 30;
        vibration_off_time = 120;
    } else if (distancia <= 165 && distancia > 140) {
        vibration_on_time = 35;
        vibration_off_time = 100;
    } else if (distancia <= 140 && distancia > 125) {
        vibration_on_time = 40;
        vibration_off_time = 80;
    } else if (distancia <= 125 && distancia > 110) {
        vibration_on_time = 45;
        vibration_off_time = 60;
    } else if (distancia <= 110 && distancia > 95) {
        vibration_on_time = 50;
        vibration_off_time = 50;
    } else if (distancia <= 95 && distancia > 80) {
        vibration_on_time = 60;
        vibration_off_time = 40;
    } else if (distancia <= 80 && distancia > 65) {
        vibration_on_time = 70;
        vibration_off_time = 30;
    } else if (distancia <= 65 && distancia > 40) {
        vibration_on_time = 80;
        vibration_off_time = 20;
    } else if (distancia <= 40 && distancia > 25) {
        vibration_on_time = 90;
        vibration_off_time = 10;
    } else if (distancia <= 25 && distancia > 10) {
        vibration_on_time = 100;
        vibration_off_time = 5;
    } else if (distancia <= 10) {
        // Vibración constante
        HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_PIN, GPIO_PIN_SET);
        return;
    } else {
        // Apagar el motor cuando está a más de 200 cm
        HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_PIN, GPIO_PIN_RESET);
        return;
    }

    // Generar la vibración si la distancia está en el rango 10-200
    HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_PIN, GPIO_PIN_SET);
    HAL_Delay(vibration_on_time);
    HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_PIN, GPIO_PIN_RESET);
    HAL_Delay(vibration_off_time);
}

static void ProcesarBoton(void)
{
    if (!flag) return;

    if (lleno) {
        tiempo_pulsacion = delay_ms - valor_anterior;
    } else {
        lleno = 1;
        tiempo_pulsacion = delay_ms;
    }

    if (posicion_cadena > 4) {
        num_total_tiempo -= cadena_tiempos_pulsacion[posicion_real];
    }
    num_total_tiempo += tiempo_pulsacion;
    cadena_tiempos_pulsacion[posicion_real] = tiempo_pulsacion;

    if (num_total_tiempo < 10000 && posicion_cadena >= 4) {
        NotificarUrgencia();
        ReiniciarSistema(cadena_tiempos_pulsacion, &num_total_tiempo, &posicion_cadena, &posicion_real, &lleno, &contador);
    } else if (tiempo_pulsacion < 3000) {
        contador++;
        if (contador >= 2) {
            NotificarNoUrgencia();
            contador = 0;
        }
    } else {
        contador = 1;
    }

    // Imprimir el número de pulsaciones
    char mensaje_pulsaciones[50];
    sprintf(mensaje_pulsaciones, "Numero de pulsaciones: %d\n\r", contador);
    HAL_UART_Transmit(&huart2, (uint8_t *)mensaje_pulsaciones, strlen(mensaje_pulsaciones), 1000);

    if (posicion_cadena >= 4) {
        if (posicion_real == 4) {
            posicion_real = 0;
        } else {
            posicion_real++;
        }
    } else {
        valor_anterior = delay_ms;
        posicion_real++;
        posicion_cadena++;
    }
    flag = 0;
}

void ReiniciarSistema(uint8_t *cadena_tiempos_pulsacion, uint8_t *num_total_tiempo, uint8_t *posicion_cadena, uint8_t *posicion_real, uint8_t *lleno, uint8_t *contador) {
    memset(cadena_tiempos_pulsacion, 0, sizeof(uint8_t) * 5);
    *num_total_tiempo = 0;
    *posicion_cadena = 0;
    *posicion_real = -1;
    *lleno = 0;
    *contador = 0;

    char cadena[50];
    sprintf(cadena, "El sistema se ha reiniciado.\n\r");
    HAL_UART_Transmit(&huart2, (uint8_t *)cadena, strlen(cadena), 1000);
}

void NotificarUrgencia(void) {
    char cadena[50];
    sprintf(cadena, "¡¡AYUDA URGENTE, AYUDA URGENTE!!\n\r");
    HAL_UART_Transmit(&huart2, (uint8_t *)cadena, strlen(cadena), 1000);
    for (int i = 0; i < 10; i++) {
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        HAL_Delay(100);
    }

    // Asegurar que el LED quede apagado al final
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}

void NotificarNoUrgencia(void) {
    char cadena[50];
    sprintf(cadena, "Necesito ayuda pero no es una urgencia.\n\r");
    HAL_UART_Transmit(&huart2, (uint8_t *)cadena, strlen(cadena), 1000);
    // HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    // HAL_Delay(3000);
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
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
