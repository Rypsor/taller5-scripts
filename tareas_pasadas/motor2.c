/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum
{
	BOTON_NO_ASIGNADO,
	BOTON_DEXTROGIRO,
	BOTON_LEVOGIRO,
	BOTON_SUBIR_VELOCIDAD,
	BOTON_BAJAR_VELOCIDAD,
	BOTON_FRENAR,
	BOTON_LEER
} BotonesMotor;

typedef enum
{
	MOTOR_APAGADO,
	MOTOR_DEXTROGIRO,
	MOTOR_LEVOGIRO,
} EstadoMotor;



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
EstadoMotor estadoActual = MOTOR_APAGADO;
uint8_t velocidad = 0;
// Definir las banderas de interrupción
// Actualizar el estado del motor según el botón presionado
volatile uint32_t g_capture_val_1 = 0;
volatile uint32_t g_capture_val_2 = 0;
volatile uint32_t g_capture_diff = 0;
volatile uint32_t g_frecuencia_medida = 0;
volatile uint8_t  g_is_first_capture = 1; // Bandera para la primera captura
volatile uint32_t g_last_capture_time = 0; // Para detectar si el motor se detuvo

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Función para actualizar el ciclo de trabajo del PWM según la velocidad

void actualizarMotor(EstadoMotor estado, int velocidad)
{
	// 1. Calcular el Duty Cycle (0-999)
	uint32_t dutyCycle = (velocidad * 999) / 100;

	// 2. Aplicar lógica de Freno/Dirección al TIM5 (htim5)
	switch (estado)
	{
		case MOTOR_DEXTROGIRO:
			// Dextro (IN1=PWM, IN2=GND) -> STM32 (CH1=dutyCycle, CH2=0)
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, dutyCycle);
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);
			break;

		case MOTOR_LEVOGIRO:
			// Levogiro (IN1=GND, IN2=PWM) -> STM32 (CH1=0, CH2=dutyCycle)
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, dutyCycle);
			break;

		case MOTOR_APAGADO:
		default:
			// Freno (IN1=GND, IN2=GND) -> STM32 (CH1=0, CH2=0)
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);
			break;
	}
}

void actualizarEstadoMotor(int botonPresionado)
{
    switch (estadoActual)
    {
    case MOTOR_APAGADO:
        if (botonPresionado == BOTON_DEXTROGIRO)
        {
            estadoActual = MOTOR_DEXTROGIRO;
            velocidad = 20; // Velocidad inicial al avanzar
            //actualizarDirección(estadoActual);
        }
        else if (botonPresionado == BOTON_LEVOGIRO)
        {
            estadoActual = MOTOR_LEVOGIRO;
            velocidad = 20; // Velocidad inicial al retroceder
           // actualizarDirección(estadoActual);
        }
        break;
    case MOTOR_DEXTROGIRO:
        if (botonPresionado == BOTON_FRENAR)
        {
            estadoActual = MOTOR_APAGADO;
            velocidad = 0;
        }
        else if (botonPresionado == BOTON_SUBIR_VELOCIDAD)
        {
            velocidad += 20; // Incrementar velocidad
            if (velocidad > 100)
                velocidad = 100; // Limitar a velocidad máxima
        }
        else if (botonPresionado == BOTON_BAJAR_VELOCIDAD)
        {
            velocidad -= 20; // reducir velocidad
            if (velocidad <= 0)
            {
                estadoActual = MOTOR_APAGADO;
                velocidad = 0;
            }
        }
        break;
    case MOTOR_LEVOGIRO:
        if (botonPresionado == BOTON_FRENAR)
        {
            estadoActual = MOTOR_APAGADO;
            velocidad = 0;
        }
        else if (botonPresionado == BOTON_SUBIR_VELOCIDAD)
        {
            velocidad += 10; // Incrementar velocidad
            if (velocidad > 100)
                velocidad = 100; // Limitar a velocidad máxima
        }
        else if (botonPresionado == BOTON_BAJAR_VELOCIDAD)
        {
            velocidad -= 20; // reducir velocidad
            if (velocidad <= 0)
            {
                estadoActual = MOTOR_APAGADO;
                velocidad = 0;
            }
        }
        break;
    default:
        break;
    }
}

/* FUNCION LEER (TECLA R)
Funcion para leer la frecuiuencia actual del motor a traves del imput capture del timer
y mandarlo a travez de la comunicación Serial al computador
*/
uint32_t leerFrecuenciaMotor(void)
{
    // 1. Espera (con bloqueo) hasta que se capture un nuevo flanco
    while (__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_CC1) == RESET)
    {
        // Espera a que la bandera de "Input Capture" se levante
    }

    // 2. Lee el valor capturado
    uint32_t captura = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1);
    
    // 3. Limpia la bandera para la próxima medición
    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_CC1);

    if (captura == 0)
    {
        return 0; // Evitar división por cero
    }

    // 4. Calcula la frecuencia del reloj del timer
    uint32_t timer_clock_freq = HAL_RCC_GetPCLK1Freq();
    if ( (RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1)
    {
        timer_clock_freq *= 2; // El reloj del Timer se duplica si APB1 Prescaler > 1
    }
    
    // 5. Divide por el prescaler que configuraste en CubeMX (n+1)
    // El '16-1' que pusiste en CubeMX significa un prescaler de 15.
    uint32_t timer_tick_freq = timer_clock_freq / (htim3.Instance->PSC + 1);
    
    // 6. Calcula la frecuencia final
    uint32_t frecuencia = timer_tick_freq / captura; 
    
    return frecuencia;
}

// Función para enviar la frecuencia del motor - CORREGIDA
void enviarFrecuenciaMotor(void)
{
    uint32_t frecuencia = leerFrecuenciaMotor();
    char buffer[50];
    
    // %lu es el formato para unsigned long (uint32_t)
    int longitud = sprintf(buffer, "Frecuencia del motor: %lu Hz\r\n", frecuencia);

    // ¡¡CORREGIDO!! Usa la función HAL para transmitir
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, longitud, HAL_MAX_DELAY);
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
  MX_USART2_UART_Init();
  MX_TIM5_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
  HAL_TIM_IC_Start(&htim3, TIM_CHANNEL_1);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		{
			// Bucle principal vacío, todo se maneja en las interrupciones
			if (Bandera_USART_RX)
			{
				Bandera_USART_RX = 0;

				switch (letraRecibida)
				{
				case 'a':
					botonPresionado = BOTON_LEVOGIRO;
					break;
				case 'd':
					botonPresionado = BOTON_DEXTROGIRO;
          break;
        case 'w':
          botonPresionado = BOTON_SUBIR_VELOCIDAD;
          break;
        case 's':
          botonPresionado = BOTON_BAJAR_VELOCIDAD;
					break;
				case 'f':
					botonPresionado = BOTON_FRENAR;
					break;
				case 'r':
                    botonPresionado = BOTON_LEER;
					break;

				default:
					break;
				}
				if (botonPresionado != BOTON_NO_ASIGNADO)
				{
					// Bandera_USART_RX = 1; // Subir la bandera de recepción
				}


				if (botonPresionado == BOTON_LEER)
				{
					enviarFrecuenciaMotor(); // Enviar la frecuencia actual del motor
				}
				else if (botonPresionado != BOTON_NO_ASIGNADO) // <-- (Mejora: usa 'else if')
                {
                    // 1. Actualiza las variables
                    actualizarEstadoMotor(botonPresionado);

                    // 2. Aplica las variables al hardware
                    actualizarMotor(estadoActual, velocidad); // <-- ¡CORREGIDO!
                }
                botonPresionado = BOTON_NO_ASIGNADO;
			}
		}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
}
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 16 -1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
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
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 16-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 999;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  HAL_UART_Receive_IT(&huart2, &g_dato_rx, 1);
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PIN_BLINKY_GPIO_Port, PIN_BLINKY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PIN_BLINKY_Pin */
  GPIO_InitStruct.Pin = PIN_BLINKY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PIN_BLINKY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* Poner esto en USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // Comprobar si la interrupción vino de la USART2
    if (huart->Instance == USART2)
    {
        letraRecibida = g_dato_rx;

        Bandera_USART_RX = 1;

        HAL_UART_Receive_IT(&huart2, &g_dato_rx, 1);
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
