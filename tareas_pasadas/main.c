/* -----------------------------------------------------------------------------
 * motor.c
 * Por: Juan Jose Zapata Moreno y Samuel Botero Rivera
 * Descripción general:
 * Este programa controla un motor DC mediante una placa STM32, usando PWM para velocidad y GPIO para dirección/freno. 
 * El usuario puede interactuar con el motor a través de comandos enviados por comunicación serial (UART), 
 * donde cada letra corresponde a una función específica:
 *   - 'a': Cambia el sentido del motor a levogiro (izquierda) cuando está apagado
 *   - 'd': Cambia el sentido del motor a dextrogiro (derecha) cuando está apagado
 *   - 'w': Aumenta la velocidad del motor
 *   - 's': Disminuye la velocidad del motor
 *   - 'f': Frena y apaga el motor
 *   - 'r': Lee y envía la frecuencia actual del motor por serial
 *   - 'b': Cambia la frecuencia del LED Blinky
 *
 * El código utiliza interrupciones para procesar los comandos seriales y para medir la frecuencia del motor mediante Input Capture. 
 * El control del motor se realiza actualizando el PWM y la dirección según el estado y el botón recibido.
 *
 * Principales funciones:
 *   - actualizarMotor: Aplica el estado y velocidad al hardware (PWM y dirección)
 *   - actualizarEstadoMotor: Cambia el estado y velocidad según el botón recibido
 *   - leerFrecuenciaMotor: Lee la frecuencia del motor usando Input Capture
 *   - enviarFrecuenciaMotor: Envía la frecuencia por UART
 *   - HAL_UART_RxCpltCallback: Interrupción de recepción serial, procesa comandos
 *   - HAL_TIM_IC_CaptureCallback: Interrupción de captura de frecuencia
 *   - HAL_TIM_PeriodElapsedCallback: Interrupción para el Blinky
 * -----------------------------------------------------------------------------
 */
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


// Enumeración de los botones disponibles para controlar el motor
typedef enum
{
	BOTON_NO_ASIGNADO,      // Sin acción
	BOTON_DEXTROGIRO,       // Sentido horario (derecha)
	BOTON_LEVOGIRO,         // Sentido antihorario (izquierda)
	BOTON_SUBIR_VELOCIDAD,  // Aumentar velocidad
	BOTON_BAJAR_VELOCIDAD,  // Disminuir velocidad
	BOTON_FRENAR,           // Freno y apagado
	BOTON_LEER,             // Leer frecuencia del motor
	BOTON_BLINKY            // Cambiar frecuencia del LED Blinky
} BotonesMotor;


// Estados posibles del motor
typedef enum
{
	MOTOR_APAGADO,      // Motor apagado/frenado
	MOTOR_DEXTROGIRO,   // Motor gira a la derecha
	MOTOR_LEVOGIRO,     // Motor gira a la izquierda
} EstadoMotor;



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

EstadoMotor estadoActual = MOTOR_APAGADO;   // Estado actual del motor
uint8_t velocidad = 0;                      // Velocidad actual (0-100)
uint8_t Bandera_USART_RX = 0;               // Bandera de recepción por UART
uint8_t botonPresionado = 0;                // Botón recibido por UART
uint8_t letraRecibida = 0;                  // Letra recibida por UART
uint8_t g_dato_rx;                          // Dato recibido por UART (buffer)

uint16_t Frecuencia_Blinky = 500;           // Frecuencia del LED Blinky en ms

// Variables para la medición de frecuencia del motor (Input Capture)
volatile uint32_t g_capture_val_1 = 0;      // Primer valor capturado
volatile uint32_t g_capture_val_2 = 0;      // Segundo valor capturado
volatile uint32_t g_capture_diff = 0;       // Diferencia entre capturas
volatile uint32_t g_frecuencia_medida = 0;  // Última frecuencia medida
volatile uint8_t  g_is_first_capture = 1;   // Bandera para saber si es la primera captura
volatile uint32_t g_last_capture_time = 0;  // Último tiempo de captura

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void actualizarMotor(EstadoMotor estado, int velocidad);
void actualizarEstadoMotor(int botonPresionado);
void enviarFrecuenciaMotor(void);
uint32_t leerFsecuenciaMotor(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


// Actualiza el ciclo de trabajo del PWM y la dirección del motor según el estado y velocidad
void actualizarMotor(EstadoMotor estado, int velocidad)
{
	// Calcular el Duty Cycle (0-999) proporcional a la velocidad (0-100)
	uint32_t dutyCycle = (velocidad * 999) / 100;

	// Aplicar la lógica de dirección y freno usando los canales del timer
	switch (estado)
	{
	case MOTOR_DEXTROGIRO:
		// Sentido horario: CH1 = PWM, CH2 = 0
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, dutyCycle);
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);
		break;
	case MOTOR_LEVOGIRO:
		// Sentido antihorario: CH1 = 0, CH2 = PWM
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, dutyCycle);
		break;
	case MOTOR_APAGADO:
	default:
		// Freno: ambos canales en 0
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);
		break;
	}
}


// Actualiza el estado y velocidad del motor según el botón recibido
void actualizarEstadoMotor(int botonPresionado)
{
	EstadoMotor estadoAnterior = estadoActual;
	uint8_t velocidadAnterior = velocidad;
	switch (estadoActual)
	{
	case MOTOR_APAGADO:
		if (botonPresionado == BOTON_DEXTROGIRO)
		{
			estadoActual = MOTOR_DEXTROGIRO;
			velocidad = 20;
		}
		else if (botonPresionado == BOTON_LEVOGIRO)
		{
			estadoActual = MOTOR_LEVOGIRO;
			velocidad = 20;
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
			velocidad += 20;
			if (velocidad > 100)
				velocidad = 100;
		}
		else if (botonPresionado == BOTON_BAJAR_VELOCIDAD)
		{
			velocidad -= 20;
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
			velocidad += 10;
			if (velocidad > 100)
				velocidad = 100;
		}
		else if (botonPresionado == BOTON_BAJAR_VELOCIDAD)
		{
			velocidad -= 20;
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

    // Imprimir por UART el estado y velocidad actual si hubo cambio
    if (estadoActual != estadoAnterior || velocidad != velocidadAnterior) {
        const char* estadoStr;

        // Asignar mensaje según estado
        if (estadoActual == MOTOR_APAGADO) {
            estadoStr = "APAGADO";
        }
        else if (estadoActual == MOTOR_DEXTROGIRO) {
            estadoStr = "DEXTROGIRO";
        }
        else if (estadoActual == MOTOR_LEVOGIRO) {
            estadoStr = "LEVOGIRO";
        }
        else {
            estadoStr = "DESCONOCIDO";
        }

        char msg[50];
        int longitud = sprintf(msg, "Estado: %s, Velocidad: %u%%\r\n", estadoStr, velocidad);
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, longitud, 1000);
    }
}


// Lee la frecuencia actual del motor usando Input Capture y la convierte a RPM
// Si no hay pulsos recientes, devuelve 0 RPM
uint32_t leerFrecuenciaMotor(void)
{
	// Si han pasado más de 500ms desde la última captura, se asume 0 RPM
	if ( (HAL_GetTick() - g_last_capture_time) > 500 )
	{
		g_frecuencia_medida = 0;
		g_is_first_capture = 1; // Reinicia la lógica de captura
	}
	// Suponiendo 1 pulso por vuelta, RPM = Hz * 60
	return g_frecuencia_medida * 60;
}



// Envía la velocidad actual del motor en RPM por UART al computador
void enviarFrecuenciaMotor(void)
{
	uint32_t rpm = leerFrecuenciaMotor();
	char buffer[50];
	int longitud = sprintf(buffer, "Velocidad del motor: %lu RPM\r\n", rpm);
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
	MX_TIM2_Init();
	/* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1); // con interrupción para

	// Enviar explicación de las teclas por UART al iniciar
	const char* info =
		"\r\nControl de motor DC por UART:\r\n"
		"a: Sentido levogiro (izquierda)\r\n"
		"d: Sentido dextrogiro (derecha)\r\n"
		"w: Aumentar velocidad\r\n"
		"s: Disminuir velocidad\r\n"
		"f: Freno y apagado\r\n"
		"r: Leer velocidad en RPM\r\n"
		"b: Cambiar frecuencia del LED Blinky\r\n"
		"\r\n";
	HAL_UART_Transmit(&huart2, (uint8_t*)info, strlen(info), HAL_MAX_DELAY);

	// Anunciar estado y velocidad inicial
	char estadoMsg[60];
	const char* estadoStr = "APAGADO";
	sprintf(estadoMsg, "Estado inicial: %s, Velocidad: %u%%\r\n", estadoStr, velocidad);
	HAL_UART_Transmit(&huart2, (uint8_t*)estadoMsg, strlen(estadoMsg), HAL_MAX_DELAY);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* Bucle principal vacío, todo se maneja en las interrupciones */
		if (Bandera_USART_RX)
		{
			Bandera_USART_RX = 0;

			// Procesa la letra recibida por UART y asigna el botón correspondiente
			switch (letraRecibida)
			{
			case 'a':
				botonPresionado = BOTON_LEVOGIRO;      // Cambia sentido a izquierda
				break;
			case 'd':
				botonPresionado = BOTON_DEXTROGIRO;     // Cambia sentido a derecha
				break;
			case 'w':
				botonPresionado = BOTON_SUBIR_VELOCIDAD;// Aumenta velocidad
				break;
			case 's':
				botonPresionado = BOTON_BAJAR_VELOCIDAD;// Disminuye velocidad
				break;
			case 'f':
				botonPresionado = BOTON_FRENAR;         // Frena y apaga
				break;
			case 'r':
				botonPresionado = BOTON_LEER;           // Lee frecuencia
				break;
			case 'b':
				botonPresionado = BOTON_BLINKY;         // Cambia frecuencia del Blinky
				break;
			default:
				break;
			}

			// Procesa la acción según el botón recibido
			if (botonPresionado == BOTON_LEER)
			{
				enviarFrecuenciaMotor(); // Enviar la frecuencia actual del motor
			}
			else if (botonPresionado == BOTON_BLINKY)
			{
				// Cambia la frecuencia del LED Blinky
				__HAL_TIM_SET_AUTORELOAD(&htim2, Frecuencia_Blinky);
				Frecuencia_Blinky *= 0.5;
				if (Frecuencia_Blinky < 50)
				{
					Frecuencia_Blinky = 500; // Reiniciar a 500 ms
				}
			}
			else if (botonPresionado != BOTON_NO_ASIGNADO)
			{
				// Actualiza el estado y velocidad del motor
				actualizarEstadoMotor(botonPresionado);
				// Aplica los cambios al hardware
				actualizarMotor(estadoActual, velocidad);
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
	htim2.Init.Prescaler = 16000-1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 500-1;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
	sConfigIC.ICFilter = 15;
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

// Interrupción de recepción UART: guarda la letra recibida y sube la bandera
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART2)
	{
		letraRecibida = g_dato_rx;      // Guarda la letra recibida
		Bandera_USART_RX = 1;           // Sube la bandera para procesar en el main
		HAL_UART_Receive_IT(&huart2, &g_dato_rx, 1); // Rehabilita la recepción
	}
}


// Interrupción de Input Capture: mide la frecuencia del motor
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM3)
	{
		g_last_capture_time = HAL_GetTick(); // Actualiza el tiempo de la última captura

		if(g_is_first_capture)
		{
			// Primera captura: guarda el valor
			g_capture_val_1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			g_is_first_capture = 0;
		}
		else
		{
			// Segunda captura: calcula la diferencia de ticks
			g_capture_val_2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			if (g_capture_val_2 > g_capture_val_1)
			{
				g_capture_diff = g_capture_val_2 - g_capture_val_1;
			}
			else if (g_capture_val_2 < g_capture_val_1)
			{
				// Timer desbordado
				g_capture_diff = (65535 - g_capture_val_1) + g_capture_val_2;
			}
			else
			{
				g_capture_diff = 0;
			}

			// Calcula la frecuencia si hay diferencia
			if(g_capture_diff > 0)
			{
				uint32_t timer_clock_freq = HAL_RCC_GetPCLK1Freq();
				if ( (RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1)
				{
					timer_clock_freq *= 2;
				}
				uint32_t timer_tick_freq = timer_clock_freq / (htim3.Instance->PSC + 1);
				g_frecuencia_medida = timer_tick_freq / g_capture_diff;
			}
			// Prepara para la próxima medición
			g_capture_val_1 = g_capture_val_2;
		}
	}
}


// Interrupción de timer: cambia el estado del LED Blinky
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM2)
	{
		HAL_GPIO_TogglePin(PIN_BLINKY_GPIO_Port, PIN_BLINKY_Pin); // Cambia el estado del LED
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
