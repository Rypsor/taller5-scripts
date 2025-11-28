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
#include <stdio.h> // Necesario para sprintf
#include <string.h>
#include "ssd1306_fonts.h" // fuentes para texto
#include "ssd1306.h" // OLED
#include "ssd1306_tests.h" // OLED
#include <stdlib.h>
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// --- 1. VARIABLES DE HARDWARE (DMA) ---
// Buffer donde el ADC guarda los datos automáticamente
// [0] = Eje X (Movimiento Lateral), [1] = Eje Y (Caída)
uint32_t joystickData[2];

// --- 2. CONSTANTES Y TABLERO ---
#define FILAS 20  // Altura (20 bloques)
#define COLS  10  // Ancho (10 bloques)

// Matriz del mundo de juego: 0 = Vacío, 1 = Bloque Fijo
uint8_t tablero[FILAS][COLS] = {0};

// --- 3. ESTADO DEL JUGADOR (Pieza Activa) ---
int8_t piezaX = 3;       // Posición Horizontal (0-9)
int8_t piezaY = 0;       // Posición Vertical (0-19)
int8_t piezaActual = 1;  // ID de la pieza (0-6)
int8_t rotacion = 0;     // Estado de rotación actual (0-3)

// --- 4. DEFINICIÓN DE PIEZAS (CON ROTACIONES) ---
// Estructura: [7 Tipos] [4 Rotaciones] [4 Filas] [4 Columnas]
const uint8_t PIEZAS[7][4][4][4] = {
    // 0. Cuadrado (O)
    {
        {{0,0,0,0},{0,1,1,0},{0,1,1,0},{0,0,0,0}},
        {{0,0,0,0},{0,1,1,0},{0,1,1,0},{0,0,0,0}}, // No cambia al rotar
        {{0,0,0,0},{0,1,1,0},{0,1,1,0},{0,0,0,0}},
        {{0,0,0,0},{0,1,1,0},{0,1,1,0},{0,0,0,0}}
    },
    // 1. Palo (I)
    {
        {{0,1,0,0},{0,1,0,0},{0,1,0,0},{0,1,0,0}}, // Vertical
        {{0,0,0,0},{1,1,1,1},{0,0,0,0},{0,0,0,0}}, // Horizontal
        {{0,1,0,0},{0,1,0,0},{0,1,0,0},{0,1,0,0}},
        {{0,0,0,0},{1,1,1,1},{0,0,0,0},{0,0,0,0}}
    },
    // 2. T (T)
    {
        {{0,0,0,0},{1,1,1,0},{0,1,0,0},{0,0,0,0}}, // T arriba
        {{0,1,0,0},{1,1,0,0},{0,1,0,0},{0,0,0,0}}, // T derecha
        {{0,1,0,0},{1,1,1,0},{0,0,0,0},{0,0,0,0}}, // T abajo
        {{0,1,0,0},{0,1,1,0},{0,1,0,0},{0,0,0,0}}  // T izquierda
    },
    // 3. L (L)
    {
        {{0,1,0,0},{0,1,0,0},{0,1,1,0},{0,0,0,0}},
        {{0,0,0,0},{1,1,1,0},{1,0,0,0},{0,0,0,0}},
        {{1,1,0,0},{0,1,0,0},{0,1,0,0},{0,0,0,0}},
        {{0,0,1,0},{1,1,1,0},{0,0,0,0},{0,0,0,0}}
    },
    // 4. J (J)
    {
        {{0,0,1,0},{0,0,1,0},{0,1,1,0},{0,0,0,0}},
        {{1,0,0,0},{1,1,1,0},{0,0,0,0},{0,0,0,0}},
        {{0,1,1,0},{0,1,0,0},{0,1,0,0},{0,0,0,0}},
        {{0,0,0,0},{1,1,1,0},{0,0,1,0},{0,0,0,0}}
    },
    // 5. S (S)
    {
        {{0,0,0,0},{0,1,1,0},{1,1,0,0},{0,0,0,0}},
        {{1,0,0,0},{1,1,0,0},{0,1,0,0},{0,0,0,0}},
        {{0,0,0,0},{0,1,1,0},{1,1,0,0},{0,0,0,0}},
        {{1,0,0,0},{1,1,0,0},{0,1,0,0},{0,0,0,0}}
    },
    // 6. Z (Z)
    {
        {{0,0,0,0},{1,1,0,0},{0,1,1,0},{0,0,0,0}},
        {{0,0,1,0},{0,1,1,0},{0,1,0,0},{0,0,0,0}},
        {{0,0,0,0},{1,1,0,0},{0,1,1,0},{0,0,0,0}},
        {{0,0,1,0},{0,1,1,0},{0,1,0,0},{0,0,0,0}}
    }
};

// --- 5. TEMPORIZADORES (TIMING) ---
uint32_t ultimoTiempoCaida = 0;
uint32_t ultimoTiempoInput = 0;
uint32_t ultimoTiempoBoton = 0; // Para el anti-rebote del botón de rotación

// --- 6. VELOCIDAD ---
uint32_t velocidadGravedadActual = 500; // Iniciamos con velocidad media
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
uint8_t CheckCollision(int8_t pX, int8_t pY, int8_t pieza, int8_t rot); // para revisar colisiones
void CheckLines(void); // para revisar si se completan lineas
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  // 1. Inicializar pantalla
    ssd1306_Init();
    ssd1306_Fill(Black);
    ssd1306_UpdateScreen();

    // 2. Calibración breve (opcional, para estabilidad eléctrica)
    HAL_Delay(100);

    // 3. ¡ARRANCAR EL DMA!
    // Le decimos al ADC: "Llena este array (joystickData) con 2 valores infinitamente"
    HAL_ADC_Start_DMA(&hadc1, joystickData, 2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

      // Variables para el control de tiempo (Debounce y repetición)
      uint32_t ultimoTiempoBoton = 0;

      while (1)
      {
          uint32_t tiempoActual = HAL_GetTick();
          uint8_t cambioEnPantalla = 0;

          // 1. LEER DMA (Joystick)
          uint32_t valLado  = joystickData[0];
          uint32_t valCaida = joystickData[1];

          // --- NUEVO: LECTURA DEL BOTÓN DE ROTACIÓN (PB5) ---
          // Configuración: Botón conectado a GND. Pin con Pull-Up interno.
          // Al pulsar, el pin se va a Tierra (RESET / 0).
          if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_RESET) {

              // Anti-rebote (Debounce): Esperamos 200ms entre pulsaciones
              if (tiempoActual - ultimoTiempoBoton > 200) {

                  // 1. Calcular cuál SERÍA la siguiente rotación
                  int8_t proxRot = rotacion + 1;
                  if (proxRot > 3) proxRot = 0;

                  // 2. PREGUNTAR AL ÁRBITRO: ¿Cabe la pieza si la giro?
                  // Pasamos 'proxRot' a la función de colisión
                  if (!CheckCollision(piezaX, piezaY, piezaActual, proxRot)) {
                      rotacion = proxRot; // ¡Aprobado! Aplicamos el giro
                      cambioEnPantalla = 1;
                  }

                  ultimoTiempoBoton = tiempoActual;
              }
          }

          // 2. CONTROL LATERAL (Mover Pieza 0-9)
          if (tiempoActual - ultimoTiempoInput > 100) {
              int8_t nuevoX = piezaX;

              if (valLado < 1000) nuevoX--;
              else if (valLado > 3000) nuevoX++;

              // Importante: Validamos movimiento usando la 'rotacion' actual
              if (nuevoX != piezaX && !CheckCollision(nuevoX, piezaY, piezaActual, rotacion)) {
                  piezaX = nuevoX;
                  cambioEnPantalla = 1;
                  ultimoTiempoInput = tiempoActual;
              }
          }

          // 3. CONTROL CAÍDA (Turbo)
          // Ajustado a tu joystick (> 3700 es ABAJO)
          if (valCaida > 3700) velocidadGravedadActual = 50;
          else velocidadGravedadActual = 500;

          // 4. GRAVEDAD
          if (tiempoActual - ultimoTiempoCaida > velocidadGravedadActual) {

              // Validamos caída con la rotación actual
              if (!CheckCollision(piezaX, piezaY + 1, piezaActual, rotacion)) {
                  piezaY++;
              }
              else {
                  // --- FIJAR PIEZA ---
                  for (int r = 0; r < 4; r++) {
                      for (int c = 0; c < 4; c++) {
                          // Guardamos la forma exacta que tiene la pieza rotada ahora mismo
                          if (PIEZAS[piezaActual][rotacion][r][c]) {
                              int realY = piezaY + r;
                              int realX = piezaX + c;
                              if(realY >= 0 && realY < FILAS && realX >= 0 && realX < COLS) {
                                  tablero[realY][realX] = 1;
                              }
                          }
                      }
                  }

                  CheckLines();

                  // RESPAWN
                  piezaY = 0;
                  piezaX = 3;
                  rotacion = 0; // Reiniciamos rotación para la pieza nueva
                  piezaActual++;
                  if (piezaActual > 6) piezaActual = 0;

                  // GAME OVER CHECK
                  if (CheckCollision(piezaX, piezaY, piezaActual, rotacion)) {
                      for(int i=0; i<FILAS; i++)
                          for(int j=0; j<COLS; j++)
                              tablero[i][j] = 0;
                  }
              }
              cambioEnPantalla = 1;
              ultimoTiempoCaida = tiempoActual;
          }

          // 5. RENDERIZADO
          if (cambioEnPantalla) {
              ssd1306_Fill(Black);

              // A. Dibujar Tablero
              for (int r = 0; r < FILAS; r++) {
                  for (int c = 0; c < COLS; c++) {
                      if (tablero[r][c]) {
                          ssd1306_FillRectangle(r*6, c*6, r*6+4, c*6+4, White);
                      }
                  }
              }

              // B. Dibujar Pieza Activa (CON ROTACIÓN)
              for (int r = 0; r < 4; r++) {
                   for (int c = 0; c < 4; c++) {
                       // Leemos la matriz usando [rotacion]
                       if (PIEZAS[piezaActual][rotacion][r][c]) {
                           int drawX = (piezaY + r) * 6;
                           int drawY = (piezaX + c) * 6;
                           ssd1306_FillRectangle(drawX, drawY, drawX+4, drawY+4, White);
                       }
                   }
              }

              // Línea de suelo
              ssd1306_Line(FILAS*6, 0, FILAS*6, COLS*6, White);

              ssd1306_UpdateScreen();
          }
      }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOTON_CONTROL1_Pin */
  GPIO_InitStruct.Pin = BOTON_CONTROL1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BOTON_CONTROL1_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
uint8_t CheckCollision(int8_t pX, int8_t pY, int8_t pieza, int8_t rot) {
    for (int r = 0; r < 4; r++) {
        for (int c = 0; c < 4; c++) {

            // Usamos la matriz de 4 dimensiones: [pieza][rotacion][fila][columna]
            if (PIEZAS[pieza][rot][r][c]) {

                int realX = pX + c;
                int realY = pY + r;

                // 1. Choca con Paredes (Izquierda/Derecha)
                if (realX < 0 || realX >= COLS) return 1;

                // 2. Choca con Suelo
                if (realY >= FILAS) return 1;

                // 3. Choca con bloques fijos del tablero
                if (realY >= 0 && tablero[realY][realX]) return 1;
            }
        }
    }
    return 0; // No hay choque
}

void CheckLines(void) {
    // Recorremos desde abajo (fila 19) hacia arriba (fila 0)
    for (int y = FILAS - 1; y >= 0; y--) {

        uint8_t filaLlena = 1;

        // 1. Verificar si la fila 'y' está llena
        for (int x = 0; x < COLS; x++) {
            if (tablero[y][x] == 0) {
                filaLlena = 0;
                break;
            }
        }

        // 2. Si está llena, BORRAR y BAJAR todo
        if (filaLlena) {
            // Desplazar todas las filas superiores hacia abajo
            for (int k = y; k > 0; k--) {
                for (int x = 0; x < COLS; x++) {
                    tablero[k][x] = tablero[k-1][x];
                }
            }
            // Limpiar la fila superior del todo (que ahora está vacía)
            for (int x = 0; x < COLS; x++) {
                tablero[0][x] = 0;
            }

            // Truco importante: Al bajar las filas, la fila actual 'y' ahora contiene
            // lo que había en 'y-1'. Debemos volver a revisarla por si TAMBIÉN estaba llena.
            y++;
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
