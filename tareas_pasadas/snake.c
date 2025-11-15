/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h> // <--- Necesario para snprintf

/* --- DEFINICIONES Y ESTRUCTURAS DEL JUEGO --- */
#define ANCHO_TABLERO 16
#define ALTO_TABLERO 16
#define LARGO_SNAKE 5

typedef struct { int x; int y; } Point;
typedef enum { ARRIBA, ABAJO, IZQUIERDA, DERECHA } Direction;

typedef struct {
    Point body[LARGO_SNAKE];
    Direction direccion;
} Snake;

/* Handles y prototipos ------------------------------------------------------*/
UART_HandleTypeDef huart2;
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void inicializar_juego(void);
void actualizar_juego(void);
void Error_Handler(void);

/* --- VARIABLES GLOBALES --- */
uint8_t last_button_state = GPIO_PIN_SET;
uint32_t first_press_time = 0;
int single_press_pending = 0;
#define DEBOUNCE_TIMEOUT 300

Snake miSnake;
int game_over = 0;
char tx_buffer[128]; // <--- Buffer para construir el mensaje

/* --- FUNCIONES DEL JUEGO --- */
void inicializar_juego(void) {
    miSnake.body[0] = (Point){4, 2}; miSnake.body[1] = (Point){3, 2};
    miSnake.body[2] = (Point){2, 2}; miSnake.body[3] = (Point){1, 2};
    miSnake.body[4] = (Point){0, 2};
    miSnake.direccion = DERECHA;
    game_over = 0;
}

void actualizar_juego(void) {
    for (int i = LARGO_SNAKE - 1; i > 0; i--) {
        miSnake.body[i] = miSnake.body[i-1];
    }
    switch(miSnake.direccion) {
        case ARRIBA:    miSnake.body[0].y--; break;
        case ABAJO:     miSnake.body[0].y++; break;
        case IZQUIERDA: miSnake.body[0].x--; break;
        case DERECHA:   miSnake.body[0].x++; break;
    }
    if (miSnake.body[0].x < 0 || miSnake.body[0].x >= ANCHO_TABLERO ||
        miSnake.body[0].y < 0 || miSnake.body[0].y >= ALTO_TABLERO) {
        game_over = 1;
        return;
    }
    for (int i = 1; i < LARGO_SNAKE; i++) {
        if (miSnake.body[0].x == miSnake.body[i].x && miSnake.body[0].y == miSnake.body[i].y) {
            game_over = 1;
            return;
        }
    }
}

/* --- FUNCIÓN PRINCIPAL --- */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART2_UART_Init();

  inicializar_juego();

  while (1)
  {
    // PASO 1: PROCESAR ENTRADA
    uint8_t current_button_state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
    if (current_button_state == GPIO_PIN_RESET && last_button_state == GPIO_PIN_SET) {
      uint32_t current_tick = HAL_GetTick();
      if (single_press_pending && (current_tick - first_press_time < DEBOUNCE_TIMEOUT)) {
        single_press_pending = 0;
        if (miSnake.direccion == DERECHA) miSnake.direccion = ARRIBA; else if (miSnake.direccion == ARRIBA) miSnake.direccion = IZQUIERDA;
        else if (miSnake.direccion == IZQUIERDA) miSnake.direccion = ABAJO; else if (miSnake.direccion == ABAJO) miSnake.direccion = DERECHA;
      } else {
        single_press_pending = 1; first_press_time = current_tick;
      }
    }
    last_button_state = current_button_state;
    if (single_press_pending && (HAL_GetTick() - first_press_time >= DEBOUNCE_TIMEOUT)) {
      single_press_pending = 0;
      if (miSnake.direccion == DERECHA) miSnake.direccion = ABAJO; else if (miSnake.direccion == ABAJO) miSnake.direccion = IZQUIERDA;
      else if (miSnake.direccion == IZQUIERDA) miSnake.direccion = ARRIBA; else if (miSnake.direccion == ARRIBA) miSnake.direccion = DERECHA;
    }

    // PASO 2: ACTUALIZAR LÓGICA
    if (!game_over) {
        actualizar_juego();
    }

    // PASO 3: CONSTRUIR Y ENVIAR ESTADO (MÉTODO ROBUSTO)
    int len = 0;
    if (game_over) {
        len = snprintf(tx_buffer, sizeof(tx_buffer), "O:1\n");
    } else {
        len += snprintf(tx_buffer + len, sizeof(tx_buffer) - len, "S:");
        for (int i = 0; i < LARGO_SNAKE; i++) {
            len += snprintf(tx_buffer + len, sizeof(tx_buffer) - len, "%d,%d", miSnake.body[i].x, miSnake.body[i].y);
            if (i < LARGO_SNAKE - 1) {
                len += snprintf(tx_buffer + len, sizeof(tx_buffer) - len, ";");
            }
        }
        len += snprintf(tx_buffer + len, sizeof(tx_buffer) - len, "|O:0\n");
    }
    HAL_UART_Transmit(&huart2, (uint8_t*)tx_buffer, len, 100);

    // PASO 4: CONTROLAR VELOCIDAD
    HAL_Delay(1000);
  }
}

/* --- El resto del código (SystemClock_Config, etc.) va aquí --- */
void SystemClock_Config(void) { /* Tu código original aquí */
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) { Error_Handler(); }
}
static void MX_USART2_UART_Init(void) { /* Tu código original aquí */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK) { Error_Handler(); }
}
static void MX_GPIO_Init(void) { /* Tu código original aquí */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}
#ifdef __GNUC__
int _write(int file, char *ptr, int len) {
  HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
  return len;
}
#endif
void Error_Handler(void) { __disable_irq(); while (1) {} }
#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) { while (1) {} }
#endif





