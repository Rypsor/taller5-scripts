
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Firmware para un trazador de curvas de transistores BJT.
  *
  * Este programa integra dos funcionalidades principales en una placa STM32:
  *
  * 1. Trazador de Curvas de Transistores BJT:
  *    - Controla la polarización de un BJT mediante PWM (Vb y Vc).
  *    - Mide los voltajes resultantes con el ADC.
  *    - Se comunica a través de UART para recibir comandos y enviar datos.
  *    - Genera curvas características (Ic-Vb, Ic-Vc) con un pipeline de
  *      post-procesamiento que incluye ordenamiento y filtro de media móvil.
  *
  * 2. Interfaz de Usuario con Display y Encoder:
  *    - Controla un display de 7 segmentos de 4 dígitos mediante multiplexación.
  *    - Permite al usuario interactuar a través de un encoder rotatorio para
  *      modificar un valor en el display.
  *    - Incluye una funcionalidad de cuenta regresiva activada por el pulsador
  *      del encoder.
  *    - Controla un LED RGB para indicar estados.
  *
  ******************************************************************************
  *
  *                       ASIGNACIÓN DE PINES
  * ============================================================================
  * -- Trazador de Curvas --
  * PA0 (TIM5_CH1) : Salida PWM para control del voltaje de base (Vb).
  * PA5 (TIM2_CH1) : Salida PWM para control del voltaje de colector (Vc).
  * PC1 (ADC1_IN11): Entrada ADC para medición del voltaje de base (Vb).
  * PC4 (ADC1_IN14): Entrada ADC para medición del voltaje de colector (Vc).
  * PA7 (ADC1_IN7) : Entrada ADC para medición del voltaje de alimentación (Vsupply).
  *
  * -- Display 7 Segmentos --
  * Seg A -> PB12      D1 (Unidades) -> PB7
  * Seg B -> PA12      D2 (Decenas)  -> PC6
  * Seg C -> PC11      D3 (Centenas) -> PC5
  * Seg D -> PC12      D4 (Millares) -> PC13
  * Seg E -> PC10
  * Seg F -> PA11
  * Seg G -> PD2
  *
  * -- Encoder Rotatorio --
  * CLK -> PB2 (con EXTI)
  * DT  -> PB1
  * SW  -> PB15 (con EXTI)
  *
  * -- LEDs y Comunicación --
  * LED Rojo -> PB4
  * LED Verde -> PA9
  * LED Azul -> PC7
  * LED Blinky -> PC9
  * PA2 (USART2_TX): Pin de transmisión UART para comunicación con el PC.
  * PA3 (USART2_RX): Pin de recepción UART para comunicación con el PC.
  * PA8 (MCO1)     : Salida de reloj del microcontrolador para depuración.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h> // Para strcmp y strncmp
#include <stdio.h>  // Para sscanf (leer números de un string)
#include <stdlib.h> // Para sscanf
#include <math.h>   // Para powf
#include <stddef.h> // Para offsetof
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    uint32_t vc_mv;
    uint32_t vb_mv;
    uint32_t vsupply_mv;
} PuntoCurva;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STATE_NORMAL 0
#define STATE_COUNTDOWN 1
#define MAX_CURVE_POINTS 110 // (1023 / 10) + 1 = 103 puntos. Usamos 110 por seguridad.
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// --- Buffers para almacenar datos de las curvas ---
PuntoCurva g_curva_buffer[MAX_CURVE_POINTS];
uint32_t g_vc_suavizado[MAX_CURVE_POINTS];
uint32_t g_vb_suavizado[MAX_CURVE_POINTS];
uint32_t g_vsupply_suavizado[MAX_CURVE_POINTS];
uint16_t g_curve_points_count = 0; // Para saber cuántos puntos se guardaron

volatile uint8_t TIMER_DISPLAY_FLAG = 0;
volatile uint8_t ENCODER_FLAG = 0;
volatile uint8_t BUTTON_FLAG = 0;

volatile uint8_t g_system_state = STATE_NORMAL;
volatile int8_t g_countdown_value = 10;
volatile uint16_t g_countdown_ms_counter = 0;
char g_tx_buffer[100];

uint16_t g_encoder_value = 0;
uint8_t Unidad_mil, Centenas, Decenas, Unidades;

// --- Variables para los comandos UART ---
#define UART_BUFFER_SIZE 50
uint8_t g_uart_rx_buffer[UART_BUFFER_SIZE]; // Buffer para guardar el comando
volatile uint8_t g_uart_rx_index = 0;       // Índice de la posición actual en el buffer
uint8_t g_uart_rx_data;                     // Variable temporal para el byte que llega

// --- Banderas para solicitar lecturas ADC desde el bucle principal ---
volatile uint8_t g_request_adc_vc = 0;
volatile uint8_t g_request_adc_vb = 0;
volatile uint8_t g_request_curva_ic_vb = 0;
volatile uint8_t g_request_adc_ic = 0; // Nueva bandera para leer Ic
volatile uint8_t g_request_curva_ic_vc = 0;   // Nueva bandera para la curva Ic-Vc

// --- Banderas y variables para ajuste asíncrono de PWM ---
volatile uint8_t g_request_ajustar_vc = 0;
volatile uint8_t g_request_ajustar_vb = 0;
volatile uint16_t g_porcentaje_vc = 0;
volatile uint16_t g_porcentaje_vb = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void lightNumber(uint8_t number);
void update_display_digits(uint16_t value);
static uint16_t leer_canal_adc(uint32_t channel);
static void enviar_float_uart(uint32_t valor_mv, char* unidad);
static void aplicar_media_movil(PuntoCurva* datos_in, uint32_t* datos_out, uint16_t count, uint8_t n, size_t offset);
void ordenar_puntos_curva(PuntoCurva* puntos, uint16_t count, uint8_t ordenar_por_vc);
void enviar_mensaje_bienvenida(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief Envía un mensaje de bienvenida con la lista de comandos al iniciar.
  */
void enviar_mensaje_bienvenida(void) {
    char buf[128];
    HAL_UART_Transmit(&huart2, (uint8_t*)"\n--- Sistema Trazador de Curvas BJT ---\n", 42, 200);
    HAL_UART_Transmit(&huart2, (uint8_t*)"Comandos disponibles:\n", 22, 200);

    sprintf(buf, "  vcXX           - Ajusta el voltaje del colector al XX%% del PWM\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 200);
    sprintf(buf, "  vbXX           - Ajusta el voltaje de la base al XX%% del PWM\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 200);
    sprintf(buf, "  leer_vc        - Lee el voltaje del colector\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 200);
    sprintf(buf, "  leer_vb        - Lee el voltaje de la base\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 200);
    sprintf(buf, "  leer_ic        - Lee la corriente del colector\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 200);
    sprintf(buf, "  rojo/verde/azul- Controla el LED RGB\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 200);
    sprintf(buf, "  apagar         - Apaga el LED RGB\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 200);
    sprintf(buf, "  curva_ic_vb_vcXX - Genera curva Ic vs Vb (Vb barrido) con Vc constante al XX%%\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 200);
    sprintf(buf, "  curva_ic_vc_vbXX - Genera curva Ic vs Vc (Vc barrido) con Vb constante al XX%%\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 200);
    HAL_UART_Transmit(&huart2, (uint8_t*)"---------------------------------------\n\n", 40, 200);
}

/**
  * @brief Ordena un array de estructuras PuntoCurva usando Bubble Sort.
  * @param puntos Array de puntos a ordenar.
  * @param count Número de puntos en el array.
  * @param ordenar_por_vc Si es 1, ordena por vc_mv; si es 0, ordena por vb_mv.
  */
void ordenar_puntos_curva(PuntoCurva* puntos, uint16_t count, uint8_t ordenar_por_vc) {
    for (uint16_t i = 0; i < count - 1; i++) {
        for (uint16_t j = 0; j < count - i - 1; j++) {
            // Selecciona el valor por el cual ordenar (Vc o Vb)
            uint32_t val_j = ordenar_por_vc ? puntos[j].vc_mv : puntos[j].vb_mv;
            uint32_t val_j1 = ordenar_por_vc ? puntos[j + 1].vc_mv : puntos[j + 1].vb_mv;

            // Si el elemento actual es mayor que el siguiente, intercambiarlos
            if (val_j > val_j1) {
                PuntoCurva temp = puntos[j];
                puntos[j] = puntos[j + 1];
                puntos[j + 1] = temp;
            }
        }
    }
}

/**
  * @brief Aplica un filtro de media móvil a un campo específico de un array de structs.
  * @param datos_in Array de estructuras PuntoCurva de entrada.
  * @param datos_out Array de uint32_t de salida para los datos suavizados.
  * @param count Número de puntos en el array de entrada.
  * @param n Tamaño de la ventana de la media móvil.
  * @param offset Offset del miembro de la struct a promediar (usar offsetof).
  */
void aplicar_media_movil(PuntoCurva* datos_in, uint32_t* datos_out, uint16_t count, uint8_t n, size_t offset)
{
    if (count < n) return; // No hay suficientes datos para el filtro

    // Itera a través de los puntos para calcular cada valor de la media móvil
    for (uint16_t i = 0; i <= count - n; i++)
    {
        uint64_t sum = 0; // Usar uint64_t para la suma para evitar desbordamientos
        // Suma los 'n' puntos de la ventana actual
        for (uint8_t j = 0; j < n; j++)
        {
            // Calcula la dirección del miembro de la struct usando el offset
            char* struct_ptr = (char*)&datos_in[i + j];
            sum += *(uint32_t*)(struct_ptr + offset);
        }
        datos_out[i] = sum / n; // Almacena el promedio
    }
}


void update_display_digits(uint16_t value)
{
    Unidad_mil = value / 1000;
    Centenas = (value % 1000) / 100;
    Decenas = (value % 100) / 10;
    Unidades = value % 10;
}

void lightNumber(uint8_t number)
{
    // Primero, apagar todos los segmentos (ánodo común)
    HAL_GPIO_WritePin(SEGMENTO_A_GPIO_Port, SEGMENTO_A_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SEGMENTO_B_GPIO_Port, SEGMENTO_B_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SEGMENTO_C_GPIO_Port, SEGMENTO_C_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SEGMENTO_D_GPIO_Port, SEGMENTO_D_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SEGMENTO_E_GPIO_Port, SEGMENTO_E_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SEGMENTO_F_GPIO_Port, SEGMENTO_F_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SEGMENTO_G_GPIO_Port, SEGMENTO_G_Pin, GPIO_PIN_SET);

    // Encender los segmentos necesarios
    switch (number)
    {
    case 0:
        HAL_GPIO_WritePin(SEGMENTO_A_GPIO_Port, SEGMENTO_A_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SEGMENTO_B_GPIO_Port, SEGMENTO_B_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SEGMENTO_C_GPIO_Port, SEGMENTO_C_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SEGMENTO_D_GPIO_Port, SEGMENTO_D_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SEGMENTO_E_GPIO_Port, SEGMENTO_E_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SEGMENTO_F_GPIO_Port, SEGMENTO_F_Pin, GPIO_PIN_RESET);
        break;
    case 1:
        HAL_GPIO_WritePin(SEGMENTO_B_GPIO_Port, SEGMENTO_B_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SEGMENTO_C_GPIO_Port, SEGMENTO_C_Pin, GPIO_PIN_RESET);
        break;
    case 2:
        HAL_GPIO_WritePin(SEGMENTO_A_GPIO_Port, SEGMENTO_A_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SEGMENTO_B_GPIO_Port, SEGMENTO_B_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SEGMENTO_D_GPIO_Port, SEGMENTO_D_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SEGMENTO_E_GPIO_Port, SEGMENTO_E_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SEGMENTO_G_GPIO_Port, SEGMENTO_G_Pin, GPIO_PIN_RESET);
        break;
    case 3:
        HAL_GPIO_WritePin(SEGMENTO_A_GPIO_Port, SEGMENTO_A_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SEGMENTO_B_GPIO_Port, SEGMENTO_B_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SEGMENTO_C_GPIO_Port, SEGMENTO_C_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SEGMENTO_D_GPIO_Port, SEGMENTO_D_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SEGMENTO_G_GPIO_Port, SEGMENTO_G_Pin, GPIO_PIN_RESET);
        break;
    case 4:
        HAL_GPIO_WritePin(SEGMENTO_B_GPIO_Port, SEGMENTO_B_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SEGMENTO_C_GPIO_Port, SEGMENTO_C_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SEGMENTO_F_GPIO_Port, SEGMENTO_F_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SEGMENTO_G_GPIO_Port, SEGMENTO_G_Pin, GPIO_PIN_RESET);
        break;
    case 5:
        HAL_GPIO_WritePin(SEGMENTO_A_GPIO_Port, SEGMENTO_A_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SEGMENTO_C_GPIO_Port, SEGMENTO_C_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SEGMENTO_D_GPIO_Port, SEGMENTO_D_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SEGMENTO_F_GPIO_Port, SEGMENTO_F_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SEGMENTO_G_GPIO_Port, SEGMENTO_G_Pin, GPIO_PIN_RESET);
        break;
    case 6:
        HAL_GPIO_WritePin(SEGMENTO_A_GPIO_Port, SEGMENTO_A_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SEGMENTO_C_GPIO_Port, SEGMENTO_C_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SEGMENTO_D_GPIO_Port, SEGMENTO_D_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SEGMENTO_E_GPIO_Port, SEGMENTO_E_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SEGMENTO_F_GPIO_Port, SEGMENTO_F_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SEGMENTO_G_GPIO_Port, SEGMENTO_G_Pin, GPIO_PIN_RESET);
        break;
    case 7:
        HAL_GPIO_WritePin(SEGMENTO_A_GPIO_Port, SEGMENTO_A_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SEGMENTO_B_GPIO_Port, SEGMENTO_B_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SEGMENTO_C_GPIO_Port, SEGMENTO_C_Pin, GPIO_PIN_RESET);
        break;
    case 8:
        HAL_GPIO_WritePin(SEGMENTO_A_GPIO_Port, SEGMENTO_A_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SEGMENTO_B_GPIO_Port, SEGMENTO_B_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SEGMENTO_C_GPIO_Port, SEGMENTO_C_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SEGMENTO_D_GPIO_Port, SEGMENTO_D_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SEGMENTO_E_GPIO_Port, SEGMENTO_E_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SEGMENTO_F_GPIO_Port, SEGMENTO_F_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SEGMENTO_G_GPIO_Port, SEGMENTO_G_Pin, GPIO_PIN_RESET);
        break;
    case 9:
        HAL_GPIO_WritePin(SEGMENTO_A_GPIO_Port, SEGMENTO_A_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SEGMENTO_B_GPIO_Port, SEGMENTO_B_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SEGMENTO_C_GPIO_Port, SEGMENTO_C_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SEGMENTO_D_GPIO_Port, SEGMENTO_D_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SEGMENTO_F_GPIO_Port, SEGMENTO_F_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SEGMENTO_G_GPIO_Port, SEGMENTO_G_Pin, GPIO_PIN_RESET);
        break;
    }
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
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  // Iniciar los PWMs de la Tarea 3
  // PA5 (Vc) usa TIM2, Canal 1
  // PA0 (Vb) usa TIM5, Canal 1
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);

  // --- Iniciar Timers Tarea #1 (Blinky y Display) ---
   HAL_TIM_Base_Start_IT(&htim4); // Inicia TIM4 para el blinky (PC9)
   HAL_TIM_Base_Start_IT(&htim3); // Inicia TIM3 para el refresco del display

   // --- Iniciar Timers Tarea #3 (PWM/DAC) ---
   HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // Inicia Pin A (PA5)
   HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1); // Inicia Pin B (PA0)

   // --- Configuración MCO (Tarea #3) ---
   // Saca el PLLCLK (100MHz) dividido por 4 = 25MHz
   HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_PLLCLK, RCC_MCO_DIV4);

   update_display_digits(g_encoder_value);

   // Habilitar interrupciones para el encoder SW (PB15)
   HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
   HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

   // --- Iniciar la recepción UART por Interrupción ---
   HAL_UART_Receive_IT(&huart2, &g_uart_rx_data, 1);

   // --- Enviar mensaje de bienvenida ---
   HAL_Delay(100); // Pausa para asegurar que el terminal serie esté listo
   enviar_mensaje_bienvenida();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // --- 1. MANEJO DEL PULSADOR (Inicia la cuenta regresiva) ---
    if (BUTTON_FLAG)
    {
      BUTTON_FLAG = 0;
      if (g_system_state == STATE_NORMAL)
      {
        g_system_state = STATE_COUNTDOWN;
        g_countdown_value = 10;
        g_countdown_ms_counter = 0;
        update_display_digits(g_countdown_value);
      }
    }

    // --- 2. MANEJO DEL DISPLAY Y LÓGICA DE ESTADOS (Controlado por Timer) ---
    if (TIMER_DISPLAY_FLAG)
    {
      TIMER_DISPLAY_FLAG = 0;

      if (g_system_state == STATE_COUNTDOWN)
      {
        // TIM3 se dispara cada 5ms. 200 * 5ms = 1000ms = 1 segundo.
        if (g_countdown_ms_counter >= 200)
        {
          g_countdown_ms_counter = 0;
          g_countdown_value--;

          if (g_countdown_value < 0)
          {
            g_system_state = STATE_NORMAL;
            g_encoder_value = 0;
            update_display_digits(g_encoder_value);
          }
          else
          {
            update_display_digits(g_countdown_value);
          }
        }
      }

      // --- Multiplexación del Display ---
      HAL_GPIO_WritePin(MILES_GPIO_Port, MILES_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(CENTENAS_GPIO_Port, CENTENAS_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(DECENAS_GPIO_Port, DECENAS_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(UNIDADES_GPIO_Port, UNIDADES_Pin, GPIO_PIN_SET);

      static uint8_t current_digit = 0;
      switch (current_digit)
      {
      case 0:
        lightNumber(Unidades);
        HAL_GPIO_WritePin(UNIDADES_GPIO_Port, UNIDADES_Pin, GPIO_PIN_RESET);
        break;
      case 1:
        lightNumber(Decenas);
        HAL_GPIO_WritePin(DECENAS_GPIO_Port, DECENAS_Pin, GPIO_PIN_RESET);
        break;
      case 2:
        lightNumber(Centenas);
        HAL_GPIO_WritePin(CENTENAS_GPIO_Port, CENTENAS_Pin, GPIO_PIN_RESET);
        break;
      case 3:
        lightNumber(Unidad_mil);
        HAL_GPIO_WritePin(MILES_GPIO_Port, MILES_Pin, GPIO_PIN_RESET);
        break;
      }
      if (++current_digit > 3)
        current_digit = 0;
    }

    // --- 3. MANEJO DEL GIRO DEL ENCODER (Solo en modo normal) ---
    if (ENCODER_FLAG && g_system_state == STATE_NORMAL)
    {
      ENCODER_FLAG = 0;
      if (HAL_GPIO_ReadPin(ENCODER_DT_GPIO_Port, ENCODER_DT_Pin) == GPIO_PIN_RESET)
      { // CCW
        if (g_encoder_value == 0)
          g_encoder_value = 4095;
        else
          g_encoder_value--;
      }
      else
      { // CW
        if (g_encoder_value == 4095)
          g_encoder_value = 0;
        else
          g_encoder_value++;
      }
      update_display_digits(g_encoder_value);
    }

    // --- 4. MANEJO DE SOLICITUDES (BANDERAS) ---
    // Estas banderas son activadas por la interrupción de UART para evitar
    // ejecutar código bloqueante (como lecturas de ADC o delays) dentro de la ISR.

    // --- Lecturas Simples de Voltaje ---
	if (g_request_adc_vc)
	{
		uint16_t adc_val = leer_canal_adc(ADC_CHANNEL_14);
		uint32_t voltage_mv = (uint32_t)adc_val * 3300 / 4095;

		HAL_UART_Transmit(&huart2, (uint8_t*)"Vc: ", 4, 100);
		enviar_float_uart(voltage_mv, "V\n");

		g_request_adc_vc = 0;
	}

	if (g_request_adc_vb)
	{
		uint16_t adc_val = leer_canal_adc(ADC_CHANNEL_11);
		uint32_t voltage_mv = (uint32_t)adc_val * 3300 / 4095;

		HAL_UART_Transmit(&huart2, (uint8_t*)"Vb: ", 4, 100);
		enviar_float_uart(voltage_mv, "V\n");

		g_request_adc_vb = 0;
	}

    // --- Generación de Curva Ic vs Vb (con Vc constante) ---
    if (g_request_curva_ic_vb)
    {
        // 1. Ajustar Vc al valor especificado en el comando y mostrar contexto.
        uint16_t pwm_vc_val = ((uint32_t)g_porcentaje_vc * 1023) / 100;
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm_vc_val);
        HAL_Delay(200); // Pausa extra para que el voltaje Vc se estabilice.

        uint16_t adc_vc_context = leer_canal_adc(ADC_CHANNEL_14);
        uint32_t vc_mv_context = (uint32_t)adc_vc_context * 3300 / 4095;
        HAL_UART_Transmit(&huart2, (uint8_t*)"datos usando vc = ", 18, 100);
        enviar_float_uart(vc_mv_context, "V\n");

        // 2. Realizar el barrido de Vb, capturando todos los puntos.
        g_curve_points_count = 0;
        for (uint16_t pwm_val = 0; pwm_val <= 1023; pwm_val += 10)
        {
            if (g_curve_points_count >= MAX_CURVE_POINTS) break;

            __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, pwm_val);
            HAL_Delay(20);

            g_curva_buffer[g_curve_points_count].vc_mv = (uint32_t)leer_canal_adc(ADC_CHANNEL_14) * 3300 / 4095;
            g_curva_buffer[g_curve_points_count].vb_mv = (uint32_t)leer_canal_adc(ADC_CHANNEL_11) * 3300 / 4095;
            g_curva_buffer[g_curve_points_count].vsupply_mv = (uint32_t)leer_canal_adc(ADC_CHANNEL_7) * 3300 / 4095;

            g_curve_points_count++;
        }

        // 3. Post-procesamiento: ordenar y filtrar los datos capturados.
        ordenar_puntos_curva(g_curva_buffer, g_curve_points_count, 0); // Ordenar por Vb

        aplicar_media_movil(g_curva_buffer, g_vc_suavizado, g_curve_points_count, 5, offsetof(PuntoCurva, vc_mv));
        aplicar_media_movil(g_curva_buffer, g_vb_suavizado, g_curve_points_count, 5, offsetof(PuntoCurva, vb_mv));
        aplicar_media_movil(g_curva_buffer, g_vsupply_suavizado, g_curve_points_count, 5, offsetof(PuntoCurva, vsupply_mv));
        uint16_t puntos_suavizados = g_curve_points_count > 4 ? g_curve_points_count - 4 : 0;

        // 4. Imprimir la tabla de resultados.
        sprintf(g_tx_buffer, "Vb(V);Ic(mA)\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)g_tx_buffer, strlen(g_tx_buffer), 100);

        for (uint16_t i = 0; i < puntos_suavizados; i++)
        {
            uint32_t vc_mv = g_vc_suavizado[i];
            uint32_t vb_mv = g_vb_suavizado[i];
            uint32_t vsupply_mv = g_vsupply_suavizado[i];

            uint32_t ic_ua = (vsupply_mv > vc_mv) ? (vsupply_mv - vc_mv) * 1000 / 220 : 0;

            enviar_float_uart(vb_mv, ";");
            enviar_float_uart(ic_ua, "\n");
        }

        g_request_curva_ic_vb = 0;
    }

    // --- Generación de Curva Ic vs Vc (con Vb constante) ---
	if (g_request_curva_ic_vc)
	{
        // 1. Ajustar Vb al valor especificado y mostrar contexto.
        uint16_t pwm_vb_val = ((uint32_t)g_porcentaje_vb * 1023) / 100;
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, pwm_vb_val);
        HAL_Delay(200);

        uint16_t adc_vb_context = leer_canal_adc(ADC_CHANNEL_11);
        uint32_t vb_mv_context = (uint32_t)adc_vb_context * 3300 / 4095;
        HAL_UART_Transmit(&huart2, (uint8_t*)"datos usando vb = ", 18, 100);
        enviar_float_uart(vb_mv_context, "V\n");

        // 2. Realizar el barrido de Vc, capturando todos los puntos.
		g_curve_points_count = 0;
		for (uint16_t pwm_val = 0; pwm_val <= 1023; pwm_val += 10)
		{
			if (g_curve_points_count >= MAX_CURVE_POINTS) break;

			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm_val);
			HAL_Delay(20);

			g_curva_buffer[g_curve_points_count].vc_mv = (uint32_t)leer_canal_adc(ADC_CHANNEL_14) * 3300 / 4095;
			g_curva_buffer[g_curve_points_count].vb_mv = (uint32_t)leer_canal_adc(ADC_CHANNEL_11) * 3300 / 4095;
			g_curva_buffer[g_curve_points_count].vsupply_mv = (uint32_t)leer_canal_adc(ADC_CHANNEL_7) * 3300 / 4095;

			g_curve_points_count++;
		}

        // 3. Post-procesamiento: ordenar y filtrar.
        ordenar_puntos_curva(g_curva_buffer, g_curve_points_count, 1); // Ordenar por Vc

        aplicar_media_movil(g_curva_buffer, g_vc_suavizado, g_curve_points_count, 5, offsetof(PuntoCurva, vc_mv));
        aplicar_media_movil(g_curva_buffer, g_vb_suavizado, g_curve_points_count, 5, offsetof(PuntoCurva, vb_mv));
        aplicar_media_movil(g_curva_buffer, g_vsupply_suavizado, g_curve_points_count, 5, offsetof(PuntoCurva, vsupply_mv));
        uint16_t puntos_suavizados = g_curve_points_count > 4 ? g_curve_points_count - 4 : 0;

        // 4. Imprimir la tabla de resultados.
        sprintf(g_tx_buffer, "Vc(V);Ic(mA)\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)g_tx_buffer, strlen(g_tx_buffer), 100);

        for (uint16_t i = 0; i < puntos_suavizados; i++)
        {
            uint32_t vc_mv = g_vc_suavizado[i];
            uint32_t vsupply_mv = g_vsupply_suavizado[i];

            uint32_t ic_ua = (vsupply_mv > vc_mv) ? (vsupply_mv - vc_mv) * 1000 / 220 : 0;

            enviar_float_uart(vc_mv, ";");
            enviar_float_uart(ic_ua, "\n");
        }

		g_request_curva_ic_vc = 0;
	}

    // --- Lectura Simple de Corriente ---
    if (g_request_adc_ic)
    {
        uint16_t adc_vsupply = leer_canal_adc(ADC_CHANNEL_7);
        uint16_t adc_vc = leer_canal_adc(ADC_CHANNEL_14);
        uint32_t vsupply_mv = (uint32_t)adc_vsupply * 3300 / 4095;
        uint32_t vc_mv = (uint32_t)adc_vc * 3300 / 4095;

        uint32_t ic_ua = (vsupply_mv > vc_mv) ? (vsupply_mv - vc_mv) * 1000 / 220 : 0;

        HAL_UART_Transmit(&huart2, (uint8_t*)"Ic: ", 4, 100);
        enviar_float_uart(ic_ua, "mA\n");

        g_request_adc_ic = 0;
    }

    // --- Ajuste Asíncrono de Voltajes ---
    if (g_request_ajustar_vc) {
        // 1. Ajustar el PWM al valor solicitado.
        uint16_t pwm_val = ((uint32_t)g_porcentaje_vc * 1023) / 100;
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm_val);
        HAL_Delay(50); // Pausa para estabilizar.

        // 2. Medir el voltaje real y enviar confirmación.
        uint16_t adc_val = leer_canal_adc(ADC_CHANNEL_14);
        uint32_t voltage_mv = (uint32_t)adc_val * 3300 / 4095;
        sprintf(g_tx_buffer, "Vc ajustado a: %u%% (PWM: %u), Medido: ", g_porcentaje_vc, pwm_val);
        HAL_UART_Transmit(&huart2, (uint8_t*)g_tx_buffer, strlen(g_tx_buffer), 100);
        enviar_float_uart(voltage_mv, "V\n");

        g_request_ajustar_vc = 0;
    }

    if (g_request_ajustar_vb) {
        uint16_t pwm_val = ((uint32_t)g_porcentaje_vb * 1023) / 100;
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, pwm_val);
        HAL_Delay(50);

        uint16_t adc_val = leer_canal_adc(ADC_CHANNEL_11);
        uint32_t voltage_mv = (uint32_t)adc_val * 3300 / 4095;
        sprintf(g_tx_buffer, "Vb ajustado a: %u%% (PWM: %u), Medido: ", g_porcentaje_vb, pwm_val);
        HAL_UART_Transmit(&huart2, (uint8_t*)g_tx_buffer, strlen(g_tx_buffer), 100);
        enviar_float_uart(voltage_mv, "V\n");

        g_request_ajustar_vb = 0;
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 200;
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
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_PLLCLK, RCC_MCODIV_1);
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
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END ADC1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */
  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1023;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  /* USER CODE BEGIN TIM3_Init 1 */
  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 9999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 49;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */
  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */
  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 9999;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 4999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END TIM4_Init 2 */
  /* USER CODE END TIM4_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */
  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 1023;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END TIM5_Init 2 */
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
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, MILES_Pin|CENTENAS_Pin|DECENAS_Pin|BLINKY_Pin
                          |SEGMENTO_E_Pin|SEGMENTO_C_Pin|SEGMENTO_D_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_AZUL_Pin|LED_VERDE_Pin|SEGMENTO_F_Pin|SEGMENTO_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_ROJO_Pin|SEGMENTO_A_Pin|UNIDADES_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SEGMENTO_G_GPIO_Port, SEGMENTO_G_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : MILES_Pin */
  GPIO_InitStruct.Pin = MILES_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MILES_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_AZUL_Pin LED_VERDE_Pin SEGMENTO_F_Pin SEGMENTO_B_Pin */
  GPIO_InitStruct.Pin = LED_AZUL_Pin|LED_VERDE_Pin|SEGMENTO_F_Pin|SEGMENTO_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CENTENAS_Pin DECENAS_Pin BLINKY_Pin SEGMENTO_E_Pin
                           SEGMENTO_C_Pin SEGMENTO_D_Pin */
  GPIO_InitStruct.Pin = CENTENAS_Pin|DECENAS_Pin|BLINKY_Pin|SEGMENTO_E_Pin
                          |SEGMENTO_C_Pin|SEGMENTO_D_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_ROJO_Pin SEGMENTO_A_Pin UNIDADES_Pin */
  GPIO_InitStruct.Pin = LED_ROJO_Pin|SEGMENTO_A_Pin|UNIDADES_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ENCODER_DT_Pin */
  GPIO_InitStruct.Pin = ENCODER_DT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ENCODER_DT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ENCODER_CLK_Pin */
  GPIO_InitStruct.Pin = ENCODER_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ENCODER_CLK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ENCODER_SW_Pin */
  GPIO_InitStruct.Pin = ENCODER_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ENCODER_SW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SEGMENTO_G_Pin */
  GPIO_InitStruct.Pin = SEGMENTO_G_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SEGMENTO_G_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}


/* USER CODE BEGIN 4 */

// --- Nueva función de ayuda para leer un solo canal del ADC ---
static uint16_t leer_canal_adc(uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    uint32_t adc_sum = 0;
	uint16_t adc_value = 0;
	const int num_samples = 16;

    // 1. Configurar el canal específico que queremos leer
    sConfig.Channel = channel;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES; // Aumentado para mayor estabilidad
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        return 0; // Retornar 0 en caso de error de configuración
    }

    // 2. Bucle de sobremuestreo y promediado
	for (int i = 0; i < num_samples; i++)
	{
		HAL_ADC_Start(&hadc1);
		if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
		{
			adc_sum += HAL_ADC_GetValue(&hadc1);
		}
	}

    // 3. Calcular el promedio
    adc_value = adc_sum / num_samples;

    return adc_value;
}

// --- Función de ayuda para imprimir un valor en formato X.XXX ---
static void enviar_float_uart(uint32_t valor_mv, char* unidad) {
	char temp_buf[20];
	sprintf(temp_buf, "%u", (unsigned int)(valor_mv / 1000));
	HAL_UART_Transmit(&huart2, (uint8_t*)temp_buf, strlen(temp_buf), 100);
	HAL_UART_Transmit(&huart2, (uint8_t*)",", 1, 100);
	uint32_t frac = valor_mv % 1000;
	if (frac < 100) HAL_UART_Transmit(&huart2, (uint8_t*)"0", 1, 100);
	if (frac < 10) HAL_UART_Transmit(&huart2, (uint8_t*)"0", 1, 100);
	sprintf(temp_buf, "%u", (unsigned int)frac);
	HAL_UART_Transmit(&huart2, (uint8_t*)temp_buf, strlen(temp_buf), 100);
	HAL_UART_Transmit(&huart2, (uint8_t*)unidad, strlen(unidad), 100);
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    // Verificamos si el carácter recibido es un terminador de línea
    if (g_uart_rx_data == '\r' || g_uart_rx_data == '\n')
    {
      // --- Comando Recibido: Procesar ---
      if (g_uart_rx_index > 0)
      {
        // 1. Añadimos el terminador nulo para convertir el búfer en un string de C
        g_uart_rx_buffer[g_uart_rx_index] = '\0';


        // --- TAREA 3: Comandos PWM (Formato vbXXX) ---
        if (strncmp((char*)g_uart_rx_buffer, "vb", 2) == 0)
        {
            g_porcentaje_vb = atoi((char*)&g_uart_rx_buffer[2]);
            if (g_porcentaje_vb > 100) g_porcentaje_vb = 100;
            g_request_ajustar_vb = 1;
        }
        else if (strncmp((char*)g_uart_rx_buffer, "vc", 2) == 0)
        {
            g_porcentaje_vc = atoi((char*)&g_uart_rx_buffer[2]);
            if (g_porcentaje_vc > 100) g_porcentaje_vc = 100;
            g_request_ajustar_vc = 1;
        }

		// --- TAREA 3: Comandos para leer Vc y Vb (No bloqueante) ---
		else if (strcmp((char*)g_uart_rx_buffer, "leer_vc") == 0)
		{
			g_request_adc_vc = 1; // Activar la bandera para el bucle principal
		}
		else if (strcmp((char*)g_uart_rx_buffer, "leer_vb") == 0)
		{
			g_request_adc_vb = 1; // Activar la bandera para el bucle principal
		}
        else if (strncmp((char*)g_uart_rx_buffer, "curva_ic_vb_vc", 14) == 0)
        {
            g_porcentaje_vc = atoi((char*)&g_uart_rx_buffer[14]);
            if (g_porcentaje_vc > 100) g_porcentaje_vc = 100;
            g_request_curva_ic_vb = 1;
        }
        else if (strncmp((char*)g_uart_rx_buffer, "curva_ic_vc_vb", 14) == 0)
        {
            g_porcentaje_vb = atoi((char*)&g_uart_rx_buffer[14]);
            if (g_porcentaje_vb > 100) g_porcentaje_vb = 100;
            g_request_curva_ic_vc = 1;
        }
		else if (strcmp((char*)g_uart_rx_buffer, "leer_ic") == 0)
		{
			g_request_adc_ic = 1; // Activar la bandera para el bucle principal
		}

        // --- TAREA 1: Comandos LED RGB ---
        else if (strcmp((char*)g_uart_rx_buffer, "rojo") == 0)
        {
        	HAL_GPIO_WritePin(LED_AZUL_GPIO_Port, LED_AZUL_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED_VERDE_GPIO_Port, LED_VERDE_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED_ROJO_GPIO_Port, LED_ROJO_Pin, GPIO_PIN_SET);
        }
        else if (strcmp((char*)g_uart_rx_buffer, "verde") == 0)
        {
        	HAL_GPIO_WritePin(LED_ROJO_GPIO_Port, LED_ROJO_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED_AZUL_GPIO_Port, LED_AZUL_Pin, GPIO_PIN_RESET);
        	HAL_GPIO_WritePin(LED_VERDE_GPIO_Port, LED_VERDE_Pin, GPIO_PIN_SET);
        }
        else if (strcmp((char*)g_uart_rx_buffer, "azul") == 0)
        {
        	HAL_GPIO_WritePin(LED_ROJO_GPIO_Port, LED_ROJO_Pin, GPIO_PIN_RESET);
        	HAL_GPIO_WritePin(LED_VERDE_GPIO_Port, LED_VERDE_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_AZUL_GPIO_Port, LED_AZUL_Pin, GPIO_PIN_SET);
        }
        else if (strcmp((char*)g_uart_rx_buffer, "apagar") == 0)
        {
          HAL_GPIO_WritePin(LED_ROJO_GPIO_Port, LED_ROJO_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(LED_VERDE_GPIO_Port, LED_VERDE_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(LED_AZUL_GPIO_Port, LED_AZUL_Pin, GPIO_PIN_RESET);
        }
		else
		{
			HAL_UART_Transmit(&huart2, (uint8_t*)"Comando no valido\n", 18, 100);
		}

		// 3. Reiniciamos el índice del buffer para el próximo comando
		g_uart_rx_index = 0;
	  }
	}
	else
	{
      // --- Carácter Normal: Añadir al buffer ---
	  if (g_uart_rx_index < (UART_BUFFER_SIZE - 1))
	  {
		g_uart_rx_buffer[g_uart_rx_index++] = g_uart_rx_data;
	  }
	}

	// Volvemos a armar la interrupción para "escuchar" el próximo carácter
	HAL_UART_Receive_IT(&huart2, &g_uart_rx_data, 1);
  }
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // Timer para el display (alta frecuencia)
    if (htim->Instance == TIM3)
    {
        TIMER_DISPLAY_FLAG = 1;
        if (g_system_state == STATE_COUNTDOWN)
        {
            g_countdown_ms_counter++;
        }
    }
    // Timer para el LED (baja frecuencia)
    if (htim->Instance == TIM4)
    {
        HAL_GPIO_TogglePin(BLINKY_GPIO_Port, BLINKY_Pin);
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    static uint32_t last_button_press_time = 0;

    if (GPIO_Pin == ENCODER_CLK_Pin)
    {
        ENCODER_FLAG = 1;
    }

    if (GPIO_Pin == ENCODER_SW_Pin)
    {
        // Anti-rebotes (debounce): solo procesar si han pasado > 200ms
        if (HAL_GetTick() - last_button_press_time > 200)
        {
            BUTTON_FLAG = 1;
            last_button_press_time = HAL_GetTick();
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
