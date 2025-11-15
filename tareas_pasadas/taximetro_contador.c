/********************************************************************************
 * @file           : main.c
 * @author         : [Tu Nombre]
 * @brief          : Proyecto final para el control de un display de 7 segmentos con encoder.
 ********************************************************************************
 * @description
 *
 * Este programa controla un display de 7 segmentos de 4 dígitos utilizando un
 * microcontrolador STM32F411RE. Las funcionalidades principales son:
 *
 * 1.  **LED de Estado:** Un LED en PH1 parpadea a 1 Hz (cada 500ms) para indicar
 * que el sistema está funcionando. Esta tarea es controlada por TIM4.
 *
 * 2.  **Display 7 Segmentos:** Muestra un número de 0 a 4095. El refresco del
 * display se realiza por multiplexación (POV) a alta frecuencia para evitar
 * parpadeos, controlado por la interrupción de TIM3.
 *
 * 3.  **Encoder Rotatorio:**
 * - **Giro:** Al girar en sentido horario (CW), el número en el display aumenta.
 * Al girar en sentido antihorario (CCW), disminuye. El valor se ajusta
 * automáticamente entre 0 y 4095 (wrap-around).
 * - **Pulsador (SW):** Al presionar el botón del encoder, se inicia una cuenta
 * regresiva desde 10 en el display. Durante la cuenta, el encoder se deshabilita.
 *
 * ------------------------------------------------------------------------------
 * CONEXIONES DE HARDWARE
 * ------------------------------------------------------------------------------
 * - Display (Segmentos):
 * - Seg A -> PB12
 * - Seg B -> PA12
 * - Seg C -> PC11
 * - Seg D -> PC12
 * - Seg E -> PC10
 * - Seg F -> PA11
 * - Seg G -> PD2
 * - Display (Dígitos - Transistores):
 * - D1 (Unidades) -> PB7
 * - D2 (Decenas)  -> PC6
 * - D3 (Centenas)  -> PC5
 * - D4 (Millares) -> PC13
 * - Encoder Rotatorio:
 * - CLK -> PB2 (con EXTI)
 * - DT  -> PB1
 * - SW  -> PB15 (con EXTI)
 * - LED de Estado:
 * - LED -> PH1 (Específico de esta placa)
 *
 *******************************************************************************/

/* Includes */
#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "stm32f4xx_it.h"

// ==== Definiciones de pines ====
// --- LED (Configuración específica para tu placa) ---
#define LED_PH1_PORT GPIOH
#define LED_PH1_PIN GPIO_PIN_1
// --- Display (Dígitos) ---
#define GPIO_UNIDADES GPIOB
#define GPIO_PIN_UNIDADES GPIO_PIN_7
#define GPIO_DECENAS GPIOC
#define GPIO_PIN_DECENAS GPIO_PIN_6
#define GPIO_CENTENAS GPIOC
#define GPIO_PIN_CENTENAS GPIO_PIN_5
#define GPIO_UNIDADES_MIL GPIOC
#define GPIO_PIN_UNIDADES_MIL GPIO_PIN_13
// --- Display (Segmentos) ---
#define GPIO_A_PORT GPIOB
#define GPIO_A_PIN GPIO_PIN_12
#define GPIO_B_PORT GPIOA
#define GPIO_B_PIN GPIO_PIN_12
#define GPIO_C_PORT GPIOC
#define GPIO_C_PIN GPIO_PIN_11
#define GPIO_D_PORT GPIOC
#define GPIO_D_PIN GPIO_PIN_12
#define GPIO_E_PORT GPIOC
#define GPIO_E_PIN GPIO_PIN_10
#define GPIO_F_PORT GPIOA
#define GPIO_F_PIN GPIO_PIN_11
#define GPIO_G_PORT GPIOD
#define GPIO_G_PIN GPIO_PIN_2
// --- Encoder ---
#define ENCODER_CLK_PORT GPIOB
#define ENCODER_PIN_CLK GPIO_PIN_2
#define ENCODER_DATA_PORT GPIOB
#define ENCODER_PIN_DATA GPIO_PIN_1
#define ENCODER_SW_PORT GPIOB
#define ENCODER_PIN_SW GPIO_PIN_15

// ==== Estados del programa ====
#define STATE_NORMAL 0
#define STATE_COUNTDOWN 1

// ==== Variables Globales ====
TIM_HandleTypeDef htim3; // Timer para el display
TIM_HandleTypeDef htim4; // Timer para el LED blinky

volatile uint8_t TIMER_DISPLAY_FLAG = 0; // Flag para refrescar el display
volatile uint8_t ENCODER_FLAG = 0;
volatile uint8_t BUTTON_FLAG = 0;

volatile uint8_t g_system_state = STATE_NORMAL;
volatile int8_t g_countdown_value = 10;
volatile uint16_t g_countdown_ms_counter = 0;

uint16_t g_encoder_value = 0;
uint8_t Unidad_mil, Centenas, Decenas, Unidades;

// ==== Prototipos de funciones ====
void init_display_pins(void);
void init_encoder_pins(void);
void init_display_timer(void);
void init_blinky_timer(void);
void lightNumber(uint8_t number);
void update_display_digits(uint16_t value);

int main(void)
{
    HAL_Init();

    // Inicialización de periféricos
    init_display_pins();
    init_encoder_pins();
    init_display_timer();
    init_blinky_timer();

    // Iniciar los timers en modo interrupción
    HAL_TIM_Base_Start_IT(&htim3);
    HAL_TIM_Base_Start_IT(&htim4);

    update_display_digits(g_encoder_value);

    while (1)
    {
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
            HAL_GPIO_WritePin(GPIO_UNIDADES_MIL, GPIO_PIN_UNIDADES_MIL, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIO_CENTENAS, GPIO_PIN_CENTENAS, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIO_DECENAS, GPIO_PIN_DECENAS, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIO_UNIDADES, GPIO_PIN_UNIDADES, GPIO_PIN_SET);

            static uint8_t current_digit = 0;
            switch (current_digit)
            {
            case 0:
                lightNumber(Unidades);
                HAL_GPIO_WritePin(GPIO_UNIDADES, GPIO_PIN_UNIDADES, GPIO_PIN_RESET);
                break;
            case 1:
                lightNumber(Decenas);
                HAL_GPIO_WritePin(GPIO_DECENAS, GPIO_PIN_DECENAS, GPIO_PIN_RESET);
                break;
            case 2:
                lightNumber(Centenas);
                HAL_GPIO_WritePin(GPIO_CENTENAS, GPIO_PIN_CENTENAS, GPIO_PIN_RESET);
                break;
            case 3:
                lightNumber(Unidad_mil);
                HAL_GPIO_WritePin(GPIO_UNIDADES_MIL, GPIO_PIN_UNIDADES_MIL, GPIO_PIN_RESET);
                break;
            }
            if (++current_digit > 3)
                current_digit = 0;
        }

        // --- 3. MANEJO DEL GIRO DEL ENCODER (Solo en modo normal) ---
        if (ENCODER_FLAG && g_system_state == STATE_NORMAL)
        {
            ENCODER_FLAG = 0;
            if (HAL_GPIO_ReadPin(ENCODER_DATA_PORT, ENCODER_PIN_DATA) == GPIO_PIN_RESET)
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
    }
    return 0;
}

void update_display_digits(uint16_t value)
{
    Unidad_mil = value / 1000;
    Centenas = (value % 1000) / 100;
    Decenas = (value % 100) / 10;
    Unidades = value % 10;
}

void init_display_pins(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE(); // Habilitar reloj para el puerto H

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    // Segmentos
    GPIO_InitStruct.Pin = GPIO_A_PIN;
    HAL_GPIO_Init(GPIO_A_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_B_PIN;
    HAL_GPIO_Init(GPIO_B_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_C_PIN;
    HAL_GPIO_Init(GPIO_C_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_D_PIN;
    HAL_GPIO_Init(GPIO_D_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_E_PIN;
    HAL_GPIO_Init(GPIO_E_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_F_PIN;
    HAL_GPIO_Init(GPIO_F_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_G_PIN;
    HAL_GPIO_Init(GPIO_G_PORT, &GPIO_InitStruct);
    // Dígitos
    GPIO_InitStruct.Pin = GPIO_PIN_UNIDADES;
    HAL_GPIO_Init(GPIO_UNIDADES, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_PIN_DECENAS;
    HAL_GPIO_Init(GPIO_DECENAS, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_PIN_CENTENAS;
    HAL_GPIO_Init(GPIO_CENTENAS, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_PIN_UNIDADES_MIL;
    HAL_GPIO_Init(GPIO_UNIDADES_MIL, &GPIO_InitStruct);
    // LED
    GPIO_InitStruct.Pin = LED_PH1_PIN;
    HAL_GPIO_Init(LED_PH1_PORT, &GPIO_InitStruct);

    // Estado inicial de los transistores (apagados para ánodo común)
    HAL_GPIO_WritePin(GPIO_UNIDADES_MIL, GPIO_PIN_UNIDADES_MIL, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_CENTENAS, GPIO_PIN_CENTENAS, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_DECENAS, GPIO_PIN_DECENAS, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_UNIDADES, GPIO_PIN_UNIDADES, GPIO_PIN_SET);
}

void init_encoder_pins(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    // CLK como interrupción en flanco de bajada
    GPIO_InitStruct.Pin = ENCODER_PIN_CLK;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    HAL_GPIO_Init(ENCODER_CLK_PORT, &GPIO_InitStruct);
    // DATA como entrada normal
    GPIO_InitStruct.Pin = ENCODER_PIN_DATA;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(ENCODER_DATA_PORT, &GPIO_InitStruct);
    // SW como interrupción en flanco de bajada
    GPIO_InitStruct.Pin = ENCODER_PIN_SW;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    HAL_GPIO_Init(ENCODER_SW_PORT, &GPIO_InitStruct);
    // Habilitar interrupciones y prioridades
    HAL_NVIC_SetPriority(EXTI2_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI2_IRQn);
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

// Timer de alta frecuencia para el refresco del display (200 Hz)
void init_display_timer(void)
{
    __HAL_RCC_TIM3_CLK_ENABLE();
    // Suponiendo reloj a 16MHz (HSI). 16,000,000 / 1600 = 10,000 Hz. 10,000 Hz / 50 = 200 Hz -> 5ms
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 1600 - 1;
    htim3.Init.Period = 50 - 1;
    HAL_TIM_Base_Init(&htim3);
    HAL_NVIC_SetPriority(TIM3_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

// Timer de baja frecuencia para el LED de estado (2 Hz)
void init_blinky_timer(void)
{
    __HAL_RCC_TIM4_CLK_ENABLE();
    // Suponiendo reloj a 16MHz. 16,000,000 / 16000 = 1,000 Hz. 1,000 Hz / 500 = 2 Hz -> 500ms
    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 16000 - 1;
    htim4.Init.Period = 500 - 1;
    HAL_TIM_Base_Init(&htim4);
    HAL_NVIC_SetPriority(TIM4_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
}

// Muestra un número en el display de 7 segmentos (Ánodo Común)
void lightNumber(uint8_t number)
{
    // Primero, apagar todos los segmentos (poner a SET para ánodo común)
    HAL_GPIO_WritePin(GPIO_A_PORT, GPIO_A_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_B_PORT, GPIO_B_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_C_PORT, GPIO_C_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_D_PORT, GPIO_D_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_E_PORT, GPIO_E_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_F_PORT, GPIO_F_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_G_PORT, GPIO_G_PIN, GPIO_PIN_SET);

    // Encender los segmentos necesarios para cada número (poner a RESET)
    switch (number)
    {
    case 0:
        HAL_GPIO_WritePin(GPIO_A_PORT, GPIO_A_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIO_B_PORT, GPIO_B_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIO_C_PORT, GPIO_C_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIO_D_PORT, GPIO_D_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIO_E_PORT, GPIO_E_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIO_F_PORT, GPIO_F_PIN, GPIO_PIN_RESET);
        break;
    case 1:
        HAL_GPIO_WritePin(GPIO_B_PORT, GPIO_B_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIO_C_PORT, GPIO_C_PIN, GPIO_PIN_RESET);
        break;
    case 2:
        HAL_GPIO_WritePin(GPIO_A_PORT, GPIO_A_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIO_B_PORT, GPIO_B_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIO_D_PORT, GPIO_D_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIO_E_PORT, GPIO_E_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIO_G_PORT, GPIO_G_PIN, GPIO_PIN_RESET);
        break;
    case 3:
        HAL_GPIO_WritePin(GPIO_A_PORT, GPIO_A_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIO_B_PORT, GPIO_B_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIO_C_PORT, GPIO_C_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIO_D_PORT, GPIO_D_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIO_G_PORT, GPIO_G_PIN, GPIO_PIN_RESET);
        break;
    case 4:
        HAL_GPIO_WritePin(GPIO_B_PORT, GPIO_B_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIO_C_PORT, GPIO_C_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIO_F_PORT, GPIO_F_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIO_G_PORT, GPIO_G_PIN, GPIO_PIN_RESET);
        break;
    case 5:
        HAL_GPIO_WritePin(GPIO_A_PORT, GPIO_A_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIO_C_PORT, GPIO_C_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIO_D_PORT, GPIO_D_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIO_F_PORT, GPIO_F_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIO_G_PORT, GPIO_G_PIN, GPIO_PIN_RESET);
        break;
    case 6:
        HAL_GPIO_WritePin(GPIO_A_PORT, GPIO_A_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIO_C_PORT, GPIO_C_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIO_D_PORT, GPIO_D_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIO_E_PORT, GPIO_E_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIO_F_PORT, GPIO_F_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIO_G_PORT, GPIO_G_PIN, GPIO_PIN_RESET);
        break;
    case 7:
        HAL_GPIO_WritePin(GPIO_A_PORT, GPIO_A_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIO_B_PORT, GPIO_B_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIO_C_PORT, GPIO_C_PIN, GPIO_PIN_RESET);
        break;
    case 8:
        HAL_GPIO_WritePin(GPIO_A_PORT, GPIO_A_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIO_B_PORT, GPIO_B_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIO_C_PORT, GPIO_C_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIO_D_PORT, GPIO_D_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIO_E_PORT, GPIO_E_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIO_F_PORT, GPIO_F_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIO_G_PORT, GPIO_G_PIN, GPIO_PIN_RESET);
        break;
    case 9:
        HAL_GPIO_WritePin(GPIO_A_PORT, GPIO_A_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIO_B_PORT, GPIO_B_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIO_C_PORT, GPIO_C_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIO_D_PORT, GPIO_D_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIO_F_PORT, GPIO_F_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIO_G_PORT, GPIO_G_PIN, GPIO_PIN_RESET);
        break;
    }
}

// --- MANEJADORES DE INTERRUPCIONES (IRQ HANDLERS) ---
// NOTA: STM32CubeIDE usualmente coloca estas funciones en el archivo stm32f4xx_it.c
void TIM3_IRQHandler(void) { HAL_TIM_IRQHandler(&htim3); }
void TIM4_IRQHandler(void) { HAL_TIM_IRQHandler(&htim4); }
void EXTI2_IRQHandler(void) { HAL_GPIO_EXTI_IRQHandler(ENCODER_PIN_CLK); }
void EXTI15_10_IRQHandler(void)
{
    // Manejar múltiples fuentes de interrupción en la misma línea
    if (__HAL_GPIO_EXTI_GET_IT(ENCODER_PIN_SW) != RESET)
    {
        HAL_GPIO_EXTI_IRQHandler(ENCODER_PIN_SW);
    }
}

// --- FUNCIONES DE CALLBACK ---

// Se llama cuando cualquier timer configurado en modo interrupción termina su cuenta
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
        HAL_GPIO_TogglePin(LED_PH1_PORT, LED_PH1_PIN);
    }
}

// Se llama cuando se dispara cualquier interrupción externa (pines del encoder)
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    static uint32_t last_button_press_time = 0;

    if (GPIO_Pin == ENCODER_PIN_CLK)
    {
        ENCODER_FLAG = 1;
    }

    if (GPIO_Pin == ENCODER_PIN_SW)
    {
        // Anti-rebotes (debounce): solo procesar si han pasado > 200ms
        if (HAL_GetTick() - last_button_press_time > 200)
        {
            BUTTON_FLAG = 1;
            last_button_press_time = HAL_GetTick();
        }
    }
}
