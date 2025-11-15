#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "stm32f4xx_it.h"

// ==== LED PH1 ====
#define LED_PH1_PORT          GPIOH   // <<-- NUEVO
#define LED_PH1_PIN           GPIO_PIN_1 // <<-- NUEVO

// ==== DIGITOS (D1-D4) ====
#define GPIO_UNIDADES         GPIOB
#define GPIO_PIN_UNIDADES     GPIO_PIN_7

#define GPIO_DECENAS          GPIOC
#define GPIO_PIN_DECENAS      GPIO_PIN_6

#define GPIO_CENTENAS         GPIOC
#define GPIO_PIN_CENTENAS     GPIO_PIN_5

#define GPIO_UNIDADES_MIL     GPIOC
#define GPIO_PIN_UNIDADES_MIL GPIO_PIN_13

#define ENCODER_SW_PORT      GPIOB   
#define ENCODER_PIN_SW       GPIO_PIN_15

// =======================================================================
// ASIGNACIÓN DE PINES DE SEGMENTOS
// =======================================================================
#define GPIO_A_PORT          GPIOB
#define GPIO_A_PIN           GPIO_PIN_12
#define GPIO_B_PORT          GPIOA
#define GPIO_B_PIN           GPIO_PIN_12
#define GPIO_C_PORT          GPIOC
#define GPIO_C_PIN           GPIO_PIN_11
#define GPIO_D_PORT          GPIOC
#define GPIO_D_PIN           GPIO_PIN_12
#define GPIO_E_PORT          GPIOC
#define GPIO_E_PIN           GPIO_PIN_10
#define GPIO_F_PORT          GPIOA
#define GPIO_F_PIN           GPIO_PIN_11
#define GPIO_G_PORT          GPIOD
#define GPIO_G_PIN           GPIO_PIN_2
// =======================================================================

// ==== ENCODER (CLK y DATA) ====
#define ENCODER_CLK_PORT     GPIOB
#define ENCODER_PIN_CLK      GPIO_PIN_2

#define ENCODER_DATA_PORT    GPIOB
#define ENCODER_PIN_DATA     GPIO_PIN_1


// Definir variables
uint8_t TIMER_ADVISORY_FLAG = 0;
uint8_t EXTI_AVISORY_FLAG = 0;
uint8_t BUTTON_FLAG = 0;             
uint8_t display_mode = 0;

uint16_t Tarifa = 1999;
uint8_t Unidad_mil = 0;
uint8_t Centenas = 0;
uint8_t Decenas = 0;
uint8_t Unidades = 0;

TIM_HandleTypeDef htim3;

// Prototipos de funciones
void init_timer(void);
void init_display_pins(void);
void init_encoder_pins(void);
void lightNumber(uint8_t number);

void lightSingleSegment(uint8_t segment_index);
void christmasPattern1(void);
void christmasPattern2(void);

int main (void){
    HAL_Init();
    // Importante: Asegúrate de que la configuración de reloj del sistema (SystemClock_Config)
    // NO está habilitando el HSE. Si usas la configuración por defecto (HSI), funcionará bien.
    init_display_pins();
    init_encoder_pins();
    init_timer();

    Unidad_mil = Tarifa / 1000;
    Centenas = (Tarifa % 1000) / 100;
    Decenas = (Tarifa % 100) / 10;
    Unidades = Tarifa % 10;

    while (1){
    // --- MANEJO DEL PULSADOR (CAMBIO DE MODO) ---
    // Verificamos si la bandera del botón ha sido activada por la interrupción.
    if (BUTTON_FLAG)
    {
        BUTTON_FLAG = 0; // Es crucial limpiar la bandera para no procesarla de nuevo.
        
        display_mode++;  // Avanzamos al siguiente modo de visualización.

        // Si hemos pasado el último modo (que es el 2), volvemos al principio (modo 0).
        if (display_mode > 2) 
        {
            display_mode = 0;
        }

        // Si justo acabamos de volver al modo normal, recalculamos los dígitos
        // del número 'Tarifa' para asegurarnos de que el display muestre el valor correcto.
        if (display_mode == 0) {
            Unidad_mil = Tarifa / 1000;
            Centenas = (Tarifa % 1000) / 100;
            Decenas = (Tarifa % 100) / 10;
            Unidades = Tarifa % 10;
        }
    }
    
    // --- MANEJO DEL DISPLAY (CONTROLADO POR LA INTERRUPCIÓN DEL TIMER) ---
    // Este bloque se ejecuta periódicamente gracias al Timer 3 (cada 5ms en tu config).
    if (TIMER_ADVISORY_FLAG)
    {
        TIMER_ADVISORY_FLAG = 0; // Limpiamos la bandera del timer.

        // Apagamos todos los transistores de los dígitos antes de encender el siguiente.
        // Esto evita el efecto "ghosting" (dígitos fantasma).
        HAL_GPIO_WritePin(GPIO_UNIDADES_MIL, GPIO_PIN_UNIDADES_MIL, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIO_CENTENAS, GPIO_PIN_CENTENAS, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIO_DECENAS, GPIO_PIN_DECENAS, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIO_UNIDADES, GPIO_PIN_UNIDADES, GPIO_PIN_SET);

        // Usamos una estructura switch para decidir qué se va a mostrar en el display
        // basándonos en el modo actual.
        switch(display_mode)
        {
            case 0: // MODO NORMAL: Mostrar el número de la variable 'Tarifa'.
                ; // Un case en C no puede estar vacío, se añade un ';' para solucionarlo.
                static uint8_t contador_digito = 0; // Variable estática para recordar qué dígito mostrar.
                switch(contador_digito){
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
                if (++contador_digito > 3) contador_digito = 0; // Pasamos al siguiente dígito.
                break;

            case 1: // MODO NAVIDEÑO 1: Llama a la función de la primera animación.
                christmasPattern1();
                break;
            
            case 2: // MODO NAVIDEÑO 2: Llama a la función de la segunda animación.
                christmasPattern2();
                break;
        }
    }

    // --- MANEJO DEL GIRO DEL ENCODER (CONTROLADO POR INTERRUPCIÓN EXTERNA) ---
    // Este bloque solo se ejecuta si la interrupción del pin CLK ha ocurrido.
    // (Nota: La lógica en la interrupción ahora evita que esta bandera se active
    // si no estamos en el modo 0).
    if (EXTI_AVISORY_FLAG)
    {
        // Si el pin DATA está en BAJO, decrementamos.
        if (HAL_GPIO_ReadPin(ENCODER_DATA_PORT, ENCODER_PIN_DATA) == GPIO_PIN_RESET) {
            if (Tarifa-- == 0) Tarifa = 9999;
        } 
        // Si el pin DATA está en ALTO, incrementamos.
        else {
            if (++Tarifa > 9999) Tarifa = 0;
        }

        // Actualizamos las variables de los dígitos con el nuevo valor de 'Tarifa'.
        Unidad_mil = Tarifa / 1000;
        Centenas = (Tarifa % 1000) / 100;
        Decenas = (Tarifa % 100) / 10;
        Unidades = Tarifa % 10;

        EXTI_AVISORY_FLAG = 0; // Limpiamos la bandera del encoder.
    }
}
    return 0;
}

void init_display_pins(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE(); // <<-- NUEVO: Habilitar reloj para el Puerto H

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    // Configuración de pines de segmentos
    GPIO_InitStruct.Pin = GPIO_A_PIN; HAL_GPIO_Init(GPIO_A_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_B_PIN; HAL_GPIO_Init(GPIO_B_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_C_PIN; HAL_GPIO_Init(GPIO_C_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_D_PIN; HAL_GPIO_Init(GPIO_D_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_E_PIN; HAL_GPIO_Init(GPIO_E_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_F_PIN; HAL_GPIO_Init(GPIO_F_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_G_PIN; HAL_GPIO_Init(GPIO_G_PORT, &GPIO_InitStruct);

    // Configuración de pines de control de dígitos
    GPIO_InitStruct.Pin = GPIO_PIN_UNIDADES; HAL_GPIO_Init(GPIO_UNIDADES, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_PIN_DECENAS; HAL_GPIO_Init(GPIO_DECENAS, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_PIN_CENTENAS; HAL_GPIO_Init(GPIO_CENTENAS, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_PIN_UNIDADES_MIL; HAL_GPIO_Init(GPIO_UNIDADES_MIL, &GPIO_InitStruct);

    // <<-- NUEVO: Configuración del pin del LED PH1
    GPIO_InitStruct.Pin = LED_PH1_PIN;
    HAL_GPIO_Init(LED_PH1_PORT, &GPIO_InitStruct);
    // <<-- FIN DE LO NUEVO

    // Apagar todos los dígitos al inicio
    HAL_GPIO_WritePin(GPIO_UNIDADES_MIL, GPIO_PIN_UNIDADES_MIL, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_CENTENAS, GPIO_PIN_CENTENAS, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_DECENAS, GPIO_PIN_DECENAS, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_UNIDADES, GPIO_PIN_UNIDADES, GPIO_PIN_SET);
}

void init_encoder_pins(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // --- Configuración existente para los pines de rotación (CLK y DATA) ---
    // CLK en PB2 como interrupción
    GPIO_InitStruct.Pin = ENCODER_PIN_CLK;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(ENCODER_CLK_PORT, &GPIO_InitStruct);

    // DATA en PB1 como entrada normal
    GPIO_InitStruct.Pin = ENCODER_PIN_DATA;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(ENCODER_DATA_PORT, &GPIO_InitStruct);

    // --- Configuración del pulsador (SW) en PC3 --- // <<-- NUEVO
    GPIO_InitStruct.Pin = ENCODER_PIN_SW;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING; // Interrupción por flanco de bajada
    GPIO_InitStruct.Pull = GPIO_PULLUP;           // Usamos la resistencia interna a VCC
    HAL_GPIO_Init(ENCODER_SW_PORT, &GPIO_InitStruct);

    // --- Habilitación de las interrupciones en el NVIC ---
    // Habilita la interrupción para la línea 2 (usada por PB2)
    HAL_NVIC_SetPriority(EXTI2_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI2_IRQn);

    // Habilita la interrupción para la línea 3 (usada por PC3) 
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);         
}
void lightNumber(uint8_t number) {
    HAL_GPIO_WritePin(GPIO_A_PORT, GPIO_A_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_B_PORT, GPIO_B_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_C_PORT, GPIO_C_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_D_PORT, GPIO_D_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_E_PORT, GPIO_E_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_F_PORT, GPIO_F_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_G_PORT, GPIO_G_PIN, GPIO_PIN_SET);

    switch (number) {
        case 0: HAL_GPIO_WritePin(GPIO_A_PORT, GPIO_A_PIN, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIO_B_PORT, GPIO_B_PIN, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIO_C_PORT, GPIO_C_PIN, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIO_D_PORT, GPIO_D_PIN, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIO_E_PORT, GPIO_E_PIN, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIO_F_PORT, GPIO_F_PIN, GPIO_PIN_RESET); break;
        case 1: HAL_GPIO_WritePin(GPIO_B_PORT, GPIO_B_PIN, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIO_C_PORT, GPIO_C_PIN, GPIO_PIN_RESET); break;
        case 2: HAL_GPIO_WritePin(GPIO_A_PORT, GPIO_A_PIN, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIO_B_PORT, GPIO_B_PIN, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIO_D_PORT, GPIO_D_PIN, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIO_E_PORT, GPIO_E_PIN, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIO_G_PORT, GPIO_G_PIN, GPIO_PIN_RESET); break;
        case 3: HAL_GPIO_WritePin(GPIO_A_PORT, GPIO_A_PIN, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIO_B_PORT, GPIO_B_PIN, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIO_C_PORT, GPIO_C_PIN, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIO_D_PORT, GPIO_D_PIN, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIO_G_PORT, GPIO_G_PIN, GPIO_PIN_RESET); break;
        case 4: HAL_GPIO_WritePin(GPIO_B_PORT, GPIO_B_PIN, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIO_C_PORT, GPIO_C_PIN, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIO_F_PORT, GPIO_F_PIN, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIO_G_PORT, GPIO_G_PIN, GPIO_PIN_RESET); break;
        case 5: HAL_GPIO_WritePin(GPIO_A_PORT, GPIO_A_PIN, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIO_C_PORT, GPIO_C_PIN, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIO_D_PORT, GPIO_D_PIN, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIO_F_PORT, GPIO_F_PIN, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIO_G_PORT, GPIO_G_PIN, GPIO_PIN_RESET); break;
        case 6: HAL_GPIO_WritePin(GPIO_A_PORT, GPIO_A_PIN, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIO_C_PORT, GPIO_C_PIN, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIO_D_PORT, GPIO_D_PIN, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIO_E_PORT, GPIO_E_PIN, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIO_F_PORT, GPIO_F_PIN, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIO_G_PORT, GPIO_G_PIN, GPIO_PIN_RESET); break;
        case 7: HAL_GPIO_WritePin(GPIO_A_PORT, GPIO_A_PIN, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIO_B_PORT, GPIO_B_PIN, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIO_C_PORT, GPIO_C_PIN, GPIO_PIN_RESET); break;
        case 8: HAL_GPIO_WritePin(GPIO_A_PORT, GPIO_A_PIN, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIO_B_PORT, GPIO_B_PIN, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIO_C_PORT, GPIO_C_PIN, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIO_D_PORT, GPIO_D_PIN, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIO_E_PORT, GPIO_E_PIN, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIO_F_PORT, GPIO_F_PIN, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIO_G_PORT, GPIO_G_PIN, GPIO_PIN_RESET); break;
        case 9: HAL_GPIO_WritePin(GPIO_A_PORT, GPIO_A_PIN, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIO_B_PORT, GPIO_B_PIN, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIO_C_PORT, GPIO_C_PIN, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIO_D_PORT, GPIO_D_PIN, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIO_F_PORT, GPIO_F_PIN, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIO_G_PORT, GPIO_G_PIN, GPIO_PIN_RESET); break;
        default: break;
    }
}

// <<-- INICIO DE CÓDIGO NUEVO -->>

// Enciende un único segmento en TODOS los dígitos a la vez
void lightSingleSegment(uint8_t segment_index) {
    // Apaga todos primero
    HAL_GPIO_WritePin(GPIO_A_PORT, GPIO_A_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_B_PORT, GPIO_B_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_C_PORT, GPIO_C_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_D_PORT, GPIO_D_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_E_PORT, GPIO_E_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_F_PORT, GPIO_F_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_G_PORT, GPIO_G_PIN, GPIO_PIN_SET);
    
    switch (segment_index) {
        case 0: HAL_GPIO_WritePin(GPIO_A_PORT, GPIO_A_PIN, GPIO_PIN_RESET); break; // A
        case 1: HAL_GPIO_WritePin(GPIO_B_PORT, GPIO_B_PIN, GPIO_PIN_RESET); break; // B
        case 2: HAL_GPIO_WritePin(GPIO_C_PORT, GPIO_C_PIN, GPIO_PIN_RESET); break; // C
        case 3: HAL_GPIO_WritePin(GPIO_D_PORT, GPIO_D_PIN, GPIO_PIN_RESET); break; // D
        case 4: HAL_GPIO_WritePin(GPIO_E_PORT, GPIO_E_PIN, GPIO_PIN_RESET); break; // E
        case 5: HAL_GPIO_WritePin(GPIO_F_PORT, GPIO_F_PIN, GPIO_PIN_RESET); break; // F
        case 6: HAL_GPIO_WritePin(GPIO_G_PORT, GPIO_G_PIN, GPIO_PIN_RESET); break; // G
    }

    // Enciende todos los dígitos
    HAL_GPIO_WritePin(GPIO_UNIDADES_MIL, GPIO_PIN_UNIDADES_MIL, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIO_CENTENAS, GPIO_PIN_CENTENAS, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIO_DECENAS, GPIO_PIN_DECENAS, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIO_UNIDADES, GPIO_PIN_UNIDADES, GPIO_PIN_RESET);
}

// Patrón 1: "Segmento Giratorio"
void christmasPattern1(void) {
    static uint16_t animation_counter = 0;
    // La interrupción del timer es cada 5ms. Cambiamos cada 20 * 5ms = 100ms
    if (++animation_counter >= 20) {
        animation_counter = 0;
    }
    // Dividimos por 20 para obtener un número entre 0 y 6
    lightSingleSegment(animation_counter / 3);
}

// Patrón 2: "Dígito Persecutor"
void christmasPattern2(void) {
    static uint16_t animation_counter = 0;
    // Cambiamos cada 30 * 5ms = 150ms
    if (++animation_counter >= 120) {
        animation_counter = 0;
    }

    // Encendemos todos los segmentos de un solo dígito
    lightNumber(8); // El 8 enciende todos los segmentos

    uint8_t active_digit = animation_counter / 30; // Nos da un número de 0 a 3

    switch (active_digit) {
        case 0: HAL_GPIO_WritePin(GPIO_UNIDADES, GPIO_PIN_UNIDADES, GPIO_PIN_RESET); break;
        case 1: HAL_GPIO_WritePin(GPIO_DECENAS, GPIO_PIN_DECENAS, GPIO_PIN_RESET); break;
        case 2: HAL_GPIO_WritePin(GPIO_CENTENAS, GPIO_PIN_CENTENAS, GPIO_PIN_RESET); break;
        case 3: HAL_GPIO_WritePin(GPIO_UNIDADES_MIL, GPIO_PIN_UNIDADES_MIL, GPIO_PIN_RESET); break;
    }
}
// <<-- FIN DE CÓDIGO NUEVO -->>


void init_timer(void){
    __HAL_RCC_TIM3_CLK_ENABLE();
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 16000 - 1;
    htim3.Init.Period = 5 - 1;
    HAL_TIM_Base_Init(&htim3);
    HAL_NVIC_SetPriority(TIM3_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
    HAL_TIM_Base_Start_IT(&htim3);
}

void TIM3_IRQHandler(void){
    HAL_TIM_IRQHandler(&htim3);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM3)
    {
        TIMER_ADVISORY_FLAG = 1;

        // <<-- NUEVO: Lógica para parpadear el LED en PH1
        static uint16_t led_counter = 0;
        // La interrupción es cada 5ms. 100 * 5ms = 500ms.
        // El LED cambiará de estado cada medio segundo.
        if (++led_counter >= 100)
        {
            led_counter = 0;
            HAL_GPIO_TogglePin(LED_PH1_PORT, LED_PH1_PIN);
        }
        // <<-- FIN DE LO NUEVO
    }
}

void EXTI2_IRQHandler(void){
    HAL_GPIO_EXTI_IRQHandler(ENCODER_PIN_CLK);
}

void EXTI15_10_IRQHandler(void){ // <--- CAMBIO
    HAL_GPIO_EXTI_IRQHandler(ENCODER_PIN_SW);
}

// REEMPLAZA TU FUNCIÓN ACTUAL CON ESTA
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // Interrupción por giro del encoder
    if(GPIO_Pin == ENCODER_PIN_CLK)
    {
        // Solo procesamos el giro si estamos en modo normal (0)
        if (display_mode == 0) {
            EXTI_AVISORY_FLAG = 1;
        }
    }

    // Interrupción por pulsación del botón
    if(GPIO_Pin == ENCODER_PIN_SW)
    {
        BUTTON_FLAG = 1;
    }
}