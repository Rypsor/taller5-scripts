#include "stm32f4xx.h"

// --- SOLUCIÓN 2: Declarar el handle del temporizador como GLOBAL ---
TIM_HandleTypeDef handleTim3;
static int period = 25;
// Prototipo de la función de inicialización
void init_hardware(void);

int main(void){
    // --- SOLUCIÓN 1: Añadir HAL_Init() ---
    HAL_Init();
    // En un proyecto real, aquí también se configura el reloj del sistema.
    // SystemClock_Config();

    init_hardware();

    // El bucle principal ahora está vacío.
    // ¡Todo el trabajo lo hace la interrupción en segundo plano!
    while(1){
    }
    return 0;
}

void init_hardware(void){
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();

    // Configuración del Pin PA5 como Salida (LED)
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Configuración del Temporizador TIM3
    handleTim3.Instance = TIM3;
    // Cálculo para un parpadeo cada 250 ms (frecuencia de interrupción de 4 Hz)
    // Suponiendo un reloj de 84 MHz para los Timers en la Nucleo-F411RE
    // F_int = 84,000,000 / ((Prescaler + 1) * (Period + 1))
    // 4 Hz = 84,000,000 / (8400 * 2500)
    handleTim3.Init.Prescaler = 8400 - 1;
    handleTim3.Init.Period = period;
    handleTim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    handleTim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&handleTim3);

    // Habilitar la interrupción en el controlador de interrupciones (NVIC)
    HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);

    // Iniciar el temporizador en modo de interrupción
    HAL_TIM_Base_Start_IT(&handleTim3);
}

// --- SOLUCIÓN 3: Implementar el manejo de interrupción del HAL ---

/**
  * @brief Esta es la rutina de servicio de interrupción (ISR) para TIM3.
  * Su nombre está predefinido y no se puede cambiar.
  */
void TIM3_IRQHandler(void){
    // Cedemos el control al manejador genérico del HAL.
    HAL_TIM_IRQHandler(&handleTim3);
}

/**
  * @brief Esta es la función "callback" que es llamada por el manejador del HAL
  * cuando el contador del temporizador llega al final de su período.
  * @param htim: Puntero al handle del temporizador que generó la interrupción.
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
    // Es una buena práctica verificar que la interrupción viene del temporizador que esperamos.
    if (htim->Instance == TIM3) {
        // Aquí va tu código de aplicación: ¡hacer parpadear el LED!
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        period+= 10;
        __HAL_TIM_SET_AUTORELOAD(htim, period);
        
    }
}
