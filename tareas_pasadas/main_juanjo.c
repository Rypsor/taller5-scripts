#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"

// DIGITOS (D1-D4)

#define GPIO_UNIDADES GPIOB
#define GPIO_PIN_UNIDADES GPIO_PIN_7

#define GPIO_DECENAS GPIOC
#define GPIO_PIN_DECENAS GPIO_PIN_6

#define GPIO_CENTENAS GPIOC
#define GPIO_PIN_CENTENAS GPIO_PIN_5

#define GPIO_UNIDADES_MIL GPIOC
#define GPIO_PIN_UNIDADES_MIL GPIO_PIN_13

// SEGMENTOS (A-G)
#define GPIO_A_PORT GPIOB
#define GPIO_A_PIN GPIO_PIN_12

#define GPIO_B_PORT GPIOA
#define GPIO_B_PIN GPIO_PIN_12

#define GPIO_C_PORT GPIOC
#define GPIO_C_PIN GPIO_PIN_11

#define GPIO_D_PORT GPIOC
#define GPIO_D_PIN GPIO_PIN_10

#define GPIO_E_PORT GPIOC
#define GPIO_E_PIN GPIO_PIN_12

#define GPIO_F_PORT GPIOA
#define GPIO_F_PIN GPIO_PIN_11

#define GPIO_G_PORT GPIOD
#define GPIO_G_PIN GPIO_PIN_2

// PINS DE CONTROL DE DIGITOS
// ENCODER (CLK, DATA y SW)
#define ENCODER_CLK_PORT GPIOB
#define ENCODER_PIN_CLK GPIO_PIN_2

#define ENCODER_DATA_PORT GPIOB
#define ENCODER_PIN_DATA GPIO_PIN_1

#define ENCODER_SW_PORT GPIOB
#define ENCODER_PIN_SW GPIO_PIN_15

// DIGITOS PARA EL BLINKY
#define LED_BLINKY_PORT GPIOH
#define LED_BLINKY_PIN GPIO_PIN_1


// Definir variables
uint8_t TIMER_ADVISORY_FLAG = 0; // bandera para indicar que el timer ha interrumpido
uint8_t EXTI_AVISORY_FLAG = 0; // bandera para indicar que el EXTI ha interrumpido


//Se inicia en 2105 como prueba
uint16_t Tarifa = 0; // variable que guarda la tarifa actual
uint8_t Unidad_mil = 0;// variable que guarda las unidades de mil
uint8_t Centenas = 0; // variable que guarda las centenas
uint8_t Decenas = 0; // variable que guarda las decenas
uint8_t Unidades = 0;// variable que guarda las unidades


TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

//Definción de las funciones a utilizar
void init_timer(void);
void init_display_pins(void);
void init_encoder_pins(void);
void lightNumber(uint8_t number);


/*
Programa inicial del taxímetro
*/
int main (void){
	HAL_Init(); // inicializar la biblioteca HAL
    init_display_pins(); // inicializar los pines del display
    init_encoder_pins(); // inicializar los pines del encoder
    init_timer(); // inicializar el timer


    Unidad_mil = Tarifa / 1000;
    Centenas = (Tarifa % 1000) / 100;
    Decenas = (Tarifa % 100) / 10;
    Unidades = Tarifa % 10;

    while (1){

        if (TIMER_ADVISORY_FLAG) //trabajo del timer dentro del while
        {
        static uint8_t contador = 0;
        contador++;
        if (contador > 3) contador = 0; // reiniciar el contador cada 3 interrupciones
        HAL_GPIO_WritePin(GPIO_UNIDADES, GPIO_PIN_UNIDADES, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIO_DECENAS, GPIO_PIN_DECENAS, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIO_CENTENAS, GPIO_PIN_CENTENAS, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIO_UNIDADES_MIL, GPIO_PIN_UNIDADES_MIL, GPIO_PIN_SET);

            switch(contador){
        case 0:
            // encender unidades
            lightNumber(Unidades); // llamar a la función para encender el número
            HAL_GPIO_WritePin(GPIO_UNIDADES, GPIO_PIN_UNIDADES, GPIO_PIN_RESET);

            break;
        case 1:
            // encender decenas
            lightNumber(Decenas); // llamar a la función para encender el número
            HAL_GPIO_WritePin(GPIO_DECENAS, GPIO_PIN_DECENAS, GPIO_PIN_RESET);
            break;
        case 2:
            // encender centenas
            lightNumber(Centenas); // llamar a la función para encender el número
            HAL_GPIO_WritePin(GPIO_CENTENAS, GPIO_PIN_CENTENAS, GPIO_PIN_RESET);
            break;
        case 3:
            // encender unidades de mil
            lightNumber(Unidad_mil); // llamar a la función para encender el número
            HAL_GPIO_WritePin(GPIO_UNIDADES_MIL, GPIO_PIN_UNIDADES_MIL, GPIO_PIN_RESET);
            break;
        default:
            break;
    }
            TIMER_ADVISORY_FLAG = 0; // bajar la bandera de interrupción
            }



        if (EXTI_AVISORY_FLAG) //trabajo del EXTI dentro del while
        {
            EXTI_AVISORY_FLAG = 0; // bajar la bandera de interrupción

            //Revisar el estado del pin DT para determinar la dirección del giro
            if (HAL_GPIO_ReadPin(ENCODER_DATA_PORT, ENCODER_PIN_DATA) == GPIO_PIN_SET){
                Tarifa++; // incrementar la tarifa
                if (Tarifa > 4095) Tarifa = 0; // reiniciar la tarifa si supera 4095
            } else {
                if (Tarifa > 0) Tarifa--; // decrementar la tarifa
                else if (Tarifa == 0) Tarifa = 4095; // reiniciar si la tarifa es igual a 0
            }
            // actualizar las variables de las cifras de la tarifa
            Unidad_mil = Tarifa / 1000;
            Centenas = (Tarifa % 1000) / 100;
            Decenas = (Tarifa % 100) / 10;
            Unidades = Tarifa % 10;
            //Dentro de la rutina de servicio de interrupción del EXTI no se debe hacer nada más que subir una bandera
            //y hacer el trabajo pesado en el while principal
        }

    }
    return 0;
}




// Definir pines para el display de 7 segmentos

void init_display_pins(void)
{
    // 1. Habilita los relojes para todos los puertos que usas
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    // Si usas otro puerto (D, E...), agrégalo aquí

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    // 2. Configura los pines de los segmentos A-G
    // Segmento A
    GPIO_InitStruct.Pin = GPIO_A_PIN;
    HAL_GPIO_Init(GPIO_A_PORT, &GPIO_InitStruct);

    // Segmento B
    GPIO_InitStruct.Pin = GPIO_B_PIN;
    HAL_GPIO_Init(GPIO_B_PORT, &GPIO_InitStruct);

    // Segmento C
    GPIO_InitStruct.Pin = GPIO_C_PIN;
    HAL_GPIO_Init(GPIO_C_PORT, &GPIO_InitStruct);

    // Segmento D
    GPIO_InitStruct.Pin = GPIO_D_PIN;
    HAL_GPIO_Init(GPIO_D_PORT, &GPIO_InitStruct);

    // Segmento E
    GPIO_InitStruct.Pin = GPIO_E_PIN;
    HAL_GPIO_Init(GPIO_E_PORT, &GPIO_InitStruct);

    // Segmento F
    GPIO_InitStruct.Pin = GPIO_F_PIN;
    HAL_GPIO_Init(GPIO_F_PORT, &GPIO_InitStruct);

    // Segmento G
    GPIO_InitStruct.Pin = GPIO_G_PIN;
    HAL_GPIO_Init(GPIO_G_PORT, &GPIO_InitStruct);

    // 3. Configura los pines de los dígitos
    // Unidades
    GPIO_InitStruct.Pin = GPIO_PIN_UNIDADES;
    HAL_GPIO_Init(GPIO_UNIDADES, &GPIO_InitStruct);

    // Decenas
    GPIO_InitStruct.Pin = GPIO_PIN_DECENAS;
    HAL_GPIO_Init(GPIO_DECENAS, &GPIO_InitStruct);

    // Centenas
    GPIO_InitStruct.Pin = GPIO_PIN_CENTENAS;
    HAL_GPIO_Init(GPIO_CENTENAS, &GPIO_InitStruct);

    // Unidades de mil
    GPIO_InitStruct.Pin = GPIO_PIN_UNIDADES_MIL;
    HAL_GPIO_Init(GPIO_UNIDADES_MIL, &GPIO_InitStruct);

    // 4. Configuración del Blinky
    GPIO_InitStruct.Pin = LED_BLINKY_PIN;
    HAL_GPIO_Init(LED_BLINKY_PORT, &GPIO_InitStruct);

    // 5. Inicializa todos los pines en estado bajo (apagado)
    HAL_GPIO_WritePin(GPIO_A_PORT, GPIO_A_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIO_B_PORT, GPIO_B_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_C_PORT, GPIO_C_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_D_PORT, GPIO_D_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_E_PORT, GPIO_E_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_F_PORT, GPIO_F_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_G_PORT, GPIO_G_PIN, GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIO_UNIDADES_MIL, GPIO_PIN_UNIDADES_MIL, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_CENTENAS, GPIO_PIN_CENTENAS, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_DECENAS, GPIO_PIN_DECENAS, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_UNIDADES, GPIO_PIN_UNIDADES, GPIO_PIN_SET);

}

void init_encoder_pins(void)
{

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // 2. Configura el pin CLK como entrada con interrupción (EXTI)
    GPIO_InitStruct.Pin = ENCODER_PIN_CLK;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ENCODER_CLK_PORT, &GPIO_InitStruct);

    // 3. Configura el pin DATA como entrada normal
    GPIO_InitStruct.Pin = ENCODER_PIN_DATA;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ENCODER_DATA_PORT, &GPIO_InitStruct);

    // 4. Configura el pin SW como entrada con interrupción (EXTI)
    GPIO_InitStruct.Pin = ENCODER_PIN_SW;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ENCODER_SW_PORT, &GPIO_InitStruct);


    // 4. Configura la prioridad y habilita la interrupción en el NVIC
    HAL_NVIC_EnableIRQ(EXTI2_IRQn);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}



// Función para encender un número en el display de 7 segmentos
// Recibe un número del 0 al 9 y enciende los segmentos correspondientes
void lightNumber(uint8_t number) {
    // Definiciones de pines para un display de 7 segmentos (a-g)
    // se puede reducir el codigo apagando todos los segmentos primero y luego encendiendo unicamente
    // los necesarios pero se hizo así por claridad

    switch (number) {
        case 0: // Muestra el número 0
            HAL_GPIO_WritePin(GPIO_A_PORT, GPIO_A_PIN, GPIO_PIN_RESET); // a
            HAL_GPIO_WritePin(GPIO_B_PORT, GPIO_B_PIN, GPIO_PIN_RESET); // b
            HAL_GPIO_WritePin(GPIO_C_PORT, GPIO_C_PIN, GPIO_PIN_RESET); // c
            HAL_GPIO_WritePin(GPIO_D_PORT, GPIO_D_PIN, GPIO_PIN_RESET); // d
            HAL_GPIO_WritePin(GPIO_E_PORT, GPIO_E_PIN, GPIO_PIN_RESET); // e
            HAL_GPIO_WritePin(GPIO_F_PORT, GPIO_F_PIN, GPIO_PIN_RESET); // f
            HAL_GPIO_WritePin(GPIO_G_PORT, GPIO_G_PIN, GPIO_PIN_SET); // g (apagado)
            break;

        case 1: // Muestra el número 1
            HAL_GPIO_WritePin(GPIO_A_PORT, GPIO_A_PIN, GPIO_PIN_SET); // a (apagado)
            HAL_GPIO_WritePin(GPIO_B_PORT, GPIO_B_PIN, GPIO_PIN_RESET); // b
            HAL_GPIO_WritePin(GPIO_C_PORT, GPIO_C_PIN, GPIO_PIN_RESET); // c
            HAL_GPIO_WritePin(GPIO_D_PORT, GPIO_D_PIN, GPIO_PIN_SET); // d (apagado)
            HAL_GPIO_WritePin(GPIO_E_PORT, GPIO_E_PIN, GPIO_PIN_SET); // e (apagado)
            HAL_GPIO_WritePin(GPIO_F_PORT, GPIO_F_PIN, GPIO_PIN_SET); // f (apagado)
            HAL_GPIO_WritePin(GPIO_G_PORT, GPIO_G_PIN, GPIO_PIN_SET); // g (apagado)
            break;

        case 2: // Muestra el número 2
            HAL_GPIO_WritePin(GPIO_A_PORT, GPIO_A_PIN, GPIO_PIN_RESET); // a
            HAL_GPIO_WritePin(GPIO_B_PORT, GPIO_B_PIN, GPIO_PIN_RESET); // b
            HAL_GPIO_WritePin(GPIO_C_PORT, GPIO_C_PIN, GPIO_PIN_SET); // c (apagado)
            HAL_GPIO_WritePin(GPIO_D_PORT, GPIO_D_PIN, GPIO_PIN_RESET); // d
            HAL_GPIO_WritePin(GPIO_E_PORT, GPIO_E_PIN, GPIO_PIN_RESET); // e
            HAL_GPIO_WritePin(GPIO_F_PORT, GPIO_F_PIN, GPIO_PIN_SET); // f (apagado)
            HAL_GPIO_WritePin(GPIO_G_PORT, GPIO_G_PIN, GPIO_PIN_RESET); // g
            break;

        case 3: // Muestra el número 3
            HAL_GPIO_WritePin(GPIO_A_PORT, GPIO_A_PIN, GPIO_PIN_RESET); // a
            HAL_GPIO_WritePin(GPIO_B_PORT, GPIO_B_PIN, GPIO_PIN_RESET); // b
            HAL_GPIO_WritePin(GPIO_C_PORT, GPIO_C_PIN, GPIO_PIN_RESET); // c
            HAL_GPIO_WritePin(GPIO_D_PORT, GPIO_D_PIN, GPIO_PIN_RESET); // d
            HAL_GPIO_WritePin(GPIO_E_PORT, GPIO_E_PIN, GPIO_PIN_SET); // e (apagado)
            HAL_GPIO_WritePin(GPIO_F_PORT, GPIO_F_PIN, GPIO_PIN_SET); // f (apagado)
            HAL_GPIO_WritePin(GPIO_G_PORT, GPIO_G_PIN, GPIO_PIN_RESET); // g
            break;

        case 4: // Muestra el número 4
            HAL_GPIO_WritePin(GPIO_A_PORT, GPIO_A_PIN, GPIO_PIN_SET); // a (apagado)
            HAL_GPIO_WritePin(GPIO_B_PORT, GPIO_B_PIN, GPIO_PIN_RESET); // b
            HAL_GPIO_WritePin(GPIO_C_PORT, GPIO_C_PIN, GPIO_PIN_RESET); // c
            HAL_GPIO_WritePin(GPIO_D_PORT, GPIO_D_PIN, GPIO_PIN_SET); // d (apagado)
            HAL_GPIO_WritePin(GPIO_E_PORT, GPIO_E_PIN, GPIO_PIN_SET); // e (apagado)
            HAL_GPIO_WritePin(GPIO_F_PORT, GPIO_F_PIN, GPIO_PIN_RESET); // f
            HAL_GPIO_WritePin(GPIO_G_PORT, GPIO_G_PIN, GPIO_PIN_RESET); // g
            break;

        case 5: // Muestra el número 5
            HAL_GPIO_WritePin(GPIO_A_PORT, GPIO_A_PIN, GPIO_PIN_RESET); // a
            HAL_GPIO_WritePin(GPIO_B_PORT, GPIO_B_PIN, GPIO_PIN_SET); // b (apagado)
            HAL_GPIO_WritePin(GPIO_C_PORT, GPIO_C_PIN, GPIO_PIN_RESET); // c
            HAL_GPIO_WritePin(GPIO_D_PORT, GPIO_D_PIN, GPIO_PIN_RESET); // d
            HAL_GPIO_WritePin(GPIO_E_PORT, GPIO_E_PIN, GPIO_PIN_SET); // e (apagado)
            HAL_GPIO_WritePin(GPIO_F_PORT, GPIO_F_PIN, GPIO_PIN_RESET); // f
            HAL_GPIO_WritePin(GPIO_G_PORT, GPIO_G_PIN, GPIO_PIN_RESET); // g
            break;

        case 6: // Muestra el número 6
            HAL_GPIO_WritePin(GPIO_A_PORT, GPIO_A_PIN, GPIO_PIN_RESET); // a
            HAL_GPIO_WritePin(GPIO_B_PORT, GPIO_B_PIN, GPIO_PIN_SET); // b (apagado)
            HAL_GPIO_WritePin(GPIO_C_PORT, GPIO_C_PIN, GPIO_PIN_RESET); // c
            HAL_GPIO_WritePin(GPIO_D_PORT, GPIO_D_PIN, GPIO_PIN_RESET); // d
            HAL_GPIO_WritePin(GPIO_E_PORT, GPIO_E_PIN, GPIO_PIN_RESET); // e
            HAL_GPIO_WritePin(GPIO_F_PORT, GPIO_F_PIN, GPIO_PIN_RESET); // f
            HAL_GPIO_WritePin(GPIO_G_PORT, GPIO_G_PIN, GPIO_PIN_RESET); // g
            break;

        case 7: // Muestra el número 7
            HAL_GPIO_WritePin(GPIO_A_PORT, GPIO_A_PIN, GPIO_PIN_RESET); // a
            HAL_GPIO_WritePin(GPIO_B_PORT, GPIO_B_PIN, GPIO_PIN_RESET); // b
            HAL_GPIO_WritePin(GPIO_C_PORT, GPIO_C_PIN, GPIO_PIN_RESET); // c
            HAL_GPIO_WritePin(GPIO_D_PORT, GPIO_D_PIN, GPIO_PIN_SET); // d (apagado)
            HAL_GPIO_WritePin(GPIO_E_PORT, GPIO_E_PIN, GPIO_PIN_SET); // e (apagado)
            HAL_GPIO_WritePin(GPIO_F_PORT, GPIO_F_PIN, GPIO_PIN_SET); // f (apagado)
            HAL_GPIO_WritePin(GPIO_G_PORT, GPIO_G_PIN, GPIO_PIN_SET); // g (apagado)
            break;

        case 8: // Muestra el número 8
            HAL_GPIO_WritePin(GPIO_A_PORT, GPIO_A_PIN, GPIO_PIN_RESET); // a
            HAL_GPIO_WritePin(GPIO_B_PORT, GPIO_B_PIN, GPIO_PIN_RESET); // b
            HAL_GPIO_WritePin(GPIO_C_PORT, GPIO_C_PIN, GPIO_PIN_RESET); // c
            HAL_GPIO_WritePin(GPIO_D_PORT, GPIO_D_PIN, GPIO_PIN_RESET); // d
            HAL_GPIO_WritePin(GPIO_E_PORT, GPIO_E_PIN, GPIO_PIN_RESET); // e
            HAL_GPIO_WritePin(GPIO_F_PORT, GPIO_F_PIN, GPIO_PIN_RESET); // f
            HAL_GPIO_WritePin(GPIO_G_PORT, GPIO_G_PIN, GPIO_PIN_RESET); // g
            break;

        case 9: // Muestra el número 9
            HAL_GPIO_WritePin(GPIO_A_PORT, GPIO_A_PIN, GPIO_PIN_RESET); // a
            HAL_GPIO_WritePin(GPIO_B_PORT, GPIO_B_PIN, GPIO_PIN_RESET); // b
            HAL_GPIO_WritePin(GPIO_C_PORT, GPIO_C_PIN, GPIO_PIN_RESET); // c
            HAL_GPIO_WritePin(GPIO_D_PORT, GPIO_D_PIN, GPIO_PIN_RESET); // d
            HAL_GPIO_WritePin(GPIO_E_PORT, GPIO_E_PIN, GPIO_PIN_SET); // e (apagado)
            HAL_GPIO_WritePin(GPIO_F_PORT, GPIO_F_PIN, GPIO_PIN_RESET); // f
            HAL_GPIO_WritePin(GPIO_G_PORT, GPIO_G_PIN, GPIO_PIN_RESET); // g
            break;

        default: // Apaga todos los segmentos excepto el g (para indicar error)
            HAL_GPIO_WritePin(GPIO_A_PORT, GPIO_A_PIN, GPIO_PIN_SET); // a (apagado)
            HAL_GPIO_WritePin(GPIO_B_PORT, GPIO_B_PIN, GPIO_PIN_SET); // b (apagado)
            HAL_GPIO_WritePin(GPIO_C_PORT, GPIO_C_PIN, GPIO_PIN_SET); // c (apagado)
            HAL_GPIO_WritePin(GPIO_D_PORT, GPIO_D_PIN, GPIO_PIN_SET); // d (apagado)
            HAL_GPIO_WritePin(GPIO_E_PORT, GPIO_E_PIN, GPIO_PIN_SET); // e (apagado)
            HAL_GPIO_WritePin(GPIO_F_PORT, GPIO_F_PIN, GPIO_PIN_SET); // f (apagado)
            HAL_GPIO_WritePin(GPIO_G_PORT, GPIO_G_PIN, GPIO_PIN_RESET); //g
            break;
    }
}

// timer que controla el encendido del siete segmentos
// configuración general del timer
void init_timer(void){
    // Habilitar relojes de los timers 3 y 2
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 16000 - 1; // Prescaler para 1 kHz
    htim3.Init.Period = 5; // Valor del periodo para visualizar correctamente
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP; // Contar hacia arriba
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_Base_Init(&htim3); // Inicializar el timer 3
    HAL_NVIC_EnableIRQ(TIM3_IRQn); // Habilitar interrupción del timer 3
    HAL_TIM_Base_Start_IT(&htim3); // Iniciar el timer 3 en modo interrupción

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 16000 - 1; // Prescaler para 1 kHz
    htim2.Init.Period = 1000 - 1; // Periodo para 1 segundo (1000 ms a 1 kHz)
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP; // Contar hacia arriba
    HAL_TIM_Base_Init(&htim2); // Inicializar el timer 2
    HAL_NVIC_EnableIRQ(TIM2_IRQn); // Habilitar interrupción del timer 2
    HAL_TIM_Base_Start_IT(&htim2); // Iniciar el timer 2 en modo interrupción
}

// rutina de servicio de interrupción del timer
// Para encender los números de unidades decenas y unidades de mil en el siete segmentos cada 5 ms
// se enciende cada uno por turno
// Para encender cada número se apagan los pines de control del display correspondiente por ser PNP el transistor
void TIM3_IRQHandler(void){
    // Delegar al HAL para limpiar banderas y ejecutar el callback correspondiente
    HAL_TIM_IRQHandler(&htim3);
}

// Definir funcion EXTI para el incremnento de la tarifa con el Encoder
void EXTI2_IRQHandler(void){
    // Delegar al HAL para limpiar la bandera EXTI y ejecutar el callback
    HAL_GPIO_EXTI_IRQHandler(ENCODER_PIN_CLK);
}

// Rutina del timer para encender y apagar el led blinky para verificar funcionamiento del codigo
void TIM2_IRQHandler(void){
    // Delegar al HAL para limpiar banderas y ejecutar el callback correspondiente
    HAL_TIM_IRQHandler(&htim2);
}

void EXTI15_10_IRQHandler(void){
    // Delegar al HAL para limpiar la bandera EXTI y ejecutar el callback
    HAL_GPIO_EXTI_IRQHandler(ENCODER_PIN_SW);
}


// Callback general del EXTI: se llama cuando ocurre una interrupción externa en un pin
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
    // Para encender cada número se apagan los pines de control del display correspondiente por ser PNP el transistor
    if(GPIO_Pin == ENCODER_PIN_CLK){
        EXTI_AVISORY_FLAG = 1; // subir la bandera de interrupción del encoder
    }
    if (GPIO_Pin == ENCODER_PIN_SW)
    {
        // Aqui se modifica el la frecuencia del refresco del display
        htim3.Instance->ARR += 5; // aumentar el periodo en 5 ms
        if (htim3.Instance->ARR == 50) // si supera 50 ms
        {
            htim3.Instance->ARR = 5; // reiniciar a 5 ms
        }
    }

}



// Callback de TIM: se llama cada vez que vence el periodo (UIF)
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

    if(htim->Instance == TIM3){
        // se levanta la bandera de asesoría para el TIM3
        TIMER_ADVISORY_FLAG = 1; // subir la bandera de interrupción del TIM3
        // Aquí realizar el multiplexado del display 7 segmentos cada ~7 ms si se requiere
        // Apagar pines del dígito PNP activo, actualizar segmentos y avanzar al siguiente dígito
    }
    else if(htim->Instance == TIM2){
        // Rutina del timer para encender y apagar el led blinky para verificar funcionamiento del codigo
        HAL_GPIO_TogglePin(LED_BLINKY_PORT, LED_BLINKY_PIN);
    }
}
