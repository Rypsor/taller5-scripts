#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"

GPIO_PinState stateBtn = GPIO_PIN_RESET; // Variable to hold button state



void init_hardware(void);


int main(void){
    init_hardware();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); // Set PA5 high
    
    while(1){

        stateBtn = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13); // Read the state of the button on PC13
        if(stateBt){ // If button is pressed (active low)
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); // Turn off LED (set PA5 low)
        } else {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); // Turn on LED (set PA5 high)
        }
 
     
    }
    return 0; 
}
// Function to initialize hardware components
void init_hardware(void){
    __HAL_RCC_GPIOA_CLK_ENABLE();            // Enable clock for GPIOA
    __HAL_RCC_GPIOC_CLK_ENABLE();            // Enable clock for GPIOC

    GPIO_InitTypeDef ConfigPin = {0};        // Initialize structure to zero
    ConfigPin.Pin = GPIO_PIN_5;              // we're using pin PA5
    ConfigPin.Mode = GPIO_MODE_OUTPUT_PP;    // push-pull output
    ConfigPin.Pull = GPIO_NOPULL;            // no pull-up or pull-down resistor
    ConfigPin.Speed = GPIO_SPEED_FREQ_HIGH;  // high frequency
    HAL_GPIO_Init(GPIOA, &ConfigPin);        // Initialize GPIOA with the configuration
    __NOP();                                 // No operation (for debugging purposes)



}
