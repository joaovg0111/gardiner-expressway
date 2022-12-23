#include "main.h"
#include "DCMotor.h"


// Function to write a digital value to a GPIO pin
void writeGPIO(GPIO_TypeDef* port, uint16_t pin, GPIO_PinState value){
    HAL_GPIO_WritePin(port, pin, value);
}

// Function to toggle a GPIO pin
void toggleGPIO(GPIO_TypeDef* port, uint16_t pin){
    HAL_GPIO_TogglePin(port, pin);
}

// Structure to hold the configuration for the PWM signal of the motor
TIM_OC_InitTypeDef sConfigOC_DC_Motor = {0};


// Function to initialize the motor
void initDCMotor(TIM_HandleTypeDef *htimx, uint32_t channel, uint16_t period){
    // Start generating the PWM signal for the motor
    HAL_TIM_PWM_Start(htimx, channel);

    // Set the initial direction of the motor to forward
    setDCMotorDirection(0x02);

    // Set the initial speed of the motor to 100%
    setDCMotorSpeed(htimx,channel,period,100);
}

// Function to set the speed of the motor
void setDCMotorSpeed(TIM_HandleTypeDef *htimx, uint32_t channel, uint16_t period, uint16_t speed){
    // Calculate the pulse width of the PWM signal based on the specified speed and period
    sConfigOC_DC_Motor.Pulse = (speed*period)/100;
    sConfigOC_DC_Motor.OCMode = TIM_OCMODE_PWM1;
    sConfigOC_DC_Motor.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC_DC_Motor.OCFastMode = TIM_OCFAST_DISABLE;

    // Stop the PWM signal and re-initialize it with the new configuration
    HAL_TIM_PWM_Stop(htimx, channel);
    HAL_TIM_PWM_Init(htimx);
    HAL_TIM_PWM_ConfigChannel(htimx, &sConfigOC_DC_Motor, channel);
    HAL_TIM_PWM_Start(htimx, channel);
}

// Function to set the direction of the motor
void setDCMotorDirection(uint8_t dir_code){
    // Initialize the output bit to send to the direction control registers as low
    GPIO_PinState bit_to_send = GPIO_PIN_RESET;

    // Reset the enable and latch pins for the direction control registers
    writeGPIO(GPIOA, DIR_EN_Pin, GPIO_PIN_RESET);
    writeGPIO(GPIOA, DIR_LATCH_Pin, GPIO_PIN_RESET);

    // Reset the clock pin for the direction control registers
    writeGPIO(GPIOB, DIR_CLK_Pin, GPIO_PIN_RESET);

    // Initialize a comparator value to compare with the direction code
    unsigned char comparator = 0x80;

    // Iterate through each bit of the direction code
    for (int i = 0; i < 8; i++){
        // If the current bit of the direction code is set, set the output bit to send as high
        if (dir_code & comparator){
            bit_to_send = GPIO_PIN_SET;
        }
        // Otherwise, set the output bit to send as low
        else{
            bit_to_send = GPIO_PIN_RESET;
        }

        // Set the data pin to the output bit to send
        writeGPIO(GPIOA, DIR_SER_Pin, bit_to_send);

        // Toggle the clock pin to send the output bit to the direction control registers
        toggleGPIO(GPIOB, DIR_CLK_Pin);

        // Shift the comparator value to compare the next bit of the direction code
        comparator = comparator >> 1;
    }

    // Set the latch pin to high to latch the output bits into the direction control registers
    writeGPIO(GPIOA, DIR_LATCH_Pin, GPIO_PIN_SET);

    // Set the enable pin to high to enable the direction control registers
    writeGPIO(GPIOA, DIR_EN_Pin, GPIO_PIN_SET);
}
