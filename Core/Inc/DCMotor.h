#ifndef DCMOTOR_H_
#define DCMOTOR_H_

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "stm32f1xx_hal.h"

// Function to initialize the motor
void initDCMotor(TIM_HandleTypeDef *htimx, uint32_t channel, uint16_t period);

// Function to set the speed of the motor
void setDCMotorSpeed(TIM_HandleTypeDef *htimx, uint32_t channel, uint16_t period, uint16_t speed);

// Function to set the direction of the motor
void setDCMotorDirection(uint8_t dir_code);

// Function to write a digital value to a GPIO pin
void writeGPIO(GPIO_TypeDef* port, uint16_t pin, GPIO_PinState value);

// Function to toggle a GPIO pin
void toggleGPIO(GPIO_TypeDef* port, uint16_t pin);

#endif // DCMOTOR_H_
