#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "stm32f1xx_hal.h"

#define MIN_ANGLE -90
#define MAX_ANGLE 90
#define NEUTRAL_ANGLE 0

#define NEG_ANGLE_PERCENTAGE 0.0256
#define POS_ANGLE_PERCENTAGE 0.1240
#define NEUTRAL_ANGLE_PERCENTAGE 0.0744

/*
    Function that sets the angle of a servo using pulse width modulation.
    The angle of the servo can range from -90 to 90 degrees.
    If the input angle is outside this range, it will be set to the closest value (-90 or 90).
    @param timer - a handle to the timer that generates the PWM signal
    @param channel - the channel of the timer to use for PWM
    @param period - the period of the PWM signal
    @param angle - the angle to set the servo to, in degrees
*/
void set_servo_angle(TIM_HandleTypeDef* timer, uint32_t channel, uint16_t period, int8_t angle);

/*
    Function that sets the pulse width of a servo using pulse width modulation.
    @param timer - a handle to the timer that generates the PWM signal
    @param channel - the channel of the timer to use for PWM
    @param period - the period of the PWM signal
    @param pulse - the pulse width to set the servo to, in units of the period
*/
void set_servo_pwm(TIM_HandleTypeDef* timer, uint32_t channel, uint16_t period, uint16_t pulse);

#endif // SERVO_H
