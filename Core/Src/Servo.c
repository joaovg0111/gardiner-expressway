#include <Servo.h>

void set_servo_angle(TIM_HandleTypeDef* timer, uint32_t channel, uint16_t period, int8_t angle) {
    uint16_t pulse = 0;

    if (angle < MIN_ANGLE) {
        pulse = NEG_ANGLE_PERCENTAGE * period;
    }
    else if (angle > MAX_ANGLE) {
        pulse = POS_ANGLE_PERCENTAGE * period;
    }
    else if (angle < 0) {
        pulse = (((angle + 90.0) / 90) * (NEUTRAL_ANGLE_PERCENTAGE - NEG_ANGLE_PERCENTAGE) + NEG_ANGLE_PERCENTAGE) * period;
    }
    else if (angle >= 0) {
        pulse = (((angle) / 90.0) * (POS_ANGLE_PERCENTAGE - NEUTRAL_ANGLE_PERCENTAGE) + NEUTRAL_ANGLE_PERCENTAGE) * period;
    }

    set_servo_pwm(timer, channel, period, pulse);
}

void set_servo_pwm(TIM_HandleTypeDef* timer, uint32_t channel, uint16_t period, uint16_t pulse) {
    HAL_TIM_PWM_Start(timer, channel);
    TIM_OC_InitTypeDef sConfigOC;

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = pulse;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(timer, &sConfigOC, channel);
    HAL_TIM_PWM_Start(timer, channel);
}
