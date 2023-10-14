/******************************************************************
 * \file Servo.cpp
 * \brief 舵机相关处理函数
 *
 * \author HDC
 * \date September 2023
 ********************************************************************/
#include "Servo.h"

void Servo::begin(uint16_t max_angle, gpio_num_t ServoPin, uint16_t min_width_us, uint16_t max_width_us)
{
    servo_pin = ServoPin;
    channel = 0;
    servo_config_t servo_cfg = {
        .max_angle = max_angle,
        .min_width_us = min_width_us,
        .max_width_us = max_width_us,
        .freq = 50,
        .timer_number = LEDC_TIMER_0,
        .channels = {
            .servo_pin = {
                servo_pin,
            },
            .ch = {
                LEDC_CHANNEL_0,
            },
        },
        .channel_number = 1,
    };
    iot_servo_init(LEDC_LOW_SPEED_MODE, &servo_cfg);
}

void Servo::write(float angle)
{
    iot_servo_write_angle(LEDC_LOW_SPEED_MODE, channel, angle);
}