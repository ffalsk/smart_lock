#ifndef SERVO_H
#define SERVO_H

#include "main.h"

class Servo
{
public:
    void begin(uint16_t max_angle, gpio_num_t ServoPin, uint16_t min_width_us = 500, uint16_t max_width_us = 2500);
    void write(float angle);

private:
    gpio_num_t servo_pin;
    uint8_t channel;
};

#endif