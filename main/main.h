#ifndef MAIN_H
#define MAIN_H

#include <stdio.h>
#include <iostream>
#include <cstring>
#include <string>
#include <sstream>
#include <iomanip>
#include <cstdarg>
#include <cstdint>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_random.h"
#include "iot_servo.h"
#include "SHTC3.h"
#include "Servo.h"
#include "FingerIdentifier.h"
#include "Audio.h"

#define I2C_SDA 3
#define I2C_SCL 0
#define servo_close 180
#define servo_open 10

extern TaskHandle_t finger_identify_task_handle;
uint32_t random(uint32_t min, uint32_t max);
#endif
