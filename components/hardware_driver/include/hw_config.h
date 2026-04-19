#pragma once

#include "driver/gpio.h"
#include "driver/i2s.h"
#include "driver/ledc.h"
#include "driver/i2c.h"

// INMP441 I2S microphone pins
#define INMP441_I2S_PORT           I2S_NUM_0
#define INMP441_I2S_BCK_GPIO       GPIO_NUM_26
#define INMP441_I2S_WS_GPIO        GPIO_NUM_27
#define INMP441_I2S_DATA_GPIO      GPIO_NUM_25

// OLED display pins and address
#define OLED_I2C_PORT              I2C_NUM_0
#define OLED_I2C_SDA_GPIO          GPIO_NUM_21
#define OLED_I2C_SCL_GPIO          GPIO_NUM_22
#define OLED_ADDRESS               0x3C

// Passive buzzer output pin
#define BUZZER_GPIO                GPIO_NUM_18
#define BUZZER_LEDC_TIMER          LEDC_TIMER_1
#define BUZZER_LEDC_CHANNEL        LEDC_CHANNEL_4
#define BUZZER_LEDC_MODE           LEDC_LOW_SPEED_MODE
