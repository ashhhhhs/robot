#include "buzzer.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hw_config.h"
#include "esp_log.h"

static const char *TAG = "BUZZER";

void buzzer_init(void)
{
    esp_err_t ret;
    
    ret = gpio_reset_pin(BUZZER_GPIO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "gpio_reset_pin failed: 0x%x", ret);
        return;
    }
    
    ret = gpio_set_direction(BUZZER_GPIO, GPIO_MODE_OUTPUT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "gpio_set_direction failed: 0x%x", ret);
        return;
    }

    ledc_timer_config_t timer_cfg = {
        .speed_mode = BUZZER_LEDC_MODE,
        .timer_num = BUZZER_LEDC_TIMER,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = 2000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    
    ret = ledc_timer_config(&timer_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config failed: 0x%x", ret);
        return;
    }

    ledc_channel_config_t channel_cfg = {
        .gpio_num = BUZZER_GPIO,
        .speed_mode = BUZZER_LEDC_MODE,
        .channel = BUZZER_LEDC_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = BUZZER_LEDC_TIMER,
        .duty = 0,
        .hpoint = 0,
    };
    
    ret = ledc_channel_config(&channel_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ledc_channel_config failed: 0x%x", ret);
        return;
    }
    
    ESP_LOGI(TAG, "Buzzer initialized successfully");
}

void buzzer_beep(uint32_t frequency_hz, uint32_t duration_ms)
{
    // Validate and clamp frequency
    if (frequency_hz < 20) {
        ESP_LOGW(TAG, "Frequency too low: %lu, clamping to 20 Hz", frequency_hz);
        frequency_hz = 20;
    } else if (frequency_hz > 20000) {
        ESP_LOGW(TAG, "Frequency too high: %lu, clamping to 20000 Hz", frequency_hz);
        frequency_hz = 20000;
    }
    
    // Clamp duration
    if (duration_ms > 10000) {
        ESP_LOGW(TAG, "Duration too long: %lu, clamping to 10000 ms", duration_ms);
        duration_ms = 10000;
    }

    esp_err_t ret;
    
    ret = ledc_set_freq(BUZZER_LEDC_MODE, BUZZER_LEDC_TIMER, frequency_hz);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ledc_set_freq failed: 0x%x", ret);
        return;
    }
    
    ret = ledc_set_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, 128);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ledc_set_duty failed: 0x%x", ret);
        return;
    }
    
    ret = ledc_update_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ledc_update_duty failed: 0x%x", ret);
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(duration_ms));

    ret = ledc_set_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ledc_set_duty off failed: 0x%x", ret);
        return;
    }
    
    ret = ledc_update_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ledc_update_duty off failed: 0x%x", ret);
    }
}
