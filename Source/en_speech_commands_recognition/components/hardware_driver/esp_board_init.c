/**
 * 
 * @copyright Copyright 2021 Espressif Systems (Shanghai) Co. Ltd.
 *
 *      Licensed under the Apache License, Version 2.0 (the "License");
 *      you may not use this file except in compliance with the License.
 *      You may obtain a copy of the License at
 *
 *               http://www.apache.org/licenses/LICENSE-2.0

 *      Unless required by applicable law or agreed to in writing, software
 *      distributed under the License is distributed on an "AS IS" BASIS,
 *      WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *      See the License for the specific language governing permissions and
 *      limitations under the License.
 */
#include <stdint.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "esp_err.h"
#include "driver/i2s.h"
#include "esp_board_init.h"
#include "hw_config.h"

static const i2s_port_t s_i2s_port = INMP441_I2S_PORT;

esp_err_t esp_board_init(uint32_t sample_rate, int channel_format, int bits_per_chan)
{
    i2s_config_t i2s_cfg = {
        .mode = I2S_MODE_MASTER | I2S_MODE_RX,
        .sample_rate = sample_rate,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_I2S_MSB,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 4,
        .dma_buf_len = 512,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0,
    };

    i2s_pin_config_t pin_cfg = {
        .bck_io_num = INMP441_I2S_BCK_GPIO,
        .ws_io_num = INMP441_I2S_WS_GPIO,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = INMP441_I2S_DATA_GPIO,
    };

    esp_err_t ret = i2s_driver_install(s_i2s_port, &i2s_cfg, 0, NULL);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = i2s_set_pin(s_i2s_port, &pin_cfg);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = i2s_zero_dma_buffer(s_i2s_port);
    if (ret != ESP_OK) {
        return ret;
    }

    return ESP_OK;
}

esp_err_t esp_sdcard_init(char *mount_point, size_t max_files)
{
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t esp_sdcard_deinit(char *mount_point)
{
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t esp_get_feed_data(bool is_get_raw_channel, int16_t *buffer, int buffer_len)
{
    size_t bytes_read = 0;
    esp_err_t ret = i2s_read(s_i2s_port, buffer, buffer_len, &bytes_read, pdMS_TO_TICKS(1000));
    if (ret != ESP_OK) {
        return ret;
    }
    return ESP_OK;
}

int esp_get_feed_channel(void)
{
    return 1;
}

char* esp_get_input_format(void)
{
    return "M";
}

esp_err_t esp_audio_play(const int16_t* data, int length, TickType_t ticks_to_wait)
{
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t esp_audio_set_play_vol(int volume)
{
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t esp_audio_get_play_vol(int *volume)
{
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t FatfsComboWrite(const void* buffer, int size, int count, FILE* stream)
{
    esp_err_t res = ESP_OK;
    res = fwrite(buffer, size, count, stream);
    res |= fflush(stream);
    res |= fsync(fileno(stream));

    return res;
}