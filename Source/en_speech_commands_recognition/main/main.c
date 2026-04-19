/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include "oled.h"
#include "buzzer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "esp_wn_iface.h"
#include "esp_wn_models.h"
#include "esp_afe_sr_iface.h"
#include "esp_afe_sr_models.h"
#include "esp_mn_iface.h"
#include "esp_mn_models.h"
#include "esp_board_init.h"
#include "speech_commands_action.h"
#include "servo.h"

static const char *TAG = "ROBOT_DOG";

static const char *command_names[] = {
    "GOOD BOY",
    "SIT DOWN",
    "LIE DOWN",
    "STRETCH",
    "WALK",
    "DANCE"
};

#define CMD_GOOD_BOY 0
#define CMD_SIT_DOWN 1
#define CMD_LIE_DOWN 2
#define CMD_STRETCH 3
#define CMD_WALK 4
#define CMD_DANCE 5
#define NUM_COMMANDS 6

// State variables
static volatile int task_flag = 0;
static volatile int feed_task_running = 0;
static volatile int detect_task_running = 0;

// Global resources
static int wakeup_flag = 0;
static const esp_afe_sr_iface_t *afe_handle = NULL;
static srmodel_list_t *models = NULL;
static QueueHandle_t servo_cmd_queue = NULL;
static esp_afe_sr_data_t *global_afe_data = NULL;
static model_iface_data_t *global_model_data = NULL;
static esp_mn_iface_t *global_multinet = NULL;

void feed_Task(void *arg)
{
    esp_afe_sr_data_t *afe_data = arg;
    feed_task_running = 1;
    
    // Validate input
    if (!afe_data || !afe_handle) {
        ESP_LOGE(TAG, "feed_Task: invalid afe_data or afe_handle");
        feed_task_running = 0;
        vTaskDelete(NULL);
        return;
    }

    int audio_chunksize = afe_handle->get_feed_chunksize(afe_data);
    int nch = afe_handle->get_feed_channel_num(afe_data);
    int feed_channel = esp_get_feed_channel();
    
    if (nch != feed_channel) {
        ESP_LOGE(TAG, "feed_Task: channel mismatch - nch:%d vs feed_channel:%d", nch, feed_channel);
        feed_task_running = 0;
        vTaskDelete(NULL);
        return;
    }

    int16_t *i2s_buff = malloc(audio_chunksize * sizeof(int16_t) * feed_channel);
    if (!i2s_buff) {
        ESP_LOGE(TAG, "feed_Task: malloc failed for i2s_buff");
        feed_task_running = 0;
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "feed_Task started - chunksize:%d, channels:%d", audio_chunksize, nch);

    while (task_flag && feed_task_running)
    {
        esp_err_t ret = esp_get_feed_data(true, i2s_buff, audio_chunksize * sizeof(int16_t) * feed_channel);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "feed_Task: esp_get_feed_data failed - 0x%x", ret);
            break;
        }

        afe_handle->feed(afe_data, i2s_buff);
    }

    if (i2s_buff) {
        free(i2s_buff);
        i2s_buff = NULL;
    }

    ESP_LOGI(TAG, "feed_Task exiting");
    feed_task_running = 0;
    vTaskDelete(NULL);
}

void detect_Task(void *arg)
{
    esp_afe_sr_data_t *afe_data = arg;
    detect_task_running = 1;

    // Validate inputs
    if (!afe_data || !afe_handle || !models) {
        ESP_LOGE(TAG, "detect_Task: invalid inputs - afe_data:%p, afe_handle:%p, models:%p", 
                 afe_data, afe_handle, models);
        detect_task_running = 0;
        vTaskDelete(NULL);
        return;
    }

    int afe_chunksize = afe_handle->get_fetch_chunksize(afe_data);
    
    // Get model name with error check
    char *mn_name = esp_srmodel_filter(models, ESP_MN_PREFIX, ESP_MN_ENGLISH);
    if (!mn_name) {
        ESP_LOGE(TAG, "detect_Task: Failed to find model");
        oled_display_text("Error", "No model found");
        detect_task_running = 0;
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "detect_Task: multinet model: %s", mn_name);

    // Get multinet handle with error check
    esp_mn_iface_t *multinet = esp_mn_handle_from_name(mn_name);
    if (!multinet) {
        ESP_LOGE(TAG, "detect_Task: Failed to get multinet handle");
        oled_display_text("Error", "Init failed");
        detect_task_running = 0;
        vTaskDelete(NULL);
        return;
    }
    global_multinet = multinet;

    // Create model data with error check
    model_iface_data_t *model_data = multinet->create(mn_name, 6000);
    if (!model_data) {
        ESP_LOGE(TAG, "detect_Task: Failed to create model data");
        oled_display_text("Error", "Model create");
        detect_task_running = 0;
        vTaskDelete(NULL);
        return;
    }
    global_model_data = model_data;

    int mu_chunksize = multinet->get_samp_chunksize(model_data);
    if (mu_chunksize != afe_chunksize) {
        ESP_LOGE(TAG, "detect_Task: chunksize mismatch - mu:%d vs afe:%d", mu_chunksize, afe_chunksize);
        multinet->destroy(model_data);
        oled_display_text("Error", "Chunksize");
        detect_task_running = 0;
        vTaskDelete(NULL);
        return;
    }

    esp_mn_commands_update_from_sdkconfig(multinet, model_data);
    multinet->print_active_speech_commands(model_data);

    ESP_LOGI(TAG, "detect_Task started");
    oled_display_text("Ready", "Say wakeword");

    while (task_flag && detect_task_running)
    {
        afe_fetch_result_t *res = afe_handle->fetch(afe_data);
        if (!res || res->ret_value == ESP_FAIL)
        {
            ESP_LOGE(TAG, "detect_Task: fetch error");
            break;
        }

        if (res->wakeup_state == WAKENET_DETECTED)
        {
            ESP_LOGI(TAG, "WAKEWORD DETECTED");
            buzzer_beep(1500, 80);
            oled_display_text("Wakeword", "Listening...");
            multinet->clean(model_data);
        }

        if (res->raw_data_channels == 1 && res->wakeup_state == WAKENET_DETECTED)
        {
            wakeup_flag = 1;
        }
        else if (res->raw_data_channels > 1 && res->wakeup_state == WAKENET_CHANNEL_VERIFIED)
        {
            ESP_LOGI(TAG, "AFE_FETCH_CHANNEL_VERIFIED, channel index: %d", res->trigger_channel_id);
            wakeup_flag = 1;
        }

        if (wakeup_flag == 1)
        {
            esp_mn_state_t mn_state = multinet->detect(model_data, res->data);

            if (mn_state == ESP_MN_STATE_DETECTING)
            {
                continue;
            }

            if (mn_state == ESP_MN_STATE_DETECTED)
            {
                esp_mn_results_t *mn_result = multinet->get_results(model_data);
                if (mn_result) {
                    for (int i = 0; i < mn_result->num; i++)
                    {
                        ESP_LOGI(TAG, "TOP %d, command_id: %d, phrase_id: %d, string: %s, prob: %f",
                               i + 1, mn_result->command_id[i], mn_result->phrase_id[i], 
                               mn_result->string, mn_result->prob[i]);
                    }

                    int cmd_id = mn_result->command_id[0];
                    if (cmd_id >= 0 && cmd_id < NUM_COMMANDS) {
                        oled_display_text("Command", command_names[cmd_id]);
                        buzzer_beep(1200 + cmd_id * 100, 120);
                    } else {
                        oled_display_text("Command", "Unknown");
                    }

                    // Send with timeout to prevent queue overflow
                    if (xQueueSend(servo_cmd_queue, &cmd_id, pdMS_TO_TICKS(100)) != pdTRUE) {
                        ESP_LOGW(TAG, "detect_Task: Queue send failed (full?)");
                    }

                    ESP_LOGI(TAG, "Command sent to servo queue");
                }
            }

            if (mn_state == ESP_MN_STATE_TIMEOUT)
            {
                esp_mn_results_t *mn_result = multinet->get_results(model_data);
                if (mn_result) {
                    ESP_LOGW(TAG, "Timeout, string: %s", mn_result->string);
                }
                oled_display_text("Timeout", "Say wakeword");
                afe_handle->enable_wakenet(afe_data);
                wakeup_flag = 0;
            }
        }
    }

    if (model_data && multinet) {
        multinet->destroy(model_data);
        global_model_data = NULL;
    }

    ESP_LOGI(TAG, "detect_Task exiting");
    detect_task_running = 0;
    vTaskDelete(NULL);
}

void servo_Task(void *arg)
{
    int cmd_id;
    ESP_LOGI(TAG, "servo_Task started");
    
    while (task_flag)
    {
        // Block here until a command arrives — no CPU waste
        if (xQueueReceive(servo_cmd_queue, &cmd_id, pdMS_TO_TICKS(500)))
        {
            ESP_LOGI(TAG, "servo_Task received command: %d", cmd_id);
            
            // Validate command ID
            if (cmd_id < 0 || cmd_id >= NUM_COMMANDS) {
                ESP_LOGW(TAG, "servo_Task: invalid command ID %d", cmd_id);
                oled_display_text("Error", "Invalid cmd");
                continue;
            }

            oled_display_text("Executing", command_names[cmd_id]);

            switch (cmd_id)
            {
            case CMD_GOOD_BOY:
                ESP_LOGI(TAG, "Executing: GOOD BOY");
                anim_good_boy();
                break;
            case CMD_SIT_DOWN:
                ESP_LOGI(TAG, "Executing: SIT DOWN");
                anim_sit_down();
                break;
            case CMD_LIE_DOWN:
                ESP_LOGI(TAG, "Executing: LIE DOWN");
                anim_lie_down();
                break;
            case CMD_STRETCH:
                ESP_LOGI(TAG, "Executing: STRETCH");
                anim_stretch();
                break;
            case CMD_WALK:
                ESP_LOGI(TAG, "Executing: WALK");
                anim_walk();
                break;
            case CMD_DANCE:
                ESP_LOGI(TAG, "Executing: DANCE");
                anim_dance();
                break;
            default:
                ESP_LOGW(TAG, "servo_Task: unhandled command %d", cmd_id);
                break;
            }
            
            oled_display_text("Ready", "Say wakeword");
        }
    }
    
    ESP_LOGI(TAG, "servo_Task exiting");
    vTaskDelete(NULL);
}

void app_main()
{
    ESP_LOGI(TAG, "=== Robot Dog Booting ===");

    // Initialize peripherals
    ESP_LOGI(TAG, "Initializing buzzer...");
    buzzer_init();
    
    ESP_LOGI(TAG, "Initializing OLED...");
    oled_init();
    oled_display_text("Robot Dog", "Booting...");

    ESP_LOGI(TAG, "Initializing servos...");
    servo_init();
    servo_all_neutral();

    // Create queue for servo commands
    servo_cmd_queue = xQueueCreate(5, sizeof(int));
    if (!servo_cmd_queue) {
        ESP_LOGE(TAG, "Failed to create servo command queue");
        oled_display_text("Error", "Queue create");
        return;
    }

    // Load speech recognition models
    ESP_LOGI(TAG, "Loading speech models...");
    models = esp_srmodel_init("model");
    if (!models) {
        ESP_LOGE(TAG, "Failed to initialize models");
        oled_display_text("Error", "Model load");
        vQueueDelete(servo_cmd_queue);
        servo_cmd_queue = NULL;
        return;
    }

    // Initialize board (I2S audio)
    ESP_LOGI(TAG, "Initializing I2S audio...");
    esp_err_t ret = esp_board_init(16000, 2, 16);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize board: 0x%x", ret);
        oled_display_text("Error", "I2S init fail");
        vQueueDelete(servo_cmd_queue);
        servo_cmd_queue = NULL;
        return;
    }

    // Initialize AFE (Audio Front End)
    ESP_LOGI(TAG, "Initializing audio front end...");
    afe_config_t *afe_config = afe_config_init(esp_get_input_format(), models, AFE_TYPE_SR, AFE_MODE_LOW_COST);
    if (!afe_config) {
        ESP_LOGE(TAG, "Failed to initialize AFE config");
        oled_display_text("Error", "AFE config");
        vQueueDelete(servo_cmd_queue);
        servo_cmd_queue = NULL;
        return;
    }

    afe_handle = esp_afe_handle_from_config(afe_config);
    if (!afe_handle) {
        ESP_LOGE(TAG, "Failed to get AFE handle");
        afe_config_free(afe_config);
        oled_display_text("Error", "AFE handle");
        vQueueDelete(servo_cmd_queue);
        servo_cmd_queue = NULL;
        return;
    }

    global_afe_data = afe_handle->create_from_config(afe_config);
    if (!global_afe_data) {
        ESP_LOGE(TAG, "Failed to create AFE data");
        afe_config_free(afe_config);
        oled_display_text("Error", "AFE data");
        vQueueDelete(servo_cmd_queue);
        servo_cmd_queue = NULL;
        return;
    }

    afe_config_free(afe_config);

    // Start task flag
    task_flag = 1;

    // Create tasks on separate cores for better performance
    ESP_LOGI(TAG, "Creating tasks...");
    
    BaseType_t ret_detect = xTaskCreatePinnedToCore(
        &detect_Task, "detect", 8 * 1024, (void *)global_afe_data, 5, NULL, 1);
    if (ret_detect != pdPASS) {
        ESP_LOGE(TAG, "Failed to create detect task");
        task_flag = 0;
        oled_display_text("Error", "Task create");
        return;
    }

    BaseType_t ret_feed = xTaskCreatePinnedToCore(
        &feed_Task, "feed", 8 * 1024, (void *)global_afe_data, 5, NULL, 0);
    if (ret_feed != pdPASS) {
        ESP_LOGE(TAG, "Failed to create feed task");
        task_flag = 0;
        oled_display_text("Error", "Feed task");
        return;
    }

    BaseType_t ret_servo = xTaskCreatePinnedToCore(
        servo_Task, "servo", 4 * 1024, NULL, 5, NULL, 0);
    if (ret_servo != pdPASS) {
        ESP_LOGE(TAG, "Failed to create servo task");
        task_flag = 0;
        oled_display_text("Error", "Servo task");
        return;
    }

    ESP_LOGI(TAG, "=== Robot Dog Ready ===");
    oled_display_text("Ready", "Say wakeword");
}
