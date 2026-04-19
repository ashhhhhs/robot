/*
 * Robot Dog - Enhanced Personality Engine
 *
 * Features:
 *  - Finite State Machine (IDLE, PETTED, DANCING, EXCITED, SLEEPING)
 *  - Gesture detection: tap, long-press (triggers dance), double-tap (excited)
 *  - Auto-sleep after configurable idle timeout
 *  - Random idle blink animations
 *  - Richer personality sounds (chirp, whimper, snore, excited bark)
 *  - Graceful task shutdown with event groups
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "oled.h"
#include "buzzer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_random.h"
#include "driver/touch_pad.h"

#include "speech_commands_action.h"
#include "servo.h"

// ─────────────────────────────────────────────
//  Configuration
// ─────────────────────────────────────────────
static const char *TAG = "ROBOT_DOG";

#define TOUCH_PAD_NUM        TOUCH_PAD_NUM9
#define TOUCH_THRESHOLD      400     // Lower = more sensitive
#define LONG_PRESS_MS        800     // Hold duration to trigger dance
#define DOUBLE_TAP_WINDOW_MS 350     // Max gap between taps for double-tap
#define SLEEP_TIMEOUT_MS     15000   // Idle time before dog falls asleep
#define BLINK_INTERVAL_MIN   3000    // Minimum ms between random blinks
#define BLINK_INTERVAL_MAX   7000    // Maximum ms between random blinks
#define BLINK_DURATION_MS    120     // How long a blink lasts
#define TOUCH_POLL_MS        50      // Touch polling interval

// ─────────────────────────────────────────────
//  Commands (servo task protocol)
// ─────────────────────────────────────────────
typedef enum {
    CMD_GOOD_BOY = 0,
    CMD_DANCE    = 5,
    CMD_EXCITED  = 10,
} servo_cmd_t;

// ─────────────────────────────────────────────
//  Robot States
// ─────────────────────────────────────────────
typedef enum {
    STATE_IDLE,
    STATE_PETTED,
    STATE_DANCING,
    STATE_EXCITED,
    STATE_SLEEPING,
} robot_state_t;

// ─────────────────────────────────────────────
//  OLED face strings
// ─────────────────────────────────────────────
typedef struct {
    const char *top;
    const char *bottom;
} oled_face_t;

static const oled_face_t FACE_IDLE     = { "  ^ _ ^  ", "   Idle   " };
static const oled_face_t FACE_HAPPY    = { "  > w <  ", " Good Boy! " };
static const oled_face_t FACE_BLINK    = { "  - _ -  ", "   Idle   " };
static const oled_face_t FACE_EXCITED  = { "  ^O^!!  ", "  Excited! " };
static const oled_face_t FACE_DANCE    = { "  >  <   ", "  Dancing! " };
static const oled_face_t FACE_SLEEP    = { "  -_- zz ", "  Sleeping " };
static const oled_face_t FACE_WAKE     = { "  o _ o  ", " Waking up!" };
static const oled_face_t FACE_SNIFF    = { "  ^ . ^  ", "  *sniff*  " };

// ─────────────────────────────────────────────
//  Shared state
// ─────────────────────────────────────────────
static volatile robot_state_t current_state   = STATE_IDLE;
static volatile TickType_t    last_activity   = 0;
static QueueHandle_t          servo_cmd_queue = NULL;
static EventGroupHandle_t     task_evt_group  = NULL;

#define EVT_SHUTDOWN BIT0

// ─────────────────────────────────────────────
//  Helper: Show face
// ─────────────────────────────────────────────
static void show_face(const oled_face_t *f)
{
    oled_display_text(f->top, f->bottom);
}

// ─────────────────────────────────────────────
//  Personality Sounds
// ─────────────────────────────────────────────
static void play_boot_sound(void)
{
    buzzer_beep(600, 80);
    vTaskDelay(40 / portTICK_PERIOD_MS);
    buzzer_beep(900, 80);
    vTaskDelay(40 / portTICK_PERIOD_MS);
    buzzer_beep(1200, 80);
    vTaskDelay(40 / portTICK_PERIOD_MS);
    buzzer_beep(1600, 150);
}

static void play_happy_chirp(void)
{
    buzzer_beep(1500, 60);
    vTaskDelay(30 / portTICK_PERIOD_MS);
    buzzer_beep(2000, 100);
    vTaskDelay(30 / portTICK_PERIOD_MS);
    buzzer_beep(1800, 80);
}

static void play_excited_bark(void)
{
    for (int i = 0; i < 3; i++) {
        buzzer_beep(900, 60);
        vTaskDelay(50 / portTICK_PERIOD_MS);
        buzzer_beep(700, 60);
        vTaskDelay(80 / portTICK_PERIOD_MS);
    }
}

static void play_dance_fanfare(void)
{
    const int notes[] = { 523, 659, 784, 1047 };  // C5 E5 G5 C6
    for (int i = 0; i < 4; i++) {
        buzzer_beep(notes[i], 100);
        vTaskDelay(40 / portTICK_PERIOD_MS);
    }
}

static void play_sleep_sound(void)
{
    buzzer_beep(400, 200);
    vTaskDelay(300 / portTICK_PERIOD_MS);
    buzzer_beep(300, 300);
}

static void play_wake_sound(void)
{
    buzzer_beep(500, 80);
    vTaskDelay(60 / portTICK_PERIOD_MS);
    buzzer_beep(800, 120);
}

// ─────────────────────────────────────────────
//  State Transitions
// ─────────────────────────────────────────────
static void enter_state(robot_state_t new_state)
{
    if (current_state == new_state) return;

    ESP_LOGI(TAG, "State: %d -> %d", current_state, new_state);
    current_state  = new_state;
    last_activity  = xTaskGetTickCount();
}

static void send_servo_cmd(servo_cmd_t cmd)
{
    int c = (int)cmd;
    if (xQueueSend(servo_cmd_queue, &c, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Servo queue full — command dropped");
    }
}

// ─────────────────────────────────────────────
//  Touch Task  (gesture detector)
// ─────────────────────────────────────────────
void touch_Task(void *arg)
{
    uint16_t touch_val;
    bool     is_touched       = false;
    bool     was_touched      = false;
    TickType_t press_start    = 0;
    TickType_t last_tap_tick  = 0;
    bool     long_press_fired = false;

    ESP_LOGI(TAG, "Touch task started (GPIO32 / TOUCH_PAD_NUM9)");

    while (!(xEventGroupGetBits(task_evt_group) & EVT_SHUTDOWN)) {
        touch_pad_read(TOUCH_PAD_NUM, &touch_val);
        is_touched = (touch_val < TOUCH_THRESHOLD);

        // ── Wake from sleep on any touch ──
        if (is_touched && current_state == STATE_SLEEPING) {
            play_wake_sound();
            show_face(&FACE_WAKE);
            vTaskDelay(600 / portTICK_PERIOD_MS);
            enter_state(STATE_IDLE);
            show_face(&FACE_IDLE);
            was_touched = true;
            vTaskDelay(TOUCH_POLL_MS / portTICK_PERIOD_MS);
            continue;
        }

        // ── Ignore gestures during servo animations ──
        if (current_state == STATE_DANCING || current_state == STATE_EXCITED) {
            was_touched = is_touched;
            vTaskDelay(TOUCH_POLL_MS / portTICK_PERIOD_MS);
            continue;
        }

        // ── Rising edge: touch started ──
        if (is_touched && !was_touched) {
            press_start    = xTaskGetTickCount();
            long_press_fired = false;
        }

        // ── Held: check for long-press ──
        if (is_touched && was_touched && !long_press_fired) {
            TickType_t held_ms = pdTICKS_TO_MS(xTaskGetTickCount() - press_start);
            if (held_ms >= LONG_PRESS_MS) {
                long_press_fired = true;
                ESP_LOGI(TAG, "Gesture: LONG PRESS -> DANCE");
                enter_state(STATE_DANCING);
                show_face(&FACE_DANCE);
                play_dance_fanfare();
                send_servo_cmd(CMD_DANCE);
            }
        }

        // ── Falling edge: touch released ──
        if (!is_touched && was_touched && !long_press_fired) {
            TickType_t now     = xTaskGetTickCount();
            TickType_t gap_ms  = pdTICKS_TO_MS(now - last_tap_tick);

            if (last_tap_tick != 0 && gap_ms <= DOUBLE_TAP_WINDOW_MS) {
                // Double tap → Excited!
                ESP_LOGI(TAG, "Gesture: DOUBLE TAP -> EXCITED");
                last_tap_tick = 0;
                enter_state(STATE_EXCITED);
                show_face(&FACE_EXCITED);
                play_excited_bark();
                send_servo_cmd(CMD_EXCITED);
            } else {
                // Single tap → Good Boy
                ESP_LOGI(TAG, "Gesture: SINGLE TAP -> PETTED");
                last_tap_tick = now;
                enter_state(STATE_PETTED);
                show_face(&FACE_HAPPY);
                play_happy_chirp();
                send_servo_cmd(CMD_GOOD_BOY);
            }
        }

        was_touched = is_touched;
        vTaskDelay(TOUCH_POLL_MS / portTICK_PERIOD_MS);
    }

    ESP_LOGI(TAG, "Touch task shutting down");
    vTaskDelete(NULL);
}

// ─────────────────────────────────────────────
//  Servo Task
// ─────────────────────────────────────────────
void servo_Task(void *arg)
{
    int cmd_id;

    while (!(xEventGroupGetBits(task_evt_group) & EVT_SHUTDOWN)) {
        if (xQueueReceive(servo_cmd_queue, &cmd_id, pdMS_TO_TICKS(200)) == pdTRUE) {
            ESP_LOGI(TAG, "Servo executing cmd: %d", cmd_id);

            switch ((servo_cmd_t)cmd_id) {
                case CMD_GOOD_BOY: anim_good_boy(); break;
                case CMD_DANCE:    anim_dance();    break;
                case CMD_EXCITED:  anim_good_boy(); anim_good_boy(); break;
                default:
                    ESP_LOGW(TAG, "Unknown servo cmd: %d", cmd_id);
                    break;
            }

            // Return to idle after animation completes
            if (current_state != STATE_SLEEPING) {
                enter_state(STATE_IDLE);
                show_face(&FACE_IDLE);
            }
        }
    }

    ESP_LOGI(TAG, "Servo task shutting down");
    vTaskDelete(NULL);
}

// ─────────────────────────────────────────────
//  Idle Manager Task (sleep + blink animations)
// ─────────────────────────────────────────────
void idle_Task(void *arg)
{
    TickType_t next_blink = xTaskGetTickCount() +
                            pdMS_TO_TICKS(BLINK_INTERVAL_MIN + (esp_random() % (BLINK_INTERVAL_MAX - BLINK_INTERVAL_MIN)));

    ESP_LOGI(TAG, "Idle manager task started");

    while (!(xEventGroupGetBits(task_evt_group) & EVT_SHUTDOWN)) {
        TickType_t now    = xTaskGetTickCount();
        TickType_t idle_ms = pdTICKS_TO_MS(now - last_activity);

        // ── Auto-sleep ──
        if (current_state == STATE_IDLE && idle_ms >= SLEEP_TIMEOUT_MS) {
            ESP_LOGI(TAG, "Idle timeout — going to sleep");
            enter_state(STATE_SLEEPING);
            servo_all_neutral();
            play_sleep_sound();
            show_face(&FACE_SLEEP);
        }

        // ── Periodic blink (only while idle) ──
        if (current_state == STATE_IDLE && now >= next_blink) {
            show_face(&FACE_BLINK);
            vTaskDelay(BLINK_DURATION_MS / portTICK_PERIOD_MS);
            show_face(&FACE_IDLE);

            // Occasionally show a sniff animation
            if ((esp_random() % 4) == 0) {
                vTaskDelay(200 / portTICK_PERIOD_MS);
                show_face(&FACE_SNIFF);
                vTaskDelay(400 / portTICK_PERIOD_MS);
                show_face(&FACE_IDLE);
            }

            // Schedule next blink with random interval
            next_blink = now + pdMS_TO_TICKS(
                BLINK_INTERVAL_MIN + (esp_random() % (BLINK_INTERVAL_MAX - BLINK_INTERVAL_MIN))
            );
        }

        vTaskDelay(200 / portTICK_PERIOD_MS);
    }

    ESP_LOGI(TAG, "Idle task shutting down");
    vTaskDelete(NULL);
}

// ─────────────────────────────────────────────
//  Entry Point
// ─────────────────────────────────────────────
void app_main(void)
{
    ESP_LOGI(TAG, "=== Robot Dog Booting ===");

    // Init touch FIRST to avoid I2C bus collision with OLED
    ESP_ERROR_CHECK(touch_pad_init());
    ESP_ERROR_CHECK(touch_pad_config(TOUCH_PAD_NUM, 0));

    buzzer_init();
    oled_init();

    show_face(&FACE_WAKE);
    play_boot_sound();

    servo_init();
    servo_all_neutral();

    // Shared resources
    servo_cmd_queue = xQueueCreate(5, sizeof(int));
    task_evt_group  = xEventGroupCreate();
    configASSERT(servo_cmd_queue);
    configASSERT(task_evt_group);

    last_activity = xTaskGetTickCount();

    // Spawn tasks
    xTaskCreatePinnedToCore(servo_Task, "servo", 4 * 1024, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(touch_Task, "touch", 4 * 1024, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(idle_Task,  "idle",  3 * 1024, NULL, 3, NULL, 1);

    vTaskDelay(pdMS_TO_TICKS(1000));
    show_face(&FACE_IDLE);

    ESP_LOGI(TAG, "=== Robot Dog Ready ===");
    // app_main returns; FreeRTOS scheduler owns the CPU from here
}