#pragma once

#include <stdint.h>

void buzzer_init(void);
void buzzer_beep(uint32_t frequency_hz, uint32_t duration_ms);
