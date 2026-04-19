# ESP32-S3 Speech Controlled Robot Dog

## 🧠 Base Framework

This project is built on top of Espressif's official ESP-Skainet example:

https://github.com/espressif/esp-skainet

All credit for the speech recognition framework goes to Espressif.


## ⚙️ Board Configuration

This project is preconfigured for the XIAO ESP32S3 Sense.

> ⚠️ Important:  
> The target is not automatically selected. You must set it manually before building.

```bash
idf.py set-target esp32s3
```
Or use the menu at the bottom.

## 🧩 Custom ESP32 WROOM Hardware

If you are using an ESP32 WROOM with INMP441, a passive buzzer, and a 0.96" 1315-W OLED, use the following wiring and set the target to `esp32`.

| Component | Signal | ESP32 GPIO |
|---|---|---|
|  INMP441 SCK   | BCLK   | GPIO26     |
| INMP441 WS    | WS     | GPIO27     |
| INMP441 SD    | DIN    | GPIO25     |
| INMP441 L/R   | —      | GND (left channel) |
| INMP441 VCC   | 3.3V   | 3.3V       |
| INMP441 GND   | GND    | GND        |
| OLED SDA | SDA | GPIO21 |
| OLED SCL | SCL | GPIO22 |
| Buzzer | PWM output | GPIO18 |
| Servo FL | PWM signal | GPIO13 |
| Servo FR | PWM signal | GPIO14 |
| Servo BL | PWM signal | GPIO16 |
| Servo BR | PWM signal | GPIO17 |

> Note: This project no longer depends on board-specific BSP code for audio playback. The buzzer emits simple beeps for feedback.

## 🚀 Build Instructions

Make a full clean before building and flashing the code.

## 🎯 Commands

Example recognized commands:

- Sit  
- Sit down  
- Come here  
- Walk  
- Dance  
- Stretch

More commands can be added in the ESP IDF config menu. These commands can be generated using the tool here:

https://github.com/espressif/esp-sr/tree/66e21f6cc384d6b4aec077c187ebb0f5fbb4c5ff/speech_command_recognition/tool

## ⚠️ Disclaimer

This is a simplified implementation intended for demonstration and experimentation.  
Speech recognition accuracy depends on environment, microphone quality, and model configuration.
