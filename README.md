# ESP32 Spotify Controller
### ESP32 with Buttons, a Rotary Encoder, and a Display to control Spotify.

## Overview
This project uses the ESP-IDF framework from PlatformIO on an ESP32 Dev Board. I wrote this to work in tandem with my [base station](https://github.com/KaiserMighty/ESP32-Base-Station). This device simply handles inputs and broadcasts a UDP message to the base station. The actual spotify functionality is handled within the Base Station.

## Organization
This device assumes that the ESP32 has one I2C devices:
* SSD1306 128x32 based display

The I2C SCL and SDA lines are assumed to be connected to GPIO pins 22 and 23 respectively.  

It also assumes that you have 3 buttons and a rotary encoder setup as such:  
* Play Button on GPIO 19  
* Skip Button on GPIO 21  
* Back Button on GPIO 18  
* Rotary Encoder Button on GPIO 4  
* Volume Up on GPIO 16 (Rotary Encoder)  
* Volume Down on GPIO 17 (Rotary Encoder)  

The WiFi SSID and Password are editable within `main.c`.