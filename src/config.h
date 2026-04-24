#pragma once
#include <Arduino.h>

#define WIFI_SSID   "Makerspace"
#define WIFI_PASS   "testbricks"
#define NVS_NS      "wheelo"

// ESC — D10 / GPIO10
inline constexpr int      PIN_ESC      = 10;
inline constexpr int      ESC_MIN_US   = 1000;
inline constexpr int      ESC_MAX_US   = 2000;
inline constexpr int      ESC_MAX_PCT  = 30;

// ST3215 servo — half-duplex UART1
inline constexpr int      PIN_SRV_TX   = 20;    // D7
inline constexpr int      PIN_SRV_RX   = 21;    // D6
inline constexpr uint32_t BAUD_SERVO   = 1000000UL;
inline constexpr uint8_t  SERVO_ID     = 1;
inline constexpr int      SERVO_DEF    = 1100;
inline constexpr int      SERVO_SWING  = 350;   // ±counts from default

// MPU6050
inline constexpr uint8_t  MPU_ADDR     = 0x68;
inline constexpr int      PIN_MPU_SDA  = 6;     // D4
inline constexpr int      PIN_MPU_SCL  = 7;     // D5
