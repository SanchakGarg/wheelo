#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <ArduinoOTA.h>
#include "config.h"
#include "ESCController.h"
#include "ServoController.h"
#include "MPUSensor.h"
#include "BalancePID.h"
#include "WebUI.h"

Preferences     prefs;
WebServer       server(80);
ESCController   esc(PIN_ESC, ESC_MIN_US, ESC_MAX_US, ESC_MAX_PCT);
ServoController servo(SERVO_ID, BAUD_SERVO, PIN_SRV_RX, PIN_SRV_TX, SERVO_DEF, SERVO_SWING);
MPUSensor       mpu(MPU_ADDR, PIN_MPU_SDA, PIN_MPU_SCL);
BalancePID      pid(servo, mpu);
WebUI           ui(server, servo, esc, mpu, pid, prefs);

void setup() {
    Serial.begin(115200);
    delay(200);

    esc.begin();
    Serial.println("ESC armed at 0%");

    servo.begin(prefs);

    mpu.begin(prefs);
    pid.loadGains(prefs);

    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.print("WiFi");
    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 30000) {
        delay(300);
        Serial.print('.');
        esc.setThrottle(0);
    }
    if (WiFi.status() == WL_CONNECTED)
        Serial.printf("\nIP: %s\n", WiFi.localIP().toString().c_str());
    else
        Serial.println("\nWiFi failed.");

    ui.begin();
    Serial.println("HTTP server started.");

    // OTA
    ArduinoOTA.setHostname(OTA_HOSTNAME);
    ArduinoOTA.setPassword(OTA_PASSWORD);
    ArduinoOTA.onStart([](){ Serial.println("OTA start"); });
    ArduinoOTA.onEnd([](){ Serial.println("\nOTA done"); });
    ArduinoOTA.onProgress([](unsigned int prog, unsigned int total){
        Serial.printf("OTA: %u%%\r", prog * 100 / total);
    });
    ArduinoOTA.onError([](ota_error_t err){
        Serial.printf("OTA error[%u]\n", err);
    });
    ArduinoOTA.begin();
    Serial.printf("OTA ready — hostname: %s\n", OTA_HOSTNAME);
}

void loop() {
    ArduinoOTA.handle();
    ui.handle();

    while (Serial1.available()) Serial1.read();

    // Single 10ms tick: read filtered MPU6050 data, then run PID on fresh samples.
    static uint32_t lastLoop = 0;
    uint32_t now = millis();
    if (now - lastLoop >= 10) {
        if (mpu.read()) {
            pid.compute();
        }
        lastLoop = now;
    }
}
