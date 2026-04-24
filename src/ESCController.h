#pragma once
#include <ESP32Servo.h>

class ESCController {
public:
    int throttlePct = 0;

    ESCController(int pin, int minUs, int maxUs, int maxPct);
    void begin();
    void setThrottle(int pct);

private:
    Servo _esc;
    int   _pin, _minUs, _maxUs, _maxPct;
};
