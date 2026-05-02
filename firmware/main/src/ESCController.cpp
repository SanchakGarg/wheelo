#include "ESCController.h"
#include <Arduino.h>

ESCController::ESCController(int pin, int minUs, int maxUs, int maxPct)
    : _pin(pin), _minUs(minUs), _maxUs(maxUs), _maxPct(maxPct) {}

void ESCController::begin() {
    ESP32PWM::allocateTimer(0);
    _esc.setPeriodHertz(50);
    _esc.attach(_pin, _minUs, _maxUs);
    setThrottle(0);
}

void ESCController::setThrottle(int pct) {
    pct = constrain(pct, 0, _maxPct);
    throttlePct = pct;
    _esc.writeMicroseconds(map(pct, 0, 100, _minUs, _maxUs));
}
