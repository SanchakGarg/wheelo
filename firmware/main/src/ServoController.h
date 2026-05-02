#pragma once
#include <Arduino.h>
#include <Preferences.h>

class ServoController {
public:
    int defaultPos;
    int targetPos;
    int currentPos = 0;

    ServoController(uint8_t id, uint32_t baud, int rxPin, int txPin,
                    int defaultFallback, int swing);

    void begin(Preferences& prefs);
    void moveTo(int pos);
    void setDefault(int pos, Preferences& prefs);
    int  readPos();
    int  getLo() const;
    int  getHi() const;

private:
    uint8_t  _id;
    uint32_t _baud;
    int      _rxPin, _txPin, _swing, _defaultFallback;

    void scWrite(uint8_t reg, const uint8_t* data, uint8_t dlen);
};
