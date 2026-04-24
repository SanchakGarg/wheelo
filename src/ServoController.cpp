#include "ServoController.h"
#include "config.h"

ServoController::ServoController(uint8_t id, uint32_t baud, int rxPin, int txPin,
                                 int defaultFallback, int swing)
    : _id(id), _baud(baud), _rxPin(rxPin), _txPin(txPin),
      _defaultFallback(defaultFallback), _swing(swing),
      defaultPos(defaultFallback), targetPos(defaultFallback) {}

void ServoController::begin(Preferences& prefs) {
    prefs.begin(NVS_NS, true);
    defaultPos = prefs.getInt("servo_default", _defaultFallback);
    prefs.end();
    targetPos = defaultPos;

    Serial1.begin(_baud, SERIAL_8N1, _rxPin, _txPin);
    delay(50);
    Serial.printf("Servo default: %d\n", defaultPos);
    moveTo(defaultPos);
}

void ServoController::moveTo(int pos) {
    pos = constrain(pos, getLo(), getHi());
    targetPos = pos;
    uint8_t data[2] = { (uint8_t)(pos & 0xFF), (uint8_t)(pos >> 8) };
    scWrite(0x2A, data, 2);
}

void ServoController::setDefault(int pos, Preferences& prefs) {
    pos = constrain(pos, 0, 4095);
    defaultPos = pos;
    prefs.begin(NVS_NS, false);
    prefs.putInt("servo_default", pos);
    prefs.end();
    if (targetPos < getLo() || targetPos > getHi())
        moveTo(defaultPos);
}

int ServoController::getLo() const { return max(0,    defaultPos - _swing); }
int ServoController::getHi() const { return min(4095, defaultPos + _swing); }

void ServoController::scWrite(uint8_t reg, const uint8_t* data, uint8_t dlen) {
    uint8_t len = dlen + 3;
    uint8_t sum = _id + len + 0x03 + reg;
    for (uint8_t i = 0; i < dlen; i++) sum += data[i];
    uint8_t pkt[16], pi = 0;
    pkt[pi++] = 0xFF; pkt[pi++] = 0xFF;
    pkt[pi++] = _id;  pkt[pi++] = len;
    pkt[pi++] = 0x03; pkt[pi++] = reg;
    for (uint8_t i = 0; i < dlen; i++) pkt[pi++] = data[i];
    pkt[pi++] = ~sum;
    Serial1.write(pkt, pi);
    Serial1.flush();
}
