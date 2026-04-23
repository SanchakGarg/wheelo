// ST3215 position read test  –  XIAO ESP32-C3
//
// Wiring (half-duplex SCServo bus):
//   D7 / GPIO20  TX  ──[1kΩ]──┬── ST3215 DATA
//   D6 / GPIO21  RX  ──────────┘
//
// 12V supply GND and ESP GND must be the same node.
// Open Serial Monitor at 115200 baud.

#include <Arduino.h>

// UART1 pins on XIAO ESP32-C3
#define SERVO_RX   21   // D6
#define SERVO_TX   20   // D7
#define SERVO_BAUD 1000000UL   // ST3215 default factory baud

#define SERVO_ID   1

// SCServo register map (Feetech protocol)
#define REG_PRESENT_POS  0x38   // 2 bytes, little-endian
#define INSTR_READ       0x02

// ── packet helpers ────────────────────────────────────────────────────────────

static void flushRx() {
  while (Serial1.available()) Serial1.read();
}

// Send a READ instruction and wait up to `timeoutMs` ms for the status reply.
// Returns the position (0–4095) on success, -1 on timeout/error.
static int16_t readPosition(uint8_t id, uint16_t timeoutMs = 20) {
  uint8_t regAddr = REG_PRESENT_POS;
  uint8_t rlen    = 2;
  uint8_t sum     = id + 4 + INSTR_READ + regAddr + rlen;
  uint8_t pkt[8]  = { 0xFF, 0xFF, id, 4, INSTR_READ, regAddr, rlen, (uint8_t)(~sum) };

  flushRx();
  Serial1.write(pkt, 8);
  Serial1.flush();   // block until TX FIFO is drained

  // Drain the echo: TX and RX share the same wire so our own 8 sent bytes
  // loop back into the RX FIFO. Discard them before reading the real reply.
  uint32_t t0 = millis();
  uint8_t  echoed = 0;
  while (echoed < 8 && millis() - t0 < 10) {
    if (Serial1.available()) { Serial1.read(); echoed++; }
  }

  // ── parse status packet ─────────────────────────────────────────────────
  // Format: FF FF ID LEN ERR POS_L POS_H CSUM  (8 bytes for a 2-byte read)
  t0 = millis();
  uint8_t  buf[16];
  uint8_t  got = 0;
  const uint8_t expected = 8;

  while (millis() - t0 < timeoutMs) {
    while (Serial1.available() && got < expected) {
      buf[got++] = Serial1.read();
    }
    if (got >= expected) break;
    delayMicroseconds(50);
  }

  if (got < expected) {
    Serial.printf("[TIMEOUT] only %d / %d bytes received\n", got, expected);
    return -1;
  }

  // Scan for 0xFF 0xFF header in case there are stray bytes
  int hdr = -1;
  for (int i = 0; i <= (int)got - 2; i++) {
    if (buf[i] == 0xFF && buf[i+1] == 0xFF) { hdr = i; break; }
  }
  if (hdr < 0 || (hdr + 7) >= (int)got) {
    Serial.println("[ERROR] no valid header found");
    return -1;
  }

  uint8_t respId  = buf[hdr+2];
  // uint8_t pktLen  = buf[hdr+3];   // should be 4
  uint8_t errByte = buf[hdr+4];
  uint8_t posL    = buf[hdr+5];
  uint8_t posH    = buf[hdr+6];
  // uint8_t csum    = buf[hdr+7];

  if (respId != id) {
    Serial.printf("[ERROR] ID mismatch: got 0x%02X expected 0x%02X\n", respId, id);
    return -1;
  }
  if (errByte != 0) {
    Serial.printf("[ERROR] servo error byte: 0x%02X\n", errByte);
    return -1;
  }

  return (int16_t)(posL | (posH << 8));
}

// ── setup / loop ──────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  delay(800);
  Serial.println("\n\n=== ST3215 Position Read Test ===");
  Serial.printf("UART1: TX=GPIO%d  RX=GPIO%d  baud=%lu\n",
                SERVO_TX, SERVO_RX, SERVO_BAUD);
  Serial.printf("Target servo ID: %d\n\n", SERVO_ID);

  Serial1.begin(SERVO_BAUD, SERIAL_8N1, SERVO_RX, SERVO_TX);
  delay(50);
}

void loop() {
  int16_t pos = readPosition(SERVO_ID);

  if (pos >= 0) {
    float deg = pos * 360.0f / 4096.0f;
    Serial.printf("Position: %4d  (%.2f deg)\n", pos, deg);
  } else {
    Serial.println("-- no response --  check 12V supply, GND, and servo ID");
  }

  delay(500);
}
