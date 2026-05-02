#pragma once
#include <WebServer.h>
#include <Preferences.h>
#include "ServoController.h"
#include "ESCController.h"
#include "MPUSensor.h"
#include "BalancePID.h"

class WebUI {
public:
    WebUI(WebServer& server, ServoController& servo, ESCController& esc,
          MPUSensor& mpu, BalancePID& pid, Preferences& prefs);

    void begin();
    void handle();

private:
    WebServer&       _server;
    ServoController& _servo;
    ESCController&   _esc;
    MPUSensor&       _mpu;
    BalancePID&      _pid;
    Preferences&     _prefs;

    void handleRoot();
    void handleSet();
    void handleState();
    void handleServoSet();
    void handleServoSetDefault();
    void handleMpu();
    void handleMpuCalibrate();
    void handleMpuNoiseCal();
    void handleMpuResetAngles();
    void handleMpuSetAngle();
    void handleBalanceStart();
    void handleBalanceStop();
    void handleBalancePid();
    void handleBalanceAccum();
    void handleBalanceState();
};
