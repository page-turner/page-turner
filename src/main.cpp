#include "Derivs_Limiter.h"
#include "ESP32_easy_wifi_data.h"
#include "JMotor.h"
#include "TwoAxisArmKinematics.h"
#include <Arduino.h>
JMotorDriverEsp32Servo servo1Driver = JMotorDriverEsp32Servo(8, 25); //pwm channel, pin
JServoControllerAdvanced servo1 = JServoControllerAdvanced(servo1Driver);
JMotorDriverEsp32Servo servo2Driver = JMotorDriverEsp32Servo(9, 26); //pwm channel, pin
JServoControllerAdvanced servo2 = JServoControllerAdvanced(servo2Driver);
bool enabled = false;
float xTarg = 0;
float yTarg = 0;
float x = 0;
float y = 0;
float theta1 = 0;
float theta2 = 0;
const float length_arm_1 = 14; // in cm
const float length_arm_2 = 11; // in cm
const float theta1Min = -90;
const float theta1Max = 40;
const float theta2Min = -100;
const float theta2Max = 70;
#define ARM_SETTINGS length_arm_1, length_arm_2, theta1Min, theta1Max, theta2Min, theta2Max

unsigned long millis_since_last_state_update = 0;
unsigned long millis_when_state_changed = 0;
bool did_state_change = true;

Derivs_Limiter xLimiter = Derivs_Limiter(3, 3);
Derivs_Limiter yLimiter = Derivs_Limiter(3, 3);

typedef enum
{
    START = 0,
    IDLE = 1,
    PAGE_TURN_LEFT = 2,
    PAGE_TURN_RIGHT = 3,
    SWING = 4
} state;
state PREVIOUS_STATE = START;
state CURRENT_STATE = IDLE;

void WifiDataToParse()
{
    enabled = EWD::recvBl();
    xTarg = EWD::recvFl();
    yTarg = EWD::recvFl();
    //add data to read here: (EWD::recvBl, EWD::recvBy, EWD::recvIn, EWD::recvFl)(boolean, byte, int, float)
}
void WifiDataToSend()
{
    EWD::sendFl(servo1.getPos());
    EWD::sendFl(servo2.getPos());
    //add data to send here:
}
void setup()
{
    Serial.begin(115200);
    servo1.setVelAccelLimits(250, 400);
    servo1.setServoRangeValues(410, 1840);
    servo1.setSetAngles(theta1Max, theta1Min);
    servo1.setAngleLimits(theta1Min, theta1Max);
    servo1.setAngleImmediate(0);

    servo2.setVelAccelLimits(250, 400);
    servo2.setServoRangeValues(544, 2200);
    servo2.setSetAngles(theta2Max, theta2Min);
    servo2.setAngleLimits(theta2Min, theta2Max);
    servo2.setAngleImmediate(0);

    EWD::routerName = "Brown-Guest"; //name of the wifi network you want to connect to
    EWD::routerPass = "-open-network-"; //password for your wifi network (enter "-open-network-" if the network has no password) (default: -open-network-)
    EWD::wifiPort = 25210; //what port the esp32 communicates on if connected to a wifi network (default: 25210)
    EWD::setupWifi(WifiDataToParse, WifiDataToSend);
}

void loop()
{
    EWD::runWifiCommunication();
    if (EWD::timedOut()) {
        enabled = false;
    }
    servo1.setEnable(enabled);
    servo2.setEnable(enabled);

    run_state();

    x = xLimiter.calc(xTarg);
    y = yLimiter.calc(yTarg);

    if (cartToAngles(x, y, theta1, theta2, ARM_SETTINGS)) {
        servo1.setAngleSmoothed(theta1);
        servo2.setAngleSmoothed(theta2);
    }

    servo1.run();
    servo2.run();
    delay(1);
}

void run_state()
{
    did_state_change = PREVIOUS_STATE != CURRENT_STATE;
    if (did_state_change) {
        millis_when_state_changed = millis();
    }
    millis_since_last_state_update = millis() - millis_when_state_changed;
    state NEXT_STATE = CURRENT_STATE;
    switch (CURRENT_STATE)
    {
    case IDLE:
        // code
        NEXT_STATE = state_idle();
        break;
    case PAGE_TURN_LEFT:
        // Perform routine to turn page left
        break;
    case PAGE_TURN_RIGHT:
        NEXT_STATE = state_page_turn_right();
        // Perform routine to turn page right
        break;
    default:
        break;
    }
    PREVIOUS_STATE = CURRENT_STATE;
    CURRENT_STATE = NEXT_STATE;
}

state state_idle() {
    if (1 /* turnPageLeftFlag */) {
        /* turnPageLeftFlag = false; */
        return PAGE_TURN_LEFT;
    }
    else if (1 /* turnPageLeftFlag */) {
        /* turnPageRightFlag = false; */
        return PAGE_TURN_RIGHT;
    }
    return CURRENT_STATE;
}

state state_page_turn_right() {
    /* Example Routine */

    return PAGE_TURN_RIGHT;
}