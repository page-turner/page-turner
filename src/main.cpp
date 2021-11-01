#include "ESP32_easy_wifi_data.h"
#include "JMotor.h"
#include "TwoAxisArmKinematics.h"
#include <Arduino.h>
JMotorDriverEsp32Servo servo1Driver = JMotorDriverEsp32Servo(8, 25); //pwm channel, pin
JServoControllerAdvanced servo1 = JServoControllerAdvanced(servo1Driver);
JMotorDriverEsp32Servo servo2Driver = JMotorDriverEsp32Servo(9, 26); //pwm channel, pin
JServoControllerAdvanced servo2 = JServoControllerAdvanced(servo2Driver);
bool enabled = false;
float x = 0;
float y = 0;
float theta1 = 0;
float theta2 = 0;
const float length_arm_1 = 14; // in cm
const float length_arm_2 = 11; // in cm
const float theta1Min = -90;
const float theta1Max = 40;
const float theta2Min = -70;
const float theta2Max = 100;
#define ARM_SETTINGS length_arm_1, length_arm_2, theta1Min, theta1Max, theta2Min, theta2Max

void WifiDataToParse()
{
    enabled = EWD::recvBl();
    x = EWD::recvFl();
    y = EWD::recvFl();
    //add data to read here: (EWD::recvBl, EWD::recvBy, EWD::recvIn, EWD::recvFl)(boolean, byte, int, float)
}
void WifiDataToSend()
{
    EWD::sendFl(theta1);
    EWD::sendFl(theta2);
    //add data to send here:
}
void setup()
{
    Serial.begin(115200);
    servo1.setVelAccelLimits(150, 200);
    servo1.setServoRangeValues(410, 1840);
    servo1.setSetAngles(theta1Max, theta1Min);
    servo1.setAngleLimits(theta1Min, theta1Max);
    servo1.setAngleImmediate(0);

    servo2.setVelAccelLimits(150, 200);
    servo2.setServoRangeValues(544, 2200);
    servo2.setSetAngles(theta2Min, theta2Max);
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

    if (cartToAngles(x, y, theta1, theta2, ARM_SETTINGS)) {
        servo1.setAngleSmoothed(theta1);
        servo2.setAngleSmoothed(theta2);
    }

    servo1.run();
    servo2.run();
}
