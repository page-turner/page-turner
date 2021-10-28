#include "ESP32_easy_wifi_data.h"
#include "JMotor.h"
#include "TwoAxisArmKinematics.h"
#include <Arduino.h>
JMotorDriverEsp32Servo servo1Driver = JMotorDriverEsp32Servo(1, 25);
JServoControllerAdvanced servo1 = JServoControllerAdvanced(servo1Driver);
JMotorDriverEsp32Servo servo2Driver = JMotorDriverEsp32Servo(2, 26);
JServoControllerAdvanced servo2 = JServoControllerAdvanced(servo2Driver);
bool enabled = false;
float x = 0;
float y = 0;
float theta1 = 0;
float theta2 = 0;

void WifiDataToParse()
{
    enabled = EWD::recvBl();
    x = EWD::recvFl();
    y = EWD::recvFl();
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
    servo1.setVelAccelLimits(150, 200);
    servo2.setVelAccelLimits(150, 200);
    servo1.setSetAngles(-90, 90);
    servo2.setSetAngles(-90, 90);
    servo1.setAngleLimits(-90, 90);
    servo2.setAngleLimits(-90, 90);
    servo1.setAngleImmediate(0);
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

    //calculate stuff
    theta1 = x;
    theta2 = y;

    servo1.setAngleSmoothed(theta1);
    servo2.setAngleSmoothed(theta2);

    servo1.run();
    servo2.run();
}
