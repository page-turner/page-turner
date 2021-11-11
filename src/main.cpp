#include "TwoAxisArmKinematics.h"
#include <Arduino.h>
#include <Derivs_Limiter.h>
#include <ESP32_easy_wifi_data.h>
#include <HX711.h>
#include <JMotor.h>

const byte torque1SensorDTPin = 19;
const byte torque1SensorSCKPin = 18;
const byte torque2SensorDTPin = 5;
const byte torque2SensorSCKPin = 17;

JMotorDriverEsp32Servo servo1Driver = JMotorDriverEsp32Servo(8, 25); //pwm channel, pin
JServoControllerAdvanced servo1 = JServoControllerAdvanced(servo1Driver);
JMotorDriverEsp32Servo servo2Driver = JMotorDriverEsp32Servo(9, 26); //pwm channel, pin
JServoControllerAdvanced servo2 = JServoControllerAdvanced(servo2Driver);

bool enabled = false; //received over wifi
float xTarg = 0; //for debug, received over wifi
float yTarg = 0; //for debug, received over wifi
bool go = false; //for debug, received over wifi
//global variables for force measurements
float torque1 = 0;
float torque2 = 0;
float Fx = 0;
float Fy = 0;
//arm position
float x = 0;
float y = 0;
//servo target angles
float theta1 = 0;
float theta2 = 0;
//arm constants
const float length_arm_1 = 22.5; // in cm
const float length_arm_2 = 11; // in cm
const float theta1Min = -135;
const float theta1Max = 135;
const float theta2Min = -135;
const float theta2Max = 135;
#define ARM_SETTINGS length_arm_1, length_arm_2, theta1Min, theta1Max, theta2Min, theta2Max

//state machine variables
unsigned long millis_since_last_state_update = 0;
unsigned long millis_when_state_changed = 0;
bool did_state_change = true;

//smooth acceleration for arm position
Derivs_Limiter xLimiter = Derivs_Limiter(6, 3);
Derivs_Limiter yLimiter = Derivs_Limiter(6, 3);

//for reading loadcells
HX711 torque1Sensor;
HX711 torque2Sensor;

//list of states for state machine
typedef enum {
    START = 0,
    IDLE = 1,
    A = 2,
    B = 3
} state;
state PREVIOUS_STATE = START;
state CURRENT_STATE = IDLE;

//for debug, receive and send data over wifi
void WifiDataToParse()
{
    enabled = EWD::recvBl();
    xTarg = EWD::recvFl();
    yTarg = EWD::recvFl();
    go = EWD::recvBl();
}
void WifiDataToSend()
{
    EWD::sendFl(servo1.getPos());
    EWD::sendFl(servo2.getPos());
    EWD::sendFl(torque1);
    EWD::sendFl(torque2);
    EWD::sendFl(Fx);
    EWD::sendFl(Fy);
}

void setup()
{
    Serial.begin(115200);

    //set up and calibrate servos
    servo1.setConstrainRange(false);
    servo1.setVelAccelLimits(250, 400);
    servo1.setServoRangeValues(840, 2131);
    servo1.setSetAngles(-90, 90);
    servo1.setAngleLimits(theta1Max, theta1Min);
    servo1.setAngleImmediate(0);

    servo2.setConstrainRange(false);
    servo2.setVelAccelLimits(250, 400);
    servo2.setServoRangeValues(870, 2151);
    servo2.setSetAngles(-90, 90);
    servo2.setAngleLimits(theta2Max, theta2Min);
    servo2.setAngleImmediate(0);

    //torque load cells
    torque1Sensor.begin(torque1SensorDTPin, torque1SensorSCKPin); //hx711 DT, SCK
    torque1Sensor.set_scale(18200); //calibrate sensor by changing this value
    torque1Sensor.tare();

    torque2Sensor.begin(torque2SensorDTPin, torque2SensorSCKPin); //hx711 DT, SCK
    torque2Sensor.set_scale(44000); //calibrate sensor by changing this value
    torque2Sensor.tare();

    EWD::routerName = "Brown-Guest"; //name of the wifi network you want to connect to
    EWD::routerPass = "-open-network-"; //password for your wifi network (enter "-open-network-" if the network has no password) (default: -open-network-)
    EWD::wifiPort = 25210; //what port the esp32 communicates on if connected to a wifi network (default: 25210)
    EWD::setupWifi(WifiDataToParse, WifiDataToSend);
}

#include "states.h"
/**
 * @brief  state machine framework, add state functions to states.h, and call them from here
 */
void run_state()
{
    did_state_change = PREVIOUS_STATE != CURRENT_STATE;
    if (did_state_change) {
        millis_when_state_changed = millis();
        millis_since_last_state_update = 0;
    } else {
        millis_since_last_state_update = millis() - millis_when_state_changed;
    }
    state NEXT_STATE = CURRENT_STATE;
    switch (CURRENT_STATE) {
    case IDLE:
        // code
        NEXT_STATE = state_idle();
        break;
    case A:
        NEXT_STATE = state_A();
        break;
    case B:
        NEXT_STATE = state_B();
        break;
    default:
        break;
    }
    PREVIOUS_STATE = CURRENT_STATE;
    CURRENT_STATE = NEXT_STATE;
}

void loop()
{
    EWD::runWifiCommunication();
    if (EWD::timedOut()) {
        enabled = false;
    }
    servo1.setEnable(enabled);
    servo2.setEnable(enabled);

    if (torque1Sensor.is_ready()) {
        torque1 = torque1Sensor.get_units();
    }
    if (torque2Sensor.is_ready()) {
        torque2 = -torque2Sensor.get_units(); //note negative sign because loadcell was installed backwards
    }

    torqueToForces(theta1, theta2, torque1, torque2, Fx, Fy, length_arm_1, length_arm_2);

    run_state();

    //smooth acceleration of arm movement
    x = xLimiter.calc();
    y = yLimiter.calc();
    //calculate arm kinematics (by having this separate outside the state machine, states can't directly set servo angles)
    if (cartToAngles(x, y, theta1, theta2, ARM_SETTINGS)) {
        servo1.setAngleSmoothed(theta1);
        servo2.setAngleSmoothed(theta2);
    }

    //run servo controllers
    servo1.run();
    servo2.run();
    delay(1);
}