#define EWDmaxWifiSendBufSize 100
#define EWDmaxWifiRecvBufSize 100

#include "TwoAxisArmKinematics.h"
#include "forceController.h"
#include <Arduino.h>
#include <Derivs_Limiter.h>
#include <ESP32_easy_wifi_data.h>
#include <HX711.h>
#include <JMotor.h>

const byte torque1SensorDTPin = 19;
const byte torque1SensorSCKPin = 18;
const byte torque2SensorDTPin = 5;
const byte torque2SensorSCKPin = 17;

//documentation for JMotor library here: https://joshua-8.github.io/JMotor/hierarchy.html

JMotorDriverEsp32Servo servo1Driver = JMotorDriverEsp32Servo(8, 25); //pwm channel, pin
JServoController servo1 = JServoController(servo1Driver);
JMotorDriverEsp32Servo servo2Driver = JMotorDriverEsp32Servo(9, 26); //pwm channel, pin
JServoController servo2 = JServoController(servo2Driver);

JMotorDriverEsp32Servo servoSweeperDriver = JMotorDriverEsp32Servo(10, 33); //pwm channel, pin
JServoController servoSweeper = JServoController(servoSweeperDriver);

JVoltageCompConst motorVoltageComp = JVoltageCompConst(5);
JMotorCompStandardConfig motorCompConfig = JMotorCompStandardConfig(3.0, 60, 4.5, 128, 5, 160, 50);
JMotorDriverEsp32L293 motor1Driver = JMotorDriverEsp32L293(3, 23, 22, 19); //portA //pdw channel, enable, dirA, dirB
JEncoderQuadratureAttachInterrupt motor1Encoder = JEncoderQuadratureAttachInterrupt(34, 35, 360.0 / (151.002 * 28));
JMotorCompStandard motor1Compensator = JMotorCompStandard(motorVoltageComp, motorCompConfig);
JControlLoopBasic motor1ControlLoop = JControlLoopBasic(/*P*/ 30);
JMotorControllerClosed motor1Controller
    = JMotorControllerClosed(motor1Driver, motor1Compensator, motor1Encoder, motor1ControlLoop, 150, 180, 5, false, 2);

jENCODER_MAKE_ISRS_MACRO(motor1Encoder);

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

//force controller using pid to maintain constant force when peeling a page
forceController fc = forceController();

//SETTINGS relevant to the book being used
float hoverX = 14; // coordinate to move the servo arm (x direction) when beginning turn page routine
float hoverY = 10; // coordinate to move the servo arm (y direction) when beginning turn page routine
float targetForceY = -.08; // how much force (y direction) is being applied on the book
float peelDist = .5; //how far to move tape wheel along book
float peelTime = 2; //how long peel motion should take
float liftHeight = 7; //how far to lift up after peeling up a single page
float liftX = 9; //how far to move in after peeling up a single page
float downSpeed = 1; // what speed to move arm towards page at

//direction to turn the page (BACKWARD makes page move right, FORWARD makes the page move left)
typedef enum {
    BACKWARD = -1,
    FORWARD = 1,
} direction;
direction DIRECTION = FORWARD;

//list of states for state machine
typedef enum {
    START = 0,
    IDLE = 1,
    TP_SETUP = 2,
    TP_STEP_1_BEGIN = 3,
    TP_STEP_2_DOWN = 4,
    TP_STEP_3_PEEL = 5,
    TP_STEP_4_LIFT = 6,
    TP_STEP_5a_SWEEP = 7,
    TP_STEP_5b_SWEEP = 8,
    TP_STEP_6_CLAMP = 9,
    TP_STEP_7_CLEANUP = 10,
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
    hoverX = EWD::recvFl();
    hoverY = EWD::recvFl();
    targetForceY = EWD::recvFl();
    peelDist = EWD::recvFl();
    peelTime = EWD::recvFl();
    liftHeight = EWD::recvFl();
    liftX = EWD::recvFl();
    downSpeed = EWD::recvFl();
}
void WifiDataToSend()
{
    EWD::sendFl(servo1.getPos());
    EWD::sendFl(servo2.getPos());
    EWD::sendFl(torque1);
    EWD::sendFl(torque2);
    EWD::sendFl(Fx);
    EWD::sendFl(Fy);
    EWD::sendIn(CURRENT_STATE);
}

void setup()
{
    Serial.begin(115200);
    Serial.println("---starting---");

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

    servoSweeper.setConstrainRange(false);
    servoSweeper.setVelAccelLimits(180, 180);
    servoSweeper.setServoRangeValues(1000, 2000);
    servoSweeper.setSetAngles(-45, 45);
    servoSweeper.setAngleLimits(-90, 90);
    servoSweeper.setAngleImmediate(0);

    motor1Encoder.setUpInterrupts(motor1Encoder_jENCODER_ISR_A, motor1Encoder_jENCODER_ISR_B);
    motor1Driver.enable();
    motor1Driver.set(0); //make sure motor isn't turning
    motor1Driver.disable();

    //torque load cells
    torque1Sensor.begin(torque1SensorDTPin, torque1SensorSCKPin); //hx711 DT, SCK
    torque1Sensor.set_scale(18200); //calibrate sensor by changing this value
    if (torque1Sensor.is_ready()) {
        torque1Sensor.tare();
    } else {
        Serial.println("ERROR connecting to torque sensor 1");
    }

    torque2Sensor.begin(torque2SensorDTPin, torque2SensorSCKPin); //hx711 DT, SCK
    torque2Sensor.set_scale(44000); //calibrate sensor by changing this value
    if (torque2Sensor.is_ready()) {
        torque2Sensor.tare();
    } else {
        Serial.println("ERROR connecting to torque sensor 2");
    }

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
        NEXT_STATE = state_idle();
        break;
    case TP_SETUP:
        NEXT_STATE = state_tp_setup();
        break;
    case TP_STEP_1_BEGIN:
        NEXT_STATE = state_tp_step_1_begin();
        break;
    case TP_STEP_2_DOWN:
        NEXT_STATE = state_tp_step_2_down();
        break;
    case TP_STEP_3_PEEL:
        NEXT_STATE = state_tp_step_3_peel();
        break;
    case TP_STEP_4_LIFT:
        NEXT_STATE = state_tp_step_4_lift();
        break;
    case TP_STEP_5a_SWEEP:
        NEXT_STATE = state_tp_step_5a_sweep();
        break;
    case TP_STEP_5b_SWEEP:
        NEXT_STATE = state_tp_step_5b_sweep();
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
    servoSweeper.setEnable(enabled);
    motor1Driver.setEnable(enabled);
    motor1Controller.setVelTarget(xTarg * 10, false);
    torque1 += 0.001 * (motor1Controller.controlLoop.getError() - torque1);

    // if (torque1Sensor.is_ready()) {
    //     torque1 = torque1Sensor.get_units();
    // }
    // if (torque2Sensor.is_ready()) {
    //     torque2 = -torque2Sensor.get_units(); //note negative sign because loadcell was installed backwards
    // }

    torqueToForces(theta1, theta2, torque1, torque2, Fx, Fy, length_arm_1, length_arm_2, x, y);

    run_state();

    //smooth acceleration of arm movement
    x = xLimiter.calc();
    y = yLimiter.calc();
    //calculate arm kinematics (by having this separate outside the state machine, states can't directly set servo angles)
    if (cartToAngles(x, y, theta1, theta2, ARM_SETTINGS)) {
        servo1.setAngleSmoothed(theta1);
        servo2.setAngleSmoothed(theta2);
    }

    //run motor controllers
    motor1Controller.run();
    servo1.run();
    servo2.run();
    servoSweeper.run();
    delay(1);
}