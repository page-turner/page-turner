#define EWDmaxWifiSendBufSize 100
#define EWDmaxWifiRecvBufSize 100

#include "RunningAverage.h"
#include "TwoAxisArmKinematics.h"
#include "forceController.h"
#include <Arduino.h>
#include <Derivs_Limiter.h>
#include <ESP32_easy_wifi_data.h>
#include <JMotor.h>
#include "states.h"

// Controller Libraries to abstract the interaction of servos and motors
// Documentation for JMotor library here: https://joshua-8.github.io/JMotor/hierarchy.html
JMotorDriverEsp32Servo sweeperDriver = JMotorDriverEsp32Servo(10, 26); //pwm channel, pin
JServoController sweeper = JServoController(sweeperDriver);
JMotorDriverEsp32Servo leftClampDriver = JMotorDriverEsp32Servo(11, 14); //pwm channel, pin
JServoController leftClamp = JServoController(leftClampDriver);
JMotorDriverEsp32Servo rightClampDriver = JMotorDriverEsp32Servo(12, 27); //pwm channel, pin
JServoController rightClamp = JServoController(rightClampDriver);
JMotorDriverEsp32Servo centerClampDriver = JMotorDriverEsp32Servo(13, 12); //pwm channel, pin
JServoController centerClamp = JServoController(centerClampDriver);

JVoltageCompConst motorVoltageComp = JVoltageCompConst(5);

JMotorDriverEsp32L293 motor1Driver = JMotorDriverEsp32L293(3, 19, 18, 5); //pdw channel, enable, dirA, dirB
JEncoderAS5048bI2C encoder1 = JEncoderAS5048bI2C(true, 360.0 * 34 / 25, 64, 200000, 100, true); //reverse, distPerCountFactor, address, velEnoughTime, velEnoughTicks, recognizeOutOfRange
JMotorCompStandardConfig motor1CompConfig = JMotorCompStandardConfig(2.45, 16.5, 2.8, 25, 5, 70, 50);
JMotorCompStandard motor1Compensator = JMotorCompStandard(motorVoltageComp, motor1CompConfig);
JControlLoopBasic motor1ControlLoop = JControlLoopBasic(/*P*/ 20);
JMotorControllerClosed motor1Controller
    = JMotorControllerClosed(motor1Driver, motor1Compensator, encoder1, motor1ControlLoop, 60, 80, 5, false, 2); //velLimit, accelLimit, distFromSetpointLimit, preventGoingWrongWay, maxStoppingAccel

JMotorDriverEsp32L293 motor2Driver = JMotorDriverEsp32L293(2, 4, 16, 17); //pdw channel, enable, dirA, dirB
JEncoderAS5048bI2C encoder2 = JEncoderAS5048bI2C(true, 360.0 * 27 / 25, 68, 200000, 100, true); //reverse, distPerCountFactor, address, velEnoughTime, velEnoughTicks, recognizeOutOfRange
JMotorCompStandardConfig motor2CompConfig = JMotorCompStandardConfig(3.3, 15, 3.45, 15, 5, 92, 50);
JMotorCompStandard motor2Compensator = JMotorCompStandard(motorVoltageComp, motor2CompConfig);
JControlLoopBasic motor2ControlLoop = JControlLoopBasic(/*P*/ 20);
JMotorControllerClosed motor2Controller
    = JMotorControllerClosed(motor2Driver, motor2Compensator, encoder2, motor2ControlLoop, 60, 80, 5, false, 2); //velLimit, accelLimit, distFromSetpointLimit, preventGoingWrongWay, maxStoppingAccel

RunningAverage torque1Smoother = RunningAverage(150);
RunningAverage torque2Smoother = RunningAverage(150);

bool enabled = false; //received over wifi
float xTarg = 0; //for debug, received over wifi
float yTarg = 0; //for debug, received over wifi
bool go = false; //for debug, received over wifi
//global variables for force measurements
float torque1 = 0;
float torque2 = 0;
float torque1Zero = 0;
float torque2Zero = 0;
float Fx = 0;
float Fy = 0;
const float torque1Calibration = 0.28;
const float torque2Calibration = 1.0;
//arm position
float x = 0;
float y = 0;
//arm angles
float theta1 = 0;
float theta2 = 0;
//arm constants
const float MIN_Y = 0; // if book not detected while arm going down, continue to next step anyways if y goes below this value
const float length_arm_1 = 14; // in cm
const float length_arm_2 = 13; // in cm
const float theta1Min = -55;
const float theta1Max = 55;
const float theta2Min = -175;
const float theta2Max = 175;
#define ARM_SETTINGS length_arm_1, length_arm_2, theta1Min, theta1Max, theta2Min, theta2Max

//raw encoder units
int encoder1Zero = 10745;
int encoder2Zero = 8131;

//state machine variables
unsigned long millis_since_last_state_update = 0;
unsigned long millis_when_state_changed = 0;
bool did_state_change = true;

//smooth acceleration for arm position
Derivs_Limiter xLimiter = Derivs_Limiter(8, 6); //vel, accel
Derivs_Limiter yLimiter = Derivs_Limiter(8, 6);

//force controller using pid to maintain constant force when peeling a page
forceController fc = forceController();

//SETTINGS relevant to the book being used
float hoverX = 14; // coordinate to move the servo arm (x direction) when beginning turn page routine
float hoverY = 18; // coordinate to move the servo arm (y direction) when beginning turn page routine
float targetForceY = -.9; // how much force (y direction) is being applied on the book
float peelDist = 1; //how far to move tape wheel along book
float peelTime = 2; //how long peel motion should take
float liftHeight = 15; //how far to lift up after peeling up a single page
float liftX = 9; //how far to move in after peeling up a single page
float downSpeed = 3; // what speed to move arm towards page at
float DIST_DOWN_BEFORE_TARE = 5; //how far to go down before tareing and enabling sensing (this distance allows the torque measurements to stabilize)
float clampOpenAngle = 1; // angle to open the 3 clamps at
float clampClosedAngle = 0; // angle to close the 3 clamps at, also the resting position

//direction to turn the page (BACKWARD makes page move right, FORWARD makes the page move left)
typedef enum {
    BACKWARD = -1,
    FORWARD = 1,
} direction;
direction DIRECTION = FORWARD;

//list of states for state machine
typedef enum {
    START,
    IDLE,
    TP_SETUP,
    TP_STEP_1_BEGIN,
    TP_STEP_2A_DOWN,
    TP_STEP_2B_DOWN,
    TP_STEP_3_OPEN_SIDE_CLAMP,
    TP_STEP_4_PEEL,
    TP_STEP_5_LIFT,
    TP_STEP_6_CLOSE_SIDE_CLAMP,
    TP_STEP_7a_SWEEP,
    TP_STEP_7b_SWEEP,
    TP_STEP_8_OPEN_SIDE_CLAMP,
    TP_STEP_9_CLOSE_SIDE_CLAMP,
    TP_STEP_10_CLEANUP,
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
    EWD::sendFl(motor1Controller.getPos());
    EWD::sendFl(motor2Controller.getPos());
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

    motor1Driver.reverse = true;
    motor1Driver.pwmDriver.setFrequencyAndResolution(5000);
    motor1Driver.enable();
    motor1Driver.set(0); //make sure motor isn't turning
    motor1Driver.disable();

    motor2Driver.reverse = false;
    motor2Driver.pwmDriver.setFrequencyAndResolution(5000);
    motor2Driver.enable();
    motor2Driver.set(0); //make sure motor isn't turning
    motor2Driver.disable();

    //set up and calibrate servos
    sweeper.setConstrainRange(false);
    sweeper.setVelAccelLimits(.5, .5);
    sweeper.setServoRangeValues(1000, 2000);
    sweeper.setSetAngles(BACKWARD, FORWARD);
    sweeper.setAngleLimits(BACKWARD, FORWARD);
    sweeper.setAngleImmediate(FORWARD);

    leftClamp.setServoRangeValues(550, 1600);
    leftClamp.setSetAngles(clampClosedAngle, clampOpenAngle);
    leftClamp.setAngleLimits(clampOpenAngle, clampClosedAngle);
    leftClamp.setAngleImmediate(clampClosedAngle);

    rightClamp.setServoRangeValues(550, 1600);
    rightClamp.setSetAngles(clampClosedAngle, clampOpenAngle);
    rightClamp.setAngleLimits(clampOpenAngle, clampClosedAngle);
    rightClamp.setAngleImmediate(clampClosedAngle);

    centerClamp.setServoRangeValues(550, 1600);
    centerClamp.setSetAngles(clampClosedAngle, clampOpenAngle);
    centerClamp.setAngleLimits(clampOpenAngle, clampClosedAngle);
    centerClamp.setAngleImmediate(clampClosedAngle);

    torque1Smoother.fillValue(0, torque1Smoother.getSize());
    torque2Smoother.fillValue(0, torque2Smoother.getSize());

    Wire.begin();

    EWD::routerName = "Brown-Guest"; //name of the wifi network you want to connect to
    EWD::routerPass = "-open-network-"; //password for your wifi network (enter "-open-network-" if the network has no password) (default: -open-network-)
    EWD::wifiPort = 25210; //what port the esp32 communicates on if connected to a wifi network (default: 25210)
    EWD::setupWifi(WifiDataToParse, WifiDataToSend);

    encoder1.setEncoderZero(encoder1Zero);
    encoder2.setEncoderZero(encoder2Zero);

    yLimiter.setPosition(length_arm_1 + length_arm_2);
}

/**
 * @brief  state machine framework, add state functions to states.h, and call them from here
 * Diagram of the state machine available here: https://docs.google.com/drawings/d/e/2PACX-1vSXIanYaCZT8zV0G8m5wtqqAXXVGU7EXHymEftb0oeUDFG-PJwq8bg7bHy57mrLzCQ-i5nggW-QJ09F/pub?w=894&h=1700
 * The state machine is such that the robot is only in one state at a time, and each state is a step in turning a page (eg. moving the arm above the page, peeling the page, lifting the page, etc.)
 * The intention is to break down the actions and steps of the robot to make testing and iteration easier.
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
    case TP_STEP_2A_DOWN:
        NEXT_STATE = state_tp_step_2a_down();
        break;
    case TP_STEP_2B_DOWN:
        NEXT_STATE = state_tp_step_2b_down();
        break;
    case TP_STEP_3_OPEN_SIDE_CLAMP:
        NEXT_STATE = state_tp_step_3_open_side_clamp();
        break;
    case TP_STEP_4_PEEL:
        NEXT_STATE = state_tp_step_4_peel();
        break;
    case TP_STEP_5_LIFT:
        NEXT_STATE = state_tp_step_5_lift();
        break;
    case TP_STEP_6_CLOSE_SIDE_CLAMP:
        NEXT_STATE = state_tp_step_6_close_side_clamp();
        break;
    case TP_STEP_7a_SWEEP:
        NEXT_STATE = state_tp_step_7a_sweep();
        break;
    case TP_STEP_7b_SWEEP:
        NEXT_STATE = state_tp_step_7b_sweep();
        break;
    case TP_STEP_8_OPEN_SIDE_CLAMP:
        NEXT_STATE = state_tp_step_8_open_side_clamp();
        break;
    case TP_STEP_9_CLOSE_SIDE_CLAMP:
        NEXT_STATE = state_tp_step_9_close_side_clamp();
        break;
    case TP_STEP_10_CLEANUP:
        NEXT_STATE = state_tp_step_10_cleanup();
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
    sweeper.setEnable(enabled);
    leftClamp.setEnable(enabled);
    rightClamp.setEnable(enabled);
    centerClamp.setEnable(enabled);
    motor1Controller.setEnable(enabled);
    motor2Controller.setEnable(enabled);

    torque1Smoother.addValue(torque1Calibration * motor1Controller.controlLoop.getCtrlLoopOut());
    torque1 = torque1Smoother.getAverage() - torque1Zero;
    torque2Smoother.addValue(torque2Calibration * motor2Controller.controlLoop.getCtrlLoopOut());
    torque2 = torque2Smoother.getAverage() - torque2Zero;

    run_state();

    //smooth acceleration of arm movement
    x = xLimiter.calc();
    y = yLimiter.calc();
    //calculate arm kinematics (by having this separate outside the state machine, states can't directly set servo angles)
    if (cartToAngles(x, y, theta1, theta2, ARM_SETTINGS)) {
        motor1Controller.setPosTarget(theta1);
        motor2Controller.setPosTarget(theta2 + theta1);
    }

    torqueToForces(theta1, theta2, torque1, torque2, Fx, Fy, length_arm_1, length_arm_2, x, y);

    //run motor controllers
    motor1Controller.run();
    motor2Controller.run();
    sweeper.run();
    leftClamp.run();
    rightClamp.run();
    centerClamp.run();
    delay(1);
}