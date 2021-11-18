/**
 * @brief  states.h contains functions for each state the pageturner can be in
 * Each state function should return CURRENT_STATE to get run again, or can switch to a different state by returning that state's value.
 * did_state_change: (bool) true if the state just changed
 * millis_since_last_state_update: (unsigned long) how many milliseconds has the current state been running for. 0 if state just changed
 */
#include <Arduino.h>

bool armAtTarget()
{
    return (xLimiter.isPosAtTarget() && yLimiter.isPosAtTarget() && servo1.isPosAtTarget() && servo2.isPosAtTarget());
}

/**
  * @brief  wait for signal to turn page
  */
state state_idle()
{
    if (go && enabled) { //for debug, wait for 'g' keypress from computer
        return TP_SETUP;
    }
    xLimiter.setTarget(xTarg); //for debug, manual control from wifi
    yLimiter.setTarget(yTarg);

    return CURRENT_STATE;
}

/* STATES ARE IN PROGRESS AND NEED TO BE TESTED*/
state state_tp_setup()
{
    if (did_state_change) {
        // reset variables (if any)
        // DETERMINE WHICH DIRECTION TO GO IN
        DIRECTION = FORWARD;
        servoSweeper.setAngleSmoothed(-90);
    }
    return TP_STEP_1_BEGIN;
}

state state_tp_step_1_begin()
{
    //go and hover above edge of book
    if (did_state_change) {
        xLimiter.setTarget(hoverX * DIRECTION);
    }
    //TODO: FOLLOW ARC INSTEAD??
    if (xLimiter.isPosAtTarget()) {
        yLimiter.setTarget(hoverY);
    }
    if (armAtTarget()) {
        return TP_STEP_2_DOWN;
    }
    return CURRENT_STATE;
}

state state_tp_step_2_down()
{
    if (did_state_change) {
        torque1Sensor.tare(3);
        torque2Sensor.tare(3);
        Fx = 0;
        Fy = 0;
        xLimiter.resetTime();
        yLimiter.resetTime();
        // tell servos to go down slowly
    }
    if (Fy < targetForceY) {
        return TP_STEP_3_PEEL;
    }
    yLimiter.jogPosition(-downSpeed * yLimiter.getTimeInterval());
    return CURRENT_STATE;
}

state state_tp_step_3_peel()
{
    if (did_state_change) {
        fc.forceControllerSetup(0, 0, 0, 0);
        xLimiter.setTargetTimedMovePreferred(xLimiter.getPosition() - peelDist * DIRECTION, peelTime);
    }
    if (xLimiter.isPosAtTarget()) {
        fc.forceControllerReset();
        xLimiter.resetVelLimitToOriginal();
        return TP_STEP_4_LIFT;
    }
    // [TODO] make sure y position changes with respect to force being applied (keep force constant) follow angle of book
    double changeInY = fc.forceControllerUpdate(Fx);
    yLimiter.setTarget(yLimiter.getPosition() + changeInY);
    return CURRENT_STATE;
}

state state_tp_step_4_lift()
{
    if (did_state_change) {
        yLimiter.setTarget(yLimiter.getPosition() + liftHeight);
        xLimiter.setTarget(xLimiter.getPosition() - liftX * DIRECTION);
    }
    // [TODO] try different paths for movement
    if (xLimiter.isPosAtTarget() && yLimiter.isPosAtTarget()) {
        return TP_STEP_5a_SWEEP;
    }
    return CURRENT_STATE;
}

state state_tp_step_5a_sweep()
{
    if (did_state_change) {
        servoSweeper.setAngleSmoothed(85);
    }
    if (servoSweeper.isPosAtTarget()) {
        return TP_STEP_5b_SWEEP;
    }
    return CURRENT_STATE;
}

state state_tp_step_5b_sweep()
{
    if (did_state_change) {
        servoSweeper.setAngleSmoothed(-90);
    }
    if (servoSweeper.isPosAtTarget()) {
        return IDLE; // change later to TP_STEP_6_CLAMP
    }
    return CURRENT_STATE;
}