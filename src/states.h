/**
 * @brief  states.h contains functions for each state the pageturner can be in
 * Each state function should return CURRENT_STATE to get run again, or can switch to a different state by returning that state's value.
 * did_state_change: (bool) true if the state just changed
 * millis_since_last_state_update: (unsigned long) how many milliseconds has the current state been running for. 0 if state just changed
 */
#include <Arduino.h>

bool armAtTarget()
{
    return (xLimiter.isPosAtTarget() && yLimiter.isPosAtTarget() && motor1Controller.posSetpointSmoother.isPosAtTarget() && motor2Controller.posSetpointSmoother.isPosAtTarget());
}

void tareTorques()
{
    torque1Zero = torque1Smoother.getAverage();
    torque2Zero = torque2Smoother.getAverage();
}

/**
  * @brief  wait for signal to turn page
  */
state state_idle()
{
    if (go && enabled) { //for debug, wait for 'g' keypress from computer
        tareTorques();
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
        sweeper.setAngleSmoothed(-90);
    }
    return TP_STEP_1_BEGIN;
}

state state_tp_step_1_begin()
{
    //go and hover above edge of book
    if (did_state_change) {
        xLimiter.setTarget(hoverX * DIRECTION);
        yLimiter.setTarget(hoverY);
    }
    if (armAtTarget()) {
        return TP_STEP_2A_DOWN;
    }
    return CURRENT_STATE;
}
//move down, then tare the torque
state state_tp_step_2A_down()
{
    if (y < hoverY - DIST_DOWN_BEFORE_TARE) {
        tareTorques();
        return TP_STEP_2B_DOWN;
    }
    // tell servos to go down slowly
    yLimiter.jogPosition(-downSpeed * yLimiter.getTimeInterval());
    return CURRENT_STATE;
}
//move down until force or min_y reached
state state_tp_step_2B_down()
{
    if (did_state_change) {
        Fx = 0;
        Fy = 0;
    }
    if (Fy < targetForceY || y <= MIN_Y) {
        return TP_STEP_3_OPEN_SIDE_CLAMP;
    }
    // tell servos to go down slowly
    yLimiter.jogPosition(-downSpeed * yLimiter.getTimeInterval());
    return CURRENT_STATE;
}

state state_tp_step_3_open_side_clamp()
{
    if (did_state_change) {
        // open the clamp on the side of the page that's being turned
        DIRECTION == BACKWARD ? leftClamp.setAngleSmoothed(clampOpenAngle) : rightClamp.setAngleSmoothed(clampOpenAngle);
    }
    if (millis_since_last_state_update > 200) {
        return TP_STEP_4_PEEL;
    }
    return CURRENT_STATE;
}

state state_tp_step_4_peel()
{
    if (did_state_change) {
        fc.forceControllerSetup(0, 0, 0, 0);
        xLimiter.setTargetTimedMovePreferred(xLimiter.getPosition() - peelDist * DIRECTION, peelTime);
    }
    if (xLimiter.isPosAtTarget()) {
        fc.forceControllerReset();
        xLimiter.resetVelLimitToOriginal();
        return TP_STEP_5_LIFT;
    }
    // [TODO] make sure y position changes with respect to force being applied (keep force constant) follow angle of book
    double changeInY = fc.forceControllerUpdate(Fx);
    yLimiter.setTarget(yLimiter.getPosition() + changeInY);
    return CURRENT_STATE;
}

state state_tp_step_5_lift()
{
    if (did_state_change) {
        yLimiter.setTarget(yLimiter.getPosition() + liftHeight);
        xLimiter.setTarget(xLimiter.getPosition() - liftX * DIRECTION);
    }
    // [TODO] try different paths for movement
    if (xLimiter.isPosAtTarget() && yLimiter.isPosAtTarget()) {
        return TP_STEP_6_CLOSE_SIDE_CLAMP;
    }
    return CURRENT_STATE;
}

state state_tp_step_6_close_side_clamp()
{
    if (did_state_change) {
        // close side clamp and open center clamp
        DIRECTION == BACKWARD ? leftClamp.setAngleSmoothed(clampClosedAngle) : rightClamp.setAngleSmoothed(clampClosedAngle);
        centerClamp.setAngleSmoothed(85);
    }
    if (millis_since_last_state_update > 200) {
        return TP_STEP_7a_SWEEP;
    }
    return CURRENT_STATE;
}

state state_tp_step_7a_sweep()
{
    if (did_state_change) {
        sweeper.setAngleSmoothed(85);
    }
    if (sweeper.isPosAtTarget()) {
        return TP_STEP_7b_SWEEP;
    }
    return CURRENT_STATE;
}

state state_tp_step_7b_sweep()
{
    if (did_state_change) {
        sweeper.setAngleSmoothed(-90);
    }
    if (sweeper.isPosAtTarget()) {
        return TP_STEP_8_OPEN_SIDE_CLAMP;
    }
    return CURRENT_STATE;
}

state state_tp_step_8_open_side_clamp()
{
    if (did_state_change) {
        // open clamp on newly turned page side and close center clamp
        DIRECTION == BACKWARD ? rightClamp.setAngleSmoothed(clampOpenAngle) : leftClamp.setAngleSmoothed(clampOpenAngle);
        centerClamp.setAngleSmoothed(clampClosedAngle);
    }
    if (millis_since_last_state_update > 1500) {
        return TP_STEP_9_CLOSE_SIDE_CLAMP;
    }
    return CURRENT_STATE;
}

state state_tp_step_9_close_side_clamp()
{
    if (did_state_change) {
        DIRECTION == BACKWARD ? rightClamp.setAngleSmoothed(clampClosedAngle) : leftClamp.setAngleSmoothed(clampClosedAngle);
    }
    if (millis_since_last_state_update > 200) {
        return TP_STEP_10_CLEANUP;
    }
    return CURRENT_STATE;
}

state state_tp_step_10_cleanup()
{
    // if (did_state_change) { }
    return IDLE;
}