/**
 * @brief  states.h contains functions for each state the pageturner can be in
 * Each state function should return CURRENT_STATE to get run again, or can switch to a different state by returning that state's value
 * The idea is that a state will always return CURRENT_STATE until a condition to transition to a new state has been met
 * did_state_change: (bool) true if the state just changed, useful for code that needs to be run just once in a given state
 * millis_since_last_state_update: (unsigned long) how many milliseconds has the current state been running for. 0 if state just changed
 */
#include <Arduino.h>
bool armAtTarget()
{
    return (xLimiter.isPosAtTarget() && yLimiter.isPosAtTarget() && motor1Controller.posSetpointSmoother.isPosAtTarget() && motor2Controller.posSetpointSmoother.isPosAtTarget());
}
/**
 * @brief  set clamp position of the clamp on the side where the page to turn starts
 * @param  pos: (use clampAngleOpen or clampAngleClosed)
 * @retval None
 */
void setFirstClamp(float pos)
{
    DIRECTION == BACKWARD ? leftClamp.setAngleSmoothed(pos) : rightClamp.setAngleSmoothed(pos);
}
/**
 * @brief  set clamp position of the clamp on the side where the page to turn ends
 * @param  pos: (use clampAngleOpen or clampAngleClosed)
 * @retval None
 */
void setSecondClamp(float pos)
{
    DIRECTION == FORWARD ? leftClamp.setAngleSmoothed(pos) : rightClamp.setAngleSmoothed(pos);
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
    if (enabled) { //for debug
        xLimiter.setTarget(xTarg);
        yLimiter.setTarget(yTarg);
    } else {
        xLimiter.setTarget(0);
        yLimiter.setTarget(length_arm_1 + length_arm_2);
    }

    bool enab = !(armAtTarget() && abs(motor1Controller.getPos() - theta1) < 5 && abs(motor2Controller.getPos() - theta2) < 5
        && abs(motor1Controller.getVel()) < 5 && abs(motor2Controller.getVel()) < 5);

    motor1Controller.setEnable(enab);
    motor2Controller.setEnable(enab);
    leftClamp.disable();
    rightClamp.disable();
    centerClamp.disable();
    sweeper.disable();

    if (go) {
        DIRECTION = BACKWARD;
        return TP_SETUP;
    }

    if (digitalRead(BUTTON_PIN) == LOW) {
        DIRECTION = FORWARD;
        return TP_SETUP;
    }
    return CURRENT_STATE;
}

/**
  * @brief  Begin the process to turn a page, set up variables
  */
state state_tp_setup()
{
    motor1Controller.enable();
    motor2Controller.enable();
    leftClamp.enable();
    rightClamp.enable();
    centerClamp.enable();
    sweeper.enable();
    sweeper.setAngleSmoothed(DIRECTION);
    if (sweeper.isPosAtTarget()) {
        sweeper.disable();
        return TP_STEP_1_BEGIN;
    }
    return CURRENT_STATE;
}

/**
  * @brief  move the arm to above the page to be turned, wait until the arm has reached it's target position
  */
state state_tp_step_1_begin()
{
    if (did_state_change) {
        xLimiter.setTarget(hoverX * DIRECTION);
        yLimiter.setTarget(hoverY);
    }
    if (armAtTarget()) {
        return TP_STEP_2A_DOWN;
    }
    return CURRENT_STATE;
}

/**
  * @brief  slowly move the arm down towards the page, wait until target position has reached, allow arm movement to smooth down before turning sensing on
  */
state state_tp_step_2a_down()
{
    if (y < hoverY - DIST_DOWN_BEFORE_TARE) {
        tareTorques();
        return TP_STEP_2B_DOWN;
    }
    // tell servos to go down slowly
    yLimiter.jogPosition(-downSpeed * yLimiter.getTimeInterval());
    return CURRENT_STATE;
}

/**
  * @brief  continue to slowly move the arm down towards the page, wait until the force target has been reached, implying that the arm is in contact with the book
  */
state state_tp_step_2b_down()
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

/**
  * @brief  open the clamp on the side of the page being turned, 
  */
state state_tp_step_3_open_side_clamp()
{
    if (did_state_change) {
        // open the clamp on the side of the page that's being turned
        setFirstClamp(clampOpenAngle);
    }
    if (millis_since_last_state_update > 200) {
        return TP_STEP_4_PEEL;
    }
    return CURRENT_STATE;
}

/**
  * @brief  move the arm along the page, wait until target position has been reached
  */
state state_tp_step_4_peel()
{
    if (did_state_change) {
        xLimiter.setTargetTimedMovePreferred(xLimiter.getPosition() - peelDist * DIRECTION, peelTime);
    }
    if (xLimiter.isPosAtTarget()) {
        xLimiter.resetVelLimitToOriginal();
        return TP_STEP_5_LIFT;
    }
    // [TODO] make sure y position changes with respect to force being applied (keep force constant) follow angle of book?
    yLimiter.setTarget(yLimiter.getPosition());
    return CURRENT_STATE;
}

/**
  * @brief  move the arm up and lift the page, wait until target position has been reached
  */
state state_tp_step_5_lift()
{
    if (did_state_change) {
        yLimiter.setTarget(yLimiter.getPosition() + liftHeight);
        xLimiter.setTarget(0.001 * DIRECTION); //basically zero, but make the arm fold the right way
    }
    // [TODO] try different paths for movement
    if (yLimiter.getTarget() - yLimiter.getPosition() < liftHeight * (1 - center_open_height)) {
        return TP_STEP_6_CLOSE_SIDE_CLAMP;
    }
    return CURRENT_STATE;
}

/**
  * @brief  close the side clamp and open the center clamp, wait for .2 seconds (approximately how long it takes for the clamp servos to complete their motion)
  */
state state_tp_step_6_close_side_clamp()
{
    if (did_state_change) {
        // close side clamp and open center clamp
        setFirstClamp(clampClosedAngle);
    }
    if (millis_since_last_state_update > 100) {
        centerClamp.setAngleSmoothed(clampOpenAngle);
    }
    if (armAtTarget()) {
        return TP_STEP_7_SWEEP;
    }
    return CURRENT_STATE;
}

/**
  * @brief  move the sweep servo from one side to the other, wait until target position has been reached
  */
state state_tp_step_7_sweep()
{
    if (did_state_change) {
        sweeper.enable();
        sweeper.setAngleSmoothed(DIRECTION * -1);
    }
    if (abs(sweeper.distToTarget()) < sideClampOpenSweeperDist) {
        setSecondClamp(clampOpenAngle);
    }
    if (abs(sweeper.distToTarget()) < centerClampCloseSweeperDist) {
        centerClamp.setAngleSmoothed(clampClosedAngle);

        return TP_STEP_8_CLAMP;
    }
    return CURRENT_STATE;
}

state state_tp_step_8_clamp()
{
    if (sweeper.isPosAtTarget()) {
        setSecondClamp(clampClosedAngle);
        return TP_STEP_9_SWEEPER_RETURN;
    }
    return CURRENT_STATE;
}

/**
  * @brief  move the sweep servo back to the original position, wait until target position has been reached
  */
state state_tp_step_9_sweeper_return()
{
    if (did_state_change) {
        xLimiter.setTarget(0);
        yLimiter.setTarget(length_arm_1 + length_arm_2);
        sweeper.setAngleSmoothed(DIRECTION);
    }

    if (sweeper.isPosAtTarget()) {
        sweeper.disable();
        return IDLE;
    }
    return CURRENT_STATE;
}

/**
  * @brief  disable then lock and require reset, for use if error detected
  */
state state_error()
{
    sweeper.disable();
    leftClamp.disable();
    rightClamp.disable();
    centerClamp.disable();
    motor1Controller.disable();
    motor2Controller.disable();
    digitalWrite(BUILTIN_LED, (millis() % 500) < 250);
    return CURRENT_STATE;
}