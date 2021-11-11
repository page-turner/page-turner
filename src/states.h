/**
 * @brief  states.h contains functions for each state the pageturner can be in
 * Each state function should return CURRENT_STATE to get run again, or can switch to a different state by returning that state's value.
 * did_state_change: (bool) true if the state just changed
 * millis_since_last_state_update: (unsigned long) how many milliseconds has the current state been running for. 0 if state just changed
 */
#include <Arduino.h>

/**
 * @brief  wait for signal to turn page
 */
state state_idle()
{
    if (go && enabled) { //for debug, wait for 'g' keypress from computer
        return A;
    }
    xLimiter.setTarget(xTarg); //for debug, manual control from wifi
    yLimiter.setTarget(yTarg);

    return CURRENT_STATE;
}

state state_A()
{
    if (did_state_change) {
        xLimiter.setTarget(10);
        yLimiter.setTarget(5);
    }

    if (xLimiter.isPosAtTarget() && yLimiter.isPosAtTarget()) {
        return B;
    }

    return CURRENT_STATE;
}
state state_B()
{
    if (did_state_change) {
        xLimiter.setTarget(5);
        yLimiter.setTarget(10);
    }

    if (xLimiter.isPosAtTarget() && yLimiter.isPosAtTarget()) {
        return IDLE;
    }

    return CURRENT_STATE;
}

/* STATES ARE IN PROGRESS AND NEED TO BE TESTED*/
state state_tp_setup()
{
    if (did_state_change) {
        // reset variables (if any)
        // 0 out load cell calibration
        // DETERMINE WHICH DIRECTION TO GO IN
        DIRECTION = LEFT;
    }
    return TP_STEP_1_BEGIN;
}

state state_tp_step_1_begin() {
    if (did_state_change) {
        // xLimiter.setTarget(5);
        // yLimiter.setTarget(10);
    }
    if (xLimiter.isPosAtTarget() && yLimiter.isPosAtTarget()) {
        return TP_STEP_2_DOWN;
    }
    return CURRENT_STATE; 
}

state state_tp_step_2_down() {
    if (did_state_change) {
        // set force needed and tell servos to go down slowly
    }
    if (0 /* calculated force is sufficient */) {
        return TP_STEP_3_PEEL;
    }
    return CURRENT_STATE; 
}

state state_tp_step_3_peel() {
    if (did_state_change) {
        // xLimiter.setTarget(5);
        // yLimiter.setTarget(10);
    }
    if (xLimiter.isPosAtTarget() && yLimiter.isPosAtTarget()) {
        return TP_STEP_4_LIFT;
    }
    // make sure xy of servos changes with respect to force being applied (keep force constant)
    return CURRENT_STATE; 
}

state state_tp_step_4_left() {
    if (did_state_change) {
        // xLimiter.setTarget(5);
        // yLimiter.setTarget(10);
    }
    if (xLimiter.isPosAtTarget() && yLimiter.isPosAtTarget()) {
        return IDLE; // change later to TP_STEP_5_SWING
    }
    return CURRENT_STATE; 
}