/**
 * @brief  states.h contains functions for each state the pageturner can be in
 * Each state function should return CURRENT_STATE to get run again, or can switch to a different state by returning that state's value.
 * did_state_change: (bool) true if the state just changed
 * millis_since_last_state_update: (unsigned long) how many milliseconds has the current state be running for. 0 if state just changed
 */
#include <Arduino.h>

/**
 * @brief  wait for signal to turn page
 */
state state_idle()
{
    if (go && enabled) { //for debug, wait for 'g' keypress from computer
        torque1Sensor.tare();
        torque2Sensor.tare();
        //return A;
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