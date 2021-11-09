#include <Arduino.h>
state state_idle()
{
    if (go && enabled) {
        return A;
    }
    xLimiter.setTarget(xTarg);
    yLimiter.setTarget(yTarg);

    return CURRENT_STATE;
}

state state_A()
{
    if (CURRENT_STATE != PREVIOUS_STATE) {
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
    if (CURRENT_STATE != PREVIOUS_STATE) {
        xLimiter.setTarget(5);
        yLimiter.setTarget(10);
    }

    if (xLimiter.isPosAtTarget() && yLimiter.isPosAtTarget()) {
        return IDLE;
    }

    return CURRENT_STATE;
}