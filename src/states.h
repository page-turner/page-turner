
state state_idle()
{
    if (go && enabled) {
        return A;
    }
    xLimiter.setTarget(0);
    yLimiter.setTarget(length_arm_1 - length_arm_2);

    return CURRENT_STATE;
}

state state_A()
{
    xLimiter.setTarget(10);
    yLimiter.setTarget(5);

    if (xLimiter.isPosAtTarget() && yLimiter.isPosAtTarget()) {
        return B;
    }

    return CURRENT_STATE;
}
state state_B()
{
    xLimiter.setTarget(5);
    yLimiter.setTarget(10);

    if (xLimiter.isPosAtTarget() && yLimiter.isPosAtTarget()) {
        return IDLE;
    }

    return CURRENT_STATE;
}