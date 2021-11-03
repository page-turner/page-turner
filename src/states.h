
state state_idle()
{
    if (go && enabled) {
        return A;
    }
    x = xLimiter.calc(0);
    y = yLimiter.calc(length_arm_1 - length_arm_2);

    if (cartToAngles(x, y, theta1, theta2, ARM_SETTINGS)) {
        servo1.setAngleSmoothed(theta1);
        servo2.setAngleSmoothed(theta2);
    }

    return CURRENT_STATE;
}

state state_A()
{
    x = xLimiter.calc(10);
    y = yLimiter.calc(5);

    if (cartToAngles(x, y, theta1, theta2, ARM_SETTINGS)) {
        servo1.setAngleSmoothed(theta1);
        servo2.setAngleSmoothed(theta2);
    }
    if (xLimiter.isPosAtTarget() && yLimiter.isPosAtTarget()) {
        return B;
    }

    return CURRENT_STATE;
}
state state_B()
{
    x = xLimiter.calc(5);
    y = yLimiter.calc(10);

    if (cartToAngles(x, y, theta1, theta2, ARM_SETTINGS)) {
        servo1.setAngleSmoothed(theta1);
        servo2.setAngleSmoothed(theta2);
    }
    if (xLimiter.isPosAtTarget() && yLimiter.isPosAtTarget()) {
        return IDLE;
    }

    return CURRENT_STATE;
}