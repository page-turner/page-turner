#ifndef _TWO_AXIS_ARM_KINEMATICS_H_
#define _TWO_AXIS_ARM_KINEMATICS_H_

/**
 * @brief  reverse kinematics formula for a 2D arm
 * @param  x: (float) x coordinate
 * @param  y: (float) y coordinate
 * @param  theta1: (float*) angle for first servo 
 * @param  theta2: (float*) angle for second servo
 * @param  d1: (float) length of first arm segment* 
 * @param  d2: (float) length of second arm segment
 * @param  theta1Min: (float) low range of servo1
 * @param  theta1Max: (float) high range of servo1
 * @param  theta2Min: (float) low range of servo2
 * @param  theta2Max: (float) high range of servo2
 * @retval (bool) true if coordinates possible to reach and angles updated, false if angles not updates
 */
bool cartToAngles(float x, float y, float* theta1, float* theta2, float d1, float d2, float theta1Min, float theta1Max, float theta2Min, float theta2Max)
{
    return false;
}

#endif