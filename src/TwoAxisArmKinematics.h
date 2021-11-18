#ifndef _TWO_AXIS_ARM_KINEMATICS_H_
#define _TWO_AXIS_ARM_KINEMATICS_H_
#include <Arduino.h>
/**
 * @brief  reverse kinematics formula for a 2D arm
 * @param  x: (float) x coordinate
 * @param  y: (float) y coordinate
 * @param  theta1: (float&) angle for first servo 
 * @param  theta2: (float&) angle for second servo
 * @param  d1: (float) length of first arm segment
 * @param  d2: (float) length of second arm segment
 * @param  theta1Min: (float) low range of servo1
 * @param  theta1Max: (float) high range of servo1
 * @param  theta2Min: (float) low range of servo2
 * @param  theta2Max: (float) high range of servo2
 * @retval (bool) true if coordinates possible to reach and angles updated, false if angles not updated
 */
bool cartToAngles(float x, float y, float& theta1, float& theta2, float d1, float d2, float theta1Min, float theta1Max, float theta2Min, float theta2Max)
{
    float potential_theta2 = acos((-sq(x) - sq(y) + sq(d1) + sq(d2)) / (2 * d1 * d2));
    if (x < 0) {
        potential_theta2 = -potential_theta2;
    }
    float potential_theta1 = asin((d2 * sin(potential_theta2)) / sqrt(sq(x) + sq(y))) + atan(y / x) - PI / 2;
    if (x < 0) {
        potential_theta1 = potential_theta1 + PI;
    }
    potential_theta1 = degrees(potential_theta1);
    potential_theta2 = degrees(potential_theta2);
    if (isnan(potential_theta1) || isnan(potential_theta2) || potential_theta1 < theta1Min || potential_theta1 > theta1Max || potential_theta2 < theta2Min || potential_theta2 > theta2Max) {
        return false;
    }
    theta1 = potential_theta1;
    theta2 = potential_theta2;
    return true;
}

/**
 * @brief  convert torque of the two load cells into force being applied by the arm
 * @param  theta1: (float) angle of first servo
 * @param  theta2: (float) angle of second servo
 * @param  torque1: (float) torque of first servo
 * @param  torque2: (float) torque of second servo
 * @param  Fx: (float&) horizontal component of force (this variable is changed)
 * @param  Fy: (float&) vertical component of force (this variable is changed)
 * @param  d1: (float) length of first arm
 * @param  d2: (float) length of second arm
 * @param  x: (float) x coordinate used to calculate thetas
 * @param  y: (float) y coordinate used to calculate thetas
 * @retval None
 */
void torqueToForces(float theta1, float theta2, float torque1, float torque2, float& Fx, float& Fy, float d1, float d2, float x, float y)
{
    theta1 = radians(theta1);
    theta2 = radians(theta2);
    //(px,py) is location of second joint
    float px = -sin(theta1) * d1;
    float py = cos(theta1) * d1;
    float r_squared = sq(x) + sq(y);
    //Thanks Eli for help with this formula!
    Fx = torque2 * (py - y) / sq(d2) + torque1 * -y / r_squared;
    Fy = torque2 * (x - px) / sq(d2) + torque1 * x / r_squared;
}

#endif