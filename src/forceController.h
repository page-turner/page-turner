#ifndef _FORCE_CONTROLLER_H_
#define _FORCE_CONTROLLER_H_
#include <Arduino.h>

class forceController {
public:
    double p, i, d; // proportional, integral, derivative
    unsigned long currentTime, previousTime;
    double elapsedTime;
    double input, output, target; // input and target (setpoint) in terms of force
    double error, lastError, cumulativeError, rateError;

    /**
     * @brief 
     * @param p
     * @param i
     * @param d
     * @param target
     * @returns None
     */
    void forceControllerSetup(double _p, double _i, double _d, double _target) {
        p = _p;
        i = _i;
        d = _d;
        target = _target;
    }

    /**
     * @brief maintain a constant force from the arm by appling small changes to it's position (PID)
     * @param
     * @returns output force, how much the force needs to be changed by
     */
    double forceControllerUpdate(double currentForce) {
        input = currentForce; 
        output = computePID();
        return output;
    }

    /**
     * @brief
     * @returns None
     */
    void forceControllerReset() {
        currentTime, previousTime = 0;
        elapsedTime = 0;
        p, i, d, target = 0;
    }

protected:
    /**
     * @brief maintain a constant force from the arm by appling small changes to it's position (PID)
     * @param
     * @returns output force, how much the force needs to be changed by
     */
    virtual double computePID(){     
        currentTime = millis();                                     // get current time
        elapsedTime = (double)(currentTime - previousTime);         // compute time elapsed from previous computation
        error = target - input;                                     // determine error
        cumulativeError += error * elapsedTime;                     // compute integral
        rateError = (error - lastError)/elapsedTime;                // compute derivative
        double _output = p*error + i*cumulativeError + d*rateError; // PID output               
        lastError = error;                                          // remember current error
        previousTime = currentTime;                                 // remember current time
        return _output;                                             // have function return the PID output
    }
};

#endif