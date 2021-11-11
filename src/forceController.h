#ifndef _FORCE_CONTROLLER_H_
#define _FORCE_CONTROLLER_H_
#include <Arduino.h>

double p, i, d; // proportional, inegral, derivative
 
unsigned long currentTime, previousTime;
double elapsedTime;
double input, output, target;
double error, lastError, cumulativeError, rateError;


/**
 * @brief maintain a constant force from the arm by appling small changes to it's position (PID)
 * @param
 * @returns output force, how much the force needs to be changed by
 */
double pidConstantForceController(double currentForce) {
    forceControllerUpdate();
}

void forceControllerSetup(double _p, double _i, double _d, double _target) {
    p = _p;
    i = _i;
    d = _d;
    target = _target;
}

void forceControllerUpdate() {
    input = analogRead(A0); 
    output = computePID(input);

    computePID(input);
}


double computePID(double _input){     
    currentTime = millis();                                     //get current time
    elapsedTime = (double)(currentTime - previousTime);         //compute time elapsed from previous computation
    error = target - _input;                                    // determine error
    cumulativeError += error * elapsedTime;                     // compute integral
    rateError = (error - lastError)/elapsedTime;                // compute derivative
    double _output = p*error + i*cumulativeError + d*rateError; //PID output               
    lastError = error;                                          //remember current error
    previousTime = currentTime;                                 //remember current time
    return _output;                                             //have function return the PID output
}

#endif