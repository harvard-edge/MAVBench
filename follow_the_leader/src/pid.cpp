#include "pid.h"
#include <iostream>
#include "ros/ros.h"

PID::PID(double Kp, double Ki, double Kd, double max, double min) :
	_Kp(Kp), _Ki(Ki), _Kd(Kd), _max(max), _min(min), _integral(0), _prev_error(0)
{
}

double PID::calculate(double sensor, double target, double dt)
{
	double result, prop, integ, deriv;
	double error = target - sensor;

	prop = _Kp * error;

	_integral += error * dt;
	integ = _Ki * _integral;

	deriv = _Kd * (error - _prev_error) / dt;
	_prev_error = error;

	result = prop + integ + deriv;
    
    //std::cout<<"error"<<error<<"result is"<<result <<std::endl;
	
    if (result >= _max){
	    //ROS_INFO_STREAM("max speed");
        return _max;
    }
	else if (result <= _min){
	    //ROS_INFO_STREAM("min speed");
        return _min;
    }
	else
        return result;
}

void PID::reset()
{
	_prev_error = 0;
	_integral = 0;
}
