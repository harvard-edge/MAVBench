#ifndef PID_H
#define PID_H

class PID
{
public:
	PID(double Kp, double Ki, double Kd, double max, double min);
	double calculate(double sensor, double target, double dt);
	void reset();

private:
	double _Kp, _Ki, _Kd, _max, _min, _integral, _prev_error;
};

#endif
