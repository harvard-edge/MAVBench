#include <iostream>
#include <cmath>
#include "Drone.h"
#include "ros/ros.h"


#ifndef COMMON_H
#define COMMON_H


float distance(float x, float y, float z);
void spin_around(Drone &drone);
void scan_around(Drone &drone, int angle);

#endif
