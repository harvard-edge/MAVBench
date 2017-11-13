#include <iostream>
#include <cmath>
#include "Drone.h"
#include "ros/ros.h"


#ifndef COMMON_H
#define COMMON_H

void sigIntHandler(int sig);
void action_upon_panic(Drone& drone);
void action_upon_future_col(Drone& drone);
float distance(float x, float y, float z);
void spin_around(Drone &drone);
void scan_around(Drone &drone, int angle);
void spin_slowly(Drone &drone, int n_pies=20);

#endif
