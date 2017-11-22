#ifndef COMMON_H
#define COMMON_H

#include <cstdarg>
#include "Drone.h"

enum slam_recovery_method { spin, backtrack };

void sigIntHandler(int sig);

void action_upon_panic(Drone& drone);
void action_upon_future_col(Drone& drone);
void action_upon_slam_loss(Drone& drone, slam_recovery_method slm...);

float distance(float x, float y, float z);
void spin_around(Drone &drone);
void scan_around(Drone &drone, int angle);
void spin_slowly(Drone &drone, int n_pies=20);

#endif

