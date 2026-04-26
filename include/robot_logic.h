#ifndef ROBOT_LOGIC_H
#define ROBOT_LOGIC_H

#include "types.h"
#include <vector>

extern std::vector<EnhancedRoutePoint> route;

void initLogic();
void updateLogic(RobotState &currentState);
void dumpEnhancedRoute(unsigned long travelTime);
void resetLogic();

#endif