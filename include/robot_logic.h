#ifndef ROBOT_LOGIC_H
#define ROBOT_LOGIC_H

#include "types.h"

extern std::vector<EnhancedRoutePoint> route;
extern std::vector<Node> nodeMap;

void initLogic();
Mode updateLogic(RobotState &currentState);
void dumpEnhancedRoute(unsigned long travelTime);
void prepareSpeedrun();
void resetLogic();

#endif