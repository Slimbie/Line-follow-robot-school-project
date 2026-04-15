#ifndef LOGIC_H
#define LOGIC_H

#include <Arduino.h>

void calculatePID();
void calculateSpeedrun(); // DEZE TOEVOEGEN
void logSpeedrunPoint(float currentDist, float lineError, uint8_t type);
void resetPID();

#endif