#ifndef MAPPING_H
#define MAPPING_H

#include <Arduino.h>

struct PathPoint {
    float distance;      // Afstand vanaf start in cm
    float curve;         // De linePosition op dat punt
    uint8_t type;        // 0=Normaal, 1=Zigzag/Afsnijden
};

// We reserveren plek voor 500 punten (500 * 5cm = 25 meter parcours)
const int MAX_POINTS = 500;
extern PathPoint trackMap[MAX_POINTS];
extern int mapIndex;

void savePoint(float dist, float curve, uint8_t type);
void dumpMap();

#endif