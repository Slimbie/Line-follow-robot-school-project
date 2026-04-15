#ifndef MAPPING_H
#define MAPPING_H

#include <Arduino.h>

struct PathPoint {
    float distance;
    float curve;
    uint8_t type; 
    int32_t leftTicks;  // NIEUW: voor je Python script
    int32_t rightTicks; // NIEUW: voor je Python script
};

const int MAX_POINTS = 500;
extern int mapIndex;

// In Mapping.h
extern PathPoint trackMap[MAX_POINTS];
extern PathPoint speedrunMap[MAX_POINTS];
extern int mapIndex;
extern int speedrunIndex;
extern unsigned long speedrunStartTime; // Voor de tijdmeting

void dumpMap(unsigned long travelTime);
void dumpSpeedrunMap(unsigned long travelTime); // Nu met tijd
// We voegen ticks toe aan de save functie
void savePoint(float dist, float curve, float obsDist, int32_t lTicks, int32_t rTicks); 
void dumpMap(unsigned long travelTime);
void resetMap(); 
int getMapIndexAtDistance(float currentDist);
void prepareSpeedrun();


#endif