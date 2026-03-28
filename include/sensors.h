#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>

struct SensorData {
    int irValues[8];      // Ruwe analoge waarden
    uint8_t sensorMask;   // De 11000011 weergave
    float linePosition;   // Waarde tussen -1.0 (links) en 1.0 (rechts). 0.0 is centraal.
    float distance;       // Gefilterde afstand in cm
    long leftTicks;
    long rightTicks;
    float distanceDriven; // Gemiddelde afstand in cm
    bool lineDetected;    // Zien we überhaupt een lijn?
};

// Maak de data beschikbaar voor main.cpp
extern volatile SensorData currentSensors;


void startSensorTask(); // Start de loop op Core 0
float getLinePosition(); // Helper functie voor PID


#endif