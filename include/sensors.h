#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include "types.h" // Haalt SensorData hier vandaan

// --- EXTERN GEGEVENS ---
// Hiermee weten andere bestanden dat deze variabelen elders (in sensors.cpp) bestaan
extern volatile SensorData currentSensors;
extern const int sensorPinnen[8];

// --- FUNCTIES ---
void startSensorTask();
float readUltrasonic();

#endif