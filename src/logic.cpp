#include "Logic.h"
#include "Config.h"
#include "Sensors.h"
#include "Motors.h"

float lastError = 0;
float integral = 0;

void calculatePID() {
    // 1. Haal de foutwaarde op (0 is perfect op de lijn)
    float error = currentSensors.linePosition;

    // Als we de lijn helemaal kwijt zijn, doen we niets (of we gebruiken de laatste waarde)
    if (!currentSensors.lineDetected) {
        // Optioneel: als error > 0 draai rechtsom, als error < 0 draai linksom om lijn te zoeken
        return; 
    }

    // 2. Bereken PID componenten
    float P = error * KP;
    integral += error;
    float I = integral * KI;
    float D = (error - lastError) * KD;

    float turnCorrection = P + I + D;
    lastError = error;

    // 3. Pas toe op motorsnelheid
    int speedL = BASE_SPEED_MAPPING + turnCorrection;
    int speedR = BASE_SPEED_MAPPING - turnCorrection;

    // Zorg dat we binnen de PWM grenzen (0-255) blijven
    speedL = constrain(speedL, 0, MAX_SPEED);
    speedR = constrain(speedR, 0, MAX_SPEED);

    setMotorSpeed(speedL, speedR);
}

void resetPID() {
    lastError = 0;
    integral = 0;
}