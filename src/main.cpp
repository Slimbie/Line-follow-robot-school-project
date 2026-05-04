#include <Arduino.h>
#include "Config.h"
#include "Motors.h"
#include "Sensors.h"

// --- Opslag instellingen ---
struct DataPoint {
    float afstand;
    float error;
};

const int MAX_POINTS = 500; // 500 punten * 50ms = 25 seconden testtijd
DataPoint storage[MAX_POINTS];
int savedPoints = 0;

// --- Test Variabelen ---
int testBaseSpeed = 100; 
float lastLineError = 0;
int state = 0; // 0=idle, 1=rijden, 2=data dumpen

void setup() {
    Serial.begin(115200);
    setupMotors();
    startSensorTask();
    pinMode(START_BUTTON_PIN, INPUT_PULLUP);
    Serial.println("Robot Gereed. Druk op de knop om te STARTEN.");
}

void loop() {
    bool buttonPressed = (digitalRead(START_BUTTON_PIN) == LOW);

    // --- STATE 0: WACHTEN OP START ---
    if (state == 0) {
        if (buttonPressed) {
            delay(500); // Debounce
            savedPoints = 0;
            state = 1;
            Serial.println("TEST GESTART - Rijden...");
        }
    }

    // --- STATE 1: RIJDEN & DATA OPSLAAN ---
    else if (state == 1) {
        float currentError = currentSensors.linePosition;
        float afstand = currentSensors.distanceDriven;

        // PID Logica
        float adjustment = (KP * currentError) + (KD * (currentError - lastLineError));
        lastLineError = currentError;

        if (currentSensors.lineDetected) {
            setMotorSpeed(testBaseSpeed + (int)adjustment, testBaseSpeed - (int)adjustment);
        } else {
            stopMotors();
        }

        // Data opslaan in RAM (elke 50ms)
        static unsigned long lastSave = 0;
        if (millis() - lastSave >= 50 && savedPoints < MAX_POINTS) {
            storage[savedPoints] = {afstand, currentError};
            savedPoints++;
            lastSave = millis();
        }

        if (buttonPressed || savedPoints >= MAX_POINTS) {
            stopMotors();
            delay(500); // Debounce
            state = 2;
            Serial.println("TEST VOLTOOID. Sluit de kabel aan en druk nogmaals voor DATA.");
        }
    }

    // --- STATE 2: DATA DUMPEN NAAR SERIËLE MONITOR ---
    else if (state == 2) {
        if (buttonPressed) {
            Serial.println("--- START DATA DUMP ---");
            Serial.println("Afstand_cm,Line_Error"); // CSV Header
            
            for (int i = 0; i < savedPoints; i++) {
                Serial.print(storage[i].afstand);
                Serial.print(",");
                Serial.println(storage[i].error);
            }
            
            Serial.println("--- EINDE DATA DUMP ---");
            state = 0; // Terug naar idle voor een nieuwe test
            delay(1000);
        }
    }
}