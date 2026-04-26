#include <Arduino.h>
#include "types.h"
#include "Config.h"
#include "Sensors.h"
#include "Motors.h"
#include "robot_logic.h"
#include "StartStop.h"

Mode currentMode = READY;
unsigned long lastSerialUpdate = 0;

void setup() {
    Serial.begin(115200);
    delay(500);
    
    setupMotors();
    initLogic();
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    startSensorTask();

    Serial.println("\n[SYSTEM] Robot Ready. Mode: READY");
}

void loop() {
    // --- BUTTON LOGIC (State Machine Switch) ---
    if (digitalRead(BUTTON_PIN) == LOW) {
        delay(200); // Debounce
        
        switch (currentMode) {
            case READY:
                resetLogic();
                resetStartSequence();
                currentMode = MAPPING;
                Serial.println(">>> MODE: MAPPING START");
                break;

            case MAPPING:
                stopMotors();
                currentMode = FINISHED;
                Serial.println(">>> MODE: FINISHED");
                break;

            case FINISHED:
                // CRASH PREVENTION: Alleen starten als er data is
                if (!route.empty()) {
                    resetStartSequence();
                    currentMode = SPEEDRUN;
                    Serial.println(">>> MODE: SPEEDRUN START");
                } else {
                    Serial.println("[ERROR] Geen route in geheugen! Map eerst.");
                }
                break;

            case SPEEDRUN:
                stopMotors();
                currentMode = DATADUMP;
                Serial.println(">>> MODE: DATADUMP");
                break;

            case DATADUMP:
                dumpEnhancedRoute(millis()); // Print alle data
                currentMode = READY;
                Serial.println(">>> MODE: READY");
                break;
        }

        while(digitalRead(BUTTON_PIN) == LOW) yield(); // Wacht op loslaten
    }

    // --- MODE EXECUTION ---
    switch (currentMode) {
        case READY:
            stopMotors();
            break;

        case MAPPING:
            // Output voor overzicht (elke 100ms)
            if (millis() - lastSerialUpdate > 100) {
                // we inverteren de Right Ticks hier visueel als de encoder fysiek omgedraaid is
                int32_t rTicks = RIGHT_MOTOR_REVERSED ? -currentSensors.rightTicks : currentSensors.rightTicks;
                Serial.printf("Dist: %.1f | L-Enc: %d | R-Enc: %d\n", 
                    currentSensors.distanceDriven, currentSensors.leftTicks, rTicks);
                lastSerialUpdate = millis();
            }

            if (handleStartLogic()) {
                // In robot_logic.cpp gebruiken we nu 'MAPPING' branch
                updateLogic(currentMode); 
            }
            break;

        case SPEEDRUN:
            if (handleStartLogic()) {
                updateLogic(currentMode);
            }
            break;

        case FINISHED:
        case DATADUMP:
            stopMotors();
            break;
    }

    yield();
}