#include <Arduino.h>
#include "Config.h"
#include "types.h"
#include "Sensors.h"
#include "Motors.h"
#include "StartStop.h"
#include "robot_logic.h"

RobotState currentState = IDLE;
unsigned long stateStartTime = 0;

void setup() {
    Serial.begin(115200);
    setupMotors();
    startSensorTask(); // Zorg dat deze decltype trick bevat als je op Core v3.0 zit
    initLogic();
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    Serial.println("Robot Ready. Press button to START MAPPING.");
}

void loop() {
    bool buttonPressed = (digitalRead(BUTTON_PIN) == LOW);
    
    // Simpele knop debounce en state wissel
    if (buttonPressed) {
        delay(200); // debounce
        if (currentState == IDLE) {
            currentState = MAPPING;
            resetLogic();
            resetStartSequence();
            stateStartTime = millis();
            Serial.println("Starting MAPPING...");
        } else if (currentState == FINISHED) {
            currentState = SPEEDRUN;
            resetStartSequence();
            stateStartTime = millis();
            Serial.println("Starting SPEEDRUN...");
        } else if (currentState == SPEEDRUN) {
            currentState = DATA_DUMP;
        }
        while(digitalRead(BUTTON_PIN) == LOW); // Wacht tot losgelaten
    }

    switch (currentState) {
        case IDLE:
            stopMotors();
            break;

        case MAPPING:
        case SPEEDRUN:
            if (handleStartLogic()) {
                updateLogic(currentState);
            }
            break;

        case FINISHED:
            stopMotors();
            Serial.println("Track Finished! Press button for Speedrun.");
            delay(1000); // Voorkom spam
            break;

        case DATA_DUMP:
            stopMotors();
            dumpEnhancedRoute(millis() - stateStartTime);
            currentState = IDLE; // Terug naar start
            break;
            
        default:
            stopMotors();
            break;
    }
    
    delay(1);
}