#include <Arduino.h>
#include "types.h"
#include "Config.h"
#include "Sensors.h"
#include "Motors.h"
#include "robot_logic.h"
#include "StartStop.h"

Mode currentMode = READY;
unsigned long lastSerialUpdate = 0;
static uint8_t buttonPressCount = 0;
static unsigned long firstButtonPressAt = 0;

static void processButtonAction() {
    if (buttonPressCount == 0) return;

    if (buttonPressCount == 1) {
        if (currentMode == READY) {
            resetLogic();
            resetStartSequence();
            currentMode = MAPPING;
            Serial.println(">>> MODE: MAPPING START");
        } else if (currentMode == FINISHED) {
            if (!route.empty()) {
                resetStartSequence();
                prepareSpeedrun();
                currentMode = SPEEDRUN;
                Serial.println(">>> MODE: SPEEDRUN START");
            } else {
                Serial.println("[ERROR] Geen route in geheugen! Map eerst.");
            }
        } else {
            Serial.println("[BUTTON] Enkelvoudige druk genegeerd in deze modus");
        }
    } else if (buttonPressCount == 2) {
        if (currentMode == MAPPING || currentMode == SPEEDRUN) {
            stopMotors();
            currentMode = FINISHED;
            Serial.println(">>> MODE: FINISHED");
        } else {
            Serial.println("[BUTTON] Stop werkt alleen tijdens een run");
        }
    } else {
        stopMotors();
        currentMode = DATADUMP;
        Serial.println(">>> MODE: DATADUMP");
    }
}

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
    if (digitalRead(BUTTON_PIN) == LOW) {
        delay(50);
        if (digitalRead(BUTTON_PIN) == LOW) {
            if (buttonPressCount == 0) {
                firstButtonPressAt = millis();
            }
            buttonPressCount++;
            Serial.printf("[BUTTON] druk %d geregistreerd\n", buttonPressCount);
            while (digitalRead(BUTTON_PIN) == LOW) {
                yield();
            }
            delay(50);
        }
    }

    if (buttonPressCount > 0 && (millis() - firstButtonPressAt) > BUTTON_PRESS_WINDOW) {
        processButtonAction();
        buttonPressCount = 0;
    }

    // --- MODE EXECUTION ---
    switch (currentMode) {
        case READY:
            stopMotors();
            break;

        case MAPPING:
            if (millis() - lastSerialUpdate > 100) {
                int32_t rTicks = RIGHT_MOTOR_REVERSED ? -currentSensors.rightTicks : currentSensors.rightTicks;
                Serial.printf("[MAPPING] Dist: %.1f | L-Enc: %d | R-Enc: %d\n",
                    currentSensors.distanceDriven, currentSensors.leftTicks, rTicks);
                lastSerialUpdate = millis();
            }

            if (handleStartLogic()) {
                RobotState state = {currentMode, true, millis(), currentSensors.distanceDriven, (int)route.size()};
                currentMode = updateLogic(state);
            }
            break;

        case SPEEDRUN:
            if (handleStartLogic()) {
                RobotState state = {currentMode, true, millis(), currentSensors.distanceDriven, (int)route.size()};
                currentMode = updateLogic(state);
            }
            break;

        case FINISHED:
            stopMotors();
            break;

        case DATADUMP:
            dumpEnhancedRoute(millis());
            currentMode = READY;
            break;
    }

    yield();
}