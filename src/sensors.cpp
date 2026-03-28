#include "Sensors.h"
#include "Config.h"
#include <ESP32Encoder.h>

// Global variabelen
volatile SensorData currentSensors;
ESP32Encoder encL;
ESP32Encoder encR;

// Interne functies
void sensorTaskLoop(void * pvParameters);
float readUltrasonic();

void startSensorTask() {
    ESP32Encoder::useInternalWeakPullResistors = UP;
    encL.attachFullQuad(ENC_L_A, ENC_L_B);
    encR.attachFullQuad(ENC_R_A, ENC_R_B);

    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    xTaskCreatePinnedToCore(sensorTaskLoop, "SensorTask", 4000, NULL, 1, NULL, 0);
}

void sensorTaskLoop(void * pvParameters) {
    for(;;) {
        float teller = 0;
        float noemersum = 0;
        uint8_t mask = 0;
        bool seesLine = false;

        // De gewichten die jij fijn vond
        int weights[8] = {-40, -30, -20, -10, 10, 20, 30, 40};

        for (int i = 0; i < 8; i++) {
            int val = analogRead(sensorPinnen[i]);
            currentSensors.irValues[i] = val;
            
            if (val > IR_THRESHOLD) {
                seesLine = true;
                mask |= (1 << (7 - i)); // Bouw de bitmask op
                teller += (float)weights[i]; 
                noemersum++;
            }
        }
        
        currentSensors.lineDetected = seesLine;
        currentSensors.sensorMask = mask;

        if (noemersum > 0) {
            currentSensors.linePosition = teller / noemersum; 
        } else {
            currentSensors.linePosition = 0; 
        }

        // Ruwe afstand lezen voor debug
        currentSensors.distance = readUltrasonic();

        currentSensors.leftTicks = encL.getCount();
        currentSensors.rightTicks = encR.getCount();
        currentSensors.distanceDriven = ((currentSensors.leftTicks + currentSensors.rightTicks) / 2.0) / TICKS_PER_CM;

        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}
float readUltrasonic() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    long duration = pulseIn(ECHO_PIN, HIGH, 30000); 
    if (duration == 0) return 999.0; 
    return duration * 0.034 / 2.0;
}