#include "Motors.h"
#include "Config.h"
#include <Arduino.h>

void setupMotors() {
    // Richting pinnen als gewone outputs
    pinMode(MOT_L_AIN1, OUTPUT);
    pinMode(MOT_L_AIN2, OUTPUT);
    pinMode(MOT_R_BIN1, OUTPUT);
    pinMode(MOT_R_BIN2, OUTPUT);

    // ESP32 v3.0 PWM Setup: Koppel de PWM pinnen direct aan freq en resolutie
    // We hoeven geen kanalen (0 of 1) meer te definiëren.
    ledcAttach(MOT_L_PWM, PWM_FREQ, PWM_RES);
    ledcAttach(MOT_R_PWM, PWM_FREQ, PWM_RES);

    stopMotors();
}

void setMotorSpeed(float speedL, float speedR) {
    // 1. Corrigeer voor de fysiek omgekeerde motor(en) op basis van Config.h
    if (LEFT_MOTOR_REVERSED)  speedL = -speedL;
    if (RIGHT_MOTOR_REVERSED) speedR = -speedR;

    // 2. Veiligheid: Begrens de waarden tot de PWM resolutie (0-255 voor 8-bit)
    int outL = constrain((int)speedL, -255, 255);
    int outR = constrain((int)speedR, -255, 255);

    // --- LINKER MOTOR (Motor A) ---
    if (outL >= 0) {
        digitalWrite(MOT_L_AIN1, HIGH);
        digitalWrite(MOT_L_AIN2, LOW);
    } else {
        digitalWrite(MOT_L_AIN1, LOW);
        digitalWrite(MOT_L_AIN2, HIGH);
    }
    // In v3.0 schrijf je direct naar de PIN
    ledcWrite(MOT_L_PWM, abs(outL));

    // --- RECHTER MOTOR (Motor B) ---
    if (outR >= 0) {
        digitalWrite(MOT_R_BIN1, HIGH);
        digitalWrite(MOT_R_BIN2, LOW);
    } else {
        digitalWrite(MOT_R_BIN1, LOW);
        digitalWrite(MOT_R_BIN2, HIGH);
    }
    // In v3.0 schrijf je direct naar de PIN
    ledcWrite(MOT_R_PWM, abs(outR));
}

void stopMotors() {
    // Zet alles stil
    digitalWrite(MOT_L_AIN1, LOW);
    digitalWrite(MOT_L_AIN2, LOW);
    digitalWrite(MOT_R_BIN1, LOW);
    digitalWrite(MOT_R_BIN2, LOW);
    ledcWrite(MOT_L_PWM, 0);
    ledcWrite(MOT_R_PWM, 0);
}