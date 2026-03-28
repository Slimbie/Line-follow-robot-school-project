#include "Motors.h"
#include "Config.h"

void setupMotors() {
    pinMode(MOT_L_AIN1, OUTPUT);
    pinMode(MOT_L_AIN2, OUTPUT);
    pinMode(MOT_R_BIN1, OUTPUT);
    pinMode(MOT_R_BIN2, OUTPUT);

    ledcSetup(CH_LEFT, PWM_FREQ, PWM_RES);
    ledcSetup(CH_RIGHT, PWM_FREQ, PWM_RES);
    ledcAttachPin(MOT_L_PWM, CH_LEFT);
    ledcAttachPin(MOT_R_PWM, CH_RIGHT);
}

void setMotorSpeed(int speedL, int speedR) {
    // Corrigeer voor de omgekeerde motor
    if (LEFT_MOTOR_REVERSED) speedL = -speedL;
    if (RIGHT_MOTOR_REVERSED) speedR = -speedR;

    // Linker Motor aansturing
    if (speedL >= 0) {
        digitalWrite(MOT_L_AIN1, HIGH);
        digitalWrite(MOT_L_AIN2, LOW);
    } else {
        digitalWrite(MOT_L_AIN1, LOW);
        digitalWrite(MOT_L_AIN2, HIGH);
    }
    ledcWrite(CH_LEFT, abs(speedL));

    // Rechter Motor aansturing
    if (speedR >= 0) {
        digitalWrite(MOT_R_BIN1, HIGH);
        digitalWrite(MOT_R_BIN2, LOW);
    } else {
        digitalWrite(MOT_R_BIN1, LOW);
        digitalWrite(MOT_R_BIN2, HIGH);
    }
    ledcWrite(CH_RIGHT, abs(speedR));
}

void stopMotors() {
    setMotorSpeed(0, 0);
}