// Config.h
#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// --- Start/Finish & Logica ---
#define USE_START_FINISH      true
#define FINISH_DRIVE_ON_CM    10.0   // Hoeveel cm doorrijden na detectie balk
#define SAVE_POINT_INTERVAL   5.0    // Interval voor opslaan mapping punten
#define MID_SENSOR_MASK       0b00011000
#define TICKS_PER_CM          58.0   // Aantal encoder ticks per centimeter

// --- Snelheden (0-255) ---
#define BASE_SPEED_MAPPING    100    // Snelheid tijdens het mappen
#define BASE_SPEED_TURNS      90     // Snelheid in bochten
#define MAX_SPEED             180    // Maximale snelheid op rechte stukken

// --- PID Waarden ---
#define KP                    1.5    // Proportional
#define KI                    0.0    // Integral
#define KD                    5.0    // Derivative

// --- IR Sensor Array ---
const int SENSOR_COUNT = 8;
const int sensorPinnen[SENSOR_COUNT] = {32, 33, 34, 35, 36, 39, 25, 26}; 
#define IR_THRESHOLD          4050 

// --- Knoppen & Switches ---
#define START_BUTTON_PIN      23
#define MODE_SWITCH_PIN       19
#define BUTTON_PIN            0      // Extra knop (bijv. BOOT knop ESP32)

// --- Ultrasoon ---
#define TRIG_PIN              5
#define ECHO_PIN              18
#define DISTANCE_THRESHOLD    5.0    // Detectie afstand in cm

// --- Motoren (L298N / OT2065) ---
// Links
#define MOT_L_PWM             27
#define MOT_L_AIN1            14
#define MOT_L_AIN2            4
// Rechts
#define MOT_R_PWM             22
#define MOT_R_BIN1            21
#define MOT_R_BIN2            16

// PWM Kanalen (ESP32 specifiek)
#define PWM_FREQ              20000
#define PWM_RES               8
#define CH_LEFT               0
#define CH_RIGHT              1

// Motor Richting
#define LEFT_MOTOR_REVERSED   true   // Fysiek 180 graden gedraaid
#define RIGHT_MOTOR_REVERSED  false

// --- Encoders ---
#define ENC_L_A               12
#define ENC_L_B               13
#define ENC_R_A               15
#define ENC_R_B               2

#endif
