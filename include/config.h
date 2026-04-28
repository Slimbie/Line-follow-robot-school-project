#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// --- SNELHEDEN & RIJDEN ---
#define BASE_SPEED_MAPPING  60    // Rustige snelheid voor mappen
#define MAX_SPEED           180   // Maximale snelheid voor de speedrun
#define BASE_SPEED_TURNS    100   // Snelheid tijdens scherpe bochten
#define TICKS_PER_CM        58.0  // Kalibratie van je encoders

// --- PID PARAMETERS ---
#define KP 2.3 
#define KI 0.0
#define KD 16.0

// --- HARDWARE PINS: KNOOP & SWITCH ---
#define BUTTON_PIN          23    // Jouw START_BUTTON_PIN
#define MODE_SWITCH_PIN     19

// --- IR SENSOR ARRAY ---
#define SENSOR_COUNT        8
const int sensorPinnen[SENSOR_COUNT] = {32, 33, 34, 35, 36, 39, 25, 26}; 
#define IR_THRESHOLD        4050  // Jouw specifieke drempelwaarde

// --- ULTRASOON ---
#define TRIG_PIN            5
#define ECHO_PIN            18
#define DISTANCE_THRESHOLD  5.0   // Stop op 5cm van obstakel

// --- MOTOREN (L298N / OT2065) ---
// Links
#define MOT_L_PWM           27
#define MOT_L_AIN1          14
#define MOT_L_AIN2          4
// Rechts
#define MOT_R_PWM           22
#define MOT_R_BIN1          21
#define MOT_R_BIN2          16

// PWM Kanalen (ESP32 specifiek)
#define PWM_FREQ            20000
#define PWM_RES             8

// --- MOTOR RICHTING ---
#define LEFT_MOTOR_REVERSED  false 
#define RIGHT_MOTOR_REVERSED true  // Deze motor zit 180 graden gedraaid

// --- ENCODERS ---
#define ENC_L_A             12
#define ENC_L_B             13
#define ENC_R_A             15
#define ENC_R_B             2

// --- LOGICA INSTELLINGEN ---
#define MID_SENSOR_MASK     0x18
#define SAVE_POINT_INTERVAL 5.0     // Elke 5cm een punt opslaan
#define BUTTON_PRESS_WINDOW 500     // ms om snelle dubbele/drievoudige drukken te tellen
#define INTERSECTION_PROBE_CM 6.0   // cm vooruit om kruispunten te classificeren
#define FINISH_FULL_BLACK_CM 10.0   // cm zwart vlak voor finish
#define DEAD_END_RECOVERY_SPEED 70  // snelheid bij omkering na doodlopend pad
#define SEARCH_HARD_TURN_SPEED 90  // snelheid tijdens herstel naar de lijn
#define JUNCTION_YIELD_SPEED 45    // trage kruispuntdetectiesnelheid
#define LOOKAHEAD_DISTANCE    18.0  // cm om voorbereide snelheid te kiezen

// --- START/FINISH ---
#define USE_START_FINISH    true
#define FINISH_DRIVE_ON_CM  10.0

#endif