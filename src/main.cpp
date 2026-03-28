#include <Arduino.h>
#include "Config.h"
#include "Sensors.h"
#include "Motors.h"
#include "Logic.h"
#include "Mapping.h"


enum State
{
  IDLE,
  COUNTDOWN,
  MAPPING,
  SPEEDRUN,
  OBSTACLE_STOP,
  DATA_DUMP
};
State currentState = IDLE;

// Variabelen voor de knop
int buttonPressCount = 0;
bool lastButtonState = HIGH; // We gaan uit van INPUT_PULLUP

void setup()
{
  Serial.begin(115200);
  setupMotors();
  startSensorTask(); // Start Core 0

  // Zorg dat de afstand hoog begint, niet op 0!
  currentSensors.distance = 999.0;

  pinMode(START_BUTTON_PIN, INPUT_PULLUP);
  pinMode(MODE_SWITCH_PIN, INPUT_PULLUP); // Zorg dat de switch pin goed staat

  Serial.println("--- Robot Ready ---");
  Serial.println("Druk 1x: Start");
  Serial.println("Druk 2x: Stop");
  Serial.println("Druk 3x: Data Dump");
}

void loop()
{
  // --- 1. KNOP LOGICA (Debounced) ---
  bool currentButtonState = digitalRead(START_BUTTON_PIN);

  // Kijk of de knop is ingedrukt (van HIGH naar LOW)
  if (lastButtonState == HIGH && currentButtonState == LOW)
  {
    delay(50); // Korte pauze tegen "denderen" van de knop
    buttonPressCount++;

    Serial.print("Knop ingedrukt! Aantal keer: ");
    Serial.println(buttonPressCount);

    if (buttonPressCount == 1)
    {
      currentState = COUNTDOWN;
    }
    else if (buttonPressCount == 2)
    {
      currentState = IDLE;
      stopMotors();
      Serial.println("Actie: GESTOPT.");
    }
    else if (buttonPressCount == 3)
    {
      currentState = DATA_DUMP;
      stopMotors();
      Serial.println("Actie: DATA DUMP START.");
    }
    else
    {
      // Na 3x resetten we de boel weer
      buttonPressCount = 0;
      currentState = IDLE;
    }
  }
  lastButtonState = currentButtonState;

  // --- 2. STATE MACHINE ---
  switch (currentState)
  {
  case IDLE:
    stopMotors();
    break;

  case COUNTDOWN:
    Serial.println("Starten in 3 seconden...");
    delay(3000);
    resetPID();

    // Lees de switch uit om te bepalen welke modus we starten
    if (digitalRead(MODE_SWITCH_PIN) == LOW)
    {
      Serial.println("Modus: MAPPING (Traag)");
      currentState = MAPPING;
    }
    else
    {
      Serial.println("Modus: SPEEDRUN (Snel)");
      currentState = SPEEDRUN;
    }
    break;

  case MAPPING:
    // 1. PID uitvoeren
    calculatePID();

    // 2. Elke 5 cm een punt opslaan
    static float lastSavedDist = 0;
    if (currentSensors.distanceDriven - lastSavedDist >= 5.0)
    {
      savePoint(currentSensors.distanceDriven, currentSensors.linePosition, 0);
      lastSavedDist = currentSensors.distanceDriven;
    }

    // 3. Print visuele feedback (Bitmask)
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 200)
    { // Iets sneller printen voor afstellen
      Serial.print("Sensoren: [");
      for (int i = 7; i >= 0; i--)
      {
        Serial.print((currentSensors.sensorMask & (1 << i)) ? "X" : "_");
      }
      Serial.printf("] | Pos: %6.1f | Dist: %5.1f\n", currentSensors.linePosition, currentSensors.distance);
      lastPrint = millis();
    }
    break;

  case SPEEDRUN:
    // Hier komt later de snelle code
    calculatePID(); // Voor nu doet hij hetzelfde als mapping
    break;

  case OBSTACLE_STOP:
    stopMotors();
    break;

  case DATA_DUMP:
    Serial.println("Hier komt later de lijst met alle meetgegevens!");
    delay(1000);          // Wacht even
    buttonPressCount = 0; // Reset
    currentState = IDLE;
    break;
  }
}