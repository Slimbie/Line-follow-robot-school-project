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
//begin variabelen
State currentState = IDLE;
State previousState = IDLE;
enum ObstaclePhase
{
  OB_NONE,
  OB_DRIVE_RIGHT,
  OB_DRIVE_STRAIGHT,  // Drive straight for ~30cm after passing obstacle
  OB_SEARCH_LINE
};
ObstaclePhase obstaclePhase = OB_NONE;
unsigned long obstacleStartTime = 0;
float obstacleStartDistance = 0;
float lastSavedDistance = 0;
float lastSavedLinePosition = 0;
unsigned long mappingStartTime = 0;
unsigned long speedrunModeStartTime = 0;
unsigned long mappingElapsedTime = 0;
unsigned long speedrunElapsedTime = 0;

// Variabelen voor de knop
int buttonPressCount = 0;
bool lastButtonState = HIGH; // We gaan uit van INPUT_PULLUP

bool obstacleInFront() {
  return currentSensors.distance > 0 && currentSensors.distance < DISTANCE_THRESHOLD;
}

void beginObstacleAvoidance(State fromState) {
  previousState = fromState;
  currentState = OBSTACLE_STOP;
  obstaclePhase = OB_DRIVE_RIGHT;
  obstacleStartTime = millis();
  obstacleStartDistance = currentSensors.distanceDriven;
  Serial.println("Obstakel gedetecteerd! Obstakeldrijving start.");
}

void handleObstacleAvoidance() {
  bool obstacleStillSeen = obstacleInFront();
  bool lineVisible = currentSensors.lineDetected;
  int baseSpeed = 55;
  const float STRAIGHT_DRIVE_DISTANCE = 30.0;  // cm to drive straight after passing obstacle

  Serial.printf("[OBSTACLE] afstand=%.1f cm, line=%d, phase=%d, dist=%.1f\n", currentSensors.distance, lineVisible, obstaclePhase, currentSensors.distanceDriven - obstacleStartDistance);

  if (obstacleStillSeen) {
    // Phase 1: Obstacle visible - turn right aggressively
    obstaclePhase = OB_DRIVE_RIGHT;
    setMotorSpeed(baseSpeed, baseSpeed - 40);  // Increased from -25 to -40 for stronger right steering
    Serial.println("Obstakel blijft zichtbaar: stuur hard rechts.");
  } else {
    // Obstacle no longer visible
    if (obstaclePhase == OB_DRIVE_RIGHT) {
      // Transition to straight-ahead phase
      obstaclePhase = OB_DRIVE_STRAIGHT;
      obstacleStartDistance = currentSensors.distanceDriven;  // Reset distance counter
      Serial.println("Object weg, rijd 30cm rechtdoor voor terugsturing begint.");
    }

    if (obstaclePhase == OB_DRIVE_STRAIGHT) {
      // Phase 2: Drive straight to clear obstacle
      float distanceTraveled = currentSensors.distanceDriven - obstacleStartDistance;
      if (distanceTraveled < STRAIGHT_DRIVE_DISTANCE) {
        setMotorSpeed(baseSpeed, baseSpeed);  // Drive straight
        Serial.printf("Rechtdoor: %.1f/%.1f cm\n", distanceTraveled, STRAIGHT_DRIVE_DISTANCE);
      } else {
        // Straight phase complete, move to line search
        obstaclePhase = OB_SEARCH_LINE;
        Serial.println("Rechtdoorfase klaar, lijn zoeken.");
      }
    } else if (obstaclePhase == OB_SEARCH_LINE) {
      // Phase 3: Search for line
      if (lineVisible) {
        float lineError = currentSensors.linePosition;
        int steer = constrain((int)(lineError * 1.2f), -50, 50);  // Increased from 0.8 to 1.2, and ±40 to ±50
        int leftPWM = constrain(baseSpeed + steer, 0, MAX_SPEED);
        int rightPWM = constrain(baseSpeed - steer, 0, MAX_SPEED);
        setMotorSpeed(leftPWM, rightPWM);
        Serial.println("Lijn gedetecteerd, stuur naar lijn.");

        if (abs(lineError) < 10.0) {
          Serial.println("Lijn teruggevonden, terug naar vorige modus.");
          currentState = previousState;
          obstaclePhase = OB_NONE;
          return;
        }
      } else {
        // No line found - search left
        setMotorSpeed(baseSpeed - 10, baseSpeed + 10);
        Serial.println("Geen lijn, langzaam naar links zoeken.");
      }
    }
  }

  if (previousState == MAPPING) {
    float distSinceLastSave = abs(currentSensors.distanceDriven - lastSavedDistance);
    if (distSinceLastSave >= 5.0) {
      savePoint(currentSensors.distanceDriven,
                lineVisible ? currentSensors.linePosition : 0.0,
                currentSensors.distance,
                currentSensors.leftTicks,
                currentSensors.rightTicks);
      lastSavedDistance = currentSensors.distanceDriven;
      Serial.println("Obstacle punt opgeslagen in map.");
    }
  } else if (previousState == SPEEDRUN) {
    logSpeedrunPoint(currentSensors.distanceDriven,
                     lineVisible ? currentSensors.linePosition : 0.0,
                     6);
  }
}

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
      if (mappingStartTime > 0) {
          mappingElapsedTime = millis() - mappingStartTime;
          mappingStartTime = 0;
      }
      if (speedrunModeStartTime > 0) {
          speedrunElapsedTime = millis() - speedrunModeStartTime;
          speedrunModeStartTime = 0;
      }
      currentState = IDLE;
      stopMotors();
      Serial.println("Actie: GESTOPT.");
    }
    else if (buttonPressCount == 3)
    {
      if (mappingStartTime > 0) {
          mappingElapsedTime = millis() - mappingStartTime;
          mappingStartTime = 0;
      }
      if (speedrunModeStartTime > 0) {
          speedrunElapsedTime = millis() - speedrunModeStartTime;
          speedrunModeStartTime = 0;
      }
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
    Serial.println("Starten in 3...");
    delay(3000);
    
    encL.setCount(0);
    encR.setCount(0);
    currentSensors.distanceDriven = 0; 
    lastSavedDistance = 0;
    lastSavedLinePosition = 0;
    
    if (digitalRead(MODE_SWITCH_PIN) == LOW) {
        resetMap();
        mappingStartTime = millis();
        mappingElapsedTime = 0;
        speedrunModeStartTime = 0;
        speedrunElapsedTime = 0;
        currentState = MAPPING;
    } else {
        prepareSpeedrun();    // <--- RESET DE LOGICA VARIABELEN
        speedrunIndex = 0;
        speedrunModeStartTime = millis(); // De timer start NU
        speedrunElapsedTime = 0;
        mappingStartTime = 0;
        mappingElapsedTime = 0;
        currentState = SPEEDRUN;
    }
    break;

  case MAPPING: {
    if (obstacleInFront()) {
      beginObstacleAvoidance(currentState);
      break;
    }

    calculatePID(); 

    // DEBUG: Print elke 200ms de status inclusief gereden afstand
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 200) {
        Serial.print("Sensoren: [");
        for (int i = 7; i >= 0; i--) {
            Serial.print((currentSensors.sensorMask & (1 << i)) ? "X" : "_");
        }
        // NU OOK MET AFSTAND (Driven):
        Serial.printf("] | Pos: %5.1f | Dist: %5.1f | Driven: %5.1f cm\n", 
                      currentSensors.linePosition, 
                      currentSensors.distance, 
                      currentSensors.distanceDriven);
        lastPrint = millis();
    }

    // OPSLAAN LOGICA:
    // We gebruiken abs() voor het geval de encoders negatief tellen
    // OPSLAAN LOGICA:
    bool shouldSave = false;
    float distSinceLastSave = abs(currentSensors.distanceDriven - lastSavedDistance);
    float lineShift = abs(currentSensors.linePosition - lastSavedLinePosition);

    if (distSinceLastSave >= 5.0) {
        shouldSave = true;
    } else if (abs(currentSensors.linePosition) > 20.0 && distSinceLastSave >= 2.0 && lineShift > 8.0) {
        shouldSave = true;
    }

    if (shouldSave) {
        savePoint(currentSensors.distanceDriven, currentSensors.linePosition, currentSensors.distance, currentSensors.leftTicks, currentSensors.rightTicks);
        lastSavedDistance = currentSensors.distanceDriven;
        lastSavedLinePosition = currentSensors.linePosition;
        Serial.println(">>> PUNT OPGESLAGEN <<<");
    }
    }
    break;

  case SPEEDRUN:
    if (obstacleInFront()) {
      beginObstacleAvoidance(currentState);
      break;
    }

    calculateSpeedrun(); 

    // Automatische stop: als we voorbij de gemapte afstand zijn en de lijn echt kwijt raken
    if (mapIndex > 0) {
        float finalMapDistance = trackMap[mapIndex - 1].distance;
        if (currentSensors.distanceDriven > finalMapDistance + 20.0 && !currentSensors.lineDetected) {
            Serial.println("Finish bereikt! Stop.");
            currentState = IDLE;
            stopMotors();
        }
    }
    break;

  case OBSTACLE_STOP:
    handleObstacleAvoidance();
    break;

 case DATA_DUMP:
    stopMotors();
    
    // Bereken de tijden voor de dumps
    unsigned long mappingTijd = (mappingElapsedTime > 0) ? mappingElapsedTime : (mappingStartTime > 0 ? millis() - mappingStartTime : 0);
    unsigned long speedrunTijd = (speedrunElapsedTime > 0) ? speedrunElapsedTime : (speedrunModeStartTime > 0 ? millis() - speedrunModeStartTime : 0);
    
    // 1. Dump de originele kaart (Mapping)
    dumpMap(mappingTijd); 
    
    delay(1000); // Korte pauze zodat de seriële buffer niet overloopt
    
    // 2. Dump de debug kaart (Speedrun)
    // Deze is leeg als je nog geen speedrun hebt gedaan
    dumpSpeedrunMap(speedrunTijd); 
    
    buttonPressCount = 0;
    mappingStartTime = 0;
    speedrunModeStartTime = 0;
    mappingElapsedTime = 0;
    speedrunElapsedTime = 0;
    currentState = IDLE;
    break;
  }
}