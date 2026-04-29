#include <Arduino.h>
#include "Config.h"
#include "Sensors.h"
#include "Motors.h"
#include "Logic.h"
#include "Mapping.h"
#include "StartStop.h"
#include "Navigation.h"


//begin variabelen
RobotState currentState = IDLE;
RobotState previousState = IDLE;
enum ObstaclePhase
{
  OB_NONE,
  OB_DRIVE_RIGHT,
  OB_SEARCH_LINE
};

ObstaclePhase obstaclePhase = OB_NONE;
unsigned long obstacleStartTime = 0;
float obstacleStartDistance = 0;
int obstacleDetectCount = 0;
int obstacleClearCount = 0;
const int OBSTACLE_CONFIRM_COUNT = 3;
const int OBSTACLE_CLEAR_COUNT = 3;
const float OBSTACLE_PASS_DISTANCE = 27.0; // cm rechtdoor rijden voorbij het object
const int OBSTACLE_RIGHT_SPEED_HIGH = 95;
const int OBSTACLE_RIGHT_SPEED_LOW = 25;
const int OBSTACLE_RIGHT_SPEED_AFTER = 85;
const int OBSTACLE_RIGHT_SPEED_BIAS = 10;
const int OBSTACLE_LEFT_SEARCH_SPEED = 20;
const float OBSTACLE_LINE_STEER_FACTOR = 1.0f;
const int OBSTACLE_LINE_STEER_LIMIT = 70;
float lastSavedDistance = 0;
float lastSavedLinePosition = 0;
unsigned long mappingStartTime = 0;
unsigned long speedrunModeStartTime = 0;
unsigned long mappingElapsedTime = 0;
unsigned long speedrunElapsedTime = 0;
static uint8_t lastSensorMask = 0;
static unsigned long finishDetectStartTime = 0;
static bool finishPaused = false;

// Variabelen voor de knop
int buttonPressCount = 0;
bool lastButtonState = HIGH; // We gaan uit van INPUT_PULLUP

// Helper: Detect sharp turn direction based on sensor pattern
// Returns: -1 for LEFT turn, 1 for RIGHT turn, 0 for no turn
inline int detectSharpTurnDirection(uint8_t mask) {
    bool midSensors = (mask & MID_SENSOR_MASK) != 0;
    bool leftExtreme = (mask & 0b11000000) != 0;  // sensors 0-1
    bool rightExtreme = (mask & 0b00000011) != 0; // sensors 6-7
    
    if (!midSensors && leftExtreme && !rightExtreme) return -1;  // LEFT
    if (!midSensors && !leftExtreme && rightExtreme) return 1;   // RIGHT
    return 0; // No sharp turn
}

inline bool isFullBlack() {
    return currentSensors.sensorMask == 0xFF;
}
/*
inline uint8_t getBranchMask(uint8_t mask) {
    uint8_t branches = 0;
    if (mask & LEFT_SIDE_MASK) branches |= BRANCH_MASK_LEFT;
    if (mask & MID_SENSOR_MASK) branches |= BRANCH_MASK_STRAIGHT;
    if (mask & RIGHT_SIDE_MASK) branches |= BRANCH_MASK_RIGHT;
    return branches;
}*/
/*
inline uint8_t chooseLeftHandBranch(uint8_t branches) {
    if (branches & BRANCH_MASK_LEFT) return BRANCH_LEFT;
    if (branches & BRANCH_MASK_STRAIGHT) return BRANCH_STRAIGHT;
    if (branches & BRANCH_MASK_RIGHT) return BRANCH_RIGHT;
    return BRANCH_NONE;
}*/

inline uint8_t chooseFallbackBranch(uint8_t branches) {
    if (branches & BRANCH_MASK_STRAIGHT) return BRANCH_STRAIGHT;
    if (branches & BRANCH_MASK_RIGHT) return BRANCH_RIGHT;
    if (branches & BRANCH_MASK_LEFT) return BRANCH_LEFT;
    return BRANCH_NONE;
}

inline bool onlyMiddleLastSeen(uint8_t mask) {
    return (mask & MID_SENSOR_MASK) && !(mask & (LEFT_SIDE_MASK | RIGHT_SIDE_MASK));
}

void performDeadEndRecovery() {
    performSmoothDeadEndRecovery();
}

void saveIntersectionNode(float distance, uint8_t branchDirection, uint8_t branchMask) {
    savePoint(distance,
              currentSensors.linePosition,
              currentSensors.distance,
              currentSensors.leftTicks,
              currentSensors.rightTicks,
              branchDirection,
              true,
              false,
              false,
              branchMask);
}

void markLastNodeDeadEnd() {
    int idx = getLastNodeIndex(currentSensors.distanceDriven);
    if (idx >= 0) {
        trackMap[idx].isDeadEnd = true;
        
        // Find the direction we came from (the one that was dead-end)
        // and select a fallback direction
        uint8_t availableDirs = trackMap[idx].availableBranches;
        uint8_t currentDir = trackMap[idx].branchDirection;
        
        // Remove the dead-end direction from available branches
        uint8_t validDirs = availableDirs;
        if (currentDir == BRANCH_LEFT) validDirs &= ~BRANCH_MASK_LEFT;
        else if (currentDir == BRANCH_STRAIGHT) validDirs &= ~BRANCH_MASK_STRAIGHT;
        else if (currentDir == BRANCH_RIGHT) validDirs &= ~BRANCH_MASK_RIGHT;
        
        // Choose fallback: prioritize straight > right > left
        if (validDirs & BRANCH_MASK_STRAIGHT) {
            trackMap[idx].branchDirection = BRANCH_STRAIGHT;
        } else if (validDirs & BRANCH_MASK_RIGHT) {
            trackMap[idx].branchDirection = BRANCH_RIGHT;
        } else if (validDirs & BRANCH_MASK_LEFT) {
            trackMap[idx].branchDirection = BRANCH_LEFT;
        }
        
        trackMap[idx].type = 6;
        Serial.printf("[MAPPING] Node %d als doodlopend gemarkeerd, nieuwe richting: %d\n", 
                      idx, trackMap[idx].branchDirection);
    }
}
/*
uint8_t probeAvailableBranches() {
    setMotorSpeed(BASE_SPEED_MAPPING, BASE_SPEED_MAPPING);
    delay(INTERSECTION_PROBE_MS);
    uint8_t branches = getBranchMask(currentSensors.sensorMask);
    stopMotors();
    return branches;
}*/

bool obstacleInFront() {
  return currentSensors.distance > 1.0 && currentSensors.distance < DISTANCE_THRESHOLD;
}

bool obstacleConfirmed() {
  if (obstacleInFront()) {
    obstacleDetectCount++;
    obstacleClearCount = 0;
  } else {
    obstacleDetectCount = 0;
  }
  return obstacleDetectCount >= OBSTACLE_CONFIRM_COUNT;
}

bool obstacleStillDetected() {
  if (obstacleInFront()) {
    obstacleClearCount = 0;
    return true;
  }
  obstacleClearCount++;
  return obstacleClearCount < OBSTACLE_CLEAR_COUNT;
}
/*
void beginObstacleAvoidance(RobotState fromState) {
  previousState = fromState;
  currentState = OBSTACLE_STOP;
  obstaclePhase = OB_DRIVE_RIGHT;
  obstacleStartTime = millis();
  obstacleStartDistance = currentSensors.distanceDriven;
  Serial.println("Obstakel gedetecteerd! Obstakeldrijving start.");
}*/
/*
void handleObstacleAvoidance() {
  bool obstacleStillSeen = obstacleStillDetected();
  bool lineVisible = currentSensors.lineDetected;
  int baseSpeed = 55;
  float passedDistance = currentSensors.distanceDriven - obstacleStartDistance;

  Serial.printf("[OBSTACLE] Afstand=%.1f cm, Lijn=%d, Fase=%d, Voorbij=%.1f\n", 
                currentSensors.distance, lineVisible, obstaclePhase, passedDistance);

  if (obstacleStillSeen) {
    // Phase 1: Drive RIGHT in a smooth ARC around the obstacle
    obstaclePhase = OB_DRIVE_RIGHT;
    if (passedDistance < OBSTACLE_PASS_DISTANCE) {
      // Agressief rechts (boogmanoeuvre)
      int leftMotor = baseSpeed + OBSTACLE_RIGHT_SPEED_HIGH;
      int rightMotor = baseSpeed - OBSTACLE_RIGHT_SPEED_LOW;
      setMotorSpeed(leftMotor, rightMotor);
      Serial.println("[OBSTACLE] Agressief rechts om object te omzeilen (boog)");
    } else {
      // Na het object: minder agressief rechts
      int leftMotor = baseSpeed + OBSTACLE_RIGHT_SPEED_BIAS;
      int rightMotor = baseSpeed - OBSTACLE_RIGHT_SPEED_AFTER;
      setMotorSpeed(leftMotor, rightMotor);
      Serial.println("[OBSTACLE] Scherp rechts verder rond object");
    }
  } else {
    // Phase 2: Object is voorbij, nu zoeken naar lijn
    if (obstaclePhase == OB_DRIVE_RIGHT) {
      if (passedDistance < OBSTACLE_PASS_DISTANCE) {
        // Nog niet ver genoeg voorbij het object
        int leftMotor = baseSpeed + OBSTACLE_RIGHT_SPEED_BIAS;
        int rightMotor = baseSpeed - OBSTACLE_LEFT_SEARCH_SPEED;
        setMotorSpeed(leftMotor, rightMotor);
        Serial.println("[OBSTACLE] Nog niet voorbij: rechtdoor met rechtsbias");
      } else {
        // Nu kunnen we naar lijn-zoeken fase gaan
        obstaclePhase = OB_SEARCH_LINE;
        Serial.println("[OBSTACLE] Object weg en voorbij: Zoek-fase gestart");
      }
    } else {
      // Phase 3: Search for the line (OB_SEARCH_LINE)
      if (lineVisible) {
        float lineError = currentSensors.linePosition;
        
        // Smooth steering back to the line using line error
        int steer = constrain((int)(lineError * OBSTACLE_LINE_STEER_FACTOR), 
                              -OBSTACLE_LINE_STEER_LIMIT, OBSTACLE_LINE_STEER_LIMIT);
        int leftPWM = constrain(baseSpeed - 10 + steer, 0, MAX_SPEED);
        int rightPWM = constrain(baseSpeed - 10 - steer, 0, MAX_SPEED);
        setMotorSpeed(leftPWM, rightPWM);
        Serial.printf("[OBSTACLE] Lijn gedetecteerd: stuur terug (error=%.1f)\n", lineError);

        // Exit obstacle avoidance when line is stable and we've gone far enough
        if (abs(lineError) < 10.0 && passedDistance >= 30.0) {
          Serial.println("[OBSTACLE] Lijn stabiel: terugkeer naar vorige modus");
          currentState = previousState;
          obstaclePhase = OB_NONE;
          return;
        }
      } else {
        // Line still not found: search by turning left gently
        int leftMotor = baseSpeed - 20;
        int rightMotor = baseSpeed + 30;
        setMotorSpeed(leftMotor, rightMotor);
        Serial.println("[OBSTACLE] Geen lijn: zoek langzaam naar links");
      }
    }
  }

  // --- Save data during obstacle avoidance ---
  if (previousState == MAPPING) {
    float distSinceLastSave = abs(currentSensors.distanceDriven - lastSavedDistance);
    if (distSinceLastSave >= 5.0) {
      savePoint(currentSensors.distanceDriven,
                lineVisible ? currentSensors.linePosition : 0.0,
                currentSensors.distance,
                currentSensors.leftTicks,
                currentSensors.rightTicks,
                BRANCH_NONE,
                false,
                false,
                false,
                0);
      lastSavedDistance = currentSensors.distanceDriven;
      Serial.println("[OBSTACLE] Obstacle-punt opgeslagen in map");
    }
  } else if (previousState == SPEEDRUN) {
    logSpeedrunPoint(currentSensors.distanceDriven,
                     lineVisible ? currentSensors.linePosition : 0.0,
                     6); // Type 6 = Obstacle
  }
}
*/
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

  // --- Dead-end detectie ---
  if (!currentSensors.lineDetected && onlyMiddleLastSeen(lastSensorMask) && currentSensors.distanceDriven > 5.0) {
      Serial.println("Dead-end detected - Turning 180");
      markLastNodeDeadEnd();
      performDeadEndRecovery();
  }

  lastSensorMask = currentSensors.sensorMask;
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

case MAPPING:
    executeMappingState();
    break;








case SPEEDRUN:
    if (obstacleConfirmed()) {
        Serial.println("[SPEEDRUN] OBSTAKEL DETECTIE!");
        executeArcManoeuvre(); // Gebruik direct de nieuwe, blokkerende boog-functie!
        
        // Reset de tellers na de manoeuvre
        obstacleDetectCount = 0; 
        obstacleClearCount = 0;
        break;
    }

    {
        FinishState finishStatus = checkFinishStatus();
        if (finishStatus == FINISH_CONFIRMED) {
            Serial.println("[SPEEDRUN] Finish! Pauzeren...");
            stopMotors();
            // In plaats van currentState = IDLE, wachten we gewoon
            // De robot stopt, maar blijft in de loop tot de balk weg is
            while(currentSensors.sensorMask == 0xFF) {
                stopMotors();
                delay(10);
            }
            Serial.println("[SPEEDRUN] Finish balk weg, verder gaan.");
        }
    }

    executeSpeedrunState();

    // Automatische stop: als we voorbij de gemapte afstand zijn
    if (mapIndex > 0) {
        float finalMapDistance = trackMap[mapIndex - 1].distance;
        if (currentSensors.distanceDriven > finalMapDistance + 20.0 && !currentSensors.lineDetected) {
            Serial.println("[SPEEDRUN] Finish bereikt! Stop.");
            currentState = IDLE;
            stopMotors();
        }
    }
    break;
/*
  case OBSTACLE_STOP:
    // Gebruik de nieuwe vloeiende boog uit Navigation
    executeArcManoeuvre(); 
    // Na de boog keren we terug naar de vorige modus
    currentState = previousState;
    break;
*/

 case DATA_DUMP:
    stopMotors();
    {
        // Bereken de tijden voor de dumps
        unsigned long mappingTijd = (mappingElapsedTime > 0) ? mappingElapsedTime : (mappingStartTime > 0 ? millis() - mappingStartTime : 0);
        unsigned long speedrunTijd = (speedrunElapsedTime > 0) ? speedrunElapsedTime : (speedrunModeStartTime > 0 ? millis() - speedrunModeStartTime : 0);
        
        // 1. Dump de originele kaart (Mapping)
        dumpMap(mappingTijd); 
        delay(1000);
        
        // 2. Dump de debug kaart (Speedrun)
        dumpSpeedrunMap(speedrunTijd); 
        
        // Reset all state
        buttonPressCount = 0;
        mappingStartTime = 0;
        speedrunModeStartTime = 0;
        mappingElapsedTime = 0;
        speedrunElapsedTime = 0;
        currentState = IDLE;
    }
    break;
  }
}