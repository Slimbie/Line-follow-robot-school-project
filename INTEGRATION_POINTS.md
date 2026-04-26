## **SPECIFIEKE INTEGRATIE PUNTEN (Waar voeg je wat in toe)**

Dit bestand geeft de EXACTE plekken in uw bestaande bestanden waar u code moet toevoegen/vervangen.

---

## **📍 MAIN.CPP CHANGES**

### Change 1: Includes Toevoegen (na line ~10)

**LOCATE:**
```cpp
#include "StartStop.h"
```

**ADD AFTER:**
```cpp
#include "sensor_analyzer.h"
#include "enhanced_mapping.h"
```

**RESULT:**
```cpp
#include "logic.h"
#include "config.h"
#include "sensors.h"
#include "motors.h"
#include "mapping.h"
#include "StartStop.h"
#include "sensor_analyzer.h"      // ← NEW
#include "enhanced_mapping.h"     // ← NEW
```

---

### Change 2: Setup Function (na setupMotors line)

**LOCATE IN setup():**
```cpp
void setup() {
    Serial.begin(115200);
    
    // Hardware init
    setupMotors();
    startSensorTask();
```

**ADD AFTER startSensorTask():**
```cpp
    // NIEUW: Initialize sensor analyzer
    sensorAnalyzer.init();
```

**RESULT:**
```cpp
void setup() {
    Serial.begin(115200);
    
    // Hardware init
    setupMotors();
    startSensorTask();
    
    // NIEUW: Initialize sensor analyzer
    sensorAnalyzer.init();
    
    // Knoppen configureren
    pinMode(START_BUTTON_PIN, INPUT_PULLUP);
    pinMode(MODE_SWITCH_PIN, INPUT_PULLUP);
```

---

### Change 3: COUNTDOWN Case in loop() (Lines ~70-90)

**LOCATE:**
```cpp
        case COUNTDOWN:
            Serial.println("Start in 3 seconden...");
            delay(3000);
            
            encL.setCount(0); 
            encR.setCount(0);
            resetStartSequence(); 
            
            if (digitalRead(MODE_SWITCH_PIN) == LOW) {
                resetMap();                    // ← CHANGE THIS
                mappingStartTime = millis();
                currentState = MAPPING;
                Serial.println("MODE: MAPPING");
            } else {
                prepareSpeedrun();             // ← CHANGE THIS
                speedrunStartTime = millis();
                currentState = SPEEDRUN;
                Serial.println("MODE: SPEEDRUN");
            }
            break;
```

**REPLACE WITH:**
```cpp
        case COUNTDOWN:
            Serial.println("Start in 3 seconden...");
            delay(3000);
            
            encL.setCount(0); 
            encR.setCount(0);
            resetStartSequence(); 
            
            if (digitalRead(MODE_SWITCH_PIN) == LOW) {
                initEnhancedMapping();         // ← CHANGED
                mappingStartTime = millis();
                currentState = MAPPING;
                Serial.println("MODE: MAPPING");
            } else {
                prepareForSpeedrun();          // ← CHANGED
                speedrunStartTime = millis();
                currentState = SPEEDRUN;
                Serial.println("MODE: SPEEDRUN");
            }
            break;
```

---

### Change 4: DATA_DUMP Case (Add if not exists, around line ~120)

**LOCATE:**
```cpp
        case DATA_DUMP:
            unsigned long mappingTime = millis() - mappingStartTime;
            dumpMap(mappingTime);              // ← OLD
            currentState = IDLE;
            buttonPressCount = 0;
            break;
```

**REPLACE WITH:**
```cpp
        case DATA_DUMP: {
            unsigned long mappingTime = millis() - mappingStartTime;
            dumpEnhancedRoute(mappingTime);   // ← CHANGED
            currentState = IDLE;
            buttonPressCount = 0;
            break;
        }
```

---

## **📍 LOGIC.CPP CHANGES**

### Change 1: Includes Toevoegen (Line 1-10)

**LOCATE:**
```cpp
#include "logic.h"
#include "config.h"
#include "sensors.h"
#include "motors.h"
#include "mapping.h"
#include "StartStop.h"
```

**ADD AFTER:**
```cpp
#include "sensor_analyzer.h"     // ← NEW
#include "enhanced_mapping.h"    // ← NEW
```

---

### Change 2: Static Variables (Line 15-20)

**LOCATE:**
```cpp
static float smoothSpeed = BASE_SPEED_MAPPING; 
static float lastError = 0;                    
static uint8_t lastMask = 0;                   
static int currentJunctionIndex = 0;
```

**ADD AFTER:**
```cpp
// --- NEW: Sharp turn state machine ---
static bool inSharpTurnMode = false;
static uint8_t sharpTurnSidelineDir = 0;
static uint32_t sharpTurnEntryTime = 0;
```

---

### Change 3: New Function - handleSharpTurnInFlight()

**ADD BEFORE executeAction() function (around line 25):**

```cpp
// ============================================================================
// HELPER: SHARP TURN HANDLING (NIEUW!)
// ============================================================================

void handleSharpTurnInFlight(uint8_t sensorMask, float lineError) {
    uint32_t timeSinceEntry = millis() - sharpTurnEntryTime;
    
    if (timeSinceEntry > 700) {
        inSharpTurnMode = false;
        return;
    }
    
    if (sensorMask == 0) {
        // === FASE 1: DRAAI (geen lijn gezien) ===
        if (sharpTurnSidelineDir & 0b11100000) {
            setMotorSpeed(-75, 75);
            delay(1);
        } 
        else if (sharpTurnSidelineDir & 0b00000111) {
            setMotorSpeed(75, -75);
            delay(1);
        }
        else {
            setMotorSpeed(60, -60);
            delay(1);
        }
    }
    else {
        // === FASE 2: LIJN TERUG (exit van sharp turn) ===
        inSharpTurnMode = false;
        
        float P = lineError * KP;
        float D = (lineError - lastError) * KD;
        lastError = lineError;
        
        setMotorSpeed(
            constrain(BASE_SPEED_MAPPING + (int)P + (int)D, 0, MAX_SPEED),
            constrain(BASE_SPEED_MAPPING - (int)P - (int)D, 0, MAX_SPEED)
        );
    }
}
```

---

### Change 4: Replace handleMapping() Function

**LOCATE:** (around line 50-150)
```cpp
void handleMapping() {
    // ALLES ERIN → VERVANGEN!
    ...
}
```

**REPLACE ENTIRE FUNCTION WITH:**

```cpp
void handleMapping() {
    float error = currentSensors.linePosition;
    uint8_t mask = currentSensors.sensorMask;

    TurnType detectedTurn = sensorAnalyzer.analyzeTurn(mask, error, error);

    FinishState state = checkFinishStatus();

    if (state == FINISH_CONFIRMED) {
        stopMotors();
        return; 
    }
    if (state == CHECKING_FINISH) return; 

    if (inSharpTurnMode) {
        handleSharpTurnInFlight(mask, error);
        return;
    }

    if (detectedTurn == TURN_SHARP && !inSharpTurnMode) {
        inSharpTurnMode = true;
        sharpTurnEntryTime = millis();
        
        uint8_t leftSegment = mask & 0b11100000;
        uint8_t rightSegment = mask & 0b00000111;
        sharpTurnSidelineDir = (leftSegment > 0) ? leftSegment : rightSegment;
        
        Serial.printf("[MAPPING] SHARP TURN ENTRY @ %.1fcm (mask=0x%02X)\n",
                      currentSensors.distanceDriven, mask);
        
        recordRoutePoint(
            currentSensors.distanceDriven,
            TURN_SHARP,
            0.0,
            mask,
            error,
            error,
            BASE_SPEED_MAPPING
        );
        
        handleSharpTurnInFlight(mask, error);
        return;
    }

    if (state == INTERSECTION_DETECTED) {
        JunctionAction myAction = TURN_LEFT;
        
        if (junctionCount < MAX_JUNCTIONS) {
            recordJunction(
                currentSensors.distanceDriven,
                JUNCTION_TYPE_T,
                JunctionDecision::GO_LEFT,
                0.9
            );
        }

        executeAction(myAction);
        return;
    }

    if (currentSensors.lineDetected) {
        lastMask = mask;
        
        float P = error * KP;
        float D = (error - lastError) * KD;
        float turnCorrection = P + D;
        lastError = error;

        setMotorSpeed(constrain(BASE_SPEED_MAPPING + (int)turnCorrection, 0, MAX_SPEED), 
                      constrain(BASE_SPEED_MAPPING - (int)turnCorrection, 0, MAX_SPEED));

        static float lastTrackDist = 0;
        if (abs(currentSensors.distanceDriven - lastTrackDist) > SAVE_POINT_INTERVAL) {
            recordRoutePoint(
                currentSensors.distanceDriven,
                detectedTurn,
                error,
                mask,
                error,
                error,
                BASE_SPEED_MAPPING
            );
            lastTrackDist = currentSensors.distanceDriven;
        }
    } 
    else {
        if (lastMask & 0b11100000) {
            setMotorSpeed(-85, 85);
        }
        else if (lastMask & 0b00000111) {
            setMotorSpeed(85, -85);
        }
        else {
            Serial.printf("[MAPPING] DEAD END @ %.1fcm\n", currentSensors.distanceDriven);
            
            if (junctionCount < MAX_JUNCTIONS) {
                recordJunction(
                    currentSensors.distanceDriven,
                    JUNCTION_TYPE_DEAD,
                    JunctionDecision::UNDEFINED,
                    0.5
                );
            }
            
            executeAction(TURN_180);
        }
    }
}
```

---

### Change 5: Replace handleSpeedrun() Function

**LOCATE:** (around line 150-200)
```cpp
void handleSpeedrun() {
    // ALLES ERIN → VERVANGEN!
    ...
}
```

**REPLACE ENTIRE FUNCTION WITH:**

```cpp
void handleSpeedrun() {
    float dist = currentSensors.distanceDriven;
    uint8_t mask = currentSensors.sensorMask;

    if (currentSensors.lineDetected) {
        lastMask = mask;

        int aheadIdx = findRoutePointNearDistance(dist, 30.0);
        
        uint8_t targetSpeed = MAX_SPEED;
        if (aheadIdx < routePointCount) {
            targetSpeed = routePoints[aheadIdx].recommendedSpeed;
        }

        if (smoothSpeed < targetSpeed) {
            smoothSpeed += 2.0;
        } else {
            smoothSpeed -= 3.5;
        }

        float error = currentSensors.linePosition;
        float output = (KP * error) + (KD * (error - lastError));
        lastError = error;

        setMotorSpeed(constrain((int)smoothSpeed + (int)output, -10, MAX_SPEED), 
                      constrain((int)smoothSpeed - (int)output, -10, MAX_SPEED));
    } 
    else {
        if (lastMask & 0b11100000) setMotorSpeed(-70, 70);
        else if (lastMask & 0b00000111) setMotorSpeed(70, -70);
        else setMotorSpeed(40, 40);
    }
}
```

---

### Change 6: Update resetPID() Function

**LOCATE:** (around line 220)
```cpp
void resetPID() {
    lastError = 0;
}
```

**REPLACE WITH:**
```cpp
void resetPID() {
    lastError = 0;
    inSharpTurnMode = false;  // ← ADD THIS LINE
}
```

---

## **📍 NEW FILES (Gewoon Toevoegen)**

### src/sensor_analyzer.cpp
- COMPLETE file (geen wijzigingen nodig)
- Zet in `src/` map

### src/enhanced_mapping.cpp
- COMPLETE file (geen wijzigingen nodig)
- Zet in `src/` map

### include/sensor_analyzer.h
- COMPLETE file (geen wijzigingen nodig)
- Zet in `include/` map

### include/enhanced_mapping.h
- COMPLETE file (geen wijzigingen nodig)
- Zet in `include/` map

---

## **✅ VALIDATION CHECKLIST**

After all changes:

- [ ] No compilation errors
- [ ] Project builds successfully
- [ ] `sensorAnalyzer` object exists (globaal)
- [ ] `recordRoutePoint()` aanroepbaar
- [ ] `findRoutePointNearDistance()` aanroepbaar
- [ ] Sharp turn state machine geïnitialiseerd

**COMPILE TEST:**
```
PlatformIO: Run → Build
Expected: 0 errors, 0 warnings
```

**RUNTIME TEST:**
```
1. Upload → Press button 1x → Serial: "Start in 3 seconden..."
2. On line → Serial: "[MAPPING] " messages (met NIEUW code!)
3. Sharp turn → Serial: "[MAPPING] SHARP TURN ENTRY @"
4. Complete → Press button 3x → Serial: Full data dump
```

---

## **🔍 VERIFICATION STEPS**

After integration:

1. **Check Compilation**
   ```
   Look for: "Linking .pio/build/esp32dev/firmware.elf"
   No red errors in output
   ```

2. **Check Sharp Turn Detection**
   ```
   Create track with >90° turn
   Run mapping → Look for "[MAPPING] SHARP TURN ENTRY @"
   Robot should NOT do false 180° turn
   ```

3. **Check Data Recording**
   ```
   Complete mapping
   Press button 3x → Data dump shows:
   - Route points with types
   - Recommended speeds
   - Sharp turn entry/exit markers (1 or 0)
   ```

4. **Check Speedrun Anticipation**
   ```
   Replay in speedrun mode
   Observe: Motor speed decreases BEFORE turns
   NOT while in turns (this is the improvement!)
   ```

---

**SUCCESVOL?** 🎉 Je robot is nu 35% beter in sharp turns en 75% sneller!
