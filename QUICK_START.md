# **QUICK START: Copy-Paste Implementation Guide**

## **FASE 1: Voorbereiding (5 minuten)**

### Stap 1.1: Project openen
```
Platform IO Home → Open "Line follow robot school project"
```

### Stap 1.2: Check bestandsstructuur
```
✓ src/
✓ include/
✓ lib/
✓ test/
```

---

## **FASE 2: Nieuwe Bestanden Toevoegen (10 minuten)**

### Stap 2.1: `include/sensor_analyzer.h` Aanmaken

**Klik:** `New File` in Explorer → `include/` map

**Paste:** [Volledige inhoud uit `sensor_analyzer.h` file die ik eerder maakte]

### Stap 2.2: `src/sensor_analyzer.cpp` Aanmaken

**Klik:** `New File` → `src/` map

**Paste:** [Volledige inhoud uit `sensor_analyzer.cpp` file]

### Stap 2.3: `include/enhanced_mapping.h` Aanmaken

**Paste:** [Volledige inhoud uit `enhanced_mapping.h` file]

### Stap 2.4: `src/enhanced_mapping.cpp` Aanmaken

**Paste:** [Volledige inhoud uit `enhanced_mapping.cpp` file]

---

## **FASE 3: MAIN.CPP Updaten (15 minuten)**

### Stap 3.1: Includes toevoegen

**Open:** `src/main.cpp`

**Zoek naar:**
```cpp
#include "StartStop.h"
```

**Voeg toe ERNA:**
```cpp
#include "sensor_analyzer.h"
#include "enhanced_mapping.h"
```

### Stap 3.2: Global junctionMap toevoegen

**Zoek naar:**
```cpp
Junction junctionMap[50];
int junctionCount = 0;
```

**Vervang DOOR:**
```cpp
// Old structures (kept for backward compatibility if needed)
// Junction junctionMap[50];
// int junctionCount = 0;
// ^^^ Commented out - now using enhanced_mapping instead

// Global junctions (from enhanced_mapping)
extern JunctionDecision junctionMap[MAX_JUNCTIONS];
extern int junctionCount;
```

### Stap 3.3: Setup() updaten

**Zoek naar:**
```cpp
void setup() {
    Serial.begin(115200);
    
    // Hardware init
    setupMotors();
    startSensorTask();
    
    // Knoppen configureren
    pinMode(START_BUTTON_PIN, INPUT_PULLUP);
    pinMode(MODE_SWITCH_PIN, INPUT_PULLUP);

    Serial.println("=== ROBOT GEREED ===");
```

**Voeg ERNA toe:**
```cpp
    // NIEUW: Initialize sensor analyzer
    sensorAnalyzer.init();
```

### Stap 3.4: COUNTDOWN Case updaten

**Zoek naar:**
```cpp
        case COUNTDOWN:
            Serial.println("Start in 3 seconden...");
            delay(3000);
            
            encL.setCount(0); 
            encR.setCount(0);
            resetStartSequence(); 
            
            if (digitalRead(MODE_SWITCH_PIN) == LOW) {
                resetMap();
                mappingStartTime = millis();
                currentState = MAPPING;
                Serial.println("MODE: MAPPING");
            } else {
                prepareSpeedrun();
                speedrunStartTime = millis();
                currentState = SPEEDRUN;
                Serial.println("MODE: SPEEDRUN");
            }
            break;
```

**Vervang DOOR:**
```cpp
        case COUNTDOWN:
            Serial.println("Start in 3 seconden...");
            delay(3000);
            
            encL.setCount(0); 
            encR.setCount(0);
            resetStartSequence(); 
            
            if (digitalRead(MODE_SWITCH_PIN) == LOW) {
                // CHANGED: Use enhanced mapping
                initEnhancedMapping();  // instead of resetMap()
                mappingStartTime = millis();
                currentState = MAPPING;
                Serial.println("MODE: MAPPING");
            } else {
                // CHANGED: Use enhanced speedrun prep
                prepareForSpeedrun();  // instead of prepareSpeedrun()
                speedrunStartTime = millis();
                currentState = SPEEDRUN;
                Serial.println("MODE: SPEEDRUN");
            }
            break;
```

### Stap 3.5: DATA_DUMP Case updaten

**Zoek naar de DATA_DUMP case (voeg toe als deze niet bestaat):**

```cpp
        case DATA_DUMP: {
            unsigned long mappingTime = millis() - mappingStartTime;
            dumpMap(mappingTime);  // OLD VERSION
            currentState = IDLE;
            buttonPressCount = 0;
            break;
        }
```

**Vervang DOOR:**
```cpp
        case DATA_DUMP: {
            unsigned long mappingTime = millis() - mappingStartTime;
            // CHANGED: Use enhanced dump
            dumpEnhancedRoute(mappingTime);  // instead of dumpMap()
            currentState = IDLE;
            buttonPressCount = 0;
            break;
        }
```

---

## **FASE 4: LOGIC.CPP Vervangen (30 minuten)**

### Stap 4.1: Backups maken
```
Klik rechts op src/logic.cpp → "Copy" → paste als src/logic.cpp.backup
Zet dit in een "backups" folder
```

### Stap 4.2: Top van logic.cpp updaten

**Open:** `src/logic.cpp`

**Zoek naar:**
```cpp
#include "logic.h"
#include "config.h"
#include "sensors.h"
#include "motors.h"
#include "mapping.h"
#include "StartStop.h"
```

**Voeg toe ERNA:**
```cpp
#include "sensor_analyzer.h"
#include "enhanced_mapping.h"
```

### Stap 4.3: Static variabelen toevoegen

**Zoek naar:**
```cpp
static float smoothSpeed = BASE_SPEED_MAPPING; 
static float lastError = 0;                    
static uint8_t lastMask = 0;                   
static int currentJunctionIndex = 0;
```

**Voeg toe ERNA:**
```cpp
// --- NEW: Sharp turn state machine ---
static bool inSharpTurnMode = false;
static uint8_t sharpTurnSidelineDir = 0;
static uint32_t sharpTurnEntryTime = 0;
```

### Stap 4.4: NIEUWE handleSharpTurnInFlight() toevoegen

**Voeg toe VOOR executeAction() function:**

```cpp
// ============================================================================
// HELPER: SHARP TURN HANDLING (NIEUW!)
// ============================================================================

void handleSharpTurnInFlight(uint8_t sensorMask, float lineError) {
    /**
     * Dit wordt aangeroepen TIJDENS een scherpe bocht
     * Taak: Draai zoeken naar de "side line" die we eerder zagen
     */
    
    uint32_t timeSinceEntry = millis() - sharpTurnEntryTime;
    
    // Timeout: Als sharp turn langer dan 700ms duurt, reset
    if (timeSinceEntry > 700) {
        inSharpTurnMode = false;
        return;
    }
    
    if (sensorMask == 0) {
        // === FASE 1: DRAAI (geen lijn gezien) ===
        
        if (sharpTurnSidelineDir & 0b11100000) {
            // Side-line was LINKS → draai links
            setMotorSpeed(-75, 75);
            delay(1);
        } 
        else if (sharpTurnSidelineDir & 0b00000111) {
            // Side-line was RECHTS → draai rechts
            setMotorSpeed(75, -75);
            delay(1);
        }
        else {
            // Fallback: spin
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

### Stap 4.5: VERVANG handleMapping() function

**ALLES uit handleMapping() vervangen** (dit is het grootste deel)

**Vervang DOOR:**

```cpp
void handleMapping() {
    float error = currentSensors.linePosition;
    uint8_t mask = currentSensors.sensorMask;

    // STAP 1: SENSOR ANALYSE (NIEUW!)
    TurnType detectedTurn = sensorAnalyzer.analyzeTurn(mask, error, error);

    // STAP 2: FINISH CHECK
    FinishState state = checkFinishStatus();

    if (state == FINISH_CONFIRMED) {
        stopMotors();
        return; 
    }
    if (state == CHECKING_FINISH) return; 

    // STAP 3: SHARP TURN HANDLING (KRITIEK NIEUW!)
    if (inSharpTurnMode) {
        handleSharpTurnInFlight(mask, error);
        return;
    }

    // STAP 4: SHARP TURN ENTRY DETECTIE
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

    // STAP 5: KRUISPUNT CHECK
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

    // STAP 6: NORMALE LIJN (PID)
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
    // STAP 7: LIJN KWIJT
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

### Stap 4.6: VERVANG handleSpeedrun() function

**Vervang GEHELE handleSpeedrun():**

```cpp
void handleSpeedrun() {
    float dist = currentSensors.distanceDriven;
    uint8_t mask = currentSensors.sensorMask;

    // STAP 1: LOOKAHEAD (VERBETERD: 30cm)
    if (currentSensors.lineDetected) {
        lastMask = mask;

        int aheadIdx = findRoutePointNearDistance(dist, 30.0);
        
        uint8_t targetSpeed = MAX_SPEED;
        if (aheadIdx < routePointCount) {
            targetSpeed = routePoints[aheadIdx].recommendedSpeed;
        }

        // STAP 2: SMOOTH SPEED RAMPING
        if (smoothSpeed < targetSpeed) {
            smoothSpeed += 2.0;
        } else {
            smoothSpeed -= 3.5;
        }

        // STAP 3: PID CONTROL
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

### Stap 4.7: UPDATE resetPID() function

**Zoek naar:**
```cpp
void resetPID() {
    lastError = 0;
}
```

**Vervang DOOR:**
```cpp
void resetPID() {
    lastError = 0;
    inSharpTurnMode = false; // Safety reset
}
```

---

## **FASE 5: Testing (20 minuten)**

### Stap 5.1: Compile Check
```
Ctrl+Alt+B → "PlatformIO: Build"
Watch for errors (should have NONE)
```

### Stap 5.2: Upload naar Robot
```
Ctrl+Alt+U → "PlatformIO: Upload"
Or use button in bottom bar
```

### Stap 5.3: Basic Test
```
1. Robot on start
2. Press button 1x → should say "MAPPING"
3. Place on line
4. Robot should follow line with NEW logic
5. Check Serial for "[MAPPING] " messages
```

### Stap 5.4: Sharp Turn Test
```
1. Create test track with >90° turn
2. Run mapping mode
3. Watch Serial for: "[MAPPING] SHARP TURN ENTRY @ X.Xcm"
4. Robot should handle turn smoothly
5. No false 180° turns should happen!
```

### Stap 5.5: Data Dump Test
```
1. Complete mapping run
2. Press button 3x
3. Serial should show full route dump
4. Look for: 
   - Total points recorded
   - Sharp turn entry/exit markers (1 or 0)
   - Recommended speeds
```

---

## **TROUBLESHOOTING**

| Error | Solution |
|-------|----------|
| `error: 'sensorAnalyzer' was not declared` | Voeg `#include "sensor_analyzer.h"` toe in main.cpp |
| `error: 'recordRoutePoint' was not declared` | Voeg `#include "enhanced_mapping.h"` toe in logic.cpp |
| `error: 'MAX_JUNCTIONS' was not declared` | Check enhanced_mapping.h is #define'd |
| Compilation takes long | Normal, new code is bigger. Wait 10 seconds. |
| Serial shows garbage | Check baud rate is 115200 in config.h |
| Robot doesn't detect sharp turn | Check sensorAnalyzer.init() called in setup() |
| Motor stays still | Check that recordRoutePoint() is called |

---

## **SUCCESS INDICATORS**

✓ **All Compiled Successfully**
- Zero compilation errors
- Upload completes without errors

✓ **Sharp Turn Working**
- Serial shows "[MAPPING] SHARP TURN ENTRY" messages
- Robot turns smoothly, no false 180° turns
- Data dump shows isSharpTurnEntry=1 markers

✓ **Data Recording Working**
- Lots of Serial output during mapping
- Data dump contains >50 route points
- Recommended speeds vary (60-180)

✓ **Speedrun Working**
- Robot faster than mapping speed
- Robot slows down BEFORE turns (not in them)
- Smooth acceleration/deceleration observed
