#include "Navigation.h"
#include "Config.h"
#include "Sensors.h"
#include "Motors.h"
#include "Mapping.h"
#include "Logic.h"
#include "StartStop.h"

// External variables from main.cpp
extern float lastSavedDistance;

// Forward declarations for functions defined in main.cpp
extern void saveIntersectionNode(float distance, uint8_t branchDirection, uint8_t branchMask);
extern void markLastNodeDeadEnd();

// ============================================================================
// DEAD-END RECOVERY - SMOOTH 180° TURN
// ============================================================================

void performSmoothDeadEndRecovery() {
    Serial.println("[NAVIGATION] Dead-end gedetecteerd, vloeiende 180° draai starten...");
    
    // Stop motors first
    stopMotors();
    delay(50);
    
    // Begin with a smooth 180° turn
    // Use a gradual speed curve instead of sudden full speed
    const int TURN_SPEED_START = 80;
    const int TURN_SPEED_MAX = 140;
    unsigned long turnStartTime = millis();
    unsigned long turnTimeout = 3500; // Maximum 3.5 seconds for the turn
    
    // Sensor-based exit: stop when middle sensors detect the line
    while (millis() - turnStartTime < turnTimeout) {
        // Check if line is detected in the middle (after 180° turn)
        if (currentSensors.sensorMask & MID_SENSOR_MASK) {
            Serial.println("[NAVIGATION] Lijn herontdekt na 180° draai!");
            stopMotors();
            delay(100);
            return;
        }
        
        // Smooth acceleration during the turn
        unsigned long elapsed = millis() - turnStartTime;
        float progress = (float)elapsed / turnTimeout;
        
        // Ramp up speed smoothly
        int speed;
        if (progress < 0.3f) {
            speed = TURN_SPEED_START + (int)((TURN_SPEED_MAX - TURN_SPEED_START) * (progress / 0.3f));
        } else if (progress > 0.8f) {
            speed = TURN_SPEED_MAX - (int)((TURN_SPEED_MAX - TURN_SPEED_START) * ((progress - 0.8f) / 0.2f));
        } else {
            speed = TURN_SPEED_MAX;
        }
        
        // Left turn (counterclockwise when viewed from above)
        setMotorSpeed(-speed, speed);
        delay(5);
    }
    
    // Timeout reached, stop anyway
    stopMotors();
    Serial.println("[NAVIGATION] Timeout: 180° draai voltooid zonder lijn-detectie");
    delay(100);
}

// ============================================================================
// OBSTACLE AVOIDANCE - SMOOTH ARC MANOEUVRE
// ============================================================================

void executeArcManoeuvre() {
    Serial.println("[NAVIGATION] Arc-manoeuvre starten rond obstakel...");
    
    // Phase 1: Drive right in a smooth arc (already exists in main.cpp)
    // This is handled by handleObstacleAvoidance() - we just need to ensure
    // it uses smooth transitions instead of hard speed changes
    
    // The arc logic is already in handleObstacleAvoidance()
    // This function serves as documentation/wrapper
}

void searchLineAfterObstacle() {
    Serial.println("[NAVIGATION] Zoeken naar lijn na obstakel...");
    
    // Implement a spiral search pattern if line is not immediately found
    int searchSpeed = 40;
    unsigned long searchTimeout = millis() + 5000; // 5 second max search
    
    // First, try to find the line by turning left gently
    while (millis() < searchTimeout && !currentSensors.lineDetected) {
        // Spiral: gradually increase search radius
        setMotorSpeed(searchSpeed - 15, searchSpeed + 25);
        delay(10);
    }
    
    if (currentSensors.lineDetected) {
        Serial.println("[NAVIGATION] Lijn gevonden na obstakel!");
    } else {
        Serial.println("[NAVIGATION] Lijn niet gevonden na obstakel-zoekactie");
    }
}

// ============================================================================
// INTERSECTION HANDLING - LEFT-HAND RULE
// ============================================================================

uint8_t handleIntersectionLeftHandRule() {
    Serial.println("[NAVIGATION] Kruispunt gedetecteerd - linkerhandregel toepassing");
    
    // Probe available branches
    uint8_t branches = probeAvailableBranchesImproved();
    
    // Apply strict left-hand rule
    uint8_t targetDir = BRANCH_NONE;
    if (branches & BRANCH_MASK_LEFT) {
        targetDir = BRANCH_LEFT;
        Serial.println("[NAVIGATION] Kruispunt: LINKS gekozen (linkerhandregel)");
    } else if (branches & BRANCH_MASK_STRAIGHT) {
        targetDir = BRANCH_STRAIGHT;
        Serial.println("[NAVIGATION] Kruispunt: RECHTDOOR gekozen");
    } else if (branches & BRANCH_MASK_RIGHT) {
        targetDir = BRANCH_RIGHT;
        Serial.println("[NAVIGATION] Kruispunt: RECHTS gekozen");
    }
    
    // Execute the turn
    if (targetDir == BRANCH_LEFT) {
        if (turnUntilLineDetected(-1)) {
            Serial.println("[NAVIGATION] Linkerbochtdraai voltooid");
        }
    } else if (targetDir == BRANCH_RIGHT) {
        // Only turn right if there's NO middle line (left-hand rule compliance)
        if (!(currentSensors.sensorMask & MID_SENSOR_MASK)) {
            if (turnUntilLineDetected(1)) {
                Serial.println("[NAVIGATION] Rechterbochtdraai voltooid");
            }
        }
    }
    
    return targetDir;
}

bool turnUntilLineDetected(int turnDirection, unsigned long timeoutMs) {
    unsigned long turnStartTime = millis();
    int turnSpeed = BASE_SPEED_TURNS;
    
    while (millis() - turnStartTime < timeoutMs) {
        // Check if middle sensors detect the line
        if (currentSensors.sensorMask & MID_SENSOR_MASK) {
            stopMotors();
            return true;
        }
        
        // Continue turning in the specified direction
        if (turnDirection < 0) {
            // Left turn
            setMotorSpeed(-turnSpeed, turnSpeed);
        } else {
            // Right turn
            setMotorSpeed(turnSpeed, -turnSpeed);
        }
        delay(5);
    }
    
    stopMotors();
    return false;
}

// ============================================================================
// SPEEDRUN NAVIGATION - MAP-BASED ROUTE SELECTION
// ============================================================================

uint8_t selectSpeedrunBranch(float currentDistance) {
    // Find the current node in the saved map
    int nodeIndex = getMapIndexAtDistance(currentDistance);
    
    // If we don't know where we are, fall back to left-hand rule
    if (nodeIndex < 0 || nodeIndex >= mapIndex) {
        Serial.println("[SPEEDRUN] Onbekende locatie, terugval op linkerhandregel");
        return handleIntersectionLeftHandRule();
    }
    
    // Get the node information
    PathPoint &node = trackMap[nodeIndex];
    if (!node.isNode) {
        Serial.println("[SPEEDRUN] Huidige locatie is geen knoop, gebruik rechttoe rechtaan");
        return BRANCH_STRAIGHT;
    }
    
    // Build available branches from the saved data
    uint8_t availableBranches = node.availableBranches;
    
    Serial.printf("[SPEEDRUN] Node %d gevonden: available=%d, branches mask=0x%02x\n", 
                  nodeIndex, node.isNode, availableBranches);
    
    // STRATEGY: Prefer directions that DON'T lead to dead-ends
    // Order: Left > Straight > Right (left-hand rule), but skip dead-ends
    
    uint8_t recommendedDir = BRANCH_NONE;
    
    // Priority 1: Left, if available and not a dead end
    if (availableBranches & BRANCH_MASK_LEFT) {
        if (!isDeadEndBranch(nodeIndex, BRANCH_LEFT)) {
            recommendedDir = BRANCH_LEFT;
            Serial.printf("[SPEEDRUN] Node %d: Kies LINKS (niet doodlopend)\n", nodeIndex);
            return recommendedDir;
        } else {
            Serial.printf("[SPEEDRUN] Node %d: LINKS is doodlopend, skip\n", nodeIndex);
        }
    }
    
    // Priority 2: Straight, if available and not a dead end
    if (availableBranches & BRANCH_MASK_STRAIGHT) {
        if (!isDeadEndBranch(nodeIndex, BRANCH_STRAIGHT)) {
            recommendedDir = BRANCH_STRAIGHT;
            Serial.printf("[SPEEDRUN] Node %d: Kies RECHTDOOR (niet doodlopend)\n", nodeIndex);
            return recommendedDir;
        } else {
            Serial.printf("[SPEEDRUN] Node %d: RECHTDOOR is doodlopend, skip\n", nodeIndex);
        }
    }
    
    // Priority 3: Right, if available and not a dead end
    if (availableBranches & BRANCH_MASK_RIGHT) {
        if (!isDeadEndBranch(nodeIndex, BRANCH_RIGHT)) {
            recommendedDir = BRANCH_RIGHT;
            Serial.printf("[SPEEDRUN] Node %d: Kies RECHTS (niet doodlopend)\n", nodeIndex);
            return recommendedDir;
        } else {
            Serial.printf("[SPEEDRUN] Node %d: RECHTS is doodlopend, skip\n", nodeIndex);
        }
    }
    
    // FALLBACK: If all viable routes are dead-ends, use left-hand rule anyway
    Serial.printf("[SPEEDRUN] Node %d: Geen niet-doodlopende routes, fallback naar linkerhandregel\n", nodeIndex);
    if (availableBranches & BRANCH_MASK_LEFT) return BRANCH_LEFT;
    else if (availableBranches & BRANCH_MASK_STRAIGHT) return BRANCH_STRAIGHT;
    else if (availableBranches & BRANCH_MASK_RIGHT) return BRANCH_RIGHT;
    
    return BRANCH_NONE; // Should never reach here
}

bool isDeadEndBranch(int nodeIndex, uint8_t branchDirection) {
    if (nodeIndex < 0 || nodeIndex >= mapIndex) return false;
    
    PathPoint &node = trackMap[nodeIndex];
    
    // If the node itself is marked as dead end and matches this direction, it's a dead end
    if (node.isDeadEnd && node.branchDirection == branchDirection) {
        return true;
    }
    
    // Look ahead: if the next node in this direction is a dead end, mark this branch as problematic
    // (This is optional, for more advanced route planning)
    
    return false;
}

// ============================================================================
// UTILITY HELPERS
// ============================================================================

uint8_t probeAvailableBranchesImproved() {
    // Drive forward slightly to probe the intersection
    setMotorSpeed(BASE_SPEED_MAPPING, BASE_SPEED_MAPPING);
    delay(INTERSECTION_PROBE_MS);
    
    // Read the available branches
    uint8_t branches = 0;
    if (currentSensors.sensorMask & LEFT_SIDE_MASK) branches |= BRANCH_MASK_LEFT;
    if (currentSensors.sensorMask & MID_SENSOR_MASK) branches |= BRANCH_MASK_STRAIGHT;
    if (currentSensors.sensorMask & RIGHT_SIDE_MASK) branches |= BRANCH_MASK_RIGHT;
    
    stopMotors();
    return branches;
}

bool isDeadEndPattern(uint8_t sensorMask) {
    // Dead-end is when only middle sensors see the line, and all sides are white
    bool midLine = (sensorMask & MID_SENSOR_MASK);
    bool noSides = !(sensorMask & (LEFT_SIDE_MASK | RIGHT_SIDE_MASK));
    
    return midLine && noSides;
}

bool detectSharpTurnGap(uint8_t sensorMask, int &turnDirection) {
    // Sharp left gap: extreme left sensors + gap in middle-left area
    bool sharpLeftGap = (sensorMask & 0b11000000) && !(sensorMask & 0b00100000);
    
    // Sharp right gap: extreme right sensors + gap in middle-right area
    bool sharpRightGap = (sensorMask & 0b00000011) && !(sensorMask & 0b00000100);
    
    if (sharpLeftGap) {
        turnDirection = -1; // Left turn
        return true;
    }
    if (sharpRightGap) {
        turnDirection = 1; // Right turn
        return true;
    }
    
    return false;
}

// ============================================================================
// MAPPING STATE EXECUTION (Helper for main.cpp)
// ============================================================================

void executeMappingState() {
    static unsigned long whiteTime = 0; 
    static int lastSideSeen = 0; 
    static unsigned long actionTimer = 0; 
    static unsigned long lastPrint = 0;
    static unsigned long finishDetectedTime = 0;

    // --- 1. SENSOR DATA LOGGING FOR DEBUG ---
    if (millis() - lastPrint > 150) {
        Serial.print("[MAPPING] Sensoren: [");
        for (int i = 7; i >= 0; i--) Serial.print((currentSensors.sensorMask & (1 << i)) ? "X" : "_");
        int cleanDist = (currentSensors.distance > 0 && currentSensors.distance < 200) ? currentSensors.distance : 0;
        Serial.printf("] | Dist: %3d cm | Driven: %5.1f cm\n", cleanDist, currentSensors.distanceDriven);
        lastPrint = millis();
    }

    // --- 2. OBSTACLE DETECTION (WAIT MODE) ---
    if (currentSensors.distance > 1 && currentSensors.distance < 15) {
        stopMotors();
        Serial.println("[MAPPING] Obstakel gezien: wachten...");
        return;
    }

    // --- 3. FINISH DETECTION (200ms debounce) ---
    if (currentSensors.sensorMask == 0xFF) {
        if (finishDetectedTime == 0) finishDetectedTime = millis();
        
        if (millis() - finishDetectedTime > 200) { 
            stopMotors();
            Serial.println("[MAPPING] >>> FINISH BEVESTIGD <<<");
            // Mark last node distance for later reference
            return;
        }
        return; 
    } else {
        finishDetectedTime = 0; 
    }

    // --- 4. INTERSECTION & SHARP TURN DETECTION ---
    FinishState finishStatus = checkFinishStatus();
    bool midLine = (currentSensors.sensorMask & MID_SENSOR_MASK);
    
    int sharpTurnDir = 0;
    bool isSharpTurn = detectSharpTurnGap(currentSensors.sensorMask, sharpTurnDir);

    if ((finishStatus == INTERSECTION_DETECTED || isSharpTurn) && (millis() - actionTimer > 600)) {
        
        // Move forward slightly to align for the turn
        setMotorSpeed(BASE_SPEED_MAPPING, BASE_SPEED_MAPPING);
        delay(45); 
        
        // Handle sharp turns
        if (isSharpTurn && !midLine) {
            Serial.printf("[MAPPING] Scherpe bocht gedetecteerd: richting=%d\n", sharpTurnDir);
            
            if (sharpTurnDir < 0) {
                // Sharp left
                if (turnUntilLineDetected(-1)) {
                    Serial.println("[MAPPING] Scherpe linkerbochtdraai voltooid");
                    saveIntersectionNode(currentSensors.distanceDriven, BRANCH_LEFT, BRANCH_MASK_LEFT);
                }
            } else {
                // Sharp right
                if (turnUntilLineDetected(1)) {
                    Serial.println("[MAPPING] Scherpe rechterbochtdraai voltooid");
                    saveIntersectionNode(currentSensors.distanceDriven, BRANCH_RIGHT, BRANCH_MASK_RIGHT);
                }
            }
        } else if (finishStatus == INTERSECTION_DETECTED) {
            // Regular intersection: apply left-hand rule
            uint8_t targetDir = handleIntersectionLeftHandRule();
            if (targetDir != BRANCH_NONE) {
                uint8_t branches = probeAvailableBranchesImproved();
                saveIntersectionNode(currentSensors.distanceDriven, targetDir, branches);
            }
        }
        
        whiteTime = 0;
        actionTimer = millis();
        return;
    }

    // --- 5. LINE FOLLOWING OR DEAD-END SEARCH ---
    if (currentSensors.lineDetected) {
        whiteTime = 0; 
        calculatePID();
    } else {
        if (whiteTime == 0) whiteTime = millis();
        
        // Dead-end recovery after timeout
        if (millis() - whiteTime > 600) { 
            Serial.println("[MAPPING] Dead-end timeout: 180° draai initiatief");
            markLastNodeDeadEnd();
            performSmoothDeadEndRecovery();
            whiteTime = 0;
            actionTimer = millis();
            return;
        } else {
            // Search based on last seen side
            if (lastSideSeen == -1) setMotorSpeed(-BASE_SPEED_TURNS, BASE_SPEED_TURNS);
            else if (lastSideSeen == 1) setMotorSpeed(BASE_SPEED_TURNS, -BASE_SPEED_TURNS);
            else setMotorSpeed(BASE_SPEED_MAPPING - 15, BASE_SPEED_MAPPING - 15);
        }
    }

    // --- 6. MEMORY UPDATE & DATA SAVING ---
    if (currentSensors.sensorMask & 0b11100000) lastSideSeen = -1;
    else if (currentSensors.sensorMask & 0b00000111) lastSideSeen = 1;

    float distSinceLastSave = abs(currentSensors.distanceDriven - lastSavedDistance);
    if (currentSensors.lineDetected && distSinceLastSave >= NODE_SAVE_DISTANCE) {
        savePoint(currentSensors.distanceDriven, currentSensors.linePosition, 
                  currentSensors.distance, currentSensors.leftTicks, currentSensors.rightTicks);
        lastSavedDistance = currentSensors.distanceDriven;
    }
}

// ============================================================================
// SPEEDRUN STATE EXECUTION (Helper for main.cpp)
// ============================================================================

void executeSpeedrunState() {
    // Check for obstacles first
    // (This should be called from main.cpp before this function)
    
    // --- 1. FINISH DETECTION ---
    // Note: finish detection is handled in main.cpp's SPEEDRUN case
    // This function focuses on navigation only
    
    // --- 2. MAP-BASED INTERSECTION HANDLING ---
    float currentDist = currentSensors.distanceDriven;
    int sharpTurnDir = 0;
    bool isSharpTurn = detectSharpTurnGap(currentSensors.sensorMask, sharpTurnDir);
    
    // At intersections, use map to decide which way to go
    if (isSharpTurn && (currentSensors.sensorMask & MID_SENSOR_MASK) == 0) {
        // This is an intersection - choose branch based on map
        uint8_t chosenBranch = selectSpeedrunBranch(currentDist);
        
        if (chosenBranch == BRANCH_LEFT && sharpTurnDir < 0) {
            Serial.printf("[SPEEDRUN] Sharp intersection: LINKS (map-gebaseerd)\n");
        } else if (chosenBranch == BRANCH_RIGHT && sharpTurnDir > 0) {
            Serial.printf("[SPEEDRUN] Sharp intersection: RECHTS (map-gebaseerd)\n");
        } else if (chosenBranch == BRANCH_STRAIGHT) {
            Serial.printf("[SPEEDRUN] Sharp intersection: RECHTDOOR (map-gebaseerd)\n");
        }
    }

    // --- 3. EXECUTE SPEEDRUN CALCULATION (From Logic.cpp) ---
    // This handles PID, speed ramping, and motor control
    calculateSpeedrun();

    // --- 4. DISTANCE-BASED FINISH FALLBACK ---
    // If we go beyond the mapped distance without a line, assume finish
    if (mapIndex > 0) {
        float finalMapDistance = trackMap[mapIndex - 1].distance;
        if (currentDist > finalMapDistance + 30.0 && !currentSensors.lineDetected) {
            Serial.println("[SPEEDRUN] Beyond map + 30cm without line: Auto-finish");
            // Signal to main.cpp to handle the state change
        }
    }
}
