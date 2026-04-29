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

extern RobotState currentState;

// ============================================================================
// DEAD-END RECOVERY - SMOOTH 180° TURN
// ============================================================================

void performSmoothDeadEndRecovery() {
    Serial.println("[NAVIGATION] Dead-end: Strakke 180 graden draai!");
    stopMotors();
    delay(200);

    // 1. BLINDE FASE (Kom van de lijn af)
    // We draaien altijd strak over 1 kant (Links achteruit, Rechts vooruit)
    setMotorSpeed(-BASE_SPEED_TURNS, BASE_SPEED_TURNS);
    delay(400); // Gedurende 400ms negeren we de sensoren totaal

    // 2. ZOEK FASE
    unsigned long start = millis();
    while (millis() - start < 3000) {
        // Blijf strak draaien
        setMotorSpeed(-BASE_SPEED_TURNS, BASE_SPEED_TURNS);
        
        // Zodra de MIDDELSTE sensoren de lijn weer strak zien, STOP.
        if (currentSensors.sensorMask & MID_SENSOR_MASK) {
            Serial.println("[NAVIGATION] Lijn weer in het vizier!");
            break;
        }
        delay(5);
    }
    
    stopMotors();
    delay(100);
}



// ============================================================================
// OBSTACLE AVOIDANCE - SMOOTH ARC MANOEUVRE
// ============================================================================

void executeArcManoeuvre() {
    Serial.println("[NAVIGATION] OBSTAKEL: Start veilige boog linksom!");
    stopMotors();
    delay(200);

    // STAP 1: Draai naar LINKS (weg van de lijn)
    // We draaien op de plaats om de neus in de juiste richting te zetten
    setMotorSpeed(-BASE_SPEED_TURNS, BASE_SPEED_TURNS);
    delay(350); // Pas dit aan: hij moet ongeveer 45 graden naar links wijzen

    // STAP 2: Rij RECHTDOOR (om naast/voorbij het object te komen)
    // Dit zorgt ervoor dat de zijkant van de robot het object niet raakt
    setMotorSpeed(BASE_SPEED_MAPPING + 20, BASE_SPEED_MAPPING + 20);
    delay(700); // <-- VERHOOG DEZE delay als hij nog steeds de zijkant raakt

    // STAP 3: DRAAIEN EN VOORUITRIJDEN TEGELIJK (De "Zoek-boog")
    // We maken een flauwe bocht naar rechts terwijl we snelheid houden
    Serial.println("[NAVIGATION] Zoeken naar lijn met voorwaartse boog...");
    unsigned long searchStart = millis();
    bool onLine = false;

    while (millis() - searchStart < 5000) { // Maximaal 5 seconden zoeken
        // We laten de linkermotor veel sneller draaien dan de rechter
        // Hierdoor rijdt hij vooruit maar buigt hij langzaam naar rechts
        int leftSpeed = BASE_SPEED_MAPPING + 45; 
        int rightSpeed = BASE_SPEED_MAPPING - 25; // Rechterwiel draait langzaam vooruit
        
        setMotorSpeed(leftSpeed, rightSpeed);

        // Check of we de lijn raken met de sensoren
        // We kijken vooral of de rechter- of middensensoren de lijn raken
        if (currentSensors.sensorMask != 0x00) { 
            Serial.println("[NAVIGATION] Lijn gedetecteerd tijdens boog!");
            onLine = true;
            break;
        }
        delay(10);
    }

    if (!onLine) {
        Serial.println("[NAVIGATION] Lijn niet gevonden, stop voor veiligheid.");
        stopMotors();
    } else {
        // Even kort de andere kant op sturen om recht op de lijn te komen
        setMotorSpeed(-20, 20); 
        delay(50);
    }
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
    Serial.println("[NAVIGATION] Kruispunt: Linkerhandregel toepassen...");
    
    // Scan goed over het kruispunt (geeft de sensoren de tijd om de zijlijnen te zien)
    uint8_t branches = probeAvailableBranchesImproved();
    uint8_t targetDir = BRANCH_NONE;

    // We kijken heel specifiek naar de uiterste linker sensoren (mask 0b11000000)
    // Dit voorkomt dat een wiebelende PID een 'linker afslag' triggert.
    if ((branches & BRANCH_MASK_LEFT) || (currentSensors.sensorMask & 0b11000000)) {
        targetDir = BRANCH_LEFT;
        Serial.println("[NAVIGATION] Besluit: LINKS (Prioriteit 1)");
    } 
    else if (branches & BRANCH_MASK_STRAIGHT) {
        targetDir = BRANCH_STRAIGHT;
        Serial.println("[NAVIGATION] Besluit: RECHTDOOR (Prioriteit 2)");
    } 
    else if (branches & BRANCH_MASK_RIGHT) {
        targetDir = BRANCH_RIGHT;
        Serial.println("[NAVIGATION] Besluit: RECHTS (Prioriteit 3)");
    }

    // UITVOERING
    if (targetDir == BRANCH_LEFT) {
        turnUntilLineDetected(-1); // -1 is links
    } else if (targetDir == BRANCH_RIGHT) {
        turnUntilLineDetected(1);  // 1 is rechts
    } else if (targetDir == BRANCH_STRAIGHT) {
        // Belangrijk: Rij agressief door om de zijlijnen te negeren
        setMotorSpeed(BASE_SPEED_MAPPING, BASE_SPEED_MAPPING);
        delay(300); 
    }

    return targetDir;
}



bool turnUntilLineDetected(int turnDirection, unsigned long timeoutMs) {
    unsigned long turnStartTime = millis();
    
    // We verlagen de snelheid een klein beetje voor meer grip en precisie
    int turnSpeed = BASE_SPEED_TURNS - 5; 
    
    // Belangrijk: De MID_SENSOR_MASK moet idealiter 0b00011000 zijn
    // zodat de robot kaarsrecht op de lijn stopt.
    
    while (millis() - turnStartTime < timeoutMs) {
        // 1. Check of we de lijn gevonden hebben met de middelste sensoren
        if (currentSensors.sensorMask & 0b00011000) { 
            // DIRECTE REM: Zet motoren even kort in reverse voor een harde stop
            if (turnDirection < 0) setMotorSpeed(turnSpeed, -turnSpeed);
            else setMotorSpeed(-turnSpeed, turnSpeed);
            
            delay(15); // Korte 'tegendruk' om uitbollen te stoppen
            stopMotors();
            
            Serial.println("[NAV] Lijn gevonden en gestopt.");
            return true;
        }
        
        // 2. Blijf draaien
        if (turnDirection < 0) {
            setMotorSpeed(-turnSpeed, turnSpeed); // Links
        } else {
            setMotorSpeed(turnSpeed, -turnSpeed); // Rechts
        }
        
        // Kleine delay voor stabiliteit van de sensorlezingen
        delay(1); 
    }
    
    stopMotors();
    Serial.println("[NAV] Timeout: Lijn niet gevonden.");
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
    uint8_t foundBranches = 0;
    unsigned long probeStart = millis();
    
    // Rij langzaam vooruit en scan continu gedurende 200ms
    // Zo mist hij nooit een zijlijn die net iets verderop ligt
    setMotorSpeed(BASE_SPEED_MAPPING - 10, BASE_SPEED_MAPPING - 10);
    
    while (millis() - probeStart < 200) {
        if (currentSensors.sensorMask & LEFT_SIDE_MASK) foundBranches |= BRANCH_MASK_LEFT;
        if (currentSensors.sensorMask & MID_SENSOR_MASK) foundBranches |= BRANCH_MASK_STRAIGHT;
        if (currentSensors.sensorMask & RIGHT_SIDE_MASK) foundBranches |= BRANCH_MASK_RIGHT;
        delay(5);
    }
    
    stopMotors();
    return foundBranches;
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

    // --- 2. OBSTACLE DETECTION ---
    if (currentSensors.distance > 1.0 && currentSensors.distance < DISTANCE_THRESHOLD) {
        Serial.println("[MAPPING] Obstakel gezien: start boog!");
        executeArcManoeuvre(); // Nu voert hij eindelijk de boog uit in mapping!
        actionTimer = millis();
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

    // --- 4. SNELLE INTERSECTION & SHARP TURN DETECTION ---
    FinishState finishStatus = checkFinishStatus();
    uint8_t snapMask = currentSensors.sensorMask; 
    int sharpTurnDir = 0;
    bool isSharpTurn = detectSharpTurnGap(snapMask, sharpTurnDir);

    if ((finishStatus == INTERSECTION_DETECTED || snapMask == 0xFF || isSharpTurn) && (millis() - actionTimer > 600)) {
        
        // STAP A: QUICK FINISH CHECK
        if (snapMask == 0xFF) {
            delay(150); 
            if (currentSensors.sensorMask == 0xFF) {
                Serial.println("[MAPPING] >>> FINISH BEVESTIGD <<<");
                stopMotors();
                extern RobotState currentState;
                currentState = IDLE; 
                return;
            }
        }

        // STAP B: DE BESLISSING (Gefinetuned voor 90 graden)
        
        // 1. LINKS
        if (snapMask & 0b11100000) { 
            Serial.println("[MAPPING] Besluit: LINKS");
            saveIntersectionNode(currentSensors.distanceDriven, BRANCH_LEFT, snapMask | BRANCH_MASK_STRAIGHT);
            
            // KORTERE KICKSTART: Voorkomt doordraaien
            setMotorSpeed(-BASE_SPEED_TURNS, BASE_SPEED_TURNS); 
            delay(180); // <--- Verlaagd van 350 naar 180ms
            
            turnUntilLineDetected(-1); 
        } 
        
        // 2. RECHTDOOR
        else if (snapMask & 0b00011000) {
            Serial.println("[MAPPING] Besluit: RECHTDOOR");
            saveIntersectionNode(currentSensors.distanceDriven, BRANCH_STRAIGHT, snapMask);
            setMotorSpeed(BASE_SPEED_MAPPING + 10, BASE_SPEED_MAPPING + 10);
            delay(150);
        }
        
        // 3. RECHTS
        else if (snapMask & 0b00000111) {
            Serial.println("[MAPPING] Besluit: RECHTS");
            saveIntersectionNode(currentSensors.distanceDriven, BRANCH_RIGHT, snapMask);
            
            setMotorSpeed(BASE_SPEED_TURNS, -BASE_SPEED_TURNS);
            delay(180); // <--- Verlaagd van 350 naar 180ms
            
            turnUntilLineDetected(1);
        }
        
        // 4. Fallback scherpe bocht
        else if (isSharpTurn) {
            turnUntilLineDetected(sharpTurnDir);
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
    static unsigned long speedrunActionTimer = 0;
    FinishState finishStatus = checkFinishStatus();
    int sharpTurnDir = 0;
    bool isSharpTurn = detectSharpTurnGap(currentSensors.sensorMask, sharpTurnDir);
    
    if ((finishStatus == INTERSECTION_DETECTED || isSharpTurn) && (millis() - speedrunActionTimer > 600)) {
        
        // Check de opgeslagen map voor deze afstand
        uint8_t chosenBranch = selectSpeedrunBranch(currentSensors.distanceDriven);
        
        if (chosenBranch == BRANCH_LEFT) {
            Serial.println("[SPEEDRUN] Map zegt: LINKS");
            turnUntilLineDetected(-1);
        } 
        else if (chosenBranch == BRANCH_RIGHT) {
            Serial.println("[SPEEDRUN] Map zegt: RECHTS");
            turnUntilLineDetected(1);
        } 
        else if (chosenBranch == BRANCH_STRAIGHT) {
            Serial.println("[SPEEDRUN] Map zegt: RECHTDOOR");
            setMotorSpeed(BASE_SPEED_MAPPING, BASE_SPEED_MAPPING);
            delay(250); // Hard over het kruispunt heen rijden
        } 
        else if (isSharpTurn) {
            Serial.println("[SPEEDRUN] Scherpe bocht fallback");
            turnUntilLineDetected(sharpTurnDir);
        }

        speedrunActionTimer = millis();
        return;
    }

    // --- 3. EXECUTE SPEEDRUN CALCULATION (From Logic.cpp) ---
    // This handles PID, speed ramping, and motor control
    calculateSpeedrun();

    // --- 4. DISTANCE-BASED FINISH FALLBACK ---
    if (mapIndex > 0) {
        float finalMapDistance = trackMap[mapIndex - 1].distance;
        // Gebruik direct currentSensors.distanceDriven in plaats van currentDist
        if (currentSensors.distanceDriven > finalMapDistance + 30.0 && !currentSensors.lineDetected) {
            Serial.println("[SPEEDRUN] Beyond map + 30cm without line: Auto-finish");
            // Optioneel: zet de robot hier ook echt stil of zet de state op IDLE
            // stopMotors(); 
        }
    }
}
