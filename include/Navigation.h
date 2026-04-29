#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <Arduino.h>
#include "types.h"
#include "sensors.h"

// ============================================================================
// DEAD-END RECOVERY
// ============================================================================
/**
 * Performs a smooth 180° turn until the robot detects the line again.
 * Uses sensor feedback to ensure the PID is not interfered with.
 */
void performSmoothDeadEndRecovery();

// ============================================================================
// OBSTACLE AVOIDANCE - ARC MANOEUVRE
// ============================================================================
/**
 * Execute a smooth arc manoeuvre around an obstacle.
 * The robot will:
 * 1. Drive right in an arc around the obstacle
 * 2. Search for the line after the obstacle
 * 3. Return to the previous state once the line is found
 */
void executeArcManoeuvre();

/**
 * Search for the line after obstacle avoidance using a controlled search pattern.
 */
void searchLineAfterObstacle();

// ============================================================================
// INTERSECTION HANDLING - LEFT-HAND RULE
// ============================================================================
/**
 * Handle intersection with strict left-hand rule compliance.
 * Probes available branches and chooses according to:
 * 1. Left branch (highest priority)
 * 2. Straight (second priority)
 * 3. Right (lowest priority)
 * Returns the chosen branch direction
 */
uint8_t handleIntersectionLeftHandRule();

/**
 * Turns the robot in the specified direction until middle sensors detect the line.
 * Direction: -1 for left turn, 1 for right turn
 * Returns: true if line was detected, false if timeout occurred
 */
bool turnUntilLineDetected(int turnDirection, unsigned long timeoutMs = 2500);

// ============================================================================
// SPEEDRUN NAVIGATION - MAP-BASED ROUTE SELECTION
// ============================================================================
/**
 * Determine the best direction at an intersection using the saved map.
 * If the current node is marked as a dead-end in the chosen direction,
 * the function skips that direction.
 * Falls back to left-hand rule if the current location is unknown.
 */
uint8_t selectSpeedrunBranch(float currentDistance);

/**
 * Check if a given branch direction is marked as a dead-end in the saved map.
 */
bool isDeadEndBranch(int nodeIndex, uint8_t branchDirection);

// ============================================================================
// UTILITY HELPERS
// ============================================================================

/**
 * Probes available branches at the current location.
 * The robot moves forward briefly to sense all available exits.
 */
uint8_t probeAvailableBranchesImproved();

/**
 * Checks if the robot has only the middle sensors detecting the line
 * (typical dead-end scenario).
 */
bool isDeadEndPattern(uint8_t sensorMask);

/**
 * Detects if there's a gap pattern indicating a sharp turn:
 * - Left gap: extreme left sensors + gap in middle-left
 * - Right gap: extreme right sensors + gap in middle-right
 */
bool detectSharpTurnGap(uint8_t sensorMask, int &turnDirection);

/**
 * Execute the complete MAPPING state logic.
 * This is called from main.cpp's MAPPING case for cleaner code.
 */
void executeMappingState();

/**
 * Execute the complete SPEEDRUN state logic with map-based navigation.
 * This is called from main.cpp's SPEEDRUN case for cleaner code.
 */
void executeSpeedrunState();

#endif
