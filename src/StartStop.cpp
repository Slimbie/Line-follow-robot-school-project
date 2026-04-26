#include "StartStop.h"
#include "Config.h"
#include "Sensors.h" // Voor currentSensors
#include "Motors.h"

static bool isCheckingFinish = false;
static float finishCheckStartDist = 0;

FinishState checkFinishStatus() {
    if (!USE_START_FINISH) return NOT_FINISH;

    bool seesFullBlack = (currentSensors.sensorMask == 0b11111111);

    if (seesFullBlack) {
        if (!isCheckingFinish) {
            isCheckingFinish = true;
            finishCheckStartDist = currentSensors.distanceDriven;
        }

        float distDriven = currentSensors.distanceDriven - finishCheckStartDist;
        if (distDriven >= FINISH_DRIVE_ON_CM) {
            return FINISH_CONFIRMED; 
        }
        
        setMotorSpeed(BASE_SPEED_MAPPING, BASE_SPEED_MAPPING);
        return CHECKING_FINISH;
    } 
    else {
        if (isCheckingFinish) {
            isCheckingFinish = false; 
            return INTERSECTION_DETECTED; 
        }
        return NOT_FINISH;
    }
}

bool handleStartLogic() {
    static bool startSequenceFinished = false;
    if (startSequenceFinished) return true;

    if (currentSensors.lineDetected) {
        startSequenceFinished = true;
        return true;
    } else {
        setMotorSpeed(BASE_SPEED_MAPPING, BASE_SPEED_MAPPING);
        return false;
    }
}

void resetStartSequence() {
    isCheckingFinish = false;
}