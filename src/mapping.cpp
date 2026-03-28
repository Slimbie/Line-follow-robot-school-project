#include "Mapping.h"

PathPoint trackMap[MAX_POINTS];
int mapIndex = 0;

void savePoint(float dist, float curve, uint8_t type) {
    if (mapIndex < MAX_POINTS) {
        trackMap[mapIndex].distance = dist;
        trackMap[mapIndex].curve = curve;
        trackMap[mapIndex].type = type;
        mapIndex++;
    }
}

void dumpMap() {
    Serial.println("--- START CSV DATA ---");
    Serial.println("Afstand_cm;Lijn_Positie;Type");
    for (int i = 0; i < mapIndex; i++) {
        Serial.printf("%.1f;%.2f;%d\n", trackMap[i].distance, trackMap[i].curve, trackMap[i].type);
    }
    Serial.println("--- EINDE CSV DATA ---");
}