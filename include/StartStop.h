#ifndef STARTSTOP_H
#define STARTSTOP_H

#include "types.h"

// Functie definities voor de start/finish logica
void resetStartSequence();
bool handleStartLogic();
FinishState checkFinishStatus();

#endif