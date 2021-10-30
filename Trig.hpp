#pragma once

#include <cstdint>
#include <vector>

using namespace std;

#ifndef TRIG_H
#define TRIG_H

extern float gSineTable[4096];
extern float gCosineTable[4096];
extern int gArctanTable[1025];

int16_t atan2_lookup(float z, float x);
int16_t atan2s(float z, float x);

#endif
