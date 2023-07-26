#pragma once

float remapRange(
    float value, float in_min, float in_max, float out_min, float out_max);
// Returns shortest delta from current to target angle.
float angleDelta(float current, float target);
// Returns a % b, always in the range [0, b). b >= 0.
float fmodClamp(float a, float b);
// Returns angle between a and b sides of triangle abc.
float cosineLaw(float a, float b, float c);
