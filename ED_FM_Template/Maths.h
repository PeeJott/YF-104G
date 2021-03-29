#pragma once
#include <math.h>
#include "Vec3.h"
#include <algorithm>
//=========================================================================//

#undef max
#undef min

//für die Berechnungen der Airframe.h
static inline double toDegrees(double radians)
{
	return radians * 180.0 / 3.14159;
}

static inline double toRad(double degrees)
{
	return degrees * 3.14159 / 180.0;
}

static inline double clamp(double value, double min, double max)
{
	return std::max(std::min(value, max), min);
}