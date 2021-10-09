// VectorNav Programming Library v1.0.0.1
// Copyright (c) 2015 VectorNav Technologies, LLC
/// {COMMON_HEADER}
#include "vn/math/conversions.h"

#include "vn/math/consts.h"

namespace vn {
namespace math {

float rad2deg(float angleInRads)
{
	return angleInRads * 180.0f / PIf;
}

double rad2deg(double angleInRads)
{
	return angleInRads * 180.0 / PId;
}

float deg2rad(float angleInDegs)
{
	return angleInDegs * PIf / 180.0f;
}

double deg2rad(double angleInDegs)
{
	return angleInDegs * PId / 180.0f;
}

float celsius2fahren(float tempInCelsius)
{
	return (tempInCelsius * 9.0f) / 5.0f + 32.0f;
}

double celsius2fahren(double tempInCelsius)
{
	return (tempInCelsius * 9.0) / 5.0 + 32.0;
}

float fahren2celsius(float tempInFahren)
{
	return (tempInFahren - 32.0f) * 5.0f / 9.0f;
}

double fahren2celsius(double tempInFahren)
{
	return (tempInFahren - 32.0) * 5.0 / 9.0;
}

float celsius2kelvin(float tempInCelsius)
{
	return tempInCelsius + 273.15f;
}

double celsius2kelvin(double tempInCelsius)
{
	return tempInCelsius + 273.15;
}

float kelvin2celsius(float tempInKelvin)
{
	return tempInKelvin - 273.15f;
}

double kelvin2celsius(double tempInKelvin)
{
	return tempInKelvin - 273.15;
}

float fahren2kelvin(float tempInFahren)
{
	return fahren2celsius(tempInFahren) + 273.15f;
}

double fahren2kelvin(double tempInFahren)
{
	return fahren2celsius(tempInFahren) + 273.15;
}

float kelvin2fahren(float tempInKelvin)
{
	return celsius2fahren(tempInKelvin - 273.15f);
}

double kelvin2fahren(double tempInKelvin)
{
	return celsius2fahren(tempInKelvin - 273.15);
}
	
}
}
