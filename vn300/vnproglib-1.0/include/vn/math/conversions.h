// VectorNav Programming Library v1.0.0.1
// Copyright (c) 2015 VectorNav Technologies, LLC
/// \file
/// {COMMON_HEADER}
///
/// \section DESCRIPTION
/// This header files some common math conversions.
#ifndef _VN_MATH_CONVERSIONS_H_
#define _VN_MATH_CONVERSIONS_H_

namespace vn {
namespace math {

/// \defgroup angle_convertors Angle Convertors
/// \brief Methods useful for angle conversions.
/// \{

/// \brief Converts an angle in radians to degrees.
///
/// \param[in] angleInRads The angle in radians.
/// \return The converted angle.
float rad2deg(float angleInRads);

/// \brief Converts an angle in radians to degrees.
///
/// \param[in] angleInRads The angle in radians.
/// \return The converted angle.
double rad2deg(double angleInRads);

/// \brief Converts an angle in degrees to radians.
///
/// \param[in] angleInDegs The angle in degrees.
/// \return The angle converted to radians.
float deg2rad(float angleInDegs);

/// \brief Converts an angle in degrees to radians.
///
/// \param[in] angleInDegs The angle in degrees.
/// \return The angle converted to radians.
//double deg2rad(double angleInDegs);

/// \}

/// \defgroup temperature_convertors Temperature Convertors
/// \brief Methods useful for temperature conversions.
/// \{

/// \brief Converts a temperature in Celsius to Fahrenheit.
///
/// \param[in] tempInCelsius The temperature in Celsius.
/// \return The converted temperature in Fahrenheit.
float celsius2fahren(float tempInCelsius);

/// \brief Converts a temperature in Celsius to Fahrenheit.
///
/// \param[in] tempInCelsius The temperature in Celsius.
/// \return The converted temperature in Fahrenheit.
double celsius2fahren(double tempInCelsius);

/// \brief Converts a temperature in Fahrenheit to Celsius.
///
/// \param[in] tempInFahren The temperature in Fahrenheit.
/// \return The converted temperature in Celsius.
float fahren2celsius(float tempInFahren);

/// \brief Converts a temperature in Fahrenheit to Celsius.
///
/// \param[in] tempInFahren The temperature in Fahrenheit.
/// \return The converted temperature in Celsius.
double fahren2celsius(double tempInFahren);

/// \brief Converts a temperature in Celsius to Kelvin.
///
/// \param[in] tempInCelsius The temperature in Celsius.
/// \return The converted temperature in Kelvin.
float celsius2kelvin(float tempInCelsius);

/// \brief Converts a temperature in Celsius to Kelvin.
///
/// \param[in] tempInCelsius The temperature in Celsius.
/// \return The converted temperature in Kelvin.
double celsius2kelvin(double tempInCelsius);

/// \brief Converts a temperature in Kelvin to Celsius.
///
/// \param[in] tempInKelvin The temperature in Kelvin.
/// \return The converted temperature in Celsius.
float kelvin2celsius(float tempInKelvin);

/// \brief Converts a temperature in Kelvin to Celsius.
///
/// \param[in] tempInKelvin The temperature in Kelvin.
/// \return The converted temperature in Celsius.
double kelvin2celsius(double tempInKelvin);

/// \brief Converts a temperature in Fahrenheit to Kelvin.
///
/// \param[in] tempInFahren The temperature in Fahrenheit.
/// \return The converted temperature in Kelvin.
float fahren2kelvin(float tempInFahren);

/// \brief Converts a temperature in Fahrenheit to Kelvin.
///
/// \param[in] tempInFahren The temperature in Fahrenheit.
/// \return The converted temperature in Kelvin.
double fahren2kelvin(double tempInFahren);

/// \brief Converts a temperature in Kelvin to Fahrenheit.
///
/// \param[in] tempInKelvin The temperature in Kelvin.
/// \return The converted temperature in Fahrenheit.
float kelvin2fahren(float tempInKelvin);

/// \brief Converts a temperature in Kelvin to Fahrenheit.
///
/// \param[in] tempInKelvin The temperature in Kelvin.
/// \return The converted temperature in Fahrenheit.
double kelvin2fahren(double tempInKelvin);

/// \}

}
}

#endif
