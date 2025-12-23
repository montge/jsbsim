/*******************************************************************************
 * TestUtilities.h - Shared test utilities for JSBSim unit tests
 *
 * This header provides common utilities, macros, and helper functions
 * for CxxTest-based unit tests.
 *
 * Copyright (c) JSBSim Development Team
 * Licensed under LGPL
 ******************************************************************************/

#ifndef JSBSIM_TEST_UTILITIES_H
#define JSBSIM_TEST_UTILITIES_H

#include <cxxtest/TestSuite.h>
#include <cmath>
#include <limits>
#include <string>
#include <sstream>
#include <memory>

#include <input_output/FGXMLParse.h>
#include <input_output/FGXMLElement.h>

namespace JSBSimTest {

/*******************************************************************************
 * Floating-point comparison utilities
 ******************************************************************************/

// Default tolerance for floating-point comparisons
constexpr double DEFAULT_TOLERANCE = 1e-10;
constexpr double LOOSE_TOLERANCE = 1e-6;
constexpr double ANGLE_TOLERANCE = 1e-8;  // For angular values in radians

/**
 * Check if two floating-point values are approximately equal
 */
inline bool ApproxEqual(double a, double b, double tol = DEFAULT_TOLERANCE) {
    if (std::abs(a) < tol && std::abs(b) < tol) return true;
    return std::abs(a - b) <= tol * std::max(std::abs(a), std::abs(b));
}

/**
 * Check if a value is approximately zero
 */
inline bool ApproxZero(double a, double tol = DEFAULT_TOLERANCE) {
    return std::abs(a) < tol;
}

/*******************************************************************************
 * Test assertion macros with better error messages
 ******************************************************************************/

#define TS_ASSERT_APPROX(actual, expected) \
    TS_ASSERT(JSBSimTest::ApproxEqual(actual, expected))

#define TS_ASSERT_APPROX_TOL(actual, expected, tol) \
    TS_ASSERT(JSBSimTest::ApproxEqual(actual, expected, tol))

/*******************************************************************************
 * Physical constants for testing
 ******************************************************************************/

namespace Constants {
    // Standard gravity (m/s^2 and ft/s^2)
    constexpr double G_MPS2 = 9.80665;
    constexpr double G_FTPS2 = 32.174049;

    // Standard atmosphere at sea level
    constexpr double SEA_LEVEL_PRESSURE_PSF = 2116.22;
    constexpr double SEA_LEVEL_TEMP_R = 518.67;
    constexpr double SEA_LEVEL_DENSITY_SLUGFT3 = 0.002377;

    // Conversion factors
    constexpr double FT_TO_M = 0.3048;
    constexpr double KTS_TO_FTPS = 1.68781;
    constexpr double DEG_TO_RAD = 0.017453292519943295;
    constexpr double RAD_TO_DEG = 57.29577951308232;
}

/*******************************************************************************
 * XML parsing helper for tests
 ******************************************************************************/

/**
 * Parse XML from a string and return the root element
 * Used by tests that need to parse XML configuration snippets
 */
inline JSBSim::Element_ptr readFromXML(const std::string& xml) {
    std::stringstream ss;
    ss << xml;
    JSBSim::FGXMLParse xml_parse;
    readXML(ss, xml_parse);
    return xml_parse.GetDocument();
}

} // namespace JSBSimTest

#endif // JSBSIM_TEST_UTILITIES_H
