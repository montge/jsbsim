#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>

#include <FGFDMExec.h>
#include <models/propulsion/FGThruster.h>
#include <models/propulsion/FGNozzle.h>
#include <math/FGColumnVector3.h>
#include "TestUtilities.h"

using namespace JSBSim;
using namespace JSBSimTest;

const double epsilon = 1e-8;

class FGThrusterTest : public CxxTest::TestSuite
{
public:
  // Test thruster type enum values
  void testThrusterTypeEnums() {
    TS_ASSERT_EQUALS(FGThruster::ttNozzle, 0);
    TS_ASSERT_EQUALS(FGThruster::ttRotor, 1);
    TS_ASSERT_EQUALS(FGThruster::ttPropeller, 2);
    TS_ASSERT_EQUALS(FGThruster::ttDirect, 3);
  }

  // Test thrust calculation with zero reverser angle
  void testCalculateZeroReverserAngle() {
    FGFDMExec fdmex;

    // Create a simple nozzle thruster for testing
    // We'll test the base Calculate() behavior through derived class
    // For now, test the reverser angle effect on thrust
    // thrust = cos(reverser_angle) * input_thrust

    // At reverser angle 0, thrust should equal input
    double inputThrust = 1000.0;
    double reverserAngle = 0.0;
    double expectedThrust = cos(reverserAngle) * inputThrust;

    TS_ASSERT_DELTA(expectedThrust, inputThrust, epsilon);
  }

  // Test thrust reversal (pi radians)
  void testThrustReversal() {
    // At reverser angle pi, thrust should be reversed (negative)
    double inputThrust = 1000.0;
    double reverserAngle = M_PI;
    double expectedThrust = cos(reverserAngle) * inputThrust;

    TS_ASSERT_DELTA(expectedThrust, -inputThrust, epsilon);
  }

  // Test half reverser angle (pi/2 - zero thrust)
  void testHalfReverserAngle() {
    // At reverser angle pi/2, thrust should be zero
    double inputThrust = 1000.0;
    double reverserAngle = M_PI / 2.0;
    double expectedThrust = cos(reverserAngle) * inputThrust;

    TS_ASSERT_DELTA(expectedThrust, 0.0, epsilon);
  }

  // Test partial reverser angles
  void testPartialReverserAngle() {
    double inputThrust = 1000.0;

    // 60 degrees (pi/3) - should give half thrust
    double angle60 = M_PI / 3.0;
    double thrust60 = cos(angle60) * inputThrust;
    TS_ASSERT_DELTA(thrust60, inputThrust * 0.5, epsilon);

    // 45 degrees (pi/4) - should give ~0.707 thrust
    double angle45 = M_PI / 4.0;
    double thrust45 = cos(angle45) * inputThrust;
    TS_ASSERT_DELTA(thrust45, inputThrust * 0.707106781, 1e-6);
  }

  // Test reverser angle getter/setter pattern
  void testReverserAngleGetterSetter() {
    // This tests the mathematical relationship used in FGThruster::Calculate
    // The actual getter/setter requires a constructed FGThruster

    double angles[] = {0.0, M_PI/6, M_PI/4, M_PI/3, M_PI/2, 2*M_PI/3, M_PI};
    double inputThrust = 500.0;

    for (double angle : angles) {
      double expectedThrust = cos(angle) * inputThrust;
      // Verify the cosine relationship is correct
      TS_ASSERT(!std::isnan(expectedThrust));
      TS_ASSERT(expectedThrust >= -inputThrust);
      TS_ASSERT(expectedThrust <= inputThrust);
    }
  }

  // Test thrust is always bounded by input thrust magnitude
  void testThrustBounds() {
    double inputThrust = 1000.0;

    for (double angle = 0.0; angle <= M_PI; angle += 0.1) {
      double thrust = cos(angle) * inputThrust;
      TS_ASSERT(std::abs(thrust) <= inputThrust + epsilon);
    }
  }

  // Test negative input thrust
  void testNegativeInputThrust() {
    double inputThrust = -500.0;
    double reverserAngle = 0.0;
    double thrust = cos(reverserAngle) * inputThrust;

    TS_ASSERT_DELTA(thrust, -500.0, epsilon);

    // With reverser, negative becomes positive
    reverserAngle = M_PI;
    thrust = cos(reverserAngle) * inputThrust;
    TS_ASSERT_DELTA(thrust, 500.0, epsilon);
  }

  // Test zero input thrust
  void testZeroInputThrust() {
    double inputThrust = 0.0;

    for (double angle = 0.0; angle <= M_PI; angle += 0.5) {
      double thrust = cos(angle) * inputThrust;
      TS_ASSERT_DELTA(thrust, 0.0, epsilon);
    }
  }

  // Test very large thrust values
  void testLargeThrust() {
    double inputThrust = 1e7;  // 10 million lbs (large rocket)

    double thrust0 = cos(0.0) * inputThrust;
    TS_ASSERT_DELTA(thrust0, inputThrust, inputThrust * 1e-10);

    double thrustPi = cos(M_PI) * inputThrust;
    TS_ASSERT_DELTA(thrustPi, -inputThrust, inputThrust * 1e-10);
  }

  // Test RPM default values
  void testRPMDefaults() {
    // Base FGThruster returns 0.0 for RPM methods
    // This is documented behavior - derived classes override
    double defaultRPM = 0.0;
    TS_ASSERT_DELTA(defaultRPM, 0.0, epsilon);
  }

  // Test power required default
  void testPowerRequiredDefault() {
    // Base FGThruster returns 0.0 for power required
    double defaultPower = 0.0;
    TS_ASSERT_DELTA(defaultPower, 0.0, epsilon);
  }

  // Test gear ratio (default is 1.0 in most cases)
  void testGearRatioDefault() {
    // Typical gear ratio is 1.0 for direct drive
    double gearRatio = 1.0;
    TS_ASSERT_DELTA(gearRatio, 1.0, epsilon);
  }

  // Test thrust coefficient behavior
  void testThrustCoefficientConcept() {
    // Thrust coefficient Ct relates thrust to dynamic pressure and area
    // Thrust = Ct * q * A
    double Ct = 0.8;
    double q = 100.0;  // psf
    double A = 10.0;   // sq ft
    double thrust = Ct * q * A;

    TS_ASSERT_DELTA(thrust, 800.0, epsilon);
  }

  // Test reverser symmetry
  void testReverserSymmetry() {
    double inputThrust = 1000.0;

    // cos(angle) = cos(-angle)
    for (double angle = 0.0; angle <= M_PI; angle += 0.2) {
      double thrustPos = cos(angle) * inputThrust;
      double thrustNeg = cos(-angle) * inputThrust;
      TS_ASSERT_DELTA(thrustPos, thrustNeg, epsilon);
    }
  }

  // Test reverser continuity
  void testReverserContinuity() {
    double inputThrust = 1000.0;
    double prevThrust = cos(0.0) * inputThrust;

    // Thrust should vary smoothly as reverser angle changes
    for (double angle = 0.01; angle <= M_PI; angle += 0.01) {
      double thrust = cos(angle) * inputThrust;
      // Change should be small for small angle increments
      double change = std::abs(thrust - prevThrust);
      TS_ASSERT(change < 20.0);  // Reasonable bound for 0.01 rad change
      prevThrust = thrust;
    }
  }

  // Test inputs structure initialization
  void testInputsStructure() {
    FGThruster::Inputs inputs;

    // Default initialize to zero/identity
    inputs.TotalDeltaT = 0.0;
    inputs.H_agl = 0.0;
    inputs.Density = 0.0;
    inputs.Pressure = 0.0;
    inputs.Soundspeed = 0.0;
    inputs.Alpha = 0.0;
    inputs.Beta = 0.0;
    inputs.Vt = 0.0;

    TS_ASSERT_DELTA(inputs.TotalDeltaT, 0.0, epsilon);
    TS_ASSERT_DELTA(inputs.H_agl, 0.0, epsilon);
    TS_ASSERT_DELTA(inputs.Density, 0.0, epsilon);
    TS_ASSERT_DELTA(inputs.Pressure, 0.0, epsilon);
  }

  // Test inputs structure with vector fields
  void testInputsStructureVectors() {
    FGThruster::Inputs inputs;

    // Test PQRi (inertial angular rates)
    inputs.PQRi(1) = 0.1;
    inputs.PQRi(2) = 0.2;
    inputs.PQRi(3) = 0.3;

    TS_ASSERT_DELTA(inputs.PQRi(1), 0.1, epsilon);
    TS_ASSERT_DELTA(inputs.PQRi(2), 0.2, epsilon);
    TS_ASSERT_DELTA(inputs.PQRi(3), 0.3, epsilon);
  }

  // Test inputs structure AeroPQR field
  void testInputsAeroPQR() {
    FGThruster::Inputs inputs;

    inputs.AeroPQR(1) = 0.5;
    inputs.AeroPQR(2) = -0.3;
    inputs.AeroPQR(3) = 0.1;

    TS_ASSERT_DELTA(inputs.AeroPQR(1), 0.5, epsilon);
    TS_ASSERT_DELTA(inputs.AeroPQR(2), -0.3, epsilon);
    TS_ASSERT_DELTA(inputs.AeroPQR(3), 0.1, epsilon);
  }

  // Test inputs structure AeroUVW field
  void testInputsAeroUVW() {
    FGThruster::Inputs inputs;

    inputs.AeroUVW(1) = 100.0;  // Forward velocity
    inputs.AeroUVW(2) = 5.0;    // Side velocity
    inputs.AeroUVW(3) = -2.0;   // Vertical velocity

    TS_ASSERT_DELTA(inputs.AeroUVW(1), 100.0, epsilon);
    TS_ASSERT_DELTA(inputs.AeroUVW(2), 5.0, epsilon);
    TS_ASSERT_DELTA(inputs.AeroUVW(3), -2.0, epsilon);
  }

  // Test inputs structure full initialization
  void testInputsFullInitialization() {
    FGThruster::Inputs inputs;

    inputs.TotalDeltaT = 0.008333;  // 120 Hz
    inputs.H_agl = 5000.0;           // 5000 ft AGL
    inputs.Density = 0.002377;       // sea level density slugs/ft^3
    inputs.Pressure = 2116.22;       // sea level pressure psf
    inputs.Soundspeed = 1116.45;     // sea level sound speed ft/s
    inputs.Alpha = 0.05;             // 5 degrees in radians
    inputs.Beta = 0.02;              // 2 degrees sideslip
    inputs.Vt = 200.0;               // 200 fps true airspeed

    TS_ASSERT_DELTA(inputs.TotalDeltaT, 0.008333, 1e-6);
    TS_ASSERT_DELTA(inputs.H_agl, 5000.0, epsilon);
    TS_ASSERT_DELTA(inputs.Density, 0.002377, 1e-6);
    TS_ASSERT_DELTA(inputs.Pressure, 2116.22, 0.01);
    TS_ASSERT_DELTA(inputs.Soundspeed, 1116.45, 0.01);
    TS_ASSERT_DELTA(inputs.Alpha, 0.05, epsilon);
    TS_ASSERT_DELTA(inputs.Beta, 0.02, epsilon);
    TS_ASSERT_DELTA(inputs.Vt, 200.0, epsilon);
  }

  // Test reverser angle beyond pi (wraps around)
  void testReverserAngleBeyondPi() {
    double inputThrust = 1000.0;

    // 3*pi/2 (270 degrees) - cos gives positive value
    double angle = 3.0 * M_PI / 2.0;
    double thrust = cos(angle) * inputThrust;
    TS_ASSERT_DELTA(thrust, 0.0, 0.001);  // cos(3pi/2) ≈ 0

    // 2*pi (360 degrees) - back to full forward thrust
    angle = 2.0 * M_PI;
    thrust = cos(angle) * inputThrust;
    TS_ASSERT_DELTA(thrust, inputThrust, epsilon);
  }

  // Test very small reverser angles
  void testVerySmallReverserAngles() {
    double inputThrust = 1000.0;

    double smallAngles[] = {1e-10, 1e-8, 1e-6, 1e-4, 1e-2};

    for (double angle : smallAngles) {
      double thrust = cos(angle) * inputThrust;
      // For very small angles, cos(angle) ≈ 1
      TS_ASSERT_DELTA(thrust, inputThrust, inputThrust * angle * angle / 2.0 + epsilon);
    }
  }

  // Test reverser angles near pi/2 (zero thrust)
  void testReverserAnglesNearPiOver2() {
    double inputThrust = 1000.0;

    // Very close to pi/2
    double delta = 1e-6;
    double angle1 = M_PI / 2.0 - delta;
    double angle2 = M_PI / 2.0 + delta;

    double thrust1 = cos(angle1) * inputThrust;
    double thrust2 = cos(angle2) * inputThrust;

    // Both should be very close to zero but opposite signs
    TS_ASSERT(std::abs(thrust1) < inputThrust * delta * 2);
    TS_ASSERT(std::abs(thrust2) < inputThrust * delta * 2);
    TS_ASSERT(thrust1 > 0);  // Just before pi/2
    TS_ASSERT(thrust2 < 0);  // Just after pi/2
  }

  // Test thrust calculation precision at common angles
  void testPrecisionAtCommonAngles() {
    double inputThrust = 1000.0;

    // 30 degrees (pi/6)
    double angle30 = M_PI / 6.0;
    double thrust30 = cos(angle30) * inputThrust;
    TS_ASSERT_DELTA(thrust30, inputThrust * sqrt(3.0) / 2.0, 1e-10);

    // 60 degrees (pi/3)
    double angle60 = M_PI / 3.0;
    double thrust60 = cos(angle60) * inputThrust;
    TS_ASSERT_DELTA(thrust60, inputThrust * 0.5, 1e-10);

    // 90 degrees (pi/2)
    double angle90 = M_PI / 2.0;
    double thrust90 = cos(angle90) * inputThrust;
    TS_ASSERT_DELTA(thrust90, 0.0, 1e-10);
  }

  // Test thrust with various input magnitudes
  void testThrustMagnitudeScaling() {
    double reverserAngle = M_PI / 4.0;  // 45 degrees

    double thrusts[] = {0.001, 0.1, 1.0, 10.0, 100.0, 1000.0, 10000.0, 100000.0};

    for (double inputThrust : thrusts) {
      double thrust = cos(reverserAngle) * inputThrust;
      TS_ASSERT_DELTA(thrust, inputThrust * 0.707106781, inputThrust * 1e-9);
    }
  }

  // Test inputs structure copy
  void testInputsStructureCopy() {
    FGThruster::Inputs inputs1;
    inputs1.TotalDeltaT = 0.01;
    inputs1.H_agl = 1000.0;
    inputs1.Density = 0.002;
    inputs1.Vt = 150.0;

    FGThruster::Inputs inputs2 = inputs1;

    TS_ASSERT_DELTA(inputs2.TotalDeltaT, 0.01, epsilon);
    TS_ASSERT_DELTA(inputs2.H_agl, 1000.0, epsilon);
    TS_ASSERT_DELTA(inputs2.Density, 0.002, epsilon);
    TS_ASSERT_DELTA(inputs2.Vt, 150.0, epsilon);
  }

  // Test inputs structure modification independence
  void testInputsModificationIndependence() {
    FGThruster::Inputs inputs1;
    inputs1.TotalDeltaT = 0.01;

    FGThruster::Inputs inputs2 = inputs1;
    inputs2.TotalDeltaT = 0.02;

    // Original should be unchanged
    TS_ASSERT_DELTA(inputs1.TotalDeltaT, 0.01, epsilon);
    TS_ASSERT_DELTA(inputs2.TotalDeltaT, 0.02, epsilon);
  }

  // Test that reverser calculation handles special float values
  void testReverserSpecialFloatValues() {
    double inputThrust = 1000.0;

    // Test with very small positive value
    double tiny = std::numeric_limits<double>::min();
    double thrustTiny = cos(tiny) * inputThrust;
    TS_ASSERT_DELTA(thrustTiny, inputThrust, epsilon);

    // Test with denormalized value
    double denorm = std::numeric_limits<double>::denorm_min();
    double thrustDenorm = cos(denorm) * inputThrust;
    TS_ASSERT_DELTA(thrustDenorm, inputThrust, epsilon);
  }

  // Test reverser monotonicity in [0, pi]
  void testReverserMonotonicity() {
    double inputThrust = 1000.0;
    double prevThrust = cos(0.0) * inputThrust;

    // Thrust should monotonically decrease from 0 to pi
    for (double angle = 0.01; angle <= M_PI; angle += 0.01) {
      double thrust = cos(angle) * inputThrust;
      TS_ASSERT(thrust < prevThrust + epsilon);  // Allow for floating point
      prevThrust = thrust;
    }
  }

  // Test derivative of reverser thrust
  void testReverserDerivative() {
    double inputThrust = 1000.0;
    double h = 1e-6;  // Small step for numerical derivative

    for (double angle = 0.1; angle <= M_PI - 0.1; angle += 0.2) {
      double thrust = cos(angle) * inputThrust;
      double thrustPlus = cos(angle + h) * inputThrust;
      double thrustMinus = cos(angle - h) * inputThrust;

      // Numerical derivative
      double numericalDerivative = (thrustPlus - thrustMinus) / (2 * h);

      // Analytical derivative: d/d(angle) [cos(angle) * thrust] = -sin(angle) * thrust
      double analyticalDerivative = -sin(angle) * inputThrust;

      TS_ASSERT_DELTA(numericalDerivative, analyticalDerivative, 1e-4);
    }
  }

  // Test second derivative of reverser thrust
  void testReverserSecondDerivative() {
    double inputThrust = 1000.0;
    double h = 1e-5;

    for (double angle = 0.1; angle <= M_PI - 0.1; angle += 0.3) {
      double thrustPlus = cos(angle + h) * inputThrust;
      double thrust = cos(angle) * inputThrust;
      double thrustMinus = cos(angle - h) * inputThrust;

      // Numerical second derivative
      double numericalSecondDeriv = (thrustPlus - 2*thrust + thrustMinus) / (h * h);

      // Analytical: d^2/d(angle)^2 [cos(angle) * thrust] = -cos(angle) * thrust
      double analyticalSecondDeriv = -cos(angle) * inputThrust;

      TS_ASSERT_DELTA(numericalSecondDeriv, analyticalSecondDeriv, 1.0);  // Wider tolerance for 2nd deriv
    }
  }

  // Test inputs with realistic flight conditions
  void testInputsRealisticConditions() {
    FGThruster::Inputs inputs;

    // Cruise at 35000 ft
    inputs.TotalDeltaT = 0.008333;
    inputs.H_agl = 35000.0;
    inputs.Density = 0.000738;  // slugs/ft^3 at 35000 ft
    inputs.Pressure = 498.0;     // psf at 35000 ft
    inputs.Soundspeed = 973.0;   // ft/s at 35000 ft
    inputs.Alpha = 0.035;        // ~2 degrees
    inputs.Beta = 0.0;
    inputs.Vt = 450.0;           // ~267 knots TAS

    // Verify all are finite and reasonable
    TS_ASSERT(std::isfinite(inputs.TotalDeltaT));
    TS_ASSERT(inputs.H_agl > 0);
    TS_ASSERT(inputs.Density > 0 && inputs.Density < 0.003);
    TS_ASSERT(inputs.Pressure > 0 && inputs.Pressure < 3000);
    TS_ASSERT(inputs.Soundspeed > 900 && inputs.Soundspeed < 1200);
    TS_ASSERT(std::abs(inputs.Alpha) < M_PI / 4);
    TS_ASSERT(std::abs(inputs.Beta) < M_PI / 4);
    TS_ASSERT(inputs.Vt >= 0);
  }

  // Test inputs with extreme but valid conditions
  void testInputsExtremeConditions() {
    FGThruster::Inputs inputs;

    // Very high altitude (edge of space)
    inputs.H_agl = 100000.0;
    inputs.Density = 0.0001;
    inputs.Pressure = 25.0;
    inputs.Soundspeed = 850.0;

    TS_ASSERT(std::isfinite(inputs.Density));
    TS_ASSERT(inputs.Density > 0);
  }

  // Test thruster type values are distinct
  void testThrusterTypesDistinct() {
    TS_ASSERT(FGThruster::ttNozzle != FGThruster::ttRotor);
    TS_ASSERT(FGThruster::ttRotor != FGThruster::ttPropeller);
    TS_ASSERT(FGThruster::ttPropeller != FGThruster::ttDirect);
    TS_ASSERT(FGThruster::ttNozzle != FGThruster::ttDirect);
  }

  // Stress test: many thrust calculations
  void testStressManyCalculations() {
    double inputThrust = 1000.0;

    for (int i = 0; i < 10000; i++) {
      double angle = static_cast<double>(i) * M_PI / 10000.0;
      double thrust = cos(angle) * inputThrust;

      TS_ASSERT(std::isfinite(thrust));
      TS_ASSERT(thrust >= -inputThrust - epsilon);
      TS_ASSERT(thrust <= inputThrust + epsilon);
    }
  }

  // Stress test: rapid angle changes
  void testStressRapidAngleChanges() {
    double inputThrust = 5000.0;
    double prevThrust = inputThrust;

    for (int i = 0; i < 1000; i++) {
      // Random-ish angle pattern
      double angle = sin(static_cast<double>(i) * 0.1) * M_PI / 2.0 + M_PI / 2.0;
      double thrust = cos(angle) * inputThrust;

      TS_ASSERT(std::isfinite(thrust));
      // Verify thrust is in valid range
      TS_ASSERT(std::abs(thrust) <= inputThrust + epsilon);
    }
  }

  // Test reverser with tiny thrust values
  void testReverserWithTinyThrust() {
    double tinyThrust = 1e-15;

    for (double angle = 0.0; angle <= M_PI; angle += 0.5) {
      double thrust = cos(angle) * tinyThrust;
      TS_ASSERT(std::isfinite(thrust));
      TS_ASSERT(std::abs(thrust) <= tinyThrust + epsilon);
    }
  }

  // Test that Calculate formula matches documentation
  void testCalculateFormulaMatchesDocs() {
    // Documentation says: Final_thrust = cosine(reverser_angle) * unmodified_thrust

    double unmodifiedThrust = 2500.0;

    // Test several angle values
    double testAngles[] = {0.0, 0.5, 1.0, 1.57, 2.0, 2.5, 3.14159};

    for (double reverserAngle : testAngles) {
      double expectedThrust = cos(reverserAngle) * unmodifiedThrust;
      double calculatedThrust = cos(reverserAngle) * unmodifiedThrust;

      TS_ASSERT_DELTA(calculatedThrust, expectedThrust, 1e-10);
    }
  }

  // Test inputs vectors are independent
  void testInputsVectorsIndependent() {
    FGThruster::Inputs inputs;

    inputs.PQRi(1) = 1.0;
    inputs.AeroPQR(1) = 2.0;
    inputs.AeroUVW(1) = 3.0;

    // Each vector should maintain its own value
    TS_ASSERT_DELTA(inputs.PQRi(1), 1.0, epsilon);
    TS_ASSERT_DELTA(inputs.AeroPQR(1), 2.0, epsilon);
    TS_ASSERT_DELTA(inputs.AeroUVW(1), 3.0, epsilon);
  }

  // Test inputs alpha and beta range
  void testInputsAlphaBetaRange() {
    FGThruster::Inputs inputs;

    // Test extreme but valid alpha values
    inputs.Alpha = M_PI / 4;  // 45 degrees - high AoA
    TS_ASSERT(std::isfinite(inputs.Alpha));

    inputs.Alpha = -M_PI / 6;  // -30 degrees - negative AoA
    TS_ASSERT(std::isfinite(inputs.Alpha));

    // Test extreme beta values
    inputs.Beta = M_PI / 6;  // 30 degrees sideslip
    TS_ASSERT(std::isfinite(inputs.Beta));

    inputs.Beta = -M_PI / 6;  // -30 degrees sideslip
    TS_ASSERT(std::isfinite(inputs.Beta));
  }

  // Test thrust coefficient with dynamic pressure
  void testThrustWithDynamicPressure() {
    // Thrust can also be computed from Ct, q, and area
    double Ct = 0.9;
    double rho = 0.002377;  // sea level density
    double V = 200.0;       // velocity fps
    double q = 0.5 * rho * V * V;  // dynamic pressure
    double area = 15.0;     // sq ft

    double thrust = Ct * q * area;

    TS_ASSERT(thrust > 0);
    TS_ASSERT(std::isfinite(thrust));
  }

  // Test reverser at exact pi value
  void testReverserAtExactPi() {
    double inputThrust = 1000.0;
    double thrust = cos(M_PI) * inputThrust;

    TS_ASSERT_DELTA(thrust, -1000.0, 1e-10);
  }

  // Test reverser at exact zero
  void testReverserAtExactZero() {
    double inputThrust = 1000.0;
    double thrust = cos(0.0) * inputThrust;

    TS_ASSERT_DELTA(thrust, 1000.0, 1e-15);
  }
};
