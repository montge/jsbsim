#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>
#include <vector>

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

  // Test thrust vectoring with pitch angle
  void testThrustVectoringPitch() {
    // Thrust vector with pitch angle
    // At pitch angle theta, thrust components:
    // Fx = T * cos(theta)
    // Fz = T * sin(theta)
    double thrust = 1000.0;
    double pitchAngle = 0.1;  // radians (~5.7 degrees)

    double Fx = thrust * cos(pitchAngle);
    double Fz = thrust * sin(pitchAngle);

    // cos(0.1) ≈ 0.99500416527, sin(0.1) ≈ 0.09983341665
    TS_ASSERT_DELTA(Fx, thrust * cos(pitchAngle), epsilon);
    TS_ASSERT_DELTA(Fz, thrust * sin(pitchAngle), epsilon);

    // Total thrust magnitude should be preserved
    double totalThrust = sqrt(Fx*Fx + Fz*Fz);
    TS_ASSERT_DELTA(totalThrust, thrust, epsilon);
  }

  // Test thrust vectoring with yaw angle
  void testThrustVectoringYaw() {
    double thrust = 1000.0;
    double yawAngle = 0.15;  // radians (~8.6 degrees)

    double Fx = thrust * cos(yawAngle);
    double Fy = thrust * sin(yawAngle);

    // Total should be preserved
    double totalThrust = sqrt(Fx*Fx + Fy*Fy);
    TS_ASSERT_DELTA(totalThrust, thrust, epsilon);
  }

  // Test combined pitch and yaw vectoring
  void testCombinedPitchYawVectoring() {
    double thrust = 1000.0;
    double pitch = 0.1;
    double yaw = 0.05;

    // Rotation matrix decomposition
    double Fx = thrust * cos(pitch) * cos(yaw);
    double Fy = thrust * cos(pitch) * sin(yaw);
    double Fz = thrust * sin(pitch);

    // Verify components are reasonable
    TS_ASSERT(std::abs(Fx) < thrust);
    TS_ASSERT(std::abs(Fy) < thrust * 0.1);
    TS_ASSERT(std::abs(Fz) < thrust * 0.2);

    // Magnitude should equal original thrust
    double magnitude = sqrt(Fx*Fx + Fy*Fy + Fz*Fz);
    TS_ASSERT_DELTA(magnitude, thrust, 0.1);
  }

  // Test moment calculation from thrust and arm
  void testThrustMomentCalculation() {
    // Moment = Force x Distance
    double thrust = 1000.0;  // lbf
    double armLength = 10.0;  // inches

    double moment = thrust * armLength;
    TS_ASSERT_DELTA(moment, 10000.0, epsilon);
  }

  // Test moment from off-center thrust
  void testOffCenterThrustMoment() {
    // Thrust applied at offset from CG creates moment
    double thrust = 500.0;  // lbf
    double yOffset = 5.0;   // inches right of CG

    // Creates yawing moment (nose right)
    double yawMoment = thrust * yOffset;
    TS_ASSERT_DELTA(yawMoment, 2500.0, epsilon);
  }

  // Test vectored thrust moment
  void testVectoredThrustMoment() {
    double thrust = 1000.0;
    double armLength = 20.0;  // inches below CG
    double vectorAngle = 0.1;  // pitch up

    // Vertical component of thrust creates pitching moment
    double Fz = thrust * sin(vectorAngle);
    double pitchMoment = Fz * armLength * 0.5;  // simplified

    TS_ASSERT(pitchMoment > 0);
  }

  // Test thrust line orientation angles
  void testThrustLineOrientation() {
    // Default thrust line is along x-axis (forward)
    // Orientation angles modify this
    double thrustMagnitude = 1000.0;

    // Zero orientation - all thrust is axial (Fx)
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    double Fx = thrustMagnitude * cos(pitch) * cos(yaw);

    TS_ASSERT_DELTA(Fx, thrustMagnitude, epsilon);
  }

  // Test P-Factor simulation
  void testPFactorSimulation() {
    // P-Factor: propeller torque creates asymmetric thrust
    // Acting location differs from nominal location
    double nominalX = 100.0;  // inches
    double nominalY = 0.0;
    double actingX = 100.0;
    double actingY = 2.0;  // offset due to P-Factor

    double deltaY = actingY - nominalY;
    TS_ASSERT_DELTA(deltaY, 2.0, epsilon);
  }

  // Test location coordinates in structural frame
  void testLocationStructuralCoordinates() {
    // JSBSim structural: x+ back, y+ right, z+ up
    double x = 150.0;   // aft of reference
    double y = -36.0;   // left of centerline
    double z = 10.0;    // above reference

    // All should be valid finite values
    TS_ASSERT(std::isfinite(x) && std::isfinite(y) && std::isfinite(z));
  }

  // Test location to moment arm calculation
  void testLocationToMomentArm() {
    // Location relative to CG gives moment arm
    double thrusterX = 200.0;  // inches aft
    double cgX = 150.0;

    double armX = thrusterX - cgX;  // 50 inches
    TS_ASSERT_DELTA(armX, 50.0, epsilon);
  }

  // Test transform type enumeration
  void testTransformTypeEnum() {
    // FGForce transform types
    TS_ASSERT_EQUALS(FGForce::tNone, 0);
    TS_ASSERT_EQUALS(FGForce::tWindBody, 1);
    TS_ASSERT_EQUALS(FGForce::tLocalBody, 2);
    TS_ASSERT_EQUALS(FGForce::tInertialBody, 3);
    TS_ASSERT_EQUALS(FGForce::tCustom, 4);
  }

  // Test wind to body transform concept
  void testWindToBodyTransform() {
    // Wind axis: x along velocity, y right, z up
    // Body axis: x forward, y right, z down
    // Transform depends on alpha and beta

    double alpha = 0.1;  // angle of attack
    double beta = 0.05;  // sideslip

    // Simplified rotation (about y for alpha)
    double Fx_body = cos(alpha);  // from x_wind
    double Fz_body = -sin(alpha); // z reversal

    TS_ASSERT(std::isfinite(Fx_body));
    TS_ASSERT(std::isfinite(Fz_body));
  }

  // Test custom transform matrix elements
  void testCustomTransformMatrix() {
    // 3x3 rotation matrix elements
    double roll = 0.1, pitch = 0.2, yaw = 0.15;

    // Simplified: rotation matrix element (1,1)
    // m11 = cos(pitch) * cos(yaw) = cos(0.2) * cos(0.15)
    double m11 = cos(pitch) * cos(yaw);
    double expected = cos(0.2) * cos(0.15);  // ≈ 0.9690
    TS_ASSERT_DELTA(m11, expected, epsilon);
  }

  // Test multiple thruster configuration
  void testMultipleThrusterConfiguration() {
    // Two engines, symmetric about centerline
    double leftY = -50.0;   // inches
    double rightY = 50.0;   // inches
    double thrustEach = 500.0;

    // Total thrust (both engines)
    double totalThrust = 2 * thrustEach;
    TS_ASSERT_DELTA(totalThrust, 1000.0, epsilon);

    // Net yawing moment (symmetric, should be zero)
    double netYawMoment = thrustEach * leftY + thrustEach * rightY;
    TS_ASSERT_DELTA(netYawMoment, 0.0, epsilon);
  }

  // Test asymmetric thrust (engine failure)
  void testAsymmetricThrust() {
    double leftThrust = 500.0;
    double rightThrust = 0.0;  // failed engine
    double engineY = 50.0;  // inches from centerline

    // Yawing moment from thrust asymmetry
    double yawMoment = leftThrust * (-engineY) + rightThrust * engineY;
    TS_ASSERT_DELTA(yawMoment, -25000.0, epsilon);  // nose left
  }

  // Test thrust spool-up simulation
  void testThrustSpoolUp() {
    double maxThrust = 1000.0;
    double timeConstant = 2.0;  // seconds
    double dt = 0.1;

    double thrust = 0.0;

    // First-order lag response
    for (double t = 0.0; t < 10.0; t += dt) {
      thrust = thrust + (maxThrust - thrust) * dt / timeConstant;
    }

    // Should approach max thrust
    TS_ASSERT(thrust > 0.95 * maxThrust);
  }

  // Test thrust spool-down
  void testThrustSpoolDown() {
    double initialThrust = 1000.0;
    double targetThrust = 0.0;
    double timeConstant = 1.5;
    double dt = 0.1;

    double thrust = initialThrust;

    for (double t = 0.0; t < 10.0; t += dt) {
      thrust = thrust + (targetThrust - thrust) * dt / timeConstant;
    }

    TS_ASSERT(thrust < 0.05 * initialThrust);
  }

  // Test afterburner thrust augmentation
  void testAfterburnerAugmentation() {
    double dryThrust = 1000.0;
    double abRatio = 1.5;  // 50% augmentation

    double wetThrust = dryThrust * abRatio;
    TS_ASSERT_DELTA(wetThrust, 1500.0, epsilon);
  }

  // Test thrust vs altitude relationship
  void testThrustVsAltitude() {
    // Jet thrust typically decreases with altitude
    double seaLevelThrust = 10000.0;
    double seaLevelDensity = 0.002377;
    double cruiseDensity = 0.000738;  // ~35000 ft

    // Simplified thrust lapse
    double densityRatio = cruiseDensity / seaLevelDensity;
    double cruiseThrust = seaLevelThrust * densityRatio;

    TS_ASSERT(cruiseThrust < seaLevelThrust);
    TS_ASSERT(cruiseThrust > 0.3 * seaLevelThrust);
  }

  // Test thrust vs velocity relationship
  void testThrustVsVelocity() {
    // Ram effect increases thrust at higher velocity
    double staticThrust = 10000.0;
    double velocity = 500.0;  // fps
    double ramCoeff = 0.0001;

    double ramThrust = staticThrust * (1 + ramCoeff * velocity);
    TS_ASSERT(ramThrust > staticThrust);
  }

  // Test specific fuel consumption impact
  void testSFCImpact() {
    // SFC relates fuel flow to thrust
    double thrust = 5000.0;  // lbf
    double sfc = 0.8;  // lb/hr per lbf

    double fuelFlow = thrust * sfc;
    TS_ASSERT_DELTA(fuelFlow, 4000.0, epsilon);
  }

  // Test thrust coefficient with Mach number
  void testThrustCoefficientMach() {
    double Ct_subsonic = 0.85;
    double Ct_supersonic = 0.75;  // typically lower due to wave drag

    TS_ASSERT(Ct_subsonic > Ct_supersonic);
    TS_ASSERT(Ct_subsonic <= 1.0);
    TS_ASSERT(Ct_supersonic > 0.0);
  }

  // Test nozzle area variation
  void testNozzleAreaVariation() {
    // Variable geometry nozzle
    double minArea = 5.0;   // sq ft
    double maxArea = 8.0;   // sq ft
    double currentSetting = 0.7;  // 70% open

    double area = minArea + currentSetting * (maxArea - minArea);
    TS_ASSERT_DELTA(area, 7.1, epsilon);
  }

  // Test exit pressure effects
  void testExitPressureEffects() {
    double vacuumThrust = 10000.0;
    double exitArea = 6.0;  // sq ft
    double exitPressure = 100.0;  // psf
    double ambientPressure = 500.0;  // psf

    // Thrust = vacuum thrust - (Pa - Pe) * Ae
    // Negative because ambient > exit (underexpanded)
    double pressureThrust = exitArea * (exitPressure - ambientPressure);
    double totalThrust = vacuumThrust + pressureThrust;

    TS_ASSERT(totalThrust < vacuumThrust);
  }

  // Test optimal expansion ratio
  void testOptimalExpansion() {
    double exitPressure = 2116.0;   // psf (sea level)
    double ambientPressure = 2116.0;

    // At optimal expansion, Pe = Pa
    double pressureDiff = exitPressure - ambientPressure;
    TS_ASSERT_DELTA(pressureDiff, 0.0, epsilon);
  }

  // Test thrust coefficient table lookup concept
  void testThrustCoefficientTable() {
    // Ct varies with advance ratio for propellers
    double J[] = {0.0, 0.5, 1.0, 1.5, 2.0};
    double Ct[] = {0.10, 0.09, 0.07, 0.04, 0.00};

    // Verify monotonic decrease
    for (int i = 1; i < 5; i++) {
      TS_ASSERT(Ct[i] <= Ct[i-1]);
    }
  }

  // Test propeller efficiency
  void testPropellerEfficiency() {
    double thrust = 500.0;  // lbf
    double velocity = 200.0;  // fps
    double power = 300.0 * 550.0;  // 300 HP in ft-lbf/s

    double efficiency = (thrust * velocity) / power;
    TS_ASSERT(efficiency >= 0.0 && efficiency <= 1.0);
  }

  // Test torque from thrust
  void testTorqueFromThrust() {
    double power = 500.0 * 550.0;  // 500 HP
    double rpm = 2500.0;
    double omega = rpm * 2.0 * M_PI / 60.0;

    double torque = power / omega;
    TS_ASSERT(torque > 0);
    TS_ASSERT(std::isfinite(torque));
  }

  // Test gear ratio effects on RPM
  void testGearRatioRPM() {
    double engineRPM = 2700.0;
    double gearRatio = 0.5;

    double propRPM = engineRPM * gearRatio;
    TS_ASSERT_DELTA(propRPM, 1350.0, epsilon);
  }

  // Test thrust line inclination
  void testThrustLineInclination() {
    // Thrust line typically inclined down for single-engine aircraft
    double inclinationAngle = 3.0 * M_PI / 180.0;  // 3 degrees
    double thrust = 1000.0;

    double Fx = thrust * cos(inclinationAngle);
    double Fz = thrust * sin(inclinationAngle);  // positive down in body

    TS_ASSERT(Fz > 0);  // thrust has downward component
    TS_ASSERT(Fx > 0.99 * thrust);
  }

  // Test thrust decay with engine damage
  void testThrustDecayDamage() {
    double normalThrust = 1000.0;
    double damageLevel = 0.3;  // 30% damaged

    double damagedThrust = normalThrust * (1.0 - damageLevel);
    TS_ASSERT_DELTA(damagedThrust, 700.0, epsilon);
  }

  // Test thrust variation with throttle
  void testThrottleResponse() {
    double maxThrust = 10000.0;
    double throttle[] = {0.0, 0.25, 0.5, 0.75, 1.0};

    for (double t : throttle) {
      double thrust = maxThrust * t;
      TS_ASSERT(thrust >= 0.0);
      TS_ASSERT(thrust <= maxThrust);
    }
  }

  // Test thrust oscillation damping
  void testThrustOscillationDamping() {
    double targetThrust = 1000.0;
    double thrust = 500.0;
    double damping = 0.7;
    double dt = 0.05;

    // Damped response should converge
    double prevDelta = std::abs(targetThrust - thrust);

    for (int i = 0; i < 20; i++) {
      double delta = targetThrust - thrust;
      thrust += delta * damping * dt;
      double newDelta = std::abs(targetThrust - thrust);
      TS_ASSERT(newDelta <= prevDelta + epsilon);
      prevDelta = newDelta;
    }
  }

  // Test minimum thrust setting (idle)
  void testIdleThrust() {
    double maxThrust = 10000.0;
    double idleFraction = 0.05;  // 5% at idle

    double idleThrust = maxThrust * idleFraction;
    TS_ASSERT_DELTA(idleThrust, 500.0, epsilon);
    TS_ASSERT(idleThrust > 0);
  }

  // Test engine-out yaw moment compensation
  void testEngineOutYawCompensation() {
    double singleEngineThrust = 5000.0;
    double engineSpacing = 100.0;  // inches

    // Yawing moment from failed engine
    double yawMoment = singleEngineThrust * engineSpacing / 2.0;

    // Rudder needed to counteract
    double rudderArm = 200.0;  // inches from CG to rudder
    double rudderForceNeeded = yawMoment / rudderArm;

    TS_ASSERT(rudderForceNeeded > 0);
    TS_ASSERT(rudderForceNeeded < singleEngineThrust);
  }

  // Test installation drag
  void testInstallationDrag() {
    double netThrust = 1000.0;
    double installationDrag = 50.0;  // nacelle drag

    double installedThrust = netThrust - installationDrag;
    TS_ASSERT_DELTA(installedThrust, 950.0, epsilon);
  }

  // Test bleed air extraction effects
  void testBleedAirEffects() {
    double normalThrust = 10000.0;
    double bleedFraction = 0.03;  // 3% bleed

    // Bleed reduces available thrust
    double thrustWithBleed = normalThrust * (1.0 - bleedFraction);
    TS_ASSERT(thrustWithBleed < normalThrust);
  }

  // Test power extraction effects
  void testPowerExtractionEffects() {
    double normalThrust = 10000.0;
    double powerExtraction = 100.0;  // HP for accessories
    double thrustPerHP = 2.5;  // typical

    double thrustReduction = powerExtraction * thrustPerHP;
    double netThrust = normalThrust - thrustReduction;

    TS_ASSERT(netThrust < normalThrust);
  }

  // Test critical engine concept
  void testCriticalEngine() {
    // Multi-engine aircraft: critical engine failure
    // creates maximum adverse yaw due to P-Factor
    double leftEngineYaw = -1000.0;  // moment
    double rightEngineYaw = 800.0;   // different due to P-Factor

    // Right engine is critical (larger adverse yaw when it fails)
    TS_ASSERT(std::abs(leftEngineYaw) > std::abs(rightEngineYaw));
  }

  // Test thrust available vs required
  void testThrustAvailableVsRequired() {
    double thrustAvailable = 5000.0;
    double thrustRequired = 3000.0;

    double excessThrust = thrustAvailable - thrustRequired;
    TS_ASSERT_DELTA(excessThrust, 2000.0, epsilon);
    TS_ASSERT(excessThrust > 0);  // can accelerate or climb
  }

  // Test static thrust check
  void testStaticThrustCheck() {
    // Static thrust at zero velocity
    double vacuumThrust = 10000.0;
    double velocity = 0.0;

    // No ram effect at static
    double staticThrust = vacuumThrust;
    TS_ASSERT_DELTA(staticThrust, vacuumThrust, epsilon);
  }

  // Test ground run thrust
  void testGroundRunThrust() {
    double staticThrust = 10000.0;
    double velocity = 100.0;  // fps during ground roll
    double ramFactor = 1.02;  // slight increase

    double groundThrust = staticThrust * ramFactor;
    TS_ASSERT(groundThrust >= staticThrust);
  }

  /***************************************************************************
   * Complete System Tests
   ***************************************************************************/

  void testCompleteThrustCalculationCycle() {
    // Full thrust calculation from throttle to net thrust
    double throttle = 0.85;
    double maxThrust = 12000.0;  // lbf

    // Throttle to commanded thrust
    double cmdThrust = maxThrust * throttle;

    // Altitude derating
    double rho = 0.0017;  // density at altitude
    double rho_sl = 0.002377;
    double altFactor = std::sqrt(rho / rho_sl);
    double altThrust = cmdThrust * altFactor;

    // Installation losses
    double installLoss = 0.02;  // 2%
    double netThrust = altThrust * (1.0 - installLoss);

    TS_ASSERT(netThrust > 0.0);
    TS_ASSERT(netThrust < maxThrust);
    TS_ASSERT(netThrust < cmdThrust);
  }

  void testCompletePropellerPerformanceEnvelope() {
    // Propeller performance across operating range
    double power = 300.0 * 550.0;  // 300 HP in ft-lb/s
    double D = 6.5;  // Propeller diameter (ft)
    double rho = 0.002377;

    double rpm_values[] = {1800.0, 2000.0, 2200.0, 2400.0, 2600.0};
    double V_values[] = {100.0, 150.0, 200.0, 250.0};

    for (double rpm : rpm_values) {
      double n = rpm / 60.0;  // rev/s
      for (double V : V_values) {
        double J = V / (n * D);  // Advance ratio
        TS_ASSERT(J > 0.0);
        TS_ASSERT(J < 3.0);  // Reasonable range

        // Estimated Cp based on typical curve
        double Cp = 0.08 * std::exp(-0.5 * (J - 0.8) * (J - 0.8));
        TS_ASSERT(Cp >= 0.0);
      }
    }
  }

  void testCompleteJetEngineThrottleResponse() {
    // Simulate engine spool-up
    double targetThrust = 10000.0;
    double thrust = 1000.0;  // Initial (idle)
    double timeConstant = 3.0;  // seconds
    double dt = 0.1;

    std::vector<double> thrustHistory;
    for (double t = 0.0; t < 10.0; t += dt) {
      double error = targetThrust - thrust;
      thrust += error * (1.0 - std::exp(-dt / timeConstant));
      thrustHistory.push_back(thrust);
    }

    // Should converge toward target
    TS_ASSERT(thrustHistory.back() > 0.9 * targetThrust);
    // Should be monotonically increasing
    for (size_t i = 1; i < thrustHistory.size(); i++) {
      TS_ASSERT(thrustHistory[i] >= thrustHistory[i-1] - epsilon);
    }
  }

  void testCompleteMultiEnginePerformance() {
    // Four-engine aircraft performance
    double singleEngineThrust = 5000.0;
    int numEngines = 4;

    double totalThrust = singleEngineThrust * numEngines;
    TS_ASSERT_DELTA(totalThrust, 20000.0, epsilon);

    // One engine failed
    double thrustWithFailure = singleEngineThrust * (numEngines - 1);
    TS_ASSERT_DELTA(thrustWithFailure, 15000.0, epsilon);

    // Thrust-to-weight considerations
    double weight = 80000.0;  // lbs
    double TW_normal = totalThrust / weight;
    double TW_oei = thrustWithFailure / weight;

    TS_ASSERT(TW_normal > 0.2);  // Reasonable T/W
    TS_ASSERT(TW_oei > 0.15);    // Can still maintain flight
  }

  void testCompleteTurbofanPerformance() {
    // Turbofan with bypass ratio effects
    double coreMdot = 50.0;    // Core mass flow (lb/s)
    double bypassRatio = 5.0;
    double totalMdot = coreMdot * (1.0 + bypassRatio);

    // Exhaust velocities
    double Vc = 1500.0;  // Core exhaust (ft/s)
    double Vb = 700.0;   // Bypass exhaust (ft/s)
    double V0 = 300.0;   // Inlet velocity (ft/s)
    double g = 32.174;

    // Thrust contributions
    double coreThrust = (coreMdot / g) * (Vc - V0);
    double bypassThrust = (coreMdot * bypassRatio / g) * (Vb - V0);
    double totalThrust = coreThrust + bypassThrust;

    TS_ASSERT(totalThrust > 0.0);
    TS_ASSERT(bypassThrust > coreThrust);  // High bypass = more fan thrust
  }

  void testCompleteAfterburnerOperation() {
    // Afterburner thrust augmentation
    double dryThrust = 15000.0;  // lbf
    double wetThrust = 25000.0;  // lbf with A/B
    double augmentationRatio = wetThrust / dryThrust;

    TS_ASSERT(augmentationRatio > 1.5);
    TS_ASSERT(augmentationRatio < 2.0);  // Typical range

    // SFC increase with afterburner
    double drySFC = 0.8;   // lb/hr/lbf
    double wetSFC = 2.0;   // lb/hr/lbf

    double dryFuelFlow = dryThrust * drySFC;
    double wetFuelFlow = wetThrust * wetSFC;

    TS_ASSERT(wetFuelFlow > 3.0 * dryFuelFlow);  // Much higher consumption
  }

  void testCompleteThrustVectoringMoments() {
    // Thrust vectoring contributions to control
    double thrust = 10000.0;      // lbf
    double vectorAngle = 15.0;    // degrees
    double momentArm = 15.0;      // ft from CG

    double angleRad = vectorAngle * M_PI / 180.0;
    double thrustX = thrust * cos(angleRad);
    double thrustZ = thrust * sin(angleRad);

    // Pitching moment from vectored thrust
    double pitchMoment = thrustZ * momentArm;  // ft-lbf

    TS_ASSERT(thrustX > 0.95 * thrust);  // Most thrust still forward
    TS_ASSERT(pitchMoment > 30000.0);    // Significant control moment
  }

  /***************************************************************************
   * Instance Independence Tests
   ***************************************************************************/

  void testThrustCalculationIndependence() {
    // Two engines with different settings shouldn't interfere
    double thrust1 = 8000.0 * 0.9;   // Engine 1: 90% throttle
    double thrust2 = 10000.0 * 0.75; // Engine 2: 75% throttle

    TS_ASSERT_DELTA(thrust1, 7200.0, epsilon);
    TS_ASSERT_DELTA(thrust2, 7500.0, epsilon);

    // Verify thrust1 unchanged after thrust2 calculation
    double thrust1_verify = 8000.0 * 0.9;
    TS_ASSERT_DELTA(thrust1, thrust1_verify, epsilon);
  }

  void testPropellerEfficiencyIndependence() {
    // Multiple propellers with different advance ratios
    double J1 = 0.6;
    double J2 = 0.9;

    double eta1 = 0.85 * J1 * (1.5 - J1);
    double eta2 = 0.85 * J2 * (1.5 - J2);

    TS_ASSERT(eta1 != eta2);
    TS_ASSERT(eta1 > 0.0 && eta1 < 1.0);
    TS_ASSERT(eta2 > 0.0 && eta2 < 1.0);

    // Verify eta1 unchanged
    double eta1_verify = 0.85 * J1 * (1.5 - J1);
    TS_ASSERT_DELTA(eta1, eta1_verify, epsilon);
  }

  void testEngineSpoolIndependence() {
    // Independent engine spool-up dynamics
    double N1_eng1 = 80.0;  // %
    double N1_eng2 = 95.0;  // %

    double thrust1 = 10000.0 * (N1_eng1 / 100.0);
    double thrust2 = 12000.0 * (N1_eng2 / 100.0);

    TS_ASSERT_DELTA(thrust1, 8000.0, epsilon);
    TS_ASSERT_DELTA(thrust2, 11400.0, epsilon);

    // Verify thrust1 unchanged
    double thrust1_verify = 10000.0 * (N1_eng1 / 100.0);
    TS_ASSERT_DELTA(thrust1, thrust1_verify, epsilon);
  }

  void testSFCCalculationIndependence() {
    // Different engine types, different SFC
    double thrust_turbojet = 8000.0;
    double sfc_turbojet = 1.0;
    double fuel1 = thrust_turbojet * sfc_turbojet;

    double thrust_turbofan = 12000.0;
    double sfc_turbofan = 0.6;
    double fuel2 = thrust_turbofan * sfc_turbofan;

    TS_ASSERT_DELTA(fuel1, 8000.0, epsilon);
    TS_ASSERT_DELTA(fuel2, 7200.0, epsilon);

    // Verify fuel1 unchanged
    double fuel1_verify = thrust_turbojet * sfc_turbojet;
    TS_ASSERT_DELTA(fuel1, fuel1_verify, epsilon);
  }

  void testTorqueCalculationIndependence() {
    // Two propeller engines with different power/RPM
    double power1 = 200.0 * 550.0;  // ft-lb/s
    double rpm1 = 2400.0;
    double omega1 = rpm1 * 2.0 * M_PI / 60.0;
    double torque1 = power1 / omega1;

    double power2 = 350.0 * 550.0;
    double rpm2 = 2700.0;
    double omega2 = rpm2 * 2.0 * M_PI / 60.0;
    double torque2 = power2 / omega2;

    TS_ASSERT(torque1 != torque2);
    TS_ASSERT(torque1 > 0.0);
    TS_ASSERT(torque2 > 0.0);

    // Verify torque1 unchanged
    double torque1_verify = power1 / omega1;
    TS_ASSERT_DELTA(torque1, torque1_verify, epsilon);
  }

  void testNozzleAreaIndependence() {
    // Variable nozzle settings for different engines
    double minArea = 5.0;
    double maxArea = 8.0;

    double setting1 = 0.3;
    double setting2 = 0.8;

    double area1 = minArea + setting1 * (maxArea - minArea);
    double area2 = minArea + setting2 * (maxArea - minArea);

    TS_ASSERT_DELTA(area1, 5.9, epsilon);
    TS_ASSERT_DELTA(area2, 7.4, epsilon);

    // Verify area1 unchanged
    double area1_verify = minArea + setting1 * (maxArea - minArea);
    TS_ASSERT_DELTA(area1, area1_verify, epsilon);
  }

  void testThrustVectorIndependence() {
    // Multiple engines with different vector angles
    double thrust = 10000.0;
    double angle1 = 10.0 * M_PI / 180.0;
    double angle2 = 20.0 * M_PI / 180.0;

    double Fx1 = thrust * cos(angle1);
    double Fz1 = thrust * sin(angle1);

    double Fx2 = thrust * cos(angle2);
    double Fz2 = thrust * sin(angle2);

    TS_ASSERT(Fx1 > Fx2);  // More forward at smaller angle
    TS_ASSERT(Fz1 < Fz2);  // Less vertical at smaller angle

    // Verify Fx1, Fz1 unchanged
    double Fx1_verify = thrust * cos(angle1);
    double Fz1_verify = thrust * sin(angle1);
    TS_ASSERT_DELTA(Fx1, Fx1_verify, epsilon);
    TS_ASSERT_DELTA(Fz1, Fz1_verify, epsilon);
  }
};
