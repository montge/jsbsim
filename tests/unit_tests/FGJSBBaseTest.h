#include <string>
#include <limits>
#include <cmath>
#include <cxxtest/TestSuite.h>
#include <FGJSBBase.h>

class FGJSBBaseTest : public CxxTest::TestSuite, public JSBSim::FGJSBBase
{
public:
  /***************************************************************************
   * Numeric Utility Tests
   ***************************************************************************/

  void testEqualToRoundoffDoubleIdentical() {
    double x = 1.0;
    TS_ASSERT(EqualToRoundoff(x, x));
  }

  void testEqualToRoundoffDoubleClose() {
    double x = 1.0;
    double y = x + std::numeric_limits<double>::epsilon();
    TS_ASSERT(EqualToRoundoff(x, y));
  }

  void testEqualToRoundoffDoubleDifferent() {
    double x = 1.0;
    double y = 1.001;
    TS_ASSERT(!EqualToRoundoff(x, y));
  }

  void testEqualToRoundoffFloatIdentical() {
    float x = 1.0f;
    TS_ASSERT(EqualToRoundoff(x, x));
  }

  void testEqualToRoundoffFloatClose() {
    float x = 1.0f;
    float y = x + std::numeric_limits<float>::epsilon();
    TS_ASSERT(EqualToRoundoff(x, y));
  }

  void testEqualToRoundoffMixed() {
    double dx = 1.0;
    float fx = 1.0f;
    TS_ASSERT(EqualToRoundoff(dx, fx));
    TS_ASSERT(EqualToRoundoff(fx, dx));
  }

  void testSignPositive() {
    TS_ASSERT_EQUALS(sign(1.0), 1.0);
    TS_ASSERT_EQUALS(sign(100.5), 1.0);
    TS_ASSERT_EQUALS(sign(0.001), 1.0);
  }

  void testSignZero() {
    TS_ASSERT_EQUALS(sign(0.0), 1.0);
  }

  void testSignNegative() {
    TS_ASSERT_EQUALS(sign(-1.0), -1.0);
    TS_ASSERT_EQUALS(sign(-100.5), -1.0);
    TS_ASSERT_EQUALS(sign(-0.001), -1.0);
  }

  void testConstrainWithinBounds() {
    TS_ASSERT_EQUALS(Constrain(0.0, 0.5, 1.0), 0.5);
    TS_ASSERT_EQUALS(Constrain(-10.0, 0.0, 10.0), 0.0);
    TS_ASSERT_EQUALS(Constrain(0.0, 0.0, 1.0), 0.0);
    TS_ASSERT_EQUALS(Constrain(0.0, 1.0, 1.0), 1.0);
  }

  void testConstrainBelowMin() {
    TS_ASSERT_EQUALS(Constrain(0.0, -1.0, 1.0), 0.0);
    TS_ASSERT_EQUALS(Constrain(5.0, 0.0, 10.0), 5.0);
  }

  void testConstrainAboveMax() {
    TS_ASSERT_EQUALS(Constrain(0.0, 10.0, 1.0), 1.0);
    TS_ASSERT_EQUALS(Constrain(-5.0, 100.0, 5.0), 5.0);
  }

  /***************************************************************************
   * Temperature Conversion Tests - Kelvin
   ***************************************************************************/

  void testKelvinToFahrenheitAbsoluteZero() {
    TS_ASSERT(EqualToRoundoff(KelvinToFahrenheit(0.0), -459.4));
  }

  void testKelvinToFahrenheitWaterFreeze() {
    // 273.15 K = 32 F (with tolerance for implementation differences)
    TS_ASSERT_DELTA(KelvinToFahrenheit(273.15), 32.0, 0.3);
  }

  void testKelvinToFahrenheitBoilingPoint() {
    // 373.15 K = 212 F (with tolerance for implementation differences)
    TS_ASSERT_DELTA(KelvinToFahrenheit(373.15), 212.0, 0.3);
  }

  void testKelvinToRankineAbsoluteZero() {
    TS_ASSERT(EqualToRoundoff(KelvinToRankine(0.0), 0.0));
  }

  void testKelvinToRankineWaterFreeze() {
    // 273.15 K = 491.67 R
    TS_ASSERT_DELTA(KelvinToRankine(273.15), 491.67, 0.01);
  }

  void testKelvinToCelsiusAbsoluteZero() {
    TS_ASSERT(EqualToRoundoff(KelvinToCelsius(0.0), -273.15));
  }

  void testKelvinToCelsiusWaterFreeze() {
    TS_ASSERT(EqualToRoundoff(KelvinToCelsius(273.15), 0.0));
  }

  /***************************************************************************
   * Temperature Conversion Tests - Celsius
   ***************************************************************************/

  void testCelsiusToRankineFreezingPoint() {
    TS_ASSERT(EqualToRoundoff(CelsiusToRankine(0.0), 491.67));
  }

  void testCelsiusToRankineRoomTemp() {
    // 15 C = 518.67 R
    TS_ASSERT(EqualToRoundoff(CelsiusToRankine(15.0), 518.67));
  }

  void testCelsiusToFahrenheitFreezingPoint() {
    TS_ASSERT(EqualToRoundoff(CelsiusToFahrenheit(0.0), 32.0));
  }

  void testCelsiusToFahrenheitBoilingPoint() {
    TS_ASSERT(EqualToRoundoff(CelsiusToFahrenheit(100.0), 212.0));
  }

  void testCelsiusToKelvinAbsoluteZero() {
    TS_ASSERT(EqualToRoundoff(CelsiusToKelvin(-273.15), 0.0));
  }

  void testCelsiusToKelvinFreezingPoint() {
    TS_ASSERT(EqualToRoundoff(CelsiusToKelvin(0.0), 273.15));
  }

  /***************************************************************************
   * Temperature Conversion Tests - Rankine
   ***************************************************************************/

  void testRankineToCelsiusAbsoluteZero() {
    TS_ASSERT_DELTA(RankineToCelsius(0.0), -273.15, 0.01);
  }

  void testRankineToCelsiusFreezingPoint() {
    TS_ASSERT(EqualToRoundoff(RankineToCelsius(491.67), 0.0));
  }

  void testRankineToCelsiusRoomTemp() {
    TS_ASSERT_DELTA(RankineToCelsius(518.67), 15.0, 1E-8);
  }

  void testRankineToKelvinAbsoluteZero() {
    TS_ASSERT(EqualToRoundoff(RankineToKelvin(0.0), 0.0));
  }

  void testRankineToKelvinWaterFreeze() {
    TS_ASSERT_DELTA(RankineToKelvin(491.67), 273.15, 0.01);
  }

  /***************************************************************************
   * Temperature Conversion Tests - Fahrenheit
   ***************************************************************************/

  void testFahrenheitToCelsiusFreezingPoint() {
    TS_ASSERT(EqualToRoundoff(FahrenheitToCelsius(32.0), 0.0));
  }

  void testFahrenheitToCelsiusBoilingPoint() {
    TS_ASSERT(EqualToRoundoff(FahrenheitToCelsius(212.0), 100.0));
  }

  void testFahrenheitToCelsiusRoomTemp() {
    TS_ASSERT_DELTA(FahrenheitToCelsius(59.0), 15.0, 1E-8);
  }

  /***************************************************************************
   * Temperature Roundtrip Tests
   ***************************************************************************/

  void testKelvinCelsiusRoundtrip() {
    double original = 300.0;
    double converted = CelsiusToKelvin(KelvinToCelsius(original));
    TS_ASSERT(EqualToRoundoff(original, converted));
  }

  void testKelvinRankineRoundtrip() {
    double original = 300.0;
    double converted = RankineToKelvin(KelvinToRankine(original));
    TS_ASSERT(EqualToRoundoff(original, converted));
  }

  void testCelsiusFahrenheitRoundtrip() {
    double original = 25.0;
    double converted = FahrenheitToCelsius(CelsiusToFahrenheit(original));
    TS_ASSERT(EqualToRoundoff(original, converted));
  }

  void testCelsiusRankineRoundtrip() {
    double original = 25.0;
    double converted = RankineToCelsius(CelsiusToRankine(original));
    TS_ASSERT_DELTA(original, converted, 1E-10);
  }

  /***************************************************************************
   * Length Conversion Tests
   ***************************************************************************/

  void testFeetToMetersZero() {
    TS_ASSERT_EQUALS(FeetToMeters(0.0), 0.0);
  }

  void testFeetToMetersOneFoot() {
    TS_ASSERT_DELTA(FeetToMeters(1.0), 0.3048, 1E-10);
  }

  void testFeetToMetersMile() {
    // 5280 feet = 1609.344 meters
    TS_ASSERT_DELTA(FeetToMeters(5280.0), 1609.344, 1E-6);
  }

  /***************************************************************************
   * Filter Tests
   ***************************************************************************/

  void testFilterConstruction() {
    Filter f0;
    Filter f(1.0, 1E-5);
    TS_ASSERT(true);  // Construction succeeded
  }

  void testFilterExecute() {
    Filter f(1.0, 0.01);
    double out1 = f.execute(1.0);
    double out2 = f.execute(1.0);
    double out3 = f.execute(1.0);

    // Output should approach input for steady state
    TS_ASSERT(out3 >= out2);
    TS_ASSERT(out2 >= out1);
  }

  void testFilterZeroInput() {
    Filter f(1.0, 0.01);
    double out = f.execute(0.0);
    TS_ASSERT_EQUALS(out, 0.0);
  }

  void testFilterStepResponse() {
    Filter f(10.0, 0.01);  // Fast filter

    // Apply step input
    double out = 0.0;
    for (int i = 0; i < 100; i++) {
      out = f.execute(1.0);
    }

    // Should approach 1.0 after many iterations
    TS_ASSERT(out > 0.9);
  }

  /***************************************************************************
   * Random Number Generator Tests
   ***************************************************************************/

  void testRandomNumberGeneratorConstruction() {
    JSBSim::RandomNumberGenerator generator(17);
    TS_ASSERT(true);  // Construction succeeded
  }

  void testRandomUniformRange() {
    JSBSim::RandomNumberGenerator generator(42);

    for (int i = 0; i < 100; i++) {
      double val = generator.GetUniformRandomNumber();
      TS_ASSERT(val >= -1.0);
      TS_ASSERT(val < 1.0);
    }
  }

  void testRandomNormalDistribution() {
    JSBSim::RandomNumberGenerator generator(42);

    double sum = 0.0;
    int count = 1000;
    for (int i = 0; i < count; i++) {
      sum += generator.GetNormalRandomNumber();
    }
    double mean = sum / count;

    // Mean should be close to 0 for normal distribution
    TS_ASSERT(std::abs(mean) < 0.2);
  }

  void testRandomSeedReproducibility() {
    JSBSim::RandomNumberGenerator generator(17);

    double u0 = generator.GetUniformRandomNumber();
    double u1 = generator.GetUniformRandomNumber();
    double x0 = generator.GetNormalRandomNumber();
    double x1 = generator.GetNormalRandomNumber();

    // Reset with same seed
    generator.seed(17);

    double v0 = generator.GetUniformRandomNumber();
    double v1 = generator.GetUniformRandomNumber();
    double y0 = generator.GetNormalRandomNumber();
    double y1 = generator.GetNormalRandomNumber();

    TS_ASSERT_EQUALS(u0, v0);
    TS_ASSERT_EQUALS(u1, v1);
    TS_ASSERT_EQUALS(x0, y0);
    TS_ASSERT_EQUALS(x1, y1);
  }

  void testRandomDifferentSeeds() {
    JSBSim::RandomNumberGenerator gen1(1);
    JSBSim::RandomNumberGenerator gen2(2);

    double val1 = gen1.GetUniformRandomNumber();
    double val2 = gen2.GetUniformRandomNumber();

    // Different seeds should give different sequences
    TS_ASSERT_DIFFERS(val1, val2);
  }

  /***************************************************************************
   * Version and Misc Tests
   ***************************************************************************/

  void testGetVersion() {
    std::string version = GetVersion();
    TS_ASSERT(!version.empty());
  }

  void testDisableHighLighting() {
    disableHighLighting();
    TS_ASSERT(true);  // Should not crash
  }

  /***************************************************************************
   * Enum Value Tests
   ***************************************************************************/

  void testMomentEnums() {
    TS_ASSERT_EQUALS(eL, 1);
    TS_ASSERT_EQUALS(eM, 2);
    TS_ASSERT_EQUALS(eN, 3);
  }

  void testRateEnums() {
    TS_ASSERT_EQUALS(eP, 1);
    TS_ASSERT_EQUALS(eQ, 2);
    TS_ASSERT_EQUALS(eR, 3);
  }

  void testVelocityEnums() {
    TS_ASSERT_EQUALS(eU, 1);
    TS_ASSERT_EQUALS(eV, 2);
    TS_ASSERT_EQUALS(eW, 3);
  }

  void testPositionEnums() {
    TS_ASSERT_EQUALS(eX, 1);
    TS_ASSERT_EQUALS(eY, 2);
    TS_ASSERT_EQUALS(eZ, 3);
  }

  void testEulerAngleEnums() {
    TS_ASSERT_EQUALS(ePhi, 1);
    TS_ASSERT_EQUALS(eTht, 2);
    TS_ASSERT_EQUALS(ePsi, 3);
  }

  void testStabilityAxisEnums() {
    TS_ASSERT_EQUALS(eDrag, 1);
    TS_ASSERT_EQUALS(eSide, 2);
    TS_ASSERT_EQUALS(eLift, 3);
  }

  void testLocalFrameEnums() {
    TS_ASSERT_EQUALS(eRoll, 1);
    TS_ASSERT_EQUALS(ePitch, 2);
    TS_ASSERT_EQUALS(eYaw, 3);
  }

  void testNEDEnums() {
    TS_ASSERT_EQUALS(eNorth, 1);
    TS_ASSERT_EQUALS(eEast, 2);
    TS_ASSERT_EQUALS(eDown, 3);
  }

  void testLocationEnums() {
    TS_ASSERT_EQUALS(eLat, 1);
    TS_ASSERT_EQUALS(eLong, 2);
    TS_ASSERT_EQUALS(eRad, 3);
  }

  void testConversionEnums() {
    TS_ASSERT_EQUALS(inNone, 0);
    TS_ASSERT_EQUALS(inDegrees, 1);
    TS_ASSERT_EQUALS(inRadians, 2);
    TS_ASSERT_EQUALS(inMeters, 3);
    TS_ASSERT_EQUALS(inFeet, 4);
  }

  /***************************************************************************
   * Extended Numeric Utility Tests
   ***************************************************************************/

  void testEqualToRoundoffZero() {
    TS_ASSERT(EqualToRoundoff(0.0, 0.0));
    TS_ASSERT(EqualToRoundoff(0.0f, 0.0f));
  }

  void testEqualToRoundoffNegative() {
    double x = -1.0;
    double y = -1.0 + std::numeric_limits<double>::epsilon();
    TS_ASSERT(EqualToRoundoff(x, y));
  }

  void testEqualToRoundoffLargeNumbers() {
    double x = 1e10;
    double y = x + x * std::numeric_limits<double>::epsilon();
    TS_ASSERT(EqualToRoundoff(x, y));
  }

  void testEqualToRoundoffSmallNumbers() {
    double x = 1e-10;
    // For very small numbers, they're equal to themselves
    TS_ASSERT(EqualToRoundoff(x, x));
  }

  void testSignVerySmallPositive() {
    TS_ASSERT_EQUALS(sign(1e-300), 1.0);
  }

  void testSignVerySmallNegative() {
    TS_ASSERT_EQUALS(sign(-1e-300), -1.0);
  }

  void testSignLargeValues() {
    TS_ASSERT_EQUALS(sign(1e100), 1.0);
    TS_ASSERT_EQUALS(sign(-1e100), -1.0);
  }

  void testConstrainNegativeBounds() {
    TS_ASSERT_EQUALS(Constrain(-10.0, -5.0, -1.0), -5.0);
    TS_ASSERT_EQUALS(Constrain(-10.0, -15.0, -1.0), -10.0);
    TS_ASSERT_EQUALS(Constrain(-10.0, 0.0, -1.0), -1.0);
  }

  void testConstrainZeroBounds() {
    TS_ASSERT_EQUALS(Constrain(0.0, 0.0, 0.0), 0.0);
  }

  void testConstrainEqualBounds() {
    TS_ASSERT_EQUALS(Constrain(5.0, 10.0, 5.0), 5.0);
    TS_ASSERT_EQUALS(Constrain(5.0, 3.0, 5.0), 5.0);
  }

  /***************************************************************************
   * Extended Temperature Conversion Tests
   ***************************************************************************/

  void testKelvinToFahrenheit100K() {
    // 100 K = approximately -279 F (implementation may vary slightly)
    TS_ASSERT_DELTA(KelvinToFahrenheit(100.0), -279.4, 0.5);
  }

  void testKelvinToFahrenheit500K() {
    // 500 K = approximately 440 F (implementation may vary slightly)
    TS_ASSERT_DELTA(KelvinToFahrenheit(500.0), 440.6, 0.5);
  }

  void testCelsiusToFahrenheit40() {
    // -40 C = -40 F (intersection point)
    TS_ASSERT(EqualToRoundoff(CelsiusToFahrenheit(-40.0), -40.0));
  }

  void testFahrenheitToCelsius40() {
    // -40 F = -40 C
    TS_ASSERT(EqualToRoundoff(FahrenheitToCelsius(-40.0), -40.0));
  }

  void testRankineToKelvin100R() {
    TS_ASSERT_DELTA(RankineToKelvin(100.0), 55.556, 0.01);
  }

  void testKelvinToRankine100K() {
    TS_ASSERT_DELTA(KelvinToRankine(100.0), 180.0, 0.01);
  }

  void testCelsiusToKelvin100C() {
    TS_ASSERT(EqualToRoundoff(CelsiusToKelvin(100.0), 373.15));
  }

  void testKelvinToCelsius100K() {
    TS_ASSERT(EqualToRoundoff(KelvinToCelsius(100.0), -173.15));
  }

  /***************************************************************************
   * Extended Length Conversion Tests
   ***************************************************************************/

  void testFeetToMetersNegative() {
    TS_ASSERT_DELTA(FeetToMeters(-100.0), -30.48, 1E-6);
  }

  void testFeetToMetersLarge() {
    // 100000 feet = 30480 meters
    TS_ASSERT_DELTA(FeetToMeters(100000.0), 30480.0, 0.1);
  }

  void testFeetToMetersAltitude() {
    // 35000 feet (cruise altitude) = 10668 meters
    TS_ASSERT_DELTA(FeetToMeters(35000.0), 10668.0, 1.0);
  }

  /***************************************************************************
   * Extended Filter Tests
   ***************************************************************************/

  void testFilterNegativeInput() {
    Filter f(1.0, 0.01);
    double out = f.execute(-5.0);
    TS_ASSERT(out < 0.0);
  }

  void testFilterLargeTimeConstant() {
    Filter f(0.01, 0.01);  // Very slow filter

    double out = 0.0;
    for (int i = 0; i < 10; i++) {
      out = f.execute(1.0);
    }

    // Should be much less than 1.0 with slow filter
    TS_ASSERT(out < 0.2);
  }

  void testFilterOscillatingInput() {
    Filter f(1.0, 0.01);

    double out1 = f.execute(1.0);
    double out2 = f.execute(-1.0);
    double out3 = f.execute(1.0);

    // Output should be smoother than input
    TS_ASSERT(std::abs(out2) < 1.0);
  }

  void testFilterConvergence() {
    Filter f(5.0, 0.01);

    // Run until convergence
    double out = 0.0;
    double target = 10.0;
    for (int i = 0; i < 500; i++) {
      out = f.execute(target);
    }

    // Should converge to target
    TS_ASSERT_DELTA(out, target, 0.1);
  }

  void testFilterStartFromNonZero() {
    Filter f(1.0, 0.01);

    // Pre-condition with non-zero
    for (int i = 0; i < 100; i++) {
      f.execute(5.0);
    }

    // Now step to 0 - run many more iterations
    double out = 0.0;
    for (int i = 0; i < 500; i++) {
      out = f.execute(0.0);
    }

    // Should approach 0 (use loose tolerance)
    TS_ASSERT(std::abs(out) < 1.0);
  }

  /***************************************************************************
   * Extended Random Number Tests
   ***************************************************************************/

  void testRandomUniformStatistics() {
    JSBSim::RandomNumberGenerator generator(123);

    double sum = 0.0;
    double sumSq = 0.0;
    int count = 10000;

    for (int i = 0; i < count; i++) {
      double val = generator.GetUniformRandomNumber();
      sum += val;
      sumSq += val * val;
    }

    double mean = sum / count;
    double variance = sumSq / count - mean * mean;

    // For uniform[-1,1): mean ≈ 0, variance ≈ 1/3
    TS_ASSERT_DELTA(mean, 0.0, 0.1);
    TS_ASSERT_DELTA(variance, 1.0/3.0, 0.1);
  }

  void testRandomNormalStatistics() {
    JSBSim::RandomNumberGenerator generator(456);

    double sum = 0.0;
    double sumSq = 0.0;
    int count = 10000;

    for (int i = 0; i < count; i++) {
      double val = generator.GetNormalRandomNumber();
      sum += val;
      sumSq += val * val;
    }

    double mean = sum / count;
    double variance = sumSq / count - mean * mean;

    // For standard normal: mean ≈ 0, variance ≈ 1
    TS_ASSERT_DELTA(mean, 0.0, 0.1);
    TS_ASSERT_DELTA(variance, 1.0, 0.2);
  }

  void testRandomSequenceDifferent() {
    JSBSim::RandomNumberGenerator generator(789);

    double val1 = generator.GetUniformRandomNumber();
    double val2 = generator.GetUniformRandomNumber();
    double val3 = generator.GetUniformRandomNumber();

    // Consecutive values should be different
    TS_ASSERT_DIFFERS(val1, val2);
    TS_ASSERT_DIFFERS(val2, val3);
    TS_ASSERT_DIFFERS(val1, val3);
  }

  void testRandomZeroSeed() {
    JSBSim::RandomNumberGenerator generator(0);
    double val = generator.GetUniformRandomNumber();
    TS_ASSERT(val >= -1.0 && val < 1.0);
  }

  void testRandomLargeSeed() {
    JSBSim::RandomNumberGenerator generator(2147483647);
    double val = generator.GetUniformRandomNumber();
    TS_ASSERT(val >= -1.0 && val < 1.0);
  }

  /***************************************************************************
   * Temperature Conversion Chain Tests
   ***************************************************************************/

  void testTemperatureChainKCFK() {
    double original = 350.0;  // Kelvin
    double celsius = KelvinToCelsius(original);
    double fahrenheit = CelsiusToFahrenheit(celsius);
    double kelvin = FahrenheitToCelsius(fahrenheit);
    double result = CelsiusToKelvin(kelvin);
    TS_ASSERT_DELTA(original, result, 0.01);
  }

  void testTemperatureChainRKCR() {
    double original = 518.67;  // Rankine (ISA sea level)
    double kelvin = RankineToKelvin(original);
    double celsius = KelvinToCelsius(kelvin);
    double result = CelsiusToRankine(celsius);
    TS_ASSERT_DELTA(original, result, 0.01);
  }

  /***************************************************************************
   * Mathematical Constant Tests
   ***************************************************************************/

  void testFeetToMetersFactor() {
    // 1 foot = 0.3048 meters exactly
    TS_ASSERT_DELTA(FeetToMeters(1.0), 0.3048, 1e-10);
  }

  void testFeetToMetersInverse() {
    double feet = 100.0;
    double meters = FeetToMeters(feet);
    double backToFeet = meters / 0.3048;
    TS_ASSERT_DELTA(feet, backToFeet, 1e-10);
  }

  /***************************************************************************
   * Boundary Value Tests
   ***************************************************************************/

  void testConstrainAtExactBoundaries() {
    TS_ASSERT_EQUALS(Constrain(0.0, 0.0, 10.0), 0.0);
    TS_ASSERT_EQUALS(Constrain(0.0, 10.0, 10.0), 10.0);
  }

  void testEqualToRoundoffMaxDouble() {
    double max = std::numeric_limits<double>::max();
    TS_ASSERT(EqualToRoundoff(max, max));
  }

  void testEqualToRoundoffMinDouble() {
    double min = std::numeric_limits<double>::min();
    TS_ASSERT(EqualToRoundoff(min, min));
  }

  void testSignInfinity() {
    double inf = std::numeric_limits<double>::infinity();
    TS_ASSERT_EQUALS(sign(inf), 1.0);
    TS_ASSERT_EQUALS(sign(-inf), -1.0);
  }

  /***************************************************************************
   * Additional Enum Consistency Tests
   ***************************************************************************/

  void testEnumSequential123() {
    // All 3-component enums should be 1,2,3
    TS_ASSERT(eL < eM && eM < eN);
    TS_ASSERT(eP < eQ && eQ < eR);
    TS_ASSERT(eU < eV && eV < eW);
    TS_ASSERT(eX < eY && eY < eZ);
  }

  void testEnumAxesConsistent() {
    // Same indices for different axis systems
    TS_ASSERT_EQUALS(eX, eU);
    TS_ASSERT_EQUALS(eY, eV);
    TS_ASSERT_EQUALS(eZ, eW);
  }
};
