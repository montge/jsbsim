#include <cxxtest/TestSuite.h>
#include <math/FGRealValue.h>
#include <limits>
#include <cmath>

using namespace JSBSim;


class FGRealValueTest : public CxxTest::TestSuite
{
public:
  /***************************************************************************
   * Constructor Tests - Basic Values
   ***************************************************************************/

  void testConstructorPositive() {
    // GIVEN: A positive real value
    FGRealValue x(1.0);

    // THEN: GetValue returns the value
    TS_ASSERT_EQUALS(x.GetValue(), 1.0);
    TS_ASSERT_EQUALS(x.GetName(), "constant value 1.000000");
    TS_ASSERT(x.IsConstant());
  }

  void testConstructorNegative() {
    // GIVEN: A negative real value
    FGRealValue x(-5.5);

    // THEN: GetValue returns the negative value correctly
    TS_ASSERT_EQUALS(x.GetValue(), -5.5);
    TS_ASSERT(x.IsConstant());
  }

  void testConstructorZero() {
    // GIVEN: Zero value
    FGRealValue x(0.0);

    // THEN: GetValue returns zero
    TS_ASSERT_EQUALS(x.GetValue(), 0.0);
    TS_ASSERT_EQUALS(x.GetName(), "constant value 0.000000");
    TS_ASSERT(x.IsConstant());
  }

  void testConstructorVerySmall() {
    // GIVEN: A very small value
    FGRealValue x(1e-15);

    // THEN: GetValue returns the precise value
    TS_ASSERT_EQUALS(x.GetValue(), 1e-15);
    TS_ASSERT(x.IsConstant());
  }

  void testConstructorVeryLarge() {
    // GIVEN: A very large value
    FGRealValue x(1e15);

    // THEN: GetValue returns the large value
    TS_ASSERT_EQUALS(x.GetValue(), 1e15);
    TS_ASSERT(x.IsConstant());
  }

  void testConstructorPi() {
    // GIVEN: Pi value
    FGRealValue x(M_PI);

    // THEN: GetValue returns pi with precision
    TS_ASSERT_DELTA(x.GetValue(), M_PI, 1e-15);
    TS_ASSERT(x.IsConstant());
  }

  void testConstructorE() {
    // GIVEN: Euler's number
    FGRealValue x(M_E);

    // THEN: GetValue returns e with precision
    TS_ASSERT_DELTA(x.GetValue(), M_E, 1e-15);
    TS_ASSERT(x.IsConstant());
  }

  void testConstructorNegativeZero() {
    // GIVEN: Negative zero
    FGRealValue x(-0.0);

    // THEN: GetValue returns zero (negative zero equals zero)
    TS_ASSERT_EQUALS(x.GetValue(), 0.0);
    TS_ASSERT(x.IsConstant());
  }

  /***************************************************************************
   * IsConstant Tests
   ***************************************************************************/

  void testIsConstantAlwaysTrue() {
    // GIVEN: Various FGRealValue instances
    FGRealValue x1(0.0);
    FGRealValue x2(100.0);
    FGRealValue x3(-999.9);

    // THEN: IsConstant is always true (FGRealValue is immutable)
    TS_ASSERT(x1.IsConstant());
    TS_ASSERT(x2.IsConstant());
    TS_ASSERT(x3.IsConstant());
  }

  void testIsConstantForSpecialValues() {
    FGRealValue inf_val(std::numeric_limits<double>::infinity());
    FGRealValue neg_inf(-std::numeric_limits<double>::infinity());
    FGRealValue max_val(std::numeric_limits<double>::max());

    TS_ASSERT(inf_val.IsConstant());
    TS_ASSERT(neg_inf.IsConstant());
    TS_ASSERT(max_val.IsConstant());
  }

  /***************************************************************************
   * GetValue Idempotency Tests
   ***************************************************************************/

  void testGetValueIsIdempotent() {
    // GIVEN: A FGRealValue
    FGRealValue x(42.0);

    // WHEN: Calling GetValue multiple times
    double v1 = x.GetValue();
    double v2 = x.GetValue();
    double v3 = x.GetValue();

    // THEN: All calls return the same value
    TS_ASSERT_EQUALS(v1, 42.0);
    TS_ASSERT_EQUALS(v2, 42.0);
    TS_ASSERT_EQUALS(v3, 42.0);
  }

  void testGetValueIdempotentManyTimes() {
    FGRealValue x(3.14159);

    for (int i = 0; i < 1000; i++) {
      TS_ASSERT_EQUALS(x.GetValue(), 3.14159);
    }
  }

  void testGetDoubleValueAlias() {
    // getDoubleValue should return the same as GetValue
    FGRealValue x(123.456);

    TS_ASSERT_EQUALS(x.GetValue(), x.getDoubleValue());
  }

  /***************************************************************************
   * GetName Format Tests
   ***************************************************************************/

  void testGetNameFormat() {
    // GIVEN: Values with different decimal representations
    FGRealValue x1(1.5);
    FGRealValue x2(100.0);
    FGRealValue x3(0.123456);

    // THEN: GetName should contain "constant value" and the formatted number
    TS_ASSERT(x1.GetName().find("constant value") != std::string::npos);
    TS_ASSERT(x2.GetName().find("constant value") != std::string::npos);
    TS_ASSERT(x3.GetName().find("constant value") != std::string::npos);
  }

  void testGetNameZero() {
    FGRealValue x(0.0);
    std::string name = x.GetName();

    TS_ASSERT(name.find("constant value") != std::string::npos);
    TS_ASSERT(name.find("0.000000") != std::string::npos);
  }

  void testGetNameOne() {
    FGRealValue x(1.0);
    std::string name = x.GetName();

    TS_ASSERT(name.find("constant value") != std::string::npos);
    TS_ASSERT(name.find("1.000000") != std::string::npos);
  }

  void testGetNameNegative() {
    FGRealValue x(-42.5);
    std::string name = x.GetName();

    TS_ASSERT(name.find("constant value") != std::string::npos);
    TS_ASSERT(name.find("-42.5") != std::string::npos);
  }

  void testGetNameNotEmpty() {
    FGRealValue x(999.999);
    TS_ASSERT(!x.GetName().empty());
  }

  /***************************************************************************
   * Special IEEE Values Tests
   ***************************************************************************/

  void testSpecialValues() {
    // GIVEN: Special IEEE floating point values
    FGRealValue maxVal(std::numeric_limits<double>::max());
    FGRealValue minVal(std::numeric_limits<double>::min());
    FGRealValue denorm(std::numeric_limits<double>::denorm_min());

    // THEN: Values are preserved
    TS_ASSERT_EQUALS(maxVal.GetValue(), std::numeric_limits<double>::max());
    TS_ASSERT_EQUALS(minVal.GetValue(), std::numeric_limits<double>::min());
    TS_ASSERT_EQUALS(denorm.GetValue(), std::numeric_limits<double>::denorm_min());
  }

  void testInfinityValue() {
    double inf = std::numeric_limits<double>::infinity();
    FGRealValue x(inf);

    TS_ASSERT_EQUALS(x.GetValue(), inf);
    TS_ASSERT(std::isinf(x.GetValue()));
    TS_ASSERT(x.IsConstant());
  }

  void testNegativeInfinityValue() {
    double neg_inf = -std::numeric_limits<double>::infinity();
    FGRealValue x(neg_inf);

    TS_ASSERT_EQUALS(x.GetValue(), neg_inf);
    TS_ASSERT(std::isinf(x.GetValue()));
    TS_ASSERT(x.GetValue() < 0);
    TS_ASSERT(x.IsConstant());
  }

  void testNaNValue() {
    double nan_val = std::nan("");
    FGRealValue x(nan_val);

    TS_ASSERT(std::isnan(x.GetValue()));
    TS_ASSERT(x.IsConstant());
  }

  void testQuietNaN() {
    double qnan = std::numeric_limits<double>::quiet_NaN();
    FGRealValue x(qnan);

    TS_ASSERT(std::isnan(x.GetValue()));
  }

  void testEpsilon() {
    double eps = std::numeric_limits<double>::epsilon();
    FGRealValue x(eps);

    TS_ASSERT_EQUALS(x.GetValue(), eps);
  }

  void testLowestValue() {
    double lowest = std::numeric_limits<double>::lowest();
    FGRealValue x(lowest);

    TS_ASSERT_EQUALS(x.GetValue(), lowest);
  }

  /***************************************************************************
   * Precision Tests
   ***************************************************************************/

  void testPrecisionSmallDifference() {
    double a = 1.0;
    double b = 1.0 + std::numeric_limits<double>::epsilon();

    FGRealValue xa(a);
    FGRealValue xb(b);

    TS_ASSERT_DIFFERS(xa.GetValue(), xb.GetValue());
  }

  void testPrecisionMantissa() {
    // Test precision of mantissa
    double val = 1.23456789012345678901234567890;
    FGRealValue x(val);

    // Should preserve at least 15 significant digits
    TS_ASSERT_DELTA(x.GetValue(), val, 1e-15);
  }

  void testPrecisionFractions() {
    FGRealValue half(0.5);
    FGRealValue third(1.0/3.0);
    FGRealValue seventh(1.0/7.0);

    TS_ASSERT_EQUALS(half.GetValue(), 0.5);
    TS_ASSERT_DELTA(third.GetValue(), 1.0/3.0, 1e-15);
    TS_ASSERT_DELTA(seventh.GetValue(), 1.0/7.0, 1e-15);
  }

  /***************************************************************************
   * Multiple Instances Tests
   ***************************************************************************/

  void testMultipleInstancesIndependent() {
    FGRealValue x1(10.0);
    FGRealValue x2(20.0);
    FGRealValue x3(30.0);

    TS_ASSERT_EQUALS(x1.GetValue(), 10.0);
    TS_ASSERT_EQUALS(x2.GetValue(), 20.0);
    TS_ASSERT_EQUALS(x3.GetValue(), 30.0);
  }

  void testMultipleInstancesSameValue() {
    FGRealValue x1(42.0);
    FGRealValue x2(42.0);

    TS_ASSERT_EQUALS(x1.GetValue(), x2.GetValue());
    TS_ASSERT_EQUALS(x1.GetName(), x2.GetName());
  }

  void testArrayOfRealValues() {
    std::vector<FGRealValue> values;
    for (int i = 0; i < 100; i++) {
      values.emplace_back(static_cast<double>(i));
    }

    for (int i = 0; i < 100; i++) {
      TS_ASSERT_EQUALS(values[i].GetValue(), static_cast<double>(i));
    }
  }

  /***************************************************************************
   * Mathematical Constants Tests
   ***************************************************************************/

  void testMathConstantPi() {
    FGRealValue pi(M_PI);
    TS_ASSERT_DELTA(pi.GetValue(), 3.14159265358979323846, 1e-15);
  }

  void testMathConstantE() {
    FGRealValue e(M_E);
    TS_ASSERT_DELTA(e.GetValue(), 2.71828182845904523536, 1e-15);
  }

  void testMathConstantSqrt2() {
    FGRealValue sqrt2(M_SQRT2);
    TS_ASSERT_DELTA(sqrt2.GetValue(), 1.41421356237309504880, 1e-15);
  }

  void testMathConstantLn2() {
    FGRealValue ln2(M_LN2);
    TS_ASSERT_DELTA(ln2.GetValue(), 0.69314718055994530942, 1e-15);
  }

  /***************************************************************************
   * Derived Operations Tests (through FGParameter interface)
   ***************************************************************************/

  void testMultiplicationOperator() {
    SGSharedPtr<FGRealValue> p(new FGRealValue(5.0));

    TS_ASSERT_EQUALS(p * 2.0, 10.0);
    TS_ASSERT_EQUALS(2.0 * p, 10.0);
  }

  void testMultiplicationByZero() {
    SGSharedPtr<FGRealValue> p(new FGRealValue(100.0));

    TS_ASSERT_EQUALS(p * 0.0, 0.0);
    TS_ASSERT_EQUALS(0.0 * p, 0.0);
  }

  void testMultiplicationByNegative() {
    SGSharedPtr<FGRealValue> p(new FGRealValue(7.0));

    TS_ASSERT_EQUALS(p * (-3.0), -21.0);
    TS_ASSERT_EQUALS(-3.0 * p, -21.0);
  }

  void testMultiplicationByFraction() {
    SGSharedPtr<FGRealValue> p(new FGRealValue(10.0));

    TS_ASSERT_EQUALS(p * 0.5, 5.0);
    TS_ASSERT_EQUALS(0.5 * p, 5.0);
  }

  void testMultiplicationCommutativity() {
    SGSharedPtr<FGRealValue> p(new FGRealValue(4.5));
    double v = 3.0;

    TS_ASSERT_EQUALS(p * v, v * p);
  }

  /***************************************************************************
   * Pointer/Reference Tests
   ***************************************************************************/

  void testSharedPointerConstruction() {
    SGSharedPtr<FGRealValue> p(new FGRealValue(42.0));

    TS_ASSERT(p.valid());
    TS_ASSERT_EQUALS(p->GetValue(), 42.0);
    TS_ASSERT(p->IsConstant());
  }

  void testSharedPointerCopy() {
    SGSharedPtr<FGRealValue> p1(new FGRealValue(100.0));
    SGSharedPtr<FGRealValue> p2 = p1;

    TS_ASSERT(p1.valid());
    TS_ASSERT(p2.valid());
    TS_ASSERT_EQUALS(p1->GetValue(), p2->GetValue());
  }

  void testFGParameterPtrTypedef() {
    FGParameter_ptr p(new FGRealValue(55.5));

    TS_ASSERT(p.valid());
    TS_ASSERT_EQUALS(p->GetValue(), 55.5);
    TS_ASSERT(p->IsConstant());
  }

  /***************************************************************************
   * Immutability Tests
   ***************************************************************************/

  void testImmutabilityAfterCreation() {
    FGRealValue x(123.456);

    double v1 = x.GetValue();
    double v2 = x.GetValue();
    std::string n1 = x.GetName();
    std::string n2 = x.GetName();

    TS_ASSERT_EQUALS(v1, v2);
    TS_ASSERT_EQUALS(n1, n2);
    TS_ASSERT_EQUALS(v1, 123.456);
  }

  void testConstCorrectness() {
    const FGRealValue x(99.9);

    // These should compile - const methods
    double v = x.GetValue();
    std::string n = x.GetName();
    bool c = x.IsConstant();

    TS_ASSERT_EQUALS(v, 99.9);
    TS_ASSERT(!n.empty());
    TS_ASSERT(c);
  }

  /***************************************************************************
   * Scientific Notation Tests
   ***************************************************************************/

  void testScientificNotationPositive() {
    FGRealValue x1(1.5e10);
    FGRealValue x2(3.14159e-5);
    FGRealValue x3(6.022e23);

    TS_ASSERT_EQUALS(x1.GetValue(), 1.5e10);
    TS_ASSERT_EQUALS(x2.GetValue(), 3.14159e-5);
    TS_ASSERT_EQUALS(x3.GetValue(), 6.022e23);
  }

  void testScientificNotationNegative() {
    FGRealValue x1(-1.5e10);
    FGRealValue x2(-3.14159e-5);

    TS_ASSERT_EQUALS(x1.GetValue(), -1.5e10);
    TS_ASSERT_EQUALS(x2.GetValue(), -3.14159e-5);
  }

  void testScientificNotationExtremes() {
    FGRealValue x1(1e308);  // Near max
    FGRealValue x2(1e-308); // Near min positive

    TS_ASSERT_EQUALS(x1.GetValue(), 1e308);
    TS_ASSERT_EQUALS(x2.GetValue(), 1e-308);
  }

  /***************************************************************************
   * Sign Handling Tests
   ***************************************************************************/

  void testSignPositive() {
    FGRealValue x(5.5);
    TS_ASSERT(x.GetValue() > 0.0);
    TS_ASSERT(std::signbit(x.GetValue()) == false);
  }

  void testSignNegative() {
    FGRealValue x(-5.5);
    TS_ASSERT(x.GetValue() < 0.0);
    TS_ASSERT(std::signbit(x.GetValue()) == true);
  }

  void testSignZeroPositive() {
    FGRealValue x(+0.0);
    TS_ASSERT_EQUALS(x.GetValue(), 0.0);
    TS_ASSERT(std::signbit(x.GetValue()) == false);
  }

  void testSignPreservation() {
    // Test that sign is preserved through multiplication
    SGSharedPtr<FGRealValue> pos(new FGRealValue(10.0));
    SGSharedPtr<FGRealValue> neg(new FGRealValue(-10.0));

    TS_ASSERT(pos * 1.0 > 0.0);
    TS_ASSERT(neg * 1.0 < 0.0);
    TS_ASSERT(pos * (-1.0) < 0.0);
    TS_ASSERT(neg * (-1.0) > 0.0);
  }

  /***************************************************************************
   * Comparison Tests
   ***************************************************************************/

  void testComparisonEqualValues() {
    FGRealValue x1(42.0);
    FGRealValue x2(42.0);

    TS_ASSERT_EQUALS(x1.GetValue(), x2.GetValue());
  }

  void testComparisonDifferentValues() {
    FGRealValue x1(10.0);
    FGRealValue x2(20.0);

    TS_ASSERT(x1.GetValue() < x2.GetValue());
    TS_ASSERT(x2.GetValue() > x1.GetValue());
    TS_ASSERT_DIFFERS(x1.GetValue(), x2.GetValue());
  }

  void testComparisonNearValues() {
    double eps = std::numeric_limits<double>::epsilon();
    FGRealValue x1(1.0);
    FGRealValue x2(1.0 + eps);

    TS_ASSERT_DIFFERS(x1.GetValue(), x2.GetValue());
    TS_ASSERT(x1.GetValue() < x2.GetValue());
  }

  void testComparisonWithZero() {
    FGRealValue pos(0.001);
    FGRealValue zero(0.0);
    FGRealValue neg(-0.001);

    TS_ASSERT(pos.GetValue() > zero.GetValue());
    TS_ASSERT(neg.GetValue() < zero.GetValue());
  }

  /***************************************************************************
   * Copy Semantics Tests
   ***************************************************************************/

  void testCopyConstruction() {
    FGRealValue original(99.9);
    FGRealValue copy(original);

    TS_ASSERT_EQUALS(copy.GetValue(), original.GetValue());
    TS_ASSERT_EQUALS(copy.GetName(), original.GetName());
  }

  void testCopyIndependence() {
    FGRealValue x1(50.0);
    FGRealValue x2 = x1;

    // After copy, they should be independent
    TS_ASSERT_EQUALS(x1.GetValue(), 50.0);
    TS_ASSERT_EQUALS(x2.GetValue(), 50.0);
    TS_ASSERT_EQUALS(x1.GetValue(), x2.GetValue());
  }

  /***************************************************************************
   * Simulation Value Range Tests
   ***************************************************************************/

  void testAngleDegreesRange() {
    // Common angle values in degrees
    FGRealValue deg0(0.0);
    FGRealValue deg90(90.0);
    FGRealValue deg180(180.0);
    FGRealValue deg360(360.0);
    FGRealValue degNeg(-45.0);

    TS_ASSERT_EQUALS(deg0.GetValue(), 0.0);
    TS_ASSERT_EQUALS(deg90.GetValue(), 90.0);
    TS_ASSERT_EQUALS(deg180.GetValue(), 180.0);
    TS_ASSERT_EQUALS(deg360.GetValue(), 360.0);
    TS_ASSERT_EQUALS(degNeg.GetValue(), -45.0);
  }

  void testAngleRadiansRange() {
    // Common angle values in radians
    FGRealValue rad0(0.0);
    FGRealValue radPi(M_PI);
    FGRealValue radHalfPi(M_PI / 2.0);
    FGRealValue rad2Pi(2.0 * M_PI);

    TS_ASSERT_EQUALS(rad0.GetValue(), 0.0);
    TS_ASSERT_DELTA(radPi.GetValue(), M_PI, 1e-15);
    TS_ASSERT_DELTA(radHalfPi.GetValue(), M_PI / 2.0, 1e-15);
    TS_ASSERT_DELTA(rad2Pi.GetValue(), 2.0 * M_PI, 1e-15);
  }

  void testVelocityFpsRange() {
    // Velocity values in feet per second
    FGRealValue zero_fps(0.0);
    FGRealValue cruise_fps(500.0);   // ~295 knots
    FGRealValue supersonic_fps(1200.0);  // ~Mach 1.1
    FGRealValue hypersonic_fps(5500.0);  // ~Mach 5

    TS_ASSERT_EQUALS(zero_fps.GetValue(), 0.0);
    TS_ASSERT_EQUALS(cruise_fps.GetValue(), 500.0);
    TS_ASSERT_EQUALS(supersonic_fps.GetValue(), 1200.0);
    TS_ASSERT_EQUALS(hypersonic_fps.GetValue(), 5500.0);
  }

  void testAltitudeFeetRange() {
    // Altitude values in feet
    FGRealValue sea_level(0.0);
    FGRealValue low_alt(5000.0);
    FGRealValue cruise_alt(35000.0);
    FGRealValue high_alt(60000.0);
    FGRealValue space_boundary(328084.0);  // 100 km

    TS_ASSERT_EQUALS(sea_level.GetValue(), 0.0);
    TS_ASSERT_EQUALS(low_alt.GetValue(), 5000.0);
    TS_ASSERT_EQUALS(cruise_alt.GetValue(), 35000.0);
    TS_ASSERT_EQUALS(high_alt.GetValue(), 60000.0);
    TS_ASSERT_EQUALS(space_boundary.GetValue(), 328084.0);
  }

  void testForceNewtonRange() {
    // Force values in various magnitudes
    FGRealValue small_force(10.0);
    FGRealValue medium_force(10000.0);
    FGRealValue large_force(1e6);  // Aircraft thrust

    TS_ASSERT_EQUALS(small_force.GetValue(), 10.0);
    TS_ASSERT_EQUALS(medium_force.GetValue(), 10000.0);
    TS_ASSERT_EQUALS(large_force.GetValue(), 1e6);
  }

  /***************************************************************************
   * Arithmetic Chain Tests
   ***************************************************************************/

  void testMultiplicationChain() {
    SGSharedPtr<FGRealValue> p(new FGRealValue(2.0));

    double result = (p * 3.0) * 4.0;
    TS_ASSERT_EQUALS(result, 24.0);
  }

  void testMultiplicationIdentity() {
    SGSharedPtr<FGRealValue> p(new FGRealValue(42.0));

    TS_ASSERT_EQUALS(p * 1.0, p->GetValue());
    TS_ASSERT_EQUALS(1.0 * p, p->GetValue());
  }

  void testMultiplicationAssociativity() {
    SGSharedPtr<FGRealValue> p(new FGRealValue(5.0));
    double a = 3.0, b = 4.0;

    // (p * a) * b should equal p * (a * b)
    double left = (p * a) * b;
    double right = p * (a * b);
    TS_ASSERT_DELTA(left, right, 1e-15);
  }

  void testMultiplicationWithSelf() {
    SGSharedPtr<FGRealValue> p(new FGRealValue(7.0));
    double val = p->GetValue();

    TS_ASSERT_EQUALS(p * val, 49.0);
  }

  /***************************************************************************
   * GetName Special Cases Tests
   ***************************************************************************/

  void testGetNameWithInfinity() {
    FGRealValue inf_val(std::numeric_limits<double>::infinity());
    std::string name = inf_val.GetName();

    TS_ASSERT(!name.empty());
    TS_ASSERT(name.find("constant value") != std::string::npos);
  }

  void testGetNameWithNaN() {
    FGRealValue nan_val(std::nan(""));
    std::string name = nan_val.GetName();

    TS_ASSERT(!name.empty());
    TS_ASSERT(name.find("constant value") != std::string::npos);
  }

  void testGetNameWithVerySmall() {
    FGRealValue tiny(1e-300);
    std::string name = tiny.GetName();

    TS_ASSERT(!name.empty());
    TS_ASSERT(name.find("constant value") != std::string::npos);
  }

  void testGetNameWithVeryLarge() {
    FGRealValue huge(1e300);
    std::string name = huge.GetName();

    TS_ASSERT(!name.empty());
    TS_ASSERT(name.find("constant value") != std::string::npos);
  }

  /***************************************************************************
   * FGParameter Interface Compliance Tests
   ***************************************************************************/

  void testFGParameterPolymorphism() {
    FGParameter_ptr param(new FGRealValue(123.0));

    // Access through base interface
    TS_ASSERT_EQUALS(param->GetValue(), 123.0);
    TS_ASSERT(param->IsConstant());
    TS_ASSERT(!param->GetName().empty());
  }

  void testFGParameterMultiplication() {
    FGParameter_ptr param(new FGRealValue(10.0));

    TS_ASSERT_EQUALS(param * 5.0, 50.0);
    TS_ASSERT_EQUALS(5.0 * param, 50.0);
  }

  void testFGParameterGetDoubleValue() {
    FGParameter_ptr param(new FGRealValue(99.99));

    TS_ASSERT_EQUALS(param->getDoubleValue(), param->GetValue());
    TS_ASSERT_EQUALS(param->getDoubleValue(), 99.99);
  }

  /***************************************************************************
   * Binary Representation Edge Cases
   ***************************************************************************/

  void testPowerOfTwo() {
    // Powers of 2 should be exact in IEEE 754
    for (int i = -10; i <= 10; i++) {
      double val = std::pow(2.0, i);
      FGRealValue x(val);
      TS_ASSERT_EQUALS(x.GetValue(), val);
    }
  }

  void testNonRepresentableDecimals() {
    // 0.1 is not exactly representable in binary
    FGRealValue tenth(0.1);
    TS_ASSERT_DELTA(tenth.GetValue(), 0.1, 1e-16);

    // Sum of 10 tenths should be close to 1.0
    double sum = 0.0;
    for (int i = 0; i < 10; i++) {
      sum += tenth.GetValue();
    }
    TS_ASSERT_DELTA(sum, 1.0, 1e-14);
  }

  void testSubnormalNumbers() {
    double subnorm = std::numeric_limits<double>::denorm_min();
    FGRealValue x(subnorm);

    TS_ASSERT_EQUALS(x.GetValue(), subnorm);
    TS_ASSERT(x.GetValue() > 0.0);
    TS_ASSERT(std::fpclassify(x.GetValue()) == FP_SUBNORMAL);
  }

  void testSignalingNaN() {
    double snan = std::numeric_limits<double>::signaling_NaN();
    FGRealValue x(snan);

    TS_ASSERT(std::isnan(x.GetValue()));
    TS_ASSERT(x.IsConstant());
  }

  /***************************************************************************
   * Numerical Stability Tests
   ***************************************************************************/

  void testAdditionCancellation() {
    // Test values that might cause cancellation issues
    FGRealValue large(1e15);
    FGRealValue small(1.0);

    // The values should be stored correctly independently
    TS_ASSERT_EQUALS(large.GetValue(), 1e15);
    TS_ASSERT_EQUALS(small.GetValue(), 1.0);
  }

  void testMultiplicationOverflow() {
    SGSharedPtr<FGRealValue> large(new FGRealValue(1e200));

    double result = large * 1e200;
    TS_ASSERT(std::isinf(result));
  }

  void testMultiplicationUnderflow() {
    SGSharedPtr<FGRealValue> tiny(new FGRealValue(1e-200));

    double result = tiny * 1e-200;
    // Result should be 0 or subnormal
    TS_ASSERT(result == 0.0 || std::fpclassify(result) == FP_SUBNORMAL);
  }

  /***************************************************************************
   * JSBSim Constant Values Tests
   ***************************************************************************/

  void testGravityConstant() {
    // Standard gravity in ft/s^2
    FGRealValue g(32.174);
    TS_ASSERT_DELTA(g.GetValue(), 32.174, 1e-10);
  }

  void testSealevelDensity() {
    // Sea level density in slugs/ft^3
    FGRealValue rho0(0.00237689);
    TS_ASSERT_DELTA(rho0.GetValue(), 0.00237689, 1e-10);
  }

  void testSpeedOfSound() {
    // Speed of sound at sea level in ft/s
    FGRealValue a0(1116.45);
    TS_ASSERT_DELTA(a0.GetValue(), 1116.45, 1e-10);
  }

  void testDegreesToRadians() {
    // Conversion factor
    double deg2rad = M_PI / 180.0;
    FGRealValue conv(deg2rad);

    TS_ASSERT_DELTA(conv.GetValue(), deg2rad, 1e-15);
    TS_ASSERT_DELTA(conv.GetValue() * 180.0, M_PI, 1e-14);
  }

  void testFeetToMeters() {
    // Conversion factor
    FGRealValue ft2m(0.3048);
    TS_ASSERT_DELTA(ft2m.GetValue(), 0.3048, 1e-10);
  }

  /***************************************************************************
   * Vector of Pointers Tests
   ***************************************************************************/

  void testVectorOfParameterPtrs() {
    std::vector<FGParameter_ptr> params;
    for (int i = 0; i < 10; i++) {
      params.push_back(FGParameter_ptr(new FGRealValue(static_cast<double>(i))));
    }

    for (int i = 0; i < 10; i++) {
      TS_ASSERT_EQUALS(params[i]->GetValue(), static_cast<double>(i));
      TS_ASSERT(params[i]->IsConstant());
    }
  }

  void testMixedParameterOperations() {
    FGParameter_ptr p1(new FGRealValue(10.0));
    FGParameter_ptr p2(new FGRealValue(20.0));

    TS_ASSERT_EQUALS(p1->GetValue() + p2->GetValue(), 30.0);
    TS_ASSERT_EQUALS(p1->GetValue() * p2->GetValue(), 200.0);
    TS_ASSERT_EQUALS(p2->GetValue() / p1->GetValue(), 2.0);
    TS_ASSERT_EQUALS(p2->GetValue() - p1->GetValue(), 10.0);
  }

  /***************************************************************************
   * Edge Case Combinations
   ***************************************************************************/

  void testInfinityArithmetic() {
    SGSharedPtr<FGRealValue> inf(new FGRealValue(std::numeric_limits<double>::infinity()));

    double result_pos = inf * 1.0;
    double result_neg = inf * (-1.0);

    TS_ASSERT(std::isinf(result_pos) && result_pos > 0);
    TS_ASSERT(std::isinf(result_neg) && result_neg < 0);
  }

  void testZeroMultiplicationWithInfinity() {
    SGSharedPtr<FGRealValue> zero(new FGRealValue(0.0));
    double inf = std::numeric_limits<double>::infinity();

    double result = zero * inf;
    TS_ASSERT(std::isnan(result));  // 0 * inf = NaN
  }

  void testNegativeZeroPreservation() {
    // Negative zero should behave like zero for comparisons
    FGRealValue neg_zero(-0.0);
    FGRealValue pos_zero(+0.0);

    TS_ASSERT_EQUALS(neg_zero.GetValue(), pos_zero.GetValue());
    TS_ASSERT(neg_zero.GetValue() == 0.0);
    TS_ASSERT(pos_zero.GetValue() == 0.0);
  }

  /***************************************************************************
   * Complete System Tests
   ***************************************************************************/

  void testCompleteValueRange() {
    // Test values across wide range
    double values[] = {-1e10, -1000.0, -1.0, 0.0, 1.0, 1000.0, 1e10};

    for (double v : values) {
      FGRealValue rv(v);
      TS_ASSERT_DELTA(rv.GetValue(), v, std::abs(v) * 1e-10 + 1e-15);
    }
  }

  void testCompleteValueOperations() {
    FGRealValue rv1(100.0);
    FGRealValue rv2(50.0);

    double sum = rv1.GetValue() + rv2.GetValue();
    double diff = rv1.GetValue() - rv2.GetValue();
    double prod = rv1.GetValue() * rv2.GetValue();
    double quot = rv1.GetValue() / rv2.GetValue();

    TS_ASSERT_DELTA(sum, 150.0, 1e-10);
    TS_ASSERT_DELTA(diff, 50.0, 1e-10);
    TS_ASSERT_DELTA(prod, 5000.0, 1e-10);
    TS_ASSERT_DELTA(quot, 2.0, 1e-10);
  }

  void testCompletePrecisionPreservation() {
    double preciseValue = 3.141592653589793;
    FGRealValue rv(preciseValue);

    TS_ASSERT_DELTA(rv.GetValue(), preciseValue, 1e-15);
  }

  void testCompleteSpecialValueHandling() {
    // Test various special numeric patterns
    FGRealValue tiny(1e-300);
    FGRealValue large(1e300);

    TS_ASSERT(tiny.GetValue() > 0.0);
    TS_ASSERT(large.GetValue() > 0.0);
    TS_ASSERT(tiny.GetValue() < large.GetValue());
  }

  /***************************************************************************
   * Instance Independence Tests
   ***************************************************************************/

  void testIndependentValueInstances() {
    FGRealValue rv1(100.0);
    FGRealValue rv2(200.0);

    TS_ASSERT_DELTA(rv1.GetValue(), 100.0, 1e-10);
    TS_ASSERT_DELTA(rv2.GetValue(), 200.0, 1e-10);

    // Creating rv2 doesn't affect rv1
    TS_ASSERT_DELTA(rv1.GetValue(), 100.0, 1e-10);
  }

  void testIndependentNegativeValues() {
    FGRealValue neg(-42.5);
    FGRealValue pos(42.5);

    TS_ASSERT_DELTA(neg.GetValue(), -42.5, 1e-10);
    TS_ASSERT_DELTA(pos.GetValue(), 42.5, 1e-10);
    TS_ASSERT_DELTA(neg.GetValue() + pos.GetValue(), 0.0, 1e-10);
  }

  void testIndependentArrayOfValues() {
    FGRealValue values[5] = {
      FGRealValue(1.0),
      FGRealValue(2.0),
      FGRealValue(3.0),
      FGRealValue(4.0),
      FGRealValue(5.0)
    };

    for (int i = 0; i < 5; i++) {
      TS_ASSERT_DELTA(values[i].GetValue(), static_cast<double>(i + 1), 1e-10);
    }
  }

  void testIndependentScaledValues() {
    FGRealValue base(10.0);
    double v1 = base.GetValue() * 2.0;
    double v2 = base.GetValue() * 3.0;

    TS_ASSERT_DELTA(v1, 20.0, 1e-10);
    TS_ASSERT_DELTA(v2, 30.0, 1e-10);

    // Base value unchanged
    TS_ASSERT_DELTA(base.GetValue(), 10.0, 1e-10);
  }

  void testIndependentFractionalValues() {
    FGRealValue half(0.5);
    FGRealValue quarter(0.25);
    FGRealValue eighth(0.125);

    TS_ASSERT_DELTA(half.GetValue(), 0.5, 1e-15);
    TS_ASSERT_DELTA(quarter.GetValue(), 0.25, 1e-15);
    TS_ASSERT_DELTA(eighth.GetValue(), 0.125, 1e-15);

    // Sum should be 0.875
    double sum = half.GetValue() + quarter.GetValue() + eighth.GetValue();
    TS_ASSERT_DELTA(sum, 0.875, 1e-15);
  }
};
