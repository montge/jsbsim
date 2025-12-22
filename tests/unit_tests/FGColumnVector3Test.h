#include <cxxtest/TestSuite.h>
#include <math/FGColumnVector3.h>

class FGColumnVector3Test : public CxxTest::TestSuite
{
public:
  void testDefaultConstructor(void) {
    JSBSim::FGColumnVector3 v0;

    TS_ASSERT_EQUALS(v0.Entry(1), 0.0);
    TS_ASSERT_EQUALS(v0.Entry(2), 0.0);
    TS_ASSERT_EQUALS(v0.Entry(3), 0.0);
  }

  void testConstructorWithLitterals(void) {
    JSBSim::FGColumnVector3 v1(1., 0., -2.);

    TS_ASSERT_EQUALS(v1.Entry(1), 1.0);
    TS_ASSERT_EQUALS(v1.Entry(2), 0.0);
    TS_ASSERT_EQUALS(v1.Entry(3), -2.0);
  }

  void testParenthesisOperator(void) {
    JSBSim::FGColumnVector3 v1(1., 0., -2.);

    TS_ASSERT_EQUALS(v1(1), 1.0);
    TS_ASSERT_EQUALS(v1(2), 0.0);
    TS_ASSERT_EQUALS(v1(3), -2.0);
  }

  void testCopyConstructor(void) {
    JSBSim::FGColumnVector3 v1(1., 0., -2.);
    JSBSim::FGColumnVector3 v2(v1);

    // Check that v1 and v2 are identical
    TS_ASSERT_EQUALS(v2(1), v1(1));
    TS_ASSERT_EQUALS(v2(2), v1(2));
    TS_ASSERT_EQUALS(v2(3), v1(3));
  }

  void testComponentAssignment(void) {
    JSBSim::FGColumnVector3 v;

    v(1) = 1.5;
    TS_ASSERT_EQUALS(v(1), 1.5);
    TS_ASSERT_EQUALS(v(2), 0.0);
    TS_ASSERT_EQUALS(v(3), 0.0);

    v(2) = 2.5;
    TS_ASSERT_EQUALS(v(1), 1.5);
    TS_ASSERT_EQUALS(v(2), 2.5);
    TS_ASSERT_EQUALS(v(3), 0.0);

    v(3) = -1.5;
    TS_ASSERT_EQUALS(v(1), 1.5);
    TS_ASSERT_EQUALS(v(2), 2.5);
    TS_ASSERT_EQUALS(v(3), -1.5);
  }

  void testComponentAssignmentOpParent(void) {
    JSBSim::FGColumnVector3 v;

    v(1) = 1.5;
    TS_ASSERT_EQUALS(v.Entry(1), 1.5);
    TS_ASSERT_EQUALS(v.Entry(2), 0.0);
    TS_ASSERT_EQUALS(v.Entry(3), 0.0);

    v(2) = 2.5;
    TS_ASSERT_EQUALS(v.Entry(1), 1.5);
    TS_ASSERT_EQUALS(v.Entry(2), 2.5);
    TS_ASSERT_EQUALS(v.Entry(3), 0.0);

    v(3) = -1.5;
    TS_ASSERT_EQUALS(v.Entry(1), 1.5);
    TS_ASSERT_EQUALS(v.Entry(2), 2.5);
    TS_ASSERT_EQUALS(v.Entry(3), -1.5);
  }

  // Check that modifying one component has no side effect on the other
  // components
  void testComponentAssignmentNoSideEffect(void) {
    JSBSim::FGColumnVector3 v;
    JSBSim::FGColumnVector3 v1(1., 0., -2.);

    for (unsigned int i=1; i <= 3; i++) {
      v = v1;
      double x = v1(i) + 1.0;
      v(i) = x;
      for (unsigned int j=1; j <= 3; j++) {
        if (i != j) {
          TS_ASSERT_EQUALS(v(j), v1(j));
        }
        else {
          TS_ASSERT_EQUALS(v(j), x);
        }
      }
    }
  }

  // Check that v1 and v2 are distinct copies i.e. that v1 and v2 does not
  // point to the same memory location
  void testCopyConstructorDistinctInstances(void) {
    JSBSim::FGColumnVector3 v1(1., 0., -2.);
    JSBSim::FGColumnVector3 v2(v1);

    v1(1) = 5.0;
    TS_ASSERT_EQUALS(v2(1), 1.0); // v2[1] must remain unchanged
    v1(2) = 5.0;
    TS_ASSERT_EQUALS(v2(2), 0.0); // v2[2] must remain unchanged
    v1(3) = 5.0;
    TS_ASSERT_EQUALS(v2(3), -2.0); // v2[3] must remain unchanged
  }

  void testAssignment(void) {
    JSBSim::FGColumnVector3 v;
    JSBSim::FGColumnVector3 v1(1., 0., -2.);

    v = v1;

    TS_ASSERT_EQUALS(v(1), 1.0);
    TS_ASSERT_EQUALS(v(2), 0.0);
    TS_ASSERT_EQUALS(v(3), -2.0);

    // Verify that the operand is not modified
    TS_ASSERT_EQUALS(v1(1), 1.0);
    TS_ASSERT_EQUALS(v1(2), 0.0);
    TS_ASSERT_EQUALS(v1(3), -2.0);
  }

  // Check that the assignment is building distinct copies
  void testAssignmentDistinctInstances(void) {
    JSBSim::FGColumnVector3 v;
    JSBSim::FGColumnVector3 v1(1., 0., -2.);

    v = v1;

    v1(1) = -8.0;
    TS_ASSERT_EQUALS(v(1), 1.0);
    v1(2) = -8.0;
    TS_ASSERT_EQUALS(v(2), 0.0);
    v1(3) = -8.0;
    TS_ASSERT_EQUALS(v(3), -2.0);
  }

  // Check the assignment via an initializer list
  void testAssignmentInitializerList(void) {
    JSBSim::FGColumnVector3 v;

    TS_ASSERT_EQUALS(v(1), 0.0);
    TS_ASSERT_EQUALS(v(2), 0.0);
    TS_ASSERT_EQUALS(v(3), 0.0);

    v = { 1.0, 2.0, -3.0 };

    TS_ASSERT_EQUALS(v(1), 1.0);
    TS_ASSERT_EQUALS(v(2), 2.0);
    TS_ASSERT_EQUALS(v(3), -3.0);
  }

  void testEquality(void) {
    JSBSim::FGColumnVector3 v(1., 0., -2.);
    JSBSim::FGColumnVector3 v1(v);

    TS_ASSERT(v == v1);
  }

  // Check that vectors differing by all their components are found inequal
  void testInequality(void) {
    const JSBSim::FGColumnVector3 v;
    const JSBSim::FGColumnVector3 v1(1., 0.5, -2.);

    TS_ASSERT(v != v1);
  }

  // Check that vectors differing by only one component are found inequal
  void testInequalityOneComp(void) {
    JSBSim::FGColumnVector3 v;
    const JSBSim::FGColumnVector3 v1(1., 0., -2.);

    for (unsigned int i=1; i <= 3; i++) {
      v = v1;
      v(i) = v1.Entry(i) + 1.0;
      TS_ASSERT(v != v1);
    }
  }

  // Check that vectors differing by two components are found inequal
  void testInequalityTwoComp(void) {
    JSBSim::FGColumnVector3 v;
    const JSBSim::FGColumnVector3 v1(1., 0., -2.);

    for (unsigned int i=1; i <= 3; i++) {
      for (unsigned int j=1; j <= 3; j++) {
        if (i == j)
          v(i) = v1.Entry(i);
        else
          v(j) = v1.Entry(j) + 1.0;
      }

      TS_ASSERT(v != v1);
    }
  }

  void testInitMatrix(void) {
    const JSBSim::FGColumnVector3 v0;
    JSBSim::FGColumnVector3 v1(1., 0.5, -2.);

    v1.InitMatrix();
    TS_ASSERT_EQUALS(v1, v0);
  }

  void testInitMatrixOneParam(void) {
    JSBSim::FGColumnVector3 v;

    v.InitMatrix(1.0);
    TS_ASSERT_EQUALS(v(1), 1.0);
    TS_ASSERT_EQUALS(v(2), 1.0);
    TS_ASSERT_EQUALS(v(3), 1.0);
  }

  void testInitMatrixThreeParams(void) {
    JSBSim::FGColumnVector3 v;

    v.InitMatrix(-1.0, 2.0, 0.5);
    TS_ASSERT_EQUALS(v(1), -1.0);
    TS_ASSERT_EQUALS(v(2), 2.0);
    TS_ASSERT_EQUALS(v(3), 0.5);
  }

  void testLhsScalarMultiplication(void) {
    JSBSim::FGColumnVector3 v(1., 0.5, -2.);
    JSBSim::FGColumnVector3 v_twice = 2.0 * v;

    TS_ASSERT_EQUALS(v_twice(1), 2.0);
    TS_ASSERT_EQUALS(v_twice(2), 1.0);
    TS_ASSERT_EQUALS(v_twice(3), -4.0);

    // Verify that the operand is not modified
    TS_ASSERT_EQUALS(v(1), 1.0);
    TS_ASSERT_EQUALS(v(2), 0.5);
    TS_ASSERT_EQUALS(v(3), -2.0);
  }

  // Check the scalar multiplication when the operand is on both sides of the
  // assignment
  void testLhsScalarMultiplicationSelfRef(void) {
    JSBSim::FGColumnVector3 v(1., 0.5, -2.);
    v = 2.0 * v;

    TS_ASSERT_EQUALS(v(1), 2.0);
    TS_ASSERT_EQUALS(v(2), 1.0);
    TS_ASSERT_EQUALS(v(3), -4.0);
  }

  void testRhsScalarMultiplication(void) {
    JSBSim::FGColumnVector3 v(1., 0.5, -2.);
    JSBSim::FGColumnVector3 v_twice = v * 2.0;

    TS_ASSERT_EQUALS(v_twice(1), 2.0);
    TS_ASSERT_EQUALS(v_twice(2), 1.0);
    TS_ASSERT_EQUALS(v_twice(3), -4.0);

    // Verify that the operand is not modified
    TS_ASSERT_EQUALS(v(1), 1.0);
    TS_ASSERT_EQUALS(v(2), 0.5);
    TS_ASSERT_EQUALS(v(3), -2.0);
  }

  // Check the scalar multiplication when the operand is on both sides of the
  // assignment
  void testRhsScalarMultiplicationSelfRef(void) {
    JSBSim::FGColumnVector3 v(1., 0.5, -2.);
    v = v * 2.0;

    TS_ASSERT_EQUALS(v(1), 2.0);
    TS_ASSERT_EQUALS(v(2), 1.0);
    TS_ASSERT_EQUALS(v(3), -4.0);
  }

  void testScalarDivision(void) {
    JSBSim::FGColumnVector3 v(1., 0.5, -2.);
    JSBSim::FGColumnVector3 v_half = v / 2.0;

    TS_ASSERT_EQUALS(v_half, 0.5 * v);

    // Verify that the operand is not modified
    TS_ASSERT_EQUALS(v(1), 1.0);
    TS_ASSERT_EQUALS(v(2), 0.5);
    TS_ASSERT_EQUALS(v(3), -2.0);
  }

  // Check the scalar division when the operand is on both sides of the
  // assignment
  void testScalarDivisionSelfRef(void) {
    JSBSim::FGColumnVector3 v(1., 0.5, -2.);

    v = v / 2.0;

    TS_ASSERT_EQUALS(v(1), 0.5);
    TS_ASSERT_EQUALS(v(2), 0.25);
    TS_ASSERT_EQUALS(v(3), -1.0);
  }

  void testDivisionByZero(void) {
    const JSBSim::FGColumnVector3 v0;
    const JSBSim::FGColumnVector3 v(1., 0., -2.);

    TS_ASSERT_EQUALS(v / 0.0, v0);
  }

  void testAddition(void) {
    JSBSim::FGColumnVector3 v1(1., 0.5, -2.);
    JSBSim::FGColumnVector3 v2(0., 1., 2.);
    JSBSim::FGColumnVector3 v = v1 + v2;

    TS_ASSERT_EQUALS(v(1), 1.0);
    TS_ASSERT_EQUALS(v(2), 1.5);
    TS_ASSERT_EQUALS(v(3), 0.0);

    // Verify that the operands are not modified
    TS_ASSERT_EQUALS(v1(1), 1.0);
    TS_ASSERT_EQUALS(v1(2), 0.5);
    TS_ASSERT_EQUALS(v1(3), -2.0);

    TS_ASSERT_EQUALS(v2(1), 0.0);
    TS_ASSERT_EQUALS(v2(2), 1.0);
    TS_ASSERT_EQUALS(v2(3), 2.0);
  }

  // Check the addition when the operand is on both sides of the assignment
  void testAdditionSelfRefLeft(void) {
    JSBSim::FGColumnVector3 v1(1., 0.5, -2.);
    JSBSim::FGColumnVector3 v2(0., 1., 2.);
    v1 = v1 + v2;

    TS_ASSERT_EQUALS(v1(1), 1.0);
    TS_ASSERT_EQUALS(v1(2), 1.5);
    TS_ASSERT_EQUALS(v1(3), 0.0);

    // Verify that the other operand is not modified
    TS_ASSERT_EQUALS(v2(1), 0.0);
    TS_ASSERT_EQUALS(v2(2), 1.0);
    TS_ASSERT_EQUALS(v2(3), 2.0);
  }

  // Check the addition when the operand is on both sides of the assignment
  void testAdditionSelfRefRight(void) {
    JSBSim::FGColumnVector3 v1(1., 0.5, -2.);
    JSBSim::FGColumnVector3 v2(0., 1., 2.);
    v1 = v2 + v1;

    TS_ASSERT_EQUALS(v1(1), 1.0);
    TS_ASSERT_EQUALS(v1(2), 1.5);
    TS_ASSERT_EQUALS(v1(3), 0.0);

    // Verify that the other operand is not modified
    TS_ASSERT_EQUALS(v2(1), 0.0);
    TS_ASSERT_EQUALS(v2(2), 1.0);
    TS_ASSERT_EQUALS(v2(3), 2.0);
  }

  // Check the addition when the operand is on both sides of the assignment
  void testAdditionSelfSelf(void) {
    JSBSim::FGColumnVector3 v1(1., 0.5, -2.);
    v1 = v1 + v1;

    TS_ASSERT_EQUALS(v1(1), 2.0);
    TS_ASSERT_EQUALS(v1(2), 1.0);
    TS_ASSERT_EQUALS(v1(3), -4.0);
  }

  void testSubtraction(void) {
    JSBSim::FGColumnVector3 v1(1., 0.5, -2.);
    JSBSim::FGColumnVector3 v2(0., 1., 2.);
    JSBSim::FGColumnVector3 v = v1 - v2;

    TS_ASSERT_EQUALS(v(1), 1.0);
    TS_ASSERT_EQUALS(v(2), -0.5);
    TS_ASSERT_EQUALS(v(3), -4.0);

    // Verify that the operands are not modified
    TS_ASSERT_EQUALS(v1(1), 1.0);
    TS_ASSERT_EQUALS(v1(2), 0.5);
    TS_ASSERT_EQUALS(v1(3), -2.0);

    TS_ASSERT_EQUALS(v2(1), 0.0);
    TS_ASSERT_EQUALS(v2(2), 1.0);
    TS_ASSERT_EQUALS(v2(3), 2.0);
  }

  // Check the subtraction when the operand is on both sides of the assignment
  void testSubtractionSelfLeft(void) {
    JSBSim::FGColumnVector3 v1(1., 0.5, -2.);
    JSBSim::FGColumnVector3 v2(0., 1., 2.);
    v1 = v1 - v2;

    TS_ASSERT_EQUALS(v1(1), 1.0);
    TS_ASSERT_EQUALS(v1(2), -0.5);
    TS_ASSERT_EQUALS(v1(3), -4.0);

    // Verify that the operands are not modified
    TS_ASSERT_EQUALS(v2(1), 0.0);
    TS_ASSERT_EQUALS(v2(2), 1.0);
    TS_ASSERT_EQUALS(v2(3), 2.0);
  }

  // Check the subtraction when the operand is on both sides of the assignment
  void testSubtractionSelfRight(void) {
    JSBSim::FGColumnVector3 v1(1., 0.5, -2.);
    JSBSim::FGColumnVector3 v2(0., 1., 2.);
    v1 = v2 - v1;

    TS_ASSERT_EQUALS(v1(1), -1.0);
    TS_ASSERT_EQUALS(v1(2), 0.5);
    TS_ASSERT_EQUALS(v1(3), 4.0);

    // Verify that the operands are not modified
    TS_ASSERT_EQUALS(v2(1), 0.0);
    TS_ASSERT_EQUALS(v2(2), 1.0);
    TS_ASSERT_EQUALS(v2(3), 2.0);
  }

  // Check the subtraction when the operand is on both sides of the assignment
  void testSubtractionSelfSelf(void) {
    JSBSim::FGColumnVector3 v1(1., 0.5, -2.);
    v1 = v1 - v1;

    TS_ASSERT_EQUALS(v1(1), 0.0);
    TS_ASSERT_EQUALS(v1(2), 0.0);
    TS_ASSERT_EQUALS(v1(3), 0.0);
  }

  void testAdditionAssignment(void) {
    JSBSim::FGColumnVector3 v(1., 0.5, -2.);
    JSBSim::FGColumnVector3 v2(0., 1., 2.);

    v += v2;

    TS_ASSERT_EQUALS(v(1), 1.0);
    TS_ASSERT_EQUALS(v(2), 1.5);
    TS_ASSERT_EQUALS(v(3), 0.0);

    // Verify that the operand is not modified
    TS_ASSERT_EQUALS(v2(1), 0.0);
    TS_ASSERT_EQUALS(v2(2), 1.0);
    TS_ASSERT_EQUALS(v2(3), 2.0);
  }

  // Check the operation when the operand is on both sides of the assignment
  void testAdditionAssignmentSelf(void) {
    JSBSim::FGColumnVector3 v(1., 0.5, -2.);

    v += v;

    TS_ASSERT_EQUALS(v(1), 2.0);
    TS_ASSERT_EQUALS(v(2), 1.0);
    TS_ASSERT_EQUALS(v(3), -4.0);
  }

  void testSubtractionAssignment(void) {
    JSBSim::FGColumnVector3 v(1., 0.5, -2.);
    JSBSim::FGColumnVector3 v2(0., 1., 2.);

    v -= v2;

    TS_ASSERT_EQUALS(v(1), 1.0);
    TS_ASSERT_EQUALS(v(2), -0.5);
    TS_ASSERT_EQUALS(v(3), -4.0);

    // Verify that the operand is not modified
    TS_ASSERT_EQUALS(v2(1), 0.0);
    TS_ASSERT_EQUALS(v2(2), 1.0);
    TS_ASSERT_EQUALS(v2(3), 2.0);
  }

  // Check the operation when the operand is on both sides of the assignment
  void testSubtractionAssignmentSelf(void) {
    JSBSim::FGColumnVector3 v(1., 0.5, -2.);

    v -= v;

    TS_ASSERT_EQUALS(v(1), 0.0);
    TS_ASSERT_EQUALS(v(2), 0.0);
    TS_ASSERT_EQUALS(v(3), 0.0);
  }

  void testScalarMultiplicationAssignment(void) {
    JSBSim::FGColumnVector3 v(1., 0.5, -2.);

    v *= 2.0;

    TS_ASSERT_EQUALS(v(1), 2.0);
    TS_ASSERT_EQUALS(v(2), 1.0);
    TS_ASSERT_EQUALS(v(3), -4.0);
  }

  void testScalarDivisionAssignment(void) {
    JSBSim::FGColumnVector3 v(1., 0.5, -2.);

    v /= 2.0;

    TS_ASSERT_EQUALS(v(1), 0.5);
    TS_ASSERT_EQUALS(v(2), 0.25);
    TS_ASSERT_EQUALS(v(3), -1.0);
  }

  void testDivisionByZeroAssignment(void) {
    JSBSim::FGColumnVector3 v(1., 0.5, -2.);

    v /= 0.0;

    TS_ASSERT_EQUALS(v(1), 1.0);
    TS_ASSERT_EQUALS(v(2), 0.5);
    TS_ASSERT_EQUALS(v(3), -2.0);
  }

  void testDotProduct(void) {
    JSBSim::FGColumnVector3 v(1.0, -2.0, 0.5);
    JSBSim::FGColumnVector3 X(1.0,0.0,0.0);
    JSBSim::FGColumnVector3 Y(0.0,1.0,0.0);
    JSBSim::FGColumnVector3 Z(0.0,0.0,1.0);

    TS_ASSERT_EQUALS(DotProduct(v,v), 5.25);
    TS_ASSERT_EQUALS(DotProduct(X, Y), 0.0);
    TS_ASSERT_EQUALS(DotProduct(Y, Z), 0.0);
    TS_ASSERT_EQUALS(DotProduct(X, Z), 0.0);

    // Verify that the operands are not modified
    TS_ASSERT_EQUALS(v(1), 1.0);
    TS_ASSERT_EQUALS(v(2), -2.0);
    TS_ASSERT_EQUALS(v(3), 0.5);

    TS_ASSERT_EQUALS(X(1), 1.0);
    TS_ASSERT_EQUALS(X(2), 0.0);
    TS_ASSERT_EQUALS(X(3), 0.0);

    TS_ASSERT_EQUALS(Y(1), 0.0);
    TS_ASSERT_EQUALS(Y(2), 1.0);
    TS_ASSERT_EQUALS(Y(3), 0.0);

    TS_ASSERT_EQUALS(Z(1), 0.0);
    TS_ASSERT_EQUALS(Z(2), 0.0);
    TS_ASSERT_EQUALS(Z(3), 1.0);
  }

  void testCrossProduct(void) {
    JSBSim::FGColumnVector3 v0;
    JSBSim::FGColumnVector3 X(1.0,0.0,0.0);
    JSBSim::FGColumnVector3 Y(0.0,1.0,0.0);
    JSBSim::FGColumnVector3 Z(0.0,0.0,1.0);

    TS_ASSERT_EQUALS(X * Y, Z);
    TS_ASSERT_EQUALS(Y * X, -1.0 * Z);
    TS_ASSERT_EQUALS(Y * Z, X);
    TS_ASSERT_EQUALS(Z * Y, -1.0 * X);
    TS_ASSERT_EQUALS(Z * X, Y);
    TS_ASSERT_EQUALS(X * Z, -1.0 * Y);
    TS_ASSERT_EQUALS(X * X, v0);
    TS_ASSERT_EQUALS(Y * Y, v0);
    TS_ASSERT_EQUALS(Z * Z, v0);

    // Verify that the operands are not modified
    TS_ASSERT_EQUALS(v0(1), 0.0);
    TS_ASSERT_EQUALS(v0(2), 0.0);
    TS_ASSERT_EQUALS(v0(3), 0.0);

    TS_ASSERT_EQUALS(X(1), 1.0);
    TS_ASSERT_EQUALS(X(2), 0.0);
    TS_ASSERT_EQUALS(X(3), 0.0);

    TS_ASSERT_EQUALS(Y(1), 0.0);
    TS_ASSERT_EQUALS(Y(2), 1.0);
    TS_ASSERT_EQUALS(Y(3), 0.0);

    TS_ASSERT_EQUALS(Z(1), 0.0);
    TS_ASSERT_EQUALS(Z(2), 0.0);
    TS_ASSERT_EQUALS(Z(3), 1.0);
  }

  void testMagnitude(void) {
    JSBSim::FGColumnVector3 v0;
    JSBSim::FGColumnVector3 v(3.0, 4.0, 0.0);

    TS_ASSERT_EQUALS(v0.Magnitude(), 0.0);
    TS_ASSERT_EQUALS(v.Magnitude(1,3), 3.0);
    TS_ASSERT_EQUALS(v.Magnitude(2,3), 4.0);
    TS_ASSERT_EQUALS(v.Magnitude(1,2), 5.0);
    TS_ASSERT_EQUALS(v.Magnitude(), 5.0);
    TS_ASSERT_EQUALS(DotProduct(v,v), v.Magnitude() * v.Magnitude());

    // Verify that the operands are not modified
    TS_ASSERT_EQUALS(v0(1), 0.0);
    TS_ASSERT_EQUALS(v0(2), 0.0);
    TS_ASSERT_EQUALS(v0(3), 0.0);

    TS_ASSERT_EQUALS(v(1), 3.0);
    TS_ASSERT_EQUALS(v(2), 4.0);
    TS_ASSERT_EQUALS(v(3), 0.0);
  }

  void testNormalize(void) {
    JSBSim::FGColumnVector3 v0;
    JSBSim::FGColumnVector3 v(3.0, 4.0, 0.0);

    v0.Normalize();
    TS_ASSERT_EQUALS(v0(1), 0.0);
    TS_ASSERT_EQUALS(v0(2), 0.0);
    TS_ASSERT_EQUALS(v0(3), 0.0);

    v.Normalize();
    TS_ASSERT_DELTA(v(1), 0.6, 1E-9);
    TS_ASSERT_DELTA(v(2), 0.8, 1E-9);
    TS_ASSERT_EQUALS(v(3), 0.0);
  }

  void testOutput(void) {
    JSBSim::FGColumnVector3 v1(1., 0., -2.);
    std::string s = v1.Dump(" , ");

    TS_ASSERT_EQUALS(s, std::string("1 , 0 , -2"));
    std::ostringstream os;
    os << v1;
    TS_ASSERT_EQUALS(os.str(), std::string("1 , 0 , -2"));

    // Verify that the operands are not modified
    TS_ASSERT_EQUALS(v1(1), 1.0);
    TS_ASSERT_EQUALS(v1(2), 0.0);
    TS_ASSERT_EQUALS(v1(3), -2.0);
  }

  // Edge case tests for normalization
  void testNormalizeVerySmallVector(void) {
    // GIVEN: A very small vector that should still normalize correctly
    JSBSim::FGColumnVector3 v(1e-10, 0.0, 0.0);

    // WHEN: Normalizing
    v.Normalize();

    // THEN: Result should be a unit vector along x-axis
    TS_ASSERT_DELTA(v(1), 1.0, 1e-9);
    TS_ASSERT_EQUALS(v(2), 0.0);
    TS_ASSERT_EQUALS(v(3), 0.0);
  }

  void testNormalizeLargeVector(void) {
    // GIVEN: A large vector
    JSBSim::FGColumnVector3 v(1e10, 0.0, 0.0);

    // WHEN: Normalizing
    v.Normalize();

    // THEN: Result should be a unit vector along x-axis
    TS_ASSERT_DELTA(v(1), 1.0, 1e-9);
    TS_ASSERT_DELTA(v(2), 0.0, 1e-9);
    TS_ASSERT_DELTA(v(3), 0.0, 1e-9);
  }

  void testNormalizeAlreadyNormalized(void) {
    // GIVEN: An already normalized vector
    JSBSim::FGColumnVector3 v(0.6, 0.8, 0.0);

    // WHEN: Normalizing
    v.Normalize();

    // THEN: Result should remain unchanged
    TS_ASSERT_DELTA(v(1), 0.6, 1e-9);
    TS_ASSERT_DELTA(v(2), 0.8, 1e-9);
    TS_ASSERT_EQUALS(v(3), 0.0);
    TS_ASSERT_DELTA(v.Magnitude(), 1.0, 1e-9);
  }

  void testNormalizeNegativeComponents(void) {
    // GIVEN: A vector with negative components
    JSBSim::FGColumnVector3 v(-3.0, -4.0, 0.0);

    // WHEN: Normalizing
    v.Normalize();

    // THEN: Components should be negative but unit length
    TS_ASSERT_DELTA(v(1), -0.6, 1e-9);
    TS_ASSERT_DELTA(v(2), -0.8, 1e-9);
    TS_ASSERT_EQUALS(v(3), 0.0);
    TS_ASSERT_DELTA(v.Magnitude(), 1.0, 1e-9);
  }

  void testNormalize3DVector(void) {
    // GIVEN: A 3D vector with all components non-zero
    JSBSim::FGColumnVector3 v(1.0, 2.0, 2.0);  // Magnitude = 3

    // WHEN: Normalizing
    v.Normalize();

    // THEN: Each component should be scaled correctly
    TS_ASSERT_DELTA(v(1), 1.0/3.0, 1e-9);
    TS_ASSERT_DELTA(v(2), 2.0/3.0, 1e-9);
    TS_ASSERT_DELTA(v(3), 2.0/3.0, 1e-9);
    TS_ASSERT_DELTA(v.Magnitude(), 1.0, 1e-9);
  }

  // Edge case tests for magnitude
  void testMagnitudeSingleAxisVectors(void) {
    // GIVEN: Unit vectors along each axis
    JSBSim::FGColumnVector3 vx(5.0, 0.0, 0.0);
    JSBSim::FGColumnVector3 vy(0.0, 7.0, 0.0);
    JSBSim::FGColumnVector3 vz(0.0, 0.0, 3.0);

    // THEN: Magnitude should equal the absolute value of the non-zero component
    TS_ASSERT_EQUALS(vx.Magnitude(), 5.0);
    TS_ASSERT_EQUALS(vy.Magnitude(), 7.0);
    TS_ASSERT_EQUALS(vz.Magnitude(), 3.0);
  }

  void testMagnitudeNegativeComponents(void) {
    // GIVEN: A vector with negative components
    JSBSim::FGColumnVector3 v(-3.0, -4.0, 0.0);

    // THEN: Magnitude should still be 5 (positive)
    TS_ASSERT_EQUALS(v.Magnitude(), 5.0);
  }

  // Edge case tests for cross product
  void testCrossProductParallelVectors(void) {
    // GIVEN: Parallel vectors (cross product should be zero)
    JSBSim::FGColumnVector3 v1(1.0, 2.0, 3.0);
    JSBSim::FGColumnVector3 v2(2.0, 4.0, 6.0);  // v2 = 2*v1

    // WHEN: Computing cross product
    JSBSim::FGColumnVector3 result = v1 * v2;

    // THEN: Result should be zero vector
    TS_ASSERT_DELTA(result(1), 0.0, 1e-9);
    TS_ASSERT_DELTA(result(2), 0.0, 1e-9);
    TS_ASSERT_DELTA(result(3), 0.0, 1e-9);
  }

  void testCrossProductAntiParallelVectors(void) {
    // GIVEN: Anti-parallel vectors (cross product should be zero)
    JSBSim::FGColumnVector3 v1(1.0, 2.0, 3.0);
    JSBSim::FGColumnVector3 v2(-1.0, -2.0, -3.0);  // v2 = -v1

    // WHEN: Computing cross product
    JSBSim::FGColumnVector3 result = v1 * v2;

    // THEN: Result should be zero vector
    TS_ASSERT_DELTA(result(1), 0.0, 1e-9);
    TS_ASSERT_DELTA(result(2), 0.0, 1e-9);
    TS_ASSERT_DELTA(result(3), 0.0, 1e-9);
  }

  // Edge case tests for dot product
  void testDotProductPerpendicular(void) {
    // GIVEN: Perpendicular vectors
    JSBSim::FGColumnVector3 v1(1.0, 0.0, 0.0);
    JSBSim::FGColumnVector3 v2(0.0, 1.0, 0.0);

    // THEN: Dot product should be zero
    TS_ASSERT_EQUALS(DotProduct(v1, v2), 0.0);
  }

  void testDotProductSameDirection(void) {
    // GIVEN: Vectors in the same direction
    JSBSim::FGColumnVector3 v1(1.0, 2.0, 3.0);
    JSBSim::FGColumnVector3 v2(2.0, 4.0, 6.0);

    // THEN: Dot product should equal |v1| * |v2|
    double expected = v1.Magnitude() * v2.Magnitude();
    TS_ASSERT_DELTA(DotProduct(v1, v2), expected, 1e-9);
  }

  void testDotProductOppositeDirection(void) {
    // GIVEN: Vectors in opposite directions
    JSBSim::FGColumnVector3 v1(1.0, 2.0, 3.0);
    JSBSim::FGColumnVector3 v2(-1.0, -2.0, -3.0);

    // THEN: Dot product should equal -|v1| * |v2|
    double expected = -v1.Magnitude() * v2.Magnitude();
    TS_ASSERT_DELTA(DotProduct(v1, v2), expected, 1e-9);
  }

  // Numerical precision edge cases
  void testScalarMultiplicationByZero(void) {
    // GIVEN: A non-zero vector
    JSBSim::FGColumnVector3 v(1.0, 2.0, 3.0);

    // WHEN: Multiplying by zero
    JSBSim::FGColumnVector3 result = 0.0 * v;

    // THEN: Result should be zero vector
    TS_ASSERT_EQUALS(result(1), 0.0);
    TS_ASSERT_EQUALS(result(2), 0.0);
    TS_ASSERT_EQUALS(result(3), 0.0);
  }

  void testScalarMultiplicationByNegativeOne(void) {
    // GIVEN: A vector
    JSBSim::FGColumnVector3 v(1.0, 2.0, 3.0);

    // WHEN: Multiplying by -1
    JSBSim::FGColumnVector3 result = -1.0 * v;

    // THEN: Result should be negated vector
    TS_ASSERT_EQUALS(result(1), -1.0);
    TS_ASSERT_EQUALS(result(2), -2.0);
    TS_ASSERT_EQUALS(result(3), -3.0);
  }

  void testMagnitudePartialComponents(void) {
    // GIVEN: A 3D vector v = (3, 4, 12)
    JSBSim::FGColumnVector3 v(3.0, 4.0, 12.0);  // Full magnitude = 13

    // THEN: Partial magnitudes (using only components idx1 and idx2) should be correct
    TS_ASSERT_EQUALS(v.Magnitude(1, 2), 5.0);   // sqrt(3^2 + 4^2) = 5
    TS_ASSERT_DELTA(v.Magnitude(1, 3), sqrt(9.0 + 144.0), 1e-9);   // sqrt(3^2 + 12^2)
    TS_ASSERT_DELTA(v.Magnitude(2, 3), sqrt(16.0 + 144.0), 1e-9);  // sqrt(4^2 + 12^2)
    TS_ASSERT_EQUALS(v.Magnitude(), 13.0);  // Full magnitude = 13
  }
};
