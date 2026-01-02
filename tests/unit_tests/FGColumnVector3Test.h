#include <cxxtest/TestSuite.h>
#include <math/FGColumnVector3.h>
#include <FGFDMExec.h>
#include <models/FGPropagate.h>
#include <models/FGAuxiliary.h>
#include <models/FGAccelerations.h>
#include <models/FGAerodynamics.h>
#include <initialization/FGInitialCondition.h>

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

  /***************************************************************************
   * Unary Negation Tests
   ***************************************************************************/

  void testUnaryNegation(void) {
    JSBSim::FGColumnVector3 v(1.0, -2.0, 3.0);
    JSBSim::FGColumnVector3 neg = -1.0 * v;

    TS_ASSERT_EQUALS(neg(1), -1.0);
    TS_ASSERT_EQUALS(neg(2), 2.0);
    TS_ASSERT_EQUALS(neg(3), -3.0);
  }

  void testDoubleNegation(void) {
    JSBSim::FGColumnVector3 v(1.0, -2.0, 3.0);
    JSBSim::FGColumnVector3 doubleNeg = -1.0 * (-1.0 * v);

    TS_ASSERT_EQUALS(doubleNeg(1), v(1));
    TS_ASSERT_EQUALS(doubleNeg(2), v(2));
    TS_ASSERT_EQUALS(doubleNeg(3), v(3));
  }

  void testNegationZeroVector(void) {
    JSBSim::FGColumnVector3 v(0.0, 0.0, 0.0);
    JSBSim::FGColumnVector3 neg = -1.0 * v;

    TS_ASSERT_EQUALS(neg(1), 0.0);
    TS_ASSERT_EQUALS(neg(2), 0.0);
    TS_ASSERT_EQUALS(neg(3), 0.0);
  }

  /***************************************************************************
   * Algebraic Property Tests
   ***************************************************************************/

  void testVectorAdditionCommutative(void) {
    JSBSim::FGColumnVector3 v1(1.0, 2.0, 3.0);
    JSBSim::FGColumnVector3 v2(4.0, 5.0, 6.0);

    JSBSim::FGColumnVector3 sum1 = v1 + v2;
    JSBSim::FGColumnVector3 sum2 = v2 + v1;

    TS_ASSERT_EQUALS(sum1, sum2);
  }

  void testVectorAdditionAssociative(void) {
    JSBSim::FGColumnVector3 v1(1.0, 2.0, 3.0);
    JSBSim::FGColumnVector3 v2(4.0, 5.0, 6.0);
    JSBSim::FGColumnVector3 v3(7.0, 8.0, 9.0);

    JSBSim::FGColumnVector3 left = (v1 + v2) + v3;
    JSBSim::FGColumnVector3 right = v1 + (v2 + v3);

    TS_ASSERT_DELTA(left(1), right(1), 1e-12);
    TS_ASSERT_DELTA(left(2), right(2), 1e-12);
    TS_ASSERT_DELTA(left(3), right(3), 1e-12);
  }

  void testDotProductCommutative(void) {
    JSBSim::FGColumnVector3 v1(1.0, 2.0, 3.0);
    JSBSim::FGColumnVector3 v2(4.0, 5.0, 6.0);

    TS_ASSERT_EQUALS(DotProduct(v1, v2), DotProduct(v2, v1));
  }

  void testDotProductDistributive(void) {
    JSBSim::FGColumnVector3 v1(1.0, 2.0, 3.0);
    JSBSim::FGColumnVector3 v2(4.0, 5.0, 6.0);
    JSBSim::FGColumnVector3 v3(7.0, 8.0, 9.0);

    double left = DotProduct(v1, v2 + v3);
    double right = DotProduct(v1, v2) + DotProduct(v1, v3);

    TS_ASSERT_DELTA(left, right, 1e-12);
  }

  void testCrossProductAnticommutative(void) {
    JSBSim::FGColumnVector3 v1(1.0, 2.0, 3.0);
    JSBSim::FGColumnVector3 v2(4.0, 5.0, 6.0);

    JSBSim::FGColumnVector3 cross1 = v1 * v2;
    JSBSim::FGColumnVector3 cross2 = v2 * v1;

    TS_ASSERT_DELTA(cross1(1), -cross2(1), 1e-12);
    TS_ASSERT_DELTA(cross1(2), -cross2(2), 1e-12);
    TS_ASSERT_DELTA(cross1(3), -cross2(3), 1e-12);
  }

  void testCrossProductDistributive(void) {
    JSBSim::FGColumnVector3 v1(1.0, 2.0, 3.0);
    JSBSim::FGColumnVector3 v2(4.0, 5.0, 6.0);
    JSBSim::FGColumnVector3 v3(7.0, 8.0, 9.0);

    JSBSim::FGColumnVector3 left = v1 * (v2 + v3);
    JSBSim::FGColumnVector3 right = (v1 * v2) + (v1 * v3);

    TS_ASSERT_DELTA(left(1), right(1), 1e-12);
    TS_ASSERT_DELTA(left(2), right(2), 1e-12);
    TS_ASSERT_DELTA(left(3), right(3), 1e-12);
  }

  /***************************************************************************
   * Cross Product Magnitude and Properties
   ***************************************************************************/

  void testCrossProductMagnitude(void) {
    // |a × b| = |a||b|sin(θ) - for perpendicular vectors, sin(90°) = 1
    JSBSim::FGColumnVector3 v1(3.0, 0.0, 0.0);
    JSBSim::FGColumnVector3 v2(0.0, 4.0, 0.0);

    JSBSim::FGColumnVector3 cross = v1 * v2;

    // For perpendicular vectors: |a × b| = |a| * |b|
    double expectedMag = v1.Magnitude() * v2.Magnitude();
    TS_ASSERT_DELTA(cross.Magnitude(), expectedMag, 1e-9);
  }

  void testCrossProductPerpendicularity(void) {
    // Cross product is perpendicular to both input vectors
    JSBSim::FGColumnVector3 v1(1.0, 2.0, 3.0);
    JSBSim::FGColumnVector3 v2(4.0, 5.0, 6.0);

    JSBSim::FGColumnVector3 cross = v1 * v2;

    // Dot product with each input should be zero
    TS_ASSERT_DELTA(DotProduct(cross, v1), 0.0, 1e-9);
    TS_ASSERT_DELTA(DotProduct(cross, v2), 0.0, 1e-9);
  }

  void testCrossProductWithZeroVector(void) {
    JSBSim::FGColumnVector3 v(1.0, 2.0, 3.0);
    JSBSim::FGColumnVector3 zero(0.0, 0.0, 0.0);

    JSBSim::FGColumnVector3 cross = v * zero;

    TS_ASSERT_EQUALS(cross(1), 0.0);
    TS_ASSERT_EQUALS(cross(2), 0.0);
    TS_ASSERT_EQUALS(cross(3), 0.0);
  }

  /***************************************************************************
   * Scalar Triple Product Tests
   ***************************************************************************/

  void testScalarTripleProduct(void) {
    // a · (b × c) = volume of parallelepiped
    JSBSim::FGColumnVector3 a(1.0, 0.0, 0.0);
    JSBSim::FGColumnVector3 b(0.0, 1.0, 0.0);
    JSBSim::FGColumnVector3 c(0.0, 0.0, 1.0);

    double volume = DotProduct(a, b * c);

    // For unit cube, volume = 1
    TS_ASSERT_DELTA(volume, 1.0, 1e-9);
  }

  void testScalarTripleProductCyclicProperty(void) {
    // a · (b × c) = b · (c × a) = c · (a × b)
    JSBSim::FGColumnVector3 a(1.0, 2.0, 3.0);
    JSBSim::FGColumnVector3 b(4.0, 5.0, 6.0);
    JSBSim::FGColumnVector3 c(7.0, 8.0, 10.0);  // Not coplanar

    double v1 = DotProduct(a, b * c);
    double v2 = DotProduct(b, c * a);
    double v3 = DotProduct(c, a * b);

    TS_ASSERT_DELTA(v1, v2, 1e-9);
    TS_ASSERT_DELTA(v2, v3, 1e-9);
  }

  void testScalarTripleProductCoplanarVectors(void) {
    // Coplanar vectors have zero scalar triple product
    JSBSim::FGColumnVector3 a(1.0, 0.0, 0.0);
    JSBSim::FGColumnVector3 b(0.0, 1.0, 0.0);
    JSBSim::FGColumnVector3 c(1.0, 1.0, 0.0);  // In xy-plane

    double volume = DotProduct(a, b * c);

    TS_ASSERT_DELTA(volume, 0.0, 1e-9);
  }

  /***************************************************************************
   * Linear Combination Tests
   ***************************************************************************/

  void testLinearCombinationBasisVectors(void) {
    JSBSim::FGColumnVector3 X(1.0, 0.0, 0.0);
    JSBSim::FGColumnVector3 Y(0.0, 1.0, 0.0);
    JSBSim::FGColumnVector3 Z(0.0, 0.0, 1.0);

    // Any vector can be expressed as linear combination of basis
    JSBSim::FGColumnVector3 v = 3.0 * X + 4.0 * Y + 5.0 * Z;

    TS_ASSERT_EQUALS(v(1), 3.0);
    TS_ASSERT_EQUALS(v(2), 4.0);
    TS_ASSERT_EQUALS(v(3), 5.0);
  }

  void testLinearCombinationIdentity(void) {
    JSBSim::FGColumnVector3 v(1.0, 2.0, 3.0);

    // v = 1*v + 0*v
    JSBSim::FGColumnVector3 result = 1.0 * v + 0.0 * v;

    TS_ASSERT_EQUALS(result, v);
  }

  void testLinearCombinationZero(void) {
    JSBSim::FGColumnVector3 v(1.0, 2.0, 3.0);

    // 0 = v + (-1)*v
    JSBSim::FGColumnVector3 result = v + (-1.0) * v;

    TS_ASSERT_EQUALS(result(1), 0.0);
    TS_ASSERT_EQUALS(result(2), 0.0);
    TS_ASSERT_EQUALS(result(3), 0.0);
  }

  /***************************************************************************
   * Magnitude Squared (via dot product) Tests
   ***************************************************************************/

  void testMagnitudeSquaredFromDotProduct(void) {
    JSBSim::FGColumnVector3 v(3.0, 4.0, 0.0);

    // |v|² = v · v
    double magSquared = DotProduct(v, v);
    double mag = v.Magnitude();

    TS_ASSERT_DELTA(magSquared, mag * mag, 1e-12);
    TS_ASSERT_EQUALS(magSquared, 25.0);
  }

  void testMagnitudeFromDotProduct(void) {
    JSBSim::FGColumnVector3 v(1.0, 2.0, 2.0);

    // |v| = sqrt(v · v)
    double mag = sqrt(DotProduct(v, v));

    TS_ASSERT_DELTA(mag, v.Magnitude(), 1e-12);
    TS_ASSERT_DELTA(mag, 3.0, 1e-12);
  }

  /***************************************************************************
   * Flight Simulation Specific Tests
   ***************************************************************************/

  void testVelocityMagnitude(void) {
    // Typical aircraft velocity components (ft/s)
    JSBSim::FGColumnVector3 velocity(500.0, 10.0, 5.0);

    double speed = velocity.Magnitude();

    // Speed should be approximately 500 ft/s (slightly more due to side/vertical)
    TS_ASSERT(speed > 500.0);
    TS_ASSERT(speed < 510.0);
  }

  void testAccelerationDecomposition(void) {
    // Given total acceleration
    JSBSim::FGColumnVector3 accel(32.2, 0.0, -5.0);  // ft/s²

    // Decompose into forward and down components
    double forward = accel(1);
    double down = -accel(3);  // Positive down

    TS_ASSERT_EQUALS(forward, 32.2);
    TS_ASSERT_EQUALS(down, 5.0);
  }

  void testMomentArmCrossProduct(void) {
    // Force application point relative to CG
    JSBSim::FGColumnVector3 arm(10.0, 0.0, 0.0);  // 10 ft forward
    // Applied force
    JSBSim::FGColumnVector3 force(0.0, 0.0, -1000.0);  // 1000 lbs down

    // Moment = arm × force
    // (10,0,0) × (0,0,-1000) = (0*(-1000)-0*0, 0*0-10*(-1000), 10*0-0*0) = (0, 10000, 0)
    JSBSim::FGColumnVector3 moment = arm * force;

    // Should produce pitching moment about Y-axis
    TS_ASSERT_DELTA(moment(1), 0.0, 1e-9);       // No roll
    TS_ASSERT_DELTA(moment(2), 10000.0, 1e-9);  // Pitch (nose up from forward force application)
    TS_ASSERT_DELTA(moment(3), 0.0, 1e-9);       // No yaw
  }

  void testWindVectorAddition(void) {
    // Aircraft velocity relative to air
    JSBSim::FGColumnVector3 tas(400.0, 0.0, 0.0);  // True airspeed
    // Wind velocity
    JSBSim::FGColumnVector3 wind(-50.0, 20.0, 0.0);  // Headwind with crosswind

    // Ground speed = TAS + wind
    JSBSim::FGColumnVector3 groundSpeed = tas + wind;

    TS_ASSERT_EQUALS(groundSpeed(1), 350.0);  // Reduced forward
    TS_ASSERT_EQUALS(groundSpeed(2), 20.0);   // Drift
    TS_ASSERT_EQUALS(groundSpeed(3), 0.0);
  }

  /***************************************************************************
   * Edge Cases and Numerical Precision
   ***************************************************************************/

  void testVeryLargeComponents(void) {
    JSBSim::FGColumnVector3 v(1e15, 1e15, 1e15);

    double mag = v.Magnitude();

    TS_ASSERT(std::isfinite(mag));
    TS_ASSERT_DELTA(mag, sqrt(3.0) * 1e15, 1e6);
  }

  void testMixedMagnitudeComponents(void) {
    // Very different scales
    JSBSim::FGColumnVector3 v(1e10, 1.0, 1e-10);

    double mag = v.Magnitude();

    // Should be dominated by largest component
    TS_ASSERT_DELTA(mag, 1e10, 1.0);
  }

  void testMultiplicationChain(void) {
    JSBSim::FGColumnVector3 v(1.0, 2.0, 3.0);

    // (2 * 3 * 4) * v = 24 * v
    JSBSim::FGColumnVector3 result = 2.0 * (3.0 * (4.0 * v));

    TS_ASSERT_DELTA(result(1), 24.0, 1e-12);
    TS_ASSERT_DELTA(result(2), 48.0, 1e-12);
    TS_ASSERT_DELTA(result(3), 72.0, 1e-12);
  }

  void testAdditionSubtractionChain(void) {
    JSBSim::FGColumnVector3 v1(1.0, 2.0, 3.0);
    JSBSim::FGColumnVector3 v2(4.0, 5.0, 6.0);
    JSBSim::FGColumnVector3 v3(7.0, 8.0, 9.0);

    // (v1 + v2) - v3 + v1
    JSBSim::FGColumnVector3 result = (v1 + v2) - v3 + v1;

    TS_ASSERT_EQUALS(result(1), 1.0 + 4.0 - 7.0 + 1.0);  // -1
    TS_ASSERT_EQUALS(result(2), 2.0 + 5.0 - 8.0 + 2.0);  // 1
    TS_ASSERT_EQUALS(result(3), 3.0 + 6.0 - 9.0 + 3.0);  // 3
  }

  void testUnitVectorAfterNormalize(void) {
    JSBSim::FGColumnVector3 v(100.0, -200.0, 300.0);
    v.Normalize();

    // Magnitude should be exactly 1
    TS_ASSERT_DELTA(v.Magnitude(), 1.0, 1e-12);

    // Components should be consistent
    double sumSquared = v(1)*v(1) + v(2)*v(2) + v(3)*v(3);
    TS_ASSERT_DELTA(sumSquared, 1.0, 1e-12);
  }

  void testNormalizePreservesDirection(void) {
    JSBSim::FGColumnVector3 v(3.0, 4.0, 0.0);
    double ratio12 = v(1) / v(2);

    v.Normalize();

    // Ratio between components should be preserved
    TS_ASSERT_DELTA(v(1) / v(2), ratio12, 1e-12);
  }

  /***************************************************************************
   * Vector Identity Tests
   ***************************************************************************/

  void testTriangleInequality(void) {
    JSBSim::FGColumnVector3 v1(1.0, 2.0, 3.0);
    JSBSim::FGColumnVector3 v2(4.0, 5.0, 6.0);

    double sum_mag = (v1 + v2).Magnitude();
    double mag_sum = v1.Magnitude() + v2.Magnitude();

    // |v1 + v2| <= |v1| + |v2|
    TS_ASSERT(sum_mag <= mag_sum + 1e-9);
  }

  void testCauchySchwarzInequality(void) {
    JSBSim::FGColumnVector3 v1(1.0, 2.0, 3.0);
    JSBSim::FGColumnVector3 v2(4.0, 5.0, 6.0);

    double dot = fabs(DotProduct(v1, v2));
    double product = v1.Magnitude() * v2.Magnitude();

    // |v1 · v2| <= |v1| * |v2|
    TS_ASSERT(dot <= product + 1e-9);
  }

  void testLagrangeIdentity(void) {
    // |a × b|² = |a|²|b|² - (a·b)²
    JSBSim::FGColumnVector3 a(1.0, 2.0, 3.0);
    JSBSim::FGColumnVector3 b(4.0, 5.0, 6.0);

    JSBSim::FGColumnVector3 cross = a * b;
    double crossMagSq = DotProduct(cross, cross);

    double aMagSq = DotProduct(a, a);
    double bMagSq = DotProduct(b, b);
    double dotAB = DotProduct(a, b);

    double expected = aMagSq * bMagSq - dotAB * dotAB;

    TS_ASSERT_DELTA(crossMagSq, expected, 1e-9);
  }

  /***************************************************************************
   * Complete System Tests
   ***************************************************************************/

  void testCompleteVectorTransformation(void) {
    // Test a complete sequence of vector operations
    JSBSim::FGColumnVector3 position(100.0, 50.0, 25.0);
    JSBSim::FGColumnVector3 velocity(10.0, 5.0, -2.0);
    double dt = 0.1;

    // Update position: p' = p + v*dt
    JSBSim::FGColumnVector3 newPosition = position + velocity * dt;

    TS_ASSERT_EQUALS(newPosition(1), 101.0);
    TS_ASSERT_EQUALS(newPosition(2), 50.5);
    TS_ASSERT_DELTA(newPosition(3), 24.8, 1e-12);

    // Verify displacement magnitude
    JSBSim::FGColumnVector3 displacement = newPosition - position;
    TS_ASSERT_DELTA(displacement.Magnitude(), velocity.Magnitude() * dt, 1e-12);
  }

  void testCompleteRotationSequence(void) {
    // Test computing angular velocity from moment and inertia
    JSBSim::FGColumnVector3 moment(1000.0, 500.0, 200.0);  // Applied moment
    JSBSim::FGColumnVector3 inertia(5000.0, 10000.0, 15000.0);  // Inertia components

    // Angular acceleration = moment / inertia (component-wise approximation)
    JSBSim::FGColumnVector3 angAccel;
    angAccel(1) = moment(1) / inertia(1);
    angAccel(2) = moment(2) / inertia(2);
    angAccel(3) = moment(3) / inertia(3);

    TS_ASSERT_DELTA(angAccel(1), 0.2, 1e-12);
    TS_ASSERT_DELTA(angAccel(2), 0.05, 1e-12);
    TS_ASSERT_DELTA(angAccel(3), 200.0/15000.0, 1e-12);
  }

  void testCompleteForceResolution(void) {
    // Resolve total force into components
    JSBSim::FGColumnVector3 thrust(5000.0, 0.0, -100.0);
    JSBSim::FGColumnVector3 drag(-1000.0, 0.0, 50.0);
    JSBSim::FGColumnVector3 lift(0.0, 0.0, -10000.0);
    JSBSim::FGColumnVector3 weight(0.0, 0.0, 15000.0);

    JSBSim::FGColumnVector3 totalForce = thrust + drag + lift + weight;

    TS_ASSERT_EQUALS(totalForce(1), 4000.0);   // Net forward
    TS_ASSERT_EQUALS(totalForce(2), 0.0);      // No side
    TS_ASSERT_EQUALS(totalForce(3), 4950.0);   // Net vertical

    // Verify magnitude
    TS_ASSERT(totalForce.Magnitude() > 0.0);
  }

  void testCompleteProjectionOperations(void) {
    // Project vector onto unit vector
    JSBSim::FGColumnVector3 v(3.0, 4.0, 5.0);
    JSBSim::FGColumnVector3 axis(1.0, 0.0, 0.0);  // Already unit

    // Projection of v onto axis: (v·axis) * axis
    double projMag = DotProduct(v, axis);
    JSBSim::FGColumnVector3 projection = projMag * axis;

    TS_ASSERT_EQUALS(projection(1), 3.0);
    TS_ASSERT_EQUALS(projection(2), 0.0);
    TS_ASSERT_EQUALS(projection(3), 0.0);

    // Perpendicular component
    JSBSim::FGColumnVector3 perp = v - projection;
    TS_ASSERT_EQUALS(perp(1), 0.0);
    TS_ASSERT_EQUALS(perp(2), 4.0);
    TS_ASSERT_EQUALS(perp(3), 5.0);

    // Verify orthogonality
    TS_ASSERT_DELTA(DotProduct(projection, perp), 0.0, 1e-12);
  }

  /***************************************************************************
   * Instance Independence Tests
   ***************************************************************************/

  void testIndependentVectorInstances(void) {
    // Create two independent vectors
    JSBSim::FGColumnVector3 v1(1.0, 2.0, 3.0);
    JSBSim::FGColumnVector3 v2(4.0, 5.0, 6.0);

    // Modify v1
    v1(1) = 100.0;
    v1(2) = 200.0;
    v1(3) = 300.0;

    // v2 should be unchanged
    TS_ASSERT_EQUALS(v2(1), 4.0);
    TS_ASSERT_EQUALS(v2(2), 5.0);
    TS_ASSERT_EQUALS(v2(3), 6.0);
  }

  void testIndependentOperationResults(void) {
    JSBSim::FGColumnVector3 v1(1.0, 2.0, 3.0);
    JSBSim::FGColumnVector3 v2(4.0, 5.0, 6.0);

    // Compute multiple results
    JSBSim::FGColumnVector3 sum = v1 + v2;
    JSBSim::FGColumnVector3 diff = v1 - v2;
    JSBSim::FGColumnVector3 cross = v1 * v2;

    // Results should be independent
    sum(1) = 999.0;

    TS_ASSERT_EQUALS(diff(1), -3.0);
    TS_ASSERT_DELTA(cross(1), 2.0*6.0 - 3.0*5.0, 1e-12);  // -3

    // Original vectors unchanged
    TS_ASSERT_EQUALS(v1(1), 1.0);
    TS_ASSERT_EQUALS(v2(1), 4.0);
  }

  void testIndependentNormalization(void) {
    JSBSim::FGColumnVector3 v1(3.0, 4.0, 0.0);
    JSBSim::FGColumnVector3 v2(3.0, 4.0, 0.0);

    // Normalize only v1
    v1.Normalize();

    // v2 should remain unchanged
    TS_ASSERT_EQUALS(v2(1), 3.0);
    TS_ASSERT_EQUALS(v2(2), 4.0);
    TS_ASSERT_EQUALS(v2(3), 0.0);
    TS_ASSERT_EQUALS(v2.Magnitude(), 5.0);

    // v1 should be normalized
    TS_ASSERT_DELTA(v1.Magnitude(), 1.0, 1e-12);
  }
};
