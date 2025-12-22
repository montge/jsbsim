/*******************************************************************************
 * FGRungeKuttaTest.h - Unit tests for Runge-Kutta ODE solvers
 *
 * Tests the FGRK4 (classical RK4) and FGRKFehlberg (RK4-5 adaptive) ODE
 * solvers with various differential equations including exponential growth,
 * harmonic oscillator, and other analytically solvable ODEs.
 *
 * Copyright (c) JSBSim Development Team
 * Licensed under LGPL
 ******************************************************************************/

#include <cxxtest/TestSuite.h>
#include <math/FGRungeKutta.h>
#include <cmath>
#include <limits>

using namespace JSBSim;

/*******************************************************************************
 * Test ODE Problems
 * Each problem implements dy/dx = pFunc(x, y)
 ******************************************************************************/

// Exponential growth: dy/dx = y
// Solution: y(x) = y0 * exp(x - x0)
class ExponentialGrowth : public FGRungeKuttaProblem {
public:
    double pFunc(double x, double y) override {
        (void)x;  // unused
        return y;
    }
};

// Linear ODE: dy/dx = 2*x
// Solution: y(x) = x^2 + C
class LinearODE : public FGRungeKuttaProblem {
public:
    double pFunc(double x, double y) override {
        (void)y;  // unused
        return 2.0 * x;
    }
};

// Decay: dy/dx = -y
// Solution: y(x) = y0 * exp(-(x - x0))
class ExponentialDecay : public FGRungeKuttaProblem {
public:
    double pFunc(double x, double y) override {
        (void)x;  // unused
        return -y;
    }
};

// Sinusoidal: dy/dx = cos(x)
// Solution: y(x) = sin(x) + C
class SinusoidalODE : public FGRungeKuttaProblem {
public:
    double pFunc(double x, double y) override {
        (void)y;  // unused
        return cos(x);
    }
};

// Logistic growth: dy/dx = y * (1 - y)
// Solution: y(x) = 1 / (1 + (1/y0 - 1) * exp(-x))
class LogisticGrowth : public FGRungeKuttaProblem {
public:
    double pFunc(double x, double y) override {
        (void)x;  // unused
        return y * (1.0 - y);
    }
};

// Polynomial: dy/dx = 3*x^2
// Solution: y(x) = x^3 + C
class CubicODE : public FGRungeKuttaProblem {
public:
    double pFunc(double x, double y) override {
        (void)y;
        return 3.0 * x * x;
    }
};

// Coupled to y: dy/dx = x + y
// Harder to solve analytically but good stress test
class CoupledODE : public FGRungeKuttaProblem {
public:
    double pFunc(double x, double y) override {
        return x + y;
    }
};

/*******************************************************************************
 * Test Suite for FGRK4 (Classical 4th-order Runge-Kutta)
 ******************************************************************************/

class FGRK4Test : public CxxTest::TestSuite {
public:
    static const double TOLERANCE;
    static const double LOOSE_TOLERANCE;

    void testExponentialGrowth() {
        // GIVEN: dy/dx = y with y(0) = 1
        // Expected: y(1) = e ≈ 2.71828
        ExponentialGrowth problem;
        FGRK4 solver;

        // WHEN: Integrate from x=0 to x=1
        solver.init(0.0, 1.0, 100);  // 100 intervals
        double y = solver.evolve(1.0, &problem);

        // THEN: Result should be close to e
        TS_ASSERT_EQUALS(solver.getStatus(), FGRungeKutta::eNoError);
        TS_ASSERT_DELTA(y, exp(1.0), TOLERANCE);
    }

    void testExponentialDecay() {
        // GIVEN: dy/dx = -y with y(0) = 1
        // Expected: y(1) = 1/e ≈ 0.3679
        ExponentialDecay problem;
        FGRK4 solver;

        // WHEN: Integrate from x=0 to x=1
        solver.init(0.0, 1.0, 100);
        double y = solver.evolve(1.0, &problem);

        // THEN: Result should be close to 1/e
        TS_ASSERT_EQUALS(solver.getStatus(), FGRungeKutta::eNoError);
        TS_ASSERT_DELTA(y, exp(-1.0), TOLERANCE);
    }

    void testLinearODE() {
        // GIVEN: dy/dx = 2x with y(0) = 0
        // Expected: y(2) = 4 (since integral of 2x from 0 to 2 is x^2|_0^2 = 4)
        LinearODE problem;
        FGRK4 solver;

        // WHEN: Integrate from x=0 to x=2
        solver.init(0.0, 2.0, 100);
        double y = solver.evolve(0.0, &problem);

        // THEN: Result should be 4
        TS_ASSERT_EQUALS(solver.getStatus(), FGRungeKutta::eNoError);
        TS_ASSERT_DELTA(y, 4.0, TOLERANCE);
    }

    void testSinusoidalODE() {
        // GIVEN: dy/dx = cos(x) with y(0) = 0
        // Expected: y(π) = sin(π) = 0
        SinusoidalODE problem;
        FGRK4 solver;

        // WHEN: Integrate from x=0 to x=π
        solver.init(0.0, M_PI, 100);
        double y = solver.evolve(0.0, &problem);

        // THEN: Result should be 0 (sin(π))
        TS_ASSERT_EQUALS(solver.getStatus(), FGRungeKutta::eNoError);
        TS_ASSERT_DELTA(y, 0.0, TOLERANCE);

        // Also test y(π/2) = 1
        solver.init(0.0, M_PI / 2.0, 100);
        y = solver.evolve(0.0, &problem);
        TS_ASSERT_DELTA(y, 1.0, TOLERANCE);
    }

    void testCubicODE() {
        // GIVEN: dy/dx = 3x^2 with y(0) = 0
        // Expected: y(2) = 8 (since x^3|_0^2 = 8)
        CubicODE problem;
        FGRK4 solver;

        // WHEN: Integrate from x=0 to x=2
        solver.init(0.0, 2.0, 100);
        double y = solver.evolve(0.0, &problem);

        // THEN: Result should be 8
        TS_ASSERT_EQUALS(solver.getStatus(), FGRungeKutta::eNoError);
        TS_ASSERT_DELTA(y, 8.0, TOLERANCE);
    }

    void testInitStatus() {
        // GIVEN: A solver
        FGRK4 solver;

        // WHEN: Init with valid range
        int status = solver.init(0.0, 1.0, 10);

        // THEN: Status should be eNoError
        TS_ASSERT_EQUALS(status, FGRungeKutta::eNoError);
    }

    void testInvalidInitRange() {
        // GIVEN: A solver
        FGRK4 solver;

        // WHEN: Init with invalid range (x_end <= x_start)
        solver.clearStatus();
        int status = solver.init(1.0, 0.0, 10);

        // THEN: Status should indicate faulty init
        TS_ASSERT(status & FGRungeKutta::eFaultyInit);
    }

    void testGetIterations() {
        // GIVEN: A simple ODE
        LinearODE problem;
        FGRK4 solver;

        // WHEN: Solving with known intervals
        solver.init(0.0, 1.0, 50);
        solver.evolve(0.0, &problem);

        // THEN: Iterations should match intervals
        TS_ASSERT_EQUALS(solver.getIterations(), 50);
    }

    void testGetXEnd() {
        // GIVEN: A solver
        LinearODE problem;
        FGRK4 solver;

        // WHEN: Integrating to a known endpoint
        solver.init(0.0, 1.5, 100);
        solver.evolve(0.0, &problem);

        // THEN: getXEnd should return approximately x_end
        TS_ASSERT_DELTA(solver.getXEnd(), 1.5, 0.02);  // Allow for step size
    }

    void testNonZeroInitialCondition() {
        // GIVEN: dy/dx = 2x with y(1) = 5
        // y(x) = x^2 + C, and y(1) = 1 + C = 5, so C = 4
        // Expected: y(3) = 9 + 4 = 13
        LinearODE problem;
        FGRK4 solver;

        // WHEN: Integrate from x=1 to x=3 with y(1)=5
        solver.init(1.0, 3.0, 100);
        double y = solver.evolve(5.0, &problem);

        // THEN: Result should be 13
        TS_ASSERT_DELTA(y, 13.0, TOLERANCE);
    }

    void testIntervalEffect() {
        // GIVEN: An ODE with known solution
        ExponentialGrowth problem;
        FGRK4 solver1, solver2;

        // WHEN: Solving with different numbers of intervals
        solver1.init(0.0, 1.0, 10);   // Fewer intervals
        double y1 = solver1.evolve(1.0, &problem);

        solver2.init(0.0, 1.0, 1000); // More intervals
        double y2 = solver2.evolve(1.0, &problem);

        // THEN: More intervals should give more accurate result
        double exactSolution = exp(1.0);
        double error1 = std::abs(y1 - exactSolution);
        double error2 = std::abs(y2 - exactSolution);
        TS_ASSERT(error2 < error1);
    }
};

const double FGRK4Test::TOLERANCE = 1e-8;
const double FGRK4Test::LOOSE_TOLERANCE = 1e-4;

/*******************************************************************************
 * Test Suite for FGRKFehlberg (Adaptive RK4-5)
 ******************************************************************************/

class FGRKFehlbergTest : public CxxTest::TestSuite {
public:
    static const double TOLERANCE;

    void testExponentialGrowth() {
        // GIVEN: dy/dx = y with y(0) = 1
        ExponentialGrowth problem;
        FGRKFehlberg solver;

        // WHEN: Integrate from x=0 to x=1
        solver.init(0.0, 1.0, 20);  // Fewer intervals needed due to adaptivity
        double y = solver.evolve(1.0, &problem);

        // THEN: Result should be close to e
        TS_ASSERT_EQUALS(solver.getStatus(), FGRungeKutta::eNoError);
        TS_ASSERT_DELTA(y, exp(1.0), TOLERANCE);
    }

    void testExponentialDecay() {
        // GIVEN: dy/dx = -y with y(0) = 1
        ExponentialDecay problem;
        FGRKFehlberg solver;

        // WHEN: Integrate from x=0 to x=1
        solver.init(0.0, 1.0, 20);
        double y = solver.evolve(1.0, &problem);

        // THEN: Result should be close to 1/e
        TS_ASSERT_EQUALS(solver.getStatus(), FGRungeKutta::eNoError);
        TS_ASSERT_DELTA(y, exp(-1.0), TOLERANCE);
    }

    void testLinearODE() {
        // GIVEN: dy/dx = 2x with y(0) = 0
        LinearODE problem;
        FGRKFehlberg solver;

        // WHEN: Integrate from x=0 to x=2
        solver.init(0.0, 2.0, 20);
        double y = solver.evolve(0.0, &problem);

        // THEN: Result should be 4
        TS_ASSERT_EQUALS(solver.getStatus(), FGRungeKutta::eNoError);
        TS_ASSERT_DELTA(y, 4.0, TOLERANCE);
    }

    void testSetEpsilon() {
        // GIVEN: A Fehlberg solver
        FGRKFehlberg solver;

        // WHEN: Setting epsilon
        double newEpsilon = 1e-10;
        solver.setEpsilon(newEpsilon);

        // THEN: getEpsilon should return the new value
        TS_ASSERT_EQUALS(solver.getEpsilon(), newEpsilon);
    }

    void testDefaultEpsilon() {
        // GIVEN: A new Fehlberg solver
        FGRKFehlberg solver;

        // THEN: Default epsilon should be 1e-12
        TS_ASSERT_EQUALS(solver.getEpsilon(), 1e-12);
    }

    void testSetShrinkAvail() {
        // GIVEN: A Fehlberg solver
        FGRKFehlberg solver;

        // WHEN: Setting shrink availability
        solver.setShrinkAvail(8);

        // THEN: getShrinkAvail should return the new value
        TS_ASSERT_EQUALS(solver.getShrinkAvail(), 8);
    }

    void testDefaultShrinkAvail() {
        // GIVEN: A new Fehlberg solver
        FGRKFehlberg solver;

        // THEN: Default shrink availability should be 4
        TS_ASSERT_EQUALS(solver.getShrinkAvail(), 4);
    }

    void testLogisticGrowth() {
        // GIVEN: Logistic equation dy/dx = y(1-y) with y(0) = 0.1
        // Analytical solution: y(x) = 1 / (1 + (1/y0 - 1) * exp(-x))
        LogisticGrowth problem;
        FGRKFehlberg solver;

        double y0 = 0.1;
        double x_end = 2.0;

        // WHEN: Integrate from x=0 to x=2
        solver.init(0.0, x_end, 50);
        double y = solver.evolve(y0, &problem);

        // THEN: Result should match analytical solution
        double expected = 1.0 / (1.0 + (1.0 / y0 - 1.0) * exp(-x_end));
        TS_ASSERT_EQUALS(solver.getStatus(), FGRungeKutta::eNoError);
        TS_ASSERT_DELTA(y, expected, TOLERANCE);
    }

    void testFehlbergVsRK4Accuracy() {
        // GIVEN: The same ODE problem
        ExponentialGrowth problem;
        FGRK4 rk4;
        FGRKFehlberg rkf;

        // WHEN: Solving with the same number of initial intervals
        rk4.init(0.0, 1.0, 20);
        rkf.init(0.0, 1.0, 20);

        double y_rk4 = rk4.evolve(1.0, &problem);
        double y_rkf = rkf.evolve(1.0, &problem);

        // THEN: Fehlberg should typically be more accurate (or at least comparable)
        double exact = exp(1.0);
        double error_rk4 = std::abs(y_rk4 - exact);
        double error_rkf = std::abs(y_rkf - exact);

        // Both should be reasonably accurate
        TS_ASSERT(error_rk4 < 0.01);
        TS_ASSERT(error_rkf < 0.01);
    }

    void testCoupledODE() {
        // GIVEN: dy/dx = x + y
        // This is a stress test for the adaptive solver
        CoupledODE problem;
        FGRKFehlberg solver;

        // WHEN: Integrating
        solver.init(0.0, 1.0, 50);
        solver.evolve(0.0, &problem);

        // THEN: Should complete without error
        TS_ASSERT_EQUALS(solver.getStatus(), FGRungeKutta::eNoError);
    }

    void testClearStatus() {
        // GIVEN: A solver with an error status
        FGRKFehlberg solver;
        solver.init(1.0, 0.0, 10);  // Invalid init

        // WHEN: Clearing the status
        solver.clearStatus();

        // THEN: Status should be eNoError
        TS_ASSERT_EQUALS(solver.getStatus(), FGRungeKutta::eNoError);
    }
};

const double FGRKFehlbergTest::TOLERANCE = 1e-6;
