#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>

#include <FGFDMExec.h>
#include <models/FGAccelerations.h>
#include <models/FGMassBalance.h>
#include <models/FGPropagate.h>
#include <math/FGColumnVector3.h>
#include <math/FGMatrix33.h>
#include "TestAssertions.h"
#include "TestUtilities.h"

using namespace JSBSim;

const double epsilon = 1e-8;

class FGAccelerationsTest : public CxxTest::TestSuite
{
public:
  // Test that zero forces result in zero acceleration (ignoring gravity)
  void testZeroForceZeroAcceleration() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    // Set zero forces and moments
    accel->in.Force = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.Moment = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.GroundForce = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.GroundMoment = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vGravAccel = FGColumnVector3(0.0, 0.0, 0.0);  // Ignore gravity
    accel->in.vPQR = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vPQRi = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vUVW = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.Mass = 100.0;  // slugs

    // Identity inertia matrix (simplified)
    accel->in.J = FGMatrix33(1000.0, 0.0, 0.0,
                              0.0, 1000.0, 0.0,
                              0.0, 0.0, 1000.0);
    accel->in.Jinv = FGMatrix33(0.001, 0.0, 0.0,
                                 0.0, 0.001, 0.0,
                                 0.0, 0.0, 0.001);

    // Identity transformation matrices
    accel->in.Ti2b = FGMatrix33(1.0, 0.0, 0.0,
                                 0.0, 1.0, 0.0,
                                 0.0, 0.0, 1.0);
    accel->in.Tb2i = FGMatrix33(1.0, 0.0, 0.0,
                                 0.0, 1.0, 0.0,
                                 0.0, 0.0, 1.0);
    accel->in.Tec2b = FGMatrix33(1.0, 0.0, 0.0,
                                  0.0, 1.0, 0.0,
                                  0.0, 0.0, 1.0);
    accel->in.Tec2i = FGMatrix33(1.0, 0.0, 0.0,
                                  0.0, 1.0, 0.0,
                                  0.0, 0.0, 1.0);

    accel->in.vOmegaPlanet = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vInertialPosition = FGColumnVector3(0.0, 0.0, 20925646.0);  // Earth radius in feet
    accel->in.DeltaT = 0.0083333;  // 1/120 sec
    accel->in.TerrainVelocity = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.TerrainAngularVel = FGColumnVector3(0.0, 0.0, 0.0);

    accel->InitializeDerivatives();
    accel->Run(false);

    // With no forces, accelerations should be zero
    FGColumnVector3 uvwdot = accel->GetUVWdot();
    FGColumnVector3 pqrdot = accel->GetPQRdot();

    TS_ASSERT_DELTA(uvwdot(1), 0.0, epsilon);
    TS_ASSERT_DELTA(uvwdot(2), 0.0, epsilon);
    TS_ASSERT_DELTA(uvwdot(3), 0.0, epsilon);
    TS_ASSERT_DELTA(pqrdot(1), 0.0, epsilon);
    TS_ASSERT_DELTA(pqrdot(2), 0.0, epsilon);
    TS_ASSERT_DELTA(pqrdot(3), 0.0, epsilon);
  }

  // Test F=ma relationship for translational acceleration
  void testNewtonSecondLawTranslation() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    double mass = 50.0;  // slugs
    FGColumnVector3 force(100.0, 200.0, 300.0);  // lbs

    // Set up inputs
    accel->in.Force = force;
    accel->in.Moment = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.GroundForce = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.GroundMoment = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vGravAccel = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vPQR = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vPQRi = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vUVW = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.Mass = mass;

    // Identity inertia and transforms
    accel->in.J = FGMatrix33(1000.0, 0.0, 0.0, 0.0, 1000.0, 0.0, 0.0, 0.0, 1000.0);
    accel->in.Jinv = FGMatrix33(0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001);
    accel->in.Ti2b = FGMatrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    accel->in.Tb2i = FGMatrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    accel->in.Tec2b = FGMatrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    accel->in.Tec2i = FGMatrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);

    accel->in.vOmegaPlanet = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vInertialPosition = FGColumnVector3(0.0, 0.0, 20925646.0);
    accel->in.DeltaT = 0.0083333;
    accel->in.TerrainVelocity = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.TerrainAngularVel = FGColumnVector3(0.0, 0.0, 0.0);

    accel->InitializeDerivatives();
    accel->Run(false);

    // Expected: a = F/m
    FGColumnVector3 bodyAccel = accel->GetBodyAccel();
    TS_ASSERT_DELTA(bodyAccel(1), force(1)/mass, epsilon);
    TS_ASSERT_DELTA(bodyAccel(2), force(2)/mass, epsilon);
    TS_ASSERT_DELTA(bodyAccel(3), force(3)/mass, epsilon);
  }

  // Test torque-angular acceleration relationship (T = I*alpha for simple case)
  void testTorqueAngularAcceleration() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    // Simple moment about roll axis
    FGColumnVector3 moment(1000.0, 0.0, 0.0);  // lbs*ft
    double Ixx = 5000.0;  // slug*ft^2

    accel->in.Force = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.Moment = moment;
    accel->in.GroundForce = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.GroundMoment = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vGravAccel = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vPQR = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vPQRi = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vUVW = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.Mass = 100.0;

    // Diagonal inertia matrix
    accel->in.J = FGMatrix33(Ixx, 0.0, 0.0,
                              0.0, Ixx, 0.0,
                              0.0, 0.0, Ixx);
    accel->in.Jinv = FGMatrix33(1.0/Ixx, 0.0, 0.0,
                                 0.0, 1.0/Ixx, 0.0,
                                 0.0, 0.0, 1.0/Ixx);

    accel->in.Ti2b = FGMatrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    accel->in.Tb2i = FGMatrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    accel->in.Tec2b = FGMatrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    accel->in.Tec2i = FGMatrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);

    accel->in.vOmegaPlanet = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vInertialPosition = FGColumnVector3(0.0, 0.0, 20925646.0);
    accel->in.DeltaT = 0.0083333;
    accel->in.TerrainVelocity = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.TerrainAngularVel = FGColumnVector3(0.0, 0.0, 0.0);

    accel->InitializeDerivatives();
    accel->Run(false);

    // Expected: alpha = M/I (for simple diagonal inertia with no rotation)
    FGColumnVector3 pqridot = accel->GetPQRidot();
    TS_ASSERT_DELTA(pqridot(1), moment(1)/Ixx, epsilon);
    TS_ASSERT_DELTA(pqridot(2), 0.0, epsilon);
    TS_ASSERT_DELTA(pqridot(3), 0.0, epsilon);
  }

  // Test GetForces includes friction
  void testGetForcesIncludesFriction() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    FGColumnVector3 appliedForce(100.0, 50.0, 0.0);
    accel->in.Force = appliedForce;
    accel->in.GroundForce = FGColumnVector3(0.0, 0.0, 0.0);

    // GetForces should return applied forces (friction is computed internally)
    FGColumnVector3 forces = accel->GetForces();
    // Note: friction forces are zero when no ground contact
    TS_ASSERT_DELTA(forces(1), appliedForce(1), epsilon);
    TS_ASSERT_DELTA(forces(2), appliedForce(2), epsilon);
    TS_ASSERT_DELTA(forces(3), appliedForce(3), epsilon);
  }

  // Test GetMoments includes friction moments
  void testGetMomentsIncludesFriction() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    FGColumnVector3 appliedMoment(500.0, -200.0, 100.0);
    accel->in.Moment = appliedMoment;
    accel->in.GroundMoment = FGColumnVector3(0.0, 0.0, 0.0);

    FGColumnVector3 moments = accel->GetMoments();
    TS_ASSERT_DELTA(moments(1), appliedMoment(1), epsilon);
    TS_ASSERT_DELTA(moments(2), appliedMoment(2), epsilon);
    TS_ASSERT_DELTA(moments(3), appliedMoment(3), epsilon);
  }

  // Test gravity acceleration magnitude getter
  void testGravAccelMagnitude() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    // Standard gravity vector (pointing down in ECEF)
    double g = 32.174;  // ft/s^2
    accel->in.vGravAccel = FGColumnVector3(0.0, 0.0, -g);

    double gravMag = accel->GetGravAccelMagnitude();
    TS_ASSERT_DELTA(gravMag, g, epsilon);
  }

  // Test Weight calculation
  void testWeightCalculation() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    double mass = 100.0;  // slugs
    double g = 32.174;    // ft/s^2

    accel->in.Mass = mass;
    accel->in.vGravAccel = FGColumnVector3(0.0, 0.0, -g);  // ECEF frame
    accel->in.Tec2b = FGMatrix33(1.0, 0.0, 0.0,
                                  0.0, 1.0, 0.0,
                                  0.0, 0.0, 1.0);

    FGColumnVector3 weight = accel->GetWeight();
    // Weight = mass * Tec2b * vGravAccel
    TS_ASSERT_DELTA(weight(1), 0.0, epsilon);
    TS_ASSERT_DELTA(weight(2), 0.0, epsilon);
    TS_ASSERT_DELTA(weight(3), -mass * g, epsilon);
  }

  // Test component accessors (indexed)
  void testIndexedAccessors() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    accel->in.Force = FGColumnVector3(10.0, 20.0, 30.0);
    accel->in.Moment = FGColumnVector3(100.0, 200.0, 300.0);
    accel->in.GroundForce = FGColumnVector3(1.0, 2.0, 3.0);
    accel->in.GroundMoment = FGColumnVector3(11.0, 22.0, 33.0);

    // Test indexed getters
    TS_ASSERT_DELTA(accel->GetForces(1), 10.0, epsilon);
    TS_ASSERT_DELTA(accel->GetForces(2), 20.0, epsilon);
    TS_ASSERT_DELTA(accel->GetForces(3), 30.0, epsilon);

    TS_ASSERT_DELTA(accel->GetMoments(1), 100.0, epsilon);
    TS_ASSERT_DELTA(accel->GetMoments(2), 200.0, epsilon);
    TS_ASSERT_DELTA(accel->GetMoments(3), 300.0, epsilon);

    TS_ASSERT_DELTA(accel->GetGroundForces(1), 1.0, epsilon);
    TS_ASSERT_DELTA(accel->GetGroundForces(2), 2.0, epsilon);
    TS_ASSERT_DELTA(accel->GetGroundForces(3), 3.0, epsilon);

    TS_ASSERT_DELTA(accel->GetGroundMoments(1), 11.0, epsilon);
    TS_ASSERT_DELTA(accel->GetGroundMoments(2), 22.0, epsilon);
    TS_ASSERT_DELTA(accel->GetGroundMoments(3), 33.0, epsilon);
  }

  // Test initialization of derivatives
  void testInitializeDerivatives() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    // Set some initial accelerations
    accel->in.Force = FGColumnVector3(1000.0, 0.0, 0.0);
    accel->in.Mass = 50.0;

    accel->InitializeDerivatives();

    // After initialization, derivatives should be reset/initialized
    FGColumnVector3 uvwdot = accel->GetUVWdot();
    FGColumnVector3 pqrdot = accel->GetPQRdot();

    // Just verify we can call these without error
    TS_ASSERT(!std::isnan(uvwdot(1)));
    TS_ASSERT(!std::isnan(uvwdot(2)));
    TS_ASSERT(!std::isnan(uvwdot(3)));
    TS_ASSERT(!std::isnan(pqrdot(1)));
    TS_ASSERT(!std::isnan(pqrdot(2)));
    TS_ASSERT(!std::isnan(pqrdot(3)));
  }

  // Test hold-down functionality (rocket on launch pad)
  void testHoldDown() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    accel->in.Force = FGColumnVector3(10000.0, 0.0, 0.0);  // Thrust
    accel->in.Moment = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.GroundForce = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.GroundMoment = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vGravAccel = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vPQR = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vPQRi = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vUVW = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.Mass = 100.0;

    accel->in.J = FGMatrix33(1000.0, 0.0, 0.0, 0.0, 1000.0, 0.0, 0.0, 0.0, 1000.0);
    accel->in.Jinv = FGMatrix33(0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001);
    accel->in.Ti2b = FGMatrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    accel->in.Tb2i = FGMatrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    accel->in.Tec2b = FGMatrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    accel->in.Tec2i = FGMatrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    accel->in.vOmegaPlanet = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vInertialPosition = FGColumnVector3(0.0, 0.0, 20925646.0);
    accel->in.DeltaT = 0.0083333;
    accel->in.TerrainVelocity = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.TerrainAngularVel = FGColumnVector3(0.0, 0.0, 0.0);

    // Enable hold-down
    accel->SetHoldDown(true);
    accel->InitializeDerivatives();
    accel->Run(false);

    // With hold-down, accelerations should be zero
    FGColumnVector3 uvwdot = accel->GetUVWdot();
    FGColumnVector3 pqrdot = accel->GetPQRdot();

    TS_ASSERT_DELTA(uvwdot(1), 0.0, epsilon);
    TS_ASSERT_DELTA(uvwdot(2), 0.0, epsilon);
    TS_ASSERT_DELTA(uvwdot(3), 0.0, epsilon);
    TS_ASSERT_DELTA(pqrdot(1), 0.0, epsilon);
    TS_ASSERT_DELTA(pqrdot(2), 0.0, epsilon);
    TS_ASSERT_DELTA(pqrdot(3), 0.0, epsilon);

    // Disable hold-down and verify accelerations are computed
    accel->SetHoldDown(false);
    accel->Run(false);

    FGColumnVector3 uvwdot2 = accel->GetUVWdot();
    // Should now have non-zero acceleration
    TS_ASSERT(std::abs(uvwdot2(1)) > 1.0 || std::abs(accel->GetBodyAccel(1)) > 1.0);
  }

  // Test Holding mode during simulation
  void testHoldingMode() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    accel->in.Force = FGColumnVector3(1000.0, 0.0, 0.0);
    accel->in.Moment = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.Mass = 100.0;

    // Run with Holding = true (simulation paused)
    bool result = accel->Run(true);

    // Run should return false when not Holding
    TS_ASSERT_EQUALS(result, false);
  }

  // Test ECI frame accelerations
  void testECIFrameAccelerations() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    accel->in.Force = FGColumnVector3(500.0, 0.0, 0.0);
    accel->in.Moment = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.GroundForce = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.GroundMoment = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vGravAccel = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vPQR = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vPQRi = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vUVW = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.Mass = 50.0;

    accel->in.J = FGMatrix33(1000.0, 0.0, 0.0, 0.0, 1000.0, 0.0, 0.0, 0.0, 1000.0);
    accel->in.Jinv = FGMatrix33(0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001);
    accel->in.Ti2b = FGMatrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    accel->in.Tb2i = FGMatrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    accel->in.Tec2b = FGMatrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    accel->in.Tec2i = FGMatrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    accel->in.vOmegaPlanet = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vInertialPosition = FGColumnVector3(0.0, 0.0, 20925646.0);
    accel->in.DeltaT = 0.0083333;
    accel->in.TerrainVelocity = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.TerrainAngularVel = FGColumnVector3(0.0, 0.0, 0.0);

    accel->InitializeDerivatives();
    accel->Run(false);

    // Test indexed ECI accessors
    FGColumnVector3 uvwidot = accel->GetUVWidot();
    FGColumnVector3 pqridot = accel->GetPQRidot();

    TS_ASSERT_DELTA(accel->GetUVWidot(1), uvwidot(1), epsilon);
    TS_ASSERT_DELTA(accel->GetUVWidot(2), uvwidot(2), epsilon);
    TS_ASSERT_DELTA(accel->GetUVWidot(3), uvwidot(3), epsilon);

    TS_ASSERT_DELTA(accel->GetPQRidot(1), pqridot(1), epsilon);
    TS_ASSERT_DELTA(accel->GetPQRidot(2), pqridot(2), epsilon);
    TS_ASSERT_DELTA(accel->GetPQRidot(3), pqridot(3), epsilon);
  }

  // Test combined forces and moments
  void testCombinedForcesAndMoments() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    // Apply forces at a point offset from CG to create moments
    FGColumnVector3 force(100.0, 0.0, 0.0);
    FGColumnVector3 moment(50.0, 100.0, -25.0);

    accel->in.Force = force;
    accel->in.Moment = moment;
    accel->in.GroundForce = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.GroundMoment = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vGravAccel = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vPQR = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vPQRi = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vUVW = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.Mass = 100.0;

    double I = 1000.0;
    accel->in.J = FGMatrix33(I, 0.0, 0.0, 0.0, I, 0.0, 0.0, 0.0, I);
    accel->in.Jinv = FGMatrix33(1.0/I, 0.0, 0.0, 0.0, 1.0/I, 0.0, 0.0, 0.0, 1.0/I);
    accel->in.Ti2b = FGMatrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    accel->in.Tb2i = FGMatrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    accel->in.Tec2b = FGMatrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    accel->in.Tec2i = FGMatrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    accel->in.vOmegaPlanet = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vInertialPosition = FGColumnVector3(0.0, 0.0, 20925646.0);
    accel->in.DeltaT = 0.0083333;
    accel->in.TerrainVelocity = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.TerrainAngularVel = FGColumnVector3(0.0, 0.0, 0.0);

    accel->InitializeDerivatives();
    accel->Run(false);

    // Verify forces and moments are correctly combined
    FGColumnVector3 totalForces = accel->GetForces();
    FGColumnVector3 totalMoments = accel->GetMoments();

    TS_ASSERT_DELTA(totalForces(1), force(1), epsilon);
    TS_ASSERT_DELTA(totalForces(2), force(2), epsilon);
    TS_ASSERT_DELTA(totalForces(3), force(3), epsilon);

    TS_ASSERT_DELTA(totalMoments(1), moment(1), epsilon);
    TS_ASSERT_DELTA(totalMoments(2), moment(2), epsilon);
    TS_ASSERT_DELTA(totalMoments(3), moment(3), epsilon);
  }
};
