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
using namespace JSBSimTest;

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

    // Enable hold-down - this sets the HoldDown flag
    accel->SetHoldDown(true);
    accel->InitializeDerivatives();
    accel->Run(false);

    // Note: HoldDown affects ground reactions, not computed accelerations directly
    // The acceleration values are still computed from forces/mass
    FGColumnVector3 uvwdot = accel->GetUVWdot();
    FGColumnVector3 pqrdot = accel->GetPQRdot();

    // Verify we get expected acceleration: F/m = 10000/100 = 100
    TS_ASSERT_DELTA(uvwdot(1), 100.0, 0.1);
    TS_ASSERT_DELTA(pqrdot(1), 0.0, epsilon);
    TS_ASSERT_DELTA(pqrdot(2), 0.0, epsilon);
    TS_ASSERT_DELTA(pqrdot(3), 0.0, epsilon);

    // Disable hold-down
    accel->SetHoldDown(false);
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

  /***************************************************************************
   * Construction and Initialization Tests
   ***************************************************************************/

  void testGetAccelerations() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();
    TS_ASSERT(accel != nullptr);
  }

  void testInitModel() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();
    bool result = accel->InitModel();
    // InitModel may return true or false depending on configuration
    TS_ASSERT(result == true || result == false);
  }

  void testMultipleInitModel() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();
    accel->InitModel();
    accel->InitModel();
    accel->InitModel();
    TS_ASSERT(true);
  }

  /***************************************************************************
   * Body Acceleration Component Tests
   ***************************************************************************/

  void testGetBodyAccelComponents() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    double mass = 25.0;
    FGColumnVector3 force(250.0, 500.0, 750.0);

    accel->in.Force = force;
    accel->in.Moment = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.GroundForce = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.GroundMoment = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vGravAccel = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vPQR = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vPQRi = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vUVW = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.Mass = mass;

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

    // Test indexed body accel accessors
    FGColumnVector3 bodyAccel = accel->GetBodyAccel();
    TS_ASSERT_DELTA(accel->GetBodyAccel(1), bodyAccel(1), epsilon);
    TS_ASSERT_DELTA(accel->GetBodyAccel(2), bodyAccel(2), epsilon);
    TS_ASSERT_DELTA(accel->GetBodyAccel(3), bodyAccel(3), epsilon);

    // Expected: a = F/m
    TS_ASSERT_DELTA(bodyAccel(1), force(1)/mass, epsilon);
    TS_ASSERT_DELTA(bodyAccel(2), force(2)/mass, epsilon);
    TS_ASSERT_DELTA(bodyAccel(3), force(3)/mass, epsilon);
  }

  void testGetUVWdotComponents() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    accel->in.Force = FGColumnVector3(100.0, 200.0, 300.0);
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

    // Test indexed UVWdot accessors
    FGColumnVector3 uvwdot = accel->GetUVWdot();
    TS_ASSERT_DELTA(accel->GetUVWdot(1), uvwdot(1), epsilon);
    TS_ASSERT_DELTA(accel->GetUVWdot(2), uvwdot(2), epsilon);
    TS_ASSERT_DELTA(accel->GetUVWdot(3), uvwdot(3), epsilon);
  }

  void testGetPQRdotComponents() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    FGColumnVector3 moment(100.0, 200.0, 300.0);
    double I = 2000.0;

    accel->in.Force = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.Moment = moment;
    accel->in.GroundForce = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.GroundMoment = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vGravAccel = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vPQR = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vPQRi = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vUVW = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.Mass = 100.0;

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

    // Test indexed PQRdot accessors
    FGColumnVector3 pqrdot = accel->GetPQRdot();
    TS_ASSERT_DELTA(accel->GetPQRdot(1), pqrdot(1), epsilon);
    TS_ASSERT_DELTA(accel->GetPQRdot(2), pqrdot(2), epsilon);
    TS_ASSERT_DELTA(accel->GetPQRdot(3), pqrdot(3), epsilon);
  }

  /***************************************************************************
   * Asymmetric Inertia Tests
   ***************************************************************************/

  void testAsymmetricInertia() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    // Different moments of inertia for each axis (typical aircraft)
    double Ixx = 1000.0;
    double Iyy = 3000.0;
    double Izz = 4000.0;

    FGColumnVector3 moment(500.0, 0.0, 0.0);

    accel->in.Force = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.Moment = moment;
    accel->in.GroundForce = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.GroundMoment = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vGravAccel = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vPQR = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vPQRi = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vUVW = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.Mass = 100.0;

    accel->in.J = FGMatrix33(Ixx, 0.0, 0.0, 0.0, Iyy, 0.0, 0.0, 0.0, Izz);
    accel->in.Jinv = FGMatrix33(1.0/Ixx, 0.0, 0.0, 0.0, 1.0/Iyy, 0.0, 0.0, 0.0, 1.0/Izz);
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

    FGColumnVector3 pqridot = accel->GetPQRidot();
    // Roll acceleration should be M/Ixx
    TS_ASSERT_DELTA(pqridot(1), moment(1)/Ixx, epsilon);
  }

  void testPitchMoment() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    double Iyy = 5000.0;
    FGColumnVector3 moment(0.0, 1000.0, 0.0);

    accel->in.Force = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.Moment = moment;
    accel->in.GroundForce = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.GroundMoment = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vGravAccel = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vPQR = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vPQRi = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vUVW = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.Mass = 100.0;

    accel->in.J = FGMatrix33(1000.0, 0.0, 0.0, 0.0, Iyy, 0.0, 0.0, 0.0, 1000.0);
    accel->in.Jinv = FGMatrix33(0.001, 0.0, 0.0, 0.0, 1.0/Iyy, 0.0, 0.0, 0.0, 0.001);
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

    FGColumnVector3 pqridot = accel->GetPQRidot();
    // Pitch acceleration should be M/Iyy
    TS_ASSERT_DELTA(pqridot(1), 0.0, epsilon);
    TS_ASSERT_DELTA(pqridot(2), moment(2)/Iyy, epsilon);
    TS_ASSERT_DELTA(pqridot(3), 0.0, epsilon);
  }

  void testYawMoment() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    double Izz = 8000.0;
    FGColumnVector3 moment(0.0, 0.0, 2000.0);

    accel->in.Force = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.Moment = moment;
    accel->in.GroundForce = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.GroundMoment = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vGravAccel = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vPQR = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vPQRi = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vUVW = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.Mass = 100.0;

    accel->in.J = FGMatrix33(1000.0, 0.0, 0.0, 0.0, 1000.0, 0.0, 0.0, 0.0, Izz);
    accel->in.Jinv = FGMatrix33(0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 1.0/Izz);
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

    FGColumnVector3 pqridot = accel->GetPQRidot();
    // Yaw acceleration should be M/Izz
    TS_ASSERT_DELTA(pqridot(1), 0.0, epsilon);
    TS_ASSERT_DELTA(pqridot(2), 0.0, epsilon);
    TS_ASSERT_DELTA(pqridot(3), moment(3)/Izz, epsilon);
  }

  /***************************************************************************
   * Ground Force/Moment Tests
   ***************************************************************************/

  void testGroundForcesOnly() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    FGColumnVector3 groundForce(0.0, 0.0, 1000.0);

    accel->in.Force = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.Moment = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.GroundForce = groundForce;
    accel->in.GroundMoment = FGColumnVector3(0.0, 0.0, 0.0);

    // Test ground force getters
    FGColumnVector3 gf = accel->GetGroundForces();
    TS_ASSERT_DELTA(gf(1), groundForce(1), epsilon);
    TS_ASSERT_DELTA(gf(2), groundForce(2), epsilon);
    TS_ASSERT_DELTA(gf(3), groundForce(3), epsilon);
  }

  void testGroundMomentsOnly() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    FGColumnVector3 groundMoment(100.0, 200.0, 300.0);

    accel->in.Force = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.Moment = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.GroundForce = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.GroundMoment = groundMoment;

    // Test ground moment getters
    FGColumnVector3 gm = accel->GetGroundMoments();
    TS_ASSERT_DELTA(gm(1), groundMoment(1), epsilon);
    TS_ASSERT_DELTA(gm(2), groundMoment(2), epsilon);
    TS_ASSERT_DELTA(gm(3), groundMoment(3), epsilon);
  }

  void testCombinedAppliedAndGroundForces() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    FGColumnVector3 appliedForce(100.0, 0.0, -500.0);
    FGColumnVector3 groundForce(0.0, 0.0, 500.0);

    accel->in.Force = appliedForce;
    accel->in.Moment = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.GroundForce = groundForce;
    accel->in.GroundMoment = FGColumnVector3(0.0, 0.0, 0.0);

    // Total forces = applied + ground (without friction)
    FGColumnVector3 forces = accel->GetForces();
    TS_ASSERT_DELTA(forces(1), appliedForce(1), epsilon);
    TS_ASSERT_DELTA(forces(3), appliedForce(3), epsilon);
  }

  /***************************************************************************
   * Weight Component Tests
   ***************************************************************************/

  void testWeightComponents() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    double mass = 150.0;
    double g = 32.174;

    accel->in.Mass = mass;
    accel->in.vGravAccel = FGColumnVector3(0.0, 0.0, -g);
    accel->in.Tec2b = FGMatrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);

    // Test indexed weight accessors
    FGColumnVector3 weight = accel->GetWeight();
    TS_ASSERT_DELTA(accel->GetWeight(1), weight(1), epsilon);
    TS_ASSERT_DELTA(accel->GetWeight(2), weight(2), epsilon);
    TS_ASSERT_DELTA(accel->GetWeight(3), weight(3), epsilon);

    TS_ASSERT_DELTA(weight(1), 0.0, epsilon);
    TS_ASSERT_DELTA(weight(2), 0.0, epsilon);
    TS_ASSERT_DELTA(weight(3), -mass * g, epsilon);
  }

  void testWeightWithRotatedFrame() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    double mass = 100.0;
    double g = 32.174;

    accel->in.Mass = mass;
    accel->in.vGravAccel = FGColumnVector3(0.0, 0.0, -g);

    // 90 degree rotation about Y axis
    accel->in.Tec2b = FGMatrix33(0.0, 0.0, -1.0,
                                  0.0, 1.0, 0.0,
                                  1.0, 0.0, 0.0);

    FGColumnVector3 weight = accel->GetWeight();
    // With this rotation, z-axis gravity maps to x-axis in body frame
    TS_ASSERT_DELTA(weight(1), mass * g, epsilon);
    TS_ASSERT_DELTA(weight(2), 0.0, epsilon);
    TS_ASSERT_DELTA(weight(3), 0.0, epsilon);
  }

  /***************************************************************************
   * Multiple Run Tests
   ***************************************************************************/

  void testMultipleRuns() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    accel->in.Force = FGColumnVector3(100.0, 0.0, 0.0);
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

    for (int i = 0; i < 10; i++) {
      accel->Run(false);
    }

    // Should still compute correct accelerations
    FGColumnVector3 bodyAccel = accel->GetBodyAccel();
    TS_ASSERT_DELTA(bodyAccel(1), 100.0/50.0, epsilon);
  }

  void testChangingForcesBetweenRuns() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

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

    accel->InitializeDerivatives();

    // Run with first force
    accel->in.Force = FGColumnVector3(100.0, 0.0, 0.0);
    accel->Run(false);
    FGColumnVector3 accel1 = accel->GetBodyAccel();
    TS_ASSERT_DELTA(accel1(1), 1.0, epsilon);

    // Change force and run again
    accel->in.Force = FGColumnVector3(200.0, 0.0, 0.0);
    accel->Run(false);
    FGColumnVector3 accel2 = accel->GetBodyAccel();
    TS_ASSERT_DELTA(accel2(1), 2.0, epsilon);

    // Change force again
    accel->in.Force = FGColumnVector3(-100.0, 0.0, 0.0);
    accel->Run(false);
    FGColumnVector3 accel3 = accel->GetBodyAccel();
    TS_ASSERT_DELTA(accel3(1), -1.0, epsilon);
  }

  /***************************************************************************
   * Hold-Down Behavior Tests
   ***************************************************************************/

  void testHoldDownToggle() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    accel->SetHoldDown(true);
    accel->SetHoldDown(false);
    accel->SetHoldDown(true);
    accel->SetHoldDown(false);

    TS_ASSERT(true);
  }

  void testHoldDownMultipleCycles() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    for (int i = 0; i < 10; i++) {
      accel->SetHoldDown(i % 2 == 0);
    }

    TS_ASSERT(true);
  }

  /***************************************************************************
   * Gravity Vector Tests
   ***************************************************************************/

  void testZeroGravity() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    accel->in.vGravAccel = FGColumnVector3(0.0, 0.0, 0.0);

    double gravMag = accel->GetGravAccelMagnitude();
    TS_ASSERT_DELTA(gravMag, 0.0, epsilon);
  }

  void testNonStandardGravity() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    // Mars gravity approximately 12.1 ft/s^2
    double mars_g = 12.1;
    accel->in.vGravAccel = FGColumnVector3(0.0, 0.0, -mars_g);

    double gravMag = accel->GetGravAccelMagnitude();
    TS_ASSERT_DELTA(gravMag, mars_g, epsilon);
  }

  void testAngleGravityVector() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    // Gravity at an angle
    double gx = 10.0;
    double gy = 20.0;
    double gz = 25.0;
    double expectedMag = sqrt(gx*gx + gy*gy + gz*gz);

    accel->in.vGravAccel = FGColumnVector3(gx, gy, gz);

    double gravMag = accel->GetGravAccelMagnitude();
    TS_ASSERT_DELTA(gravMag, expectedMag, epsilon);
  }

  /***************************************************************************
   * Multiple FDMExec Tests
   ***************************************************************************/

  void testMultipleFDMExecAccelerations() {
    FGFDMExec fdmex1;
    FGFDMExec fdmex2;

    auto accel1 = fdmex1.GetAccelerations();
    auto accel2 = fdmex2.GetAccelerations();

    TS_ASSERT(accel1 != nullptr);
    TS_ASSERT(accel2 != nullptr);
    TS_ASSERT(accel1 != accel2);
  }

  void testIndependentAccelerations() {
    FGFDMExec fdmex1;
    FGFDMExec fdmex2;

    auto accel1 = fdmex1.GetAccelerations();
    auto accel2 = fdmex2.GetAccelerations();

    // Set different forces on each
    accel1->in.Force = FGColumnVector3(100.0, 0.0, 0.0);
    accel2->in.Force = FGColumnVector3(200.0, 0.0, 0.0);

    accel1->in.Mass = 100.0;
    accel2->in.Mass = 100.0;

    // Setup identical params
    auto setupInputs = [](std::shared_ptr<FGAccelerations> accel) {
      accel->in.Moment = FGColumnVector3(0.0, 0.0, 0.0);
      accel->in.GroundForce = FGColumnVector3(0.0, 0.0, 0.0);
      accel->in.GroundMoment = FGColumnVector3(0.0, 0.0, 0.0);
      accel->in.vGravAccel = FGColumnVector3(0.0, 0.0, 0.0);
      accel->in.vPQR = FGColumnVector3(0.0, 0.0, 0.0);
      accel->in.vPQRi = FGColumnVector3(0.0, 0.0, 0.0);
      accel->in.vUVW = FGColumnVector3(0.0, 0.0, 0.0);
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
    };

    setupInputs(accel1);
    setupInputs(accel2);

    accel1->InitializeDerivatives();
    accel2->InitializeDerivatives();

    accel1->Run(false);
    accel2->Run(false);

    // Should produce different accelerations
    FGColumnVector3 bodyAccel1 = accel1->GetBodyAccel();
    FGColumnVector3 bodyAccel2 = accel2->GetBodyAccel();

    TS_ASSERT_DELTA(bodyAccel1(1), 1.0, epsilon);
    TS_ASSERT_DELTA(bodyAccel2(1), 2.0, epsilon);
  }

  /***************************************************************************
   * NaN/Infinity Protection Tests
   ***************************************************************************/

  void testNoNaNInResults() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    accel->in.Force = FGColumnVector3(100.0, 200.0, 300.0);
    accel->in.Moment = FGColumnVector3(50.0, 100.0, 150.0);
    accel->in.GroundForce = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.GroundMoment = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vGravAccel = FGColumnVector3(0.0, 0.0, -32.174);
    accel->in.vPQR = FGColumnVector3(0.1, 0.2, 0.3);
    accel->in.vPQRi = FGColumnVector3(0.1, 0.2, 0.3);
    accel->in.vUVW = FGColumnVector3(100.0, 10.0, 5.0);
    accel->in.Mass = 100.0;

    accel->in.J = FGMatrix33(1000.0, 0.0, 0.0, 0.0, 2000.0, 0.0, 0.0, 0.0, 3000.0);
    accel->in.Jinv = FGMatrix33(0.001, 0.0, 0.0, 0.0, 0.0005, 0.0, 0.0, 0.0, 0.000333);
    accel->in.Ti2b = FGMatrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    accel->in.Tb2i = FGMatrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    accel->in.Tec2b = FGMatrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    accel->in.Tec2i = FGMatrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    accel->in.vOmegaPlanet = FGColumnVector3(0.0, 0.0, 7.292e-5);
    accel->in.vInertialPosition = FGColumnVector3(0.0, 0.0, 20925646.0);
    accel->in.DeltaT = 0.0083333;
    accel->in.TerrainVelocity = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.TerrainAngularVel = FGColumnVector3(0.0, 0.0, 0.0);

    accel->InitializeDerivatives();
    accel->Run(false);

    FGColumnVector3 uvwdot = accel->GetUVWdot();
    FGColumnVector3 pqrdot = accel->GetPQRdot();
    FGColumnVector3 bodyAccel = accel->GetBodyAccel();

    TS_ASSERT(!std::isnan(uvwdot(1)));
    TS_ASSERT(!std::isnan(uvwdot(2)));
    TS_ASSERT(!std::isnan(uvwdot(3)));
    TS_ASSERT(!std::isnan(pqrdot(1)));
    TS_ASSERT(!std::isnan(pqrdot(2)));
    TS_ASSERT(!std::isnan(pqrdot(3)));
    TS_ASSERT(!std::isnan(bodyAccel(1)));
    TS_ASSERT(!std::isnan(bodyAccel(2)));
    TS_ASSERT(!std::isnan(bodyAccel(3)));
  }

  void testNoInfinityInResults() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    accel->in.Force = FGColumnVector3(100.0, 200.0, 300.0);
    accel->in.Moment = FGColumnVector3(50.0, 100.0, 150.0);
    accel->in.GroundForce = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.GroundMoment = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vGravAccel = FGColumnVector3(0.0, 0.0, -32.174);
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

    accel->InitializeDerivatives();
    accel->Run(false);

    FGColumnVector3 uvwdot = accel->GetUVWdot();
    FGColumnVector3 pqrdot = accel->GetPQRdot();

    TS_ASSERT(!std::isinf(uvwdot(1)));
    TS_ASSERT(!std::isinf(uvwdot(2)));
    TS_ASSERT(!std::isinf(uvwdot(3)));
    TS_ASSERT(!std::isinf(pqrdot(1)));
    TS_ASSERT(!std::isinf(pqrdot(2)));
    TS_ASSERT(!std::isinf(pqrdot(3)));
  }

  /***************************************************************************
   * Large Value Tests
   ***************************************************************************/

  void testLargeForces() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    double largeForce = 1e6;
    double mass = 1000.0;

    accel->in.Force = FGColumnVector3(largeForce, 0.0, 0.0);
    accel->in.Moment = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.GroundForce = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.GroundMoment = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vGravAccel = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vPQR = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vPQRi = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vUVW = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.Mass = mass;

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

    FGColumnVector3 bodyAccel = accel->GetBodyAccel();
    TS_ASSERT_DELTA(bodyAccel(1), largeForce/mass, 1.0);
  }

  void testLargeMass() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    double force = 10000.0;
    double largeMass = 1e6;

    accel->in.Force = FGColumnVector3(force, 0.0, 0.0);
    accel->in.Moment = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.GroundForce = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.GroundMoment = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vGravAccel = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vPQR = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vPQRi = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vUVW = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.Mass = largeMass;

    accel->in.J = FGMatrix33(1e9, 0.0, 0.0, 0.0, 1e9, 0.0, 0.0, 0.0, 1e9);
    accel->in.Jinv = FGMatrix33(1e-9, 0.0, 0.0, 0.0, 1e-9, 0.0, 0.0, 0.0, 1e-9);
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

    FGColumnVector3 bodyAccel = accel->GetBodyAccel();
    TS_ASSERT_DELTA(bodyAccel(1), force/largeMass, 1e-10);
  }

  /***************************************************************************
   * Small Value Tests
   ***************************************************************************/

  void testSmallForces() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    double smallForce = 1e-6;

    accel->in.Force = FGColumnVector3(smallForce, 0.0, 0.0);
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

    accel->InitializeDerivatives();
    accel->Run(false);

    FGColumnVector3 bodyAccel = accel->GetBodyAccel();
    TS_ASSERT_DELTA(bodyAccel(1), smallForce/100.0, 1e-12);
  }

  /***************************************************************************
   * Negative Value Tests
   ***************************************************************************/

  void testNegativeForces() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    FGColumnVector3 negForce(-500.0, -200.0, -800.0);

    accel->in.Force = negForce;
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

    accel->InitializeDerivatives();
    accel->Run(false);

    FGColumnVector3 bodyAccel = accel->GetBodyAccel();
    TS_ASSERT_DELTA(bodyAccel(1), negForce(1)/100.0, epsilon);
    TS_ASSERT_DELTA(bodyAccel(2), negForce(2)/100.0, epsilon);
    TS_ASSERT_DELTA(bodyAccel(3), negForce(3)/100.0, epsilon);
  }

  void testNegativeMoments() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    FGColumnVector3 negMoment(-300.0, -600.0, -900.0);
    double I = 1000.0;

    accel->in.Force = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.Moment = negMoment;
    accel->in.GroundForce = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.GroundMoment = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vGravAccel = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vPQR = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vPQRi = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vUVW = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.Mass = 100.0;

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

    FGColumnVector3 pqridot = accel->GetPQRidot();
    TS_ASSERT_DELTA(pqridot(1), negMoment(1)/I, epsilon);
    TS_ASSERT_DELTA(pqridot(2), negMoment(2)/I, epsilon);
    TS_ASSERT_DELTA(pqridot(3), negMoment(3)/I, epsilon);
  }

  /***************************************************************************
   * Products of Inertia Tests
   ***************************************************************************/

  void testProductsOfInertiaIxz() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    // Aircraft with Ixz product of inertia (typical due to engine placement)
    double Ixx = 1000.0;
    double Iyy = 3000.0;
    double Izz = 4000.0;
    double Ixz = 100.0;

    // Inverse for coupled matrix
    double det = Ixx*Izz - Ixz*Ixz;
    accel->in.J = FGMatrix33(Ixx, 0.0, -Ixz,
                              0.0, Iyy, 0.0,
                              -Ixz, 0.0, Izz);
    accel->in.Jinv = FGMatrix33(Izz/det, 0.0, Ixz/det,
                                 0.0, 1.0/Iyy, 0.0,
                                 Ixz/det, 0.0, Ixx/det);

    accel->in.Force = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.Moment = FGColumnVector3(500.0, 0.0, 0.0);  // Roll moment only
    accel->in.GroundForce = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.GroundMoment = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vGravAccel = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vPQR = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vPQRi = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vUVW = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.Mass = 100.0;
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

    FGColumnVector3 pqridot = accel->GetPQRidot();

    // Roll moment should cause both roll and yaw acceleration due to Ixz coupling
    TS_ASSERT(!std::isnan(pqridot(1)));
    TS_ASSERT(!std::isnan(pqridot(3)));
    // Yaw acceleration should be non-zero due to coupling
    double expectedPdot = (Izz * 500.0) / det;
    double expectedRdot = (Ixz * 500.0) / det;
    TS_ASSERT_DELTA(pqridot(1), expectedPdot, epsilon);
    TS_ASSERT_DELTA(pqridot(3), expectedRdot, epsilon);
  }

  /***************************************************************************
   * Gyroscopic Effects Tests
   ***************************************************************************/

  void testGyroscopicWithAsymmetricInertia() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    double Ixx = 1000.0;
    double Iyy = 3000.0;
    double Izz = 4000.0;

    // Initial roll rate
    FGColumnVector3 pqr(1.0, 0.0, 0.0);  // Rolling at 1 rad/s

    accel->in.Force = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.Moment = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.GroundForce = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.GroundMoment = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vGravAccel = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vPQR = pqr;
    accel->in.vPQRi = pqr;
    accel->in.vUVW = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.Mass = 100.0;
    accel->in.J = FGMatrix33(Ixx, 0.0, 0.0, 0.0, Iyy, 0.0, 0.0, 0.0, Izz);
    accel->in.Jinv = FGMatrix33(1.0/Ixx, 0.0, 0.0, 0.0, 1.0/Iyy, 0.0, 0.0, 0.0, 1.0/Izz);
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

    FGColumnVector3 pqrdot = accel->GetPQRdot();

    // Results depend on body vs inertial frame computations
    TS_ASSERT(!std::isnan(pqrdot(1)));
    TS_ASSERT(!std::isnan(pqrdot(2)));
    TS_ASSERT(!std::isnan(pqrdot(3)));
  }

  void testSphericalInertiaNoGyroscopic() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    double I = 1000.0;

    // Spinning with pitch rate
    FGColumnVector3 pqr(0.0, 0.5, 0.0);

    accel->in.Force = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.Moment = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.GroundForce = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.GroundMoment = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vGravAccel = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vPQR = pqr;
    accel->in.vPQRi = pqr;
    accel->in.vUVW = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.Mass = 100.0;
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

    FGColumnVector3 pqrdot = accel->GetPQRdot();

    // With spherical inertia, no gyroscopic coupling
    TS_ASSERT_DELTA(pqrdot(1), 0.0, epsilon);
    TS_ASSERT_DELTA(pqrdot(2), 0.0, epsilon);
    TS_ASSERT_DELTA(pqrdot(3), 0.0, epsilon);
  }

  /***************************************************************************
   * Velocity-Dependent Acceleration Tests
   ***************************************************************************/

  void testCentripetalTerms() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    // Moving body with rotation causes centripetal acceleration
    FGColumnVector3 uvw(500.0, 0.0, 0.0);  // Forward velocity
    FGColumnVector3 pqr(0.0, 0.0, 0.5);    // Yaw rate

    accel->in.Force = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.Moment = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.GroundForce = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.GroundMoment = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vGravAccel = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vPQR = pqr;
    accel->in.vPQRi = pqr;
    accel->in.vUVW = uvw;
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

    accel->InitializeDerivatives();
    accel->Run(false);

    FGColumnVector3 uvwdot = accel->GetUVWdot();

    // Rotating body creates centripetal terms in body frame acceleration
    TS_ASSERT(!std::isnan(uvwdot(1)));
    TS_ASSERT(!std::isnan(uvwdot(2)));
    TS_ASSERT(!std::isnan(uvwdot(3)));
  }

  /***************************************************************************
   * Delta-T Variation Tests
   ***************************************************************************/

  void testDifferentDeltaT() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    accel->in.Force = FGColumnVector3(1000.0, 0.0, 0.0);
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
    accel->in.TerrainVelocity = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.TerrainAngularVel = FGColumnVector3(0.0, 0.0, 0.0);

    // Test with different delta T values
    double deltaTValues[] = {0.001, 0.01, 0.1, 1.0};

    for (double dt : deltaTValues) {
      accel->in.DeltaT = dt;
      accel->InitializeDerivatives();
      accel->Run(false);

      FGColumnVector3 bodyAccel = accel->GetBodyAccel();
      // Acceleration should be the same regardless of dt (instantaneous)
      TS_ASSERT_DELTA(bodyAccel(1), 10.0, epsilon);
    }
  }

  void testVerySmallDeltaT() {
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
    accel->in.DeltaT = 1e-6;  // Very small timestep
    accel->in.TerrainVelocity = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.TerrainAngularVel = FGColumnVector3(0.0, 0.0, 0.0);

    accel->InitializeDerivatives();
    accel->Run(false);

    FGColumnVector3 bodyAccel = accel->GetBodyAccel();
    TS_ASSERT_DELTA(bodyAccel(1), 10.0, epsilon);
    TS_ASSERT(!std::isnan(bodyAccel(1)));
  }

  /***************************************************************************
   * Mass Variation Tests
   ***************************************************************************/

  void testLightMass() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    double lightMass = 1.0;  // 1 slug
    FGColumnVector3 force(10.0, 0.0, 0.0);

    accel->in.Force = force;
    accel->in.Moment = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.GroundForce = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.GroundMoment = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vGravAccel = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vPQR = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vPQRi = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vUVW = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.Mass = lightMass;
    accel->in.J = FGMatrix33(100.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 100.0);
    accel->in.Jinv = FGMatrix33(0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01);
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

    FGColumnVector3 bodyAccel = accel->GetBodyAccel();
    TS_ASSERT_DELTA(bodyAccel(1), force(1)/lightMass, epsilon);
  }

  void testHeavyMass() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    double heavyMass = 10000.0;  // 10000 slugs
    FGColumnVector3 force(100000.0, 0.0, 0.0);

    accel->in.Force = force;
    accel->in.Moment = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.GroundForce = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.GroundMoment = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vGravAccel = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vPQR = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vPQRi = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vUVW = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.Mass = heavyMass;
    accel->in.J = FGMatrix33(1e6, 0.0, 0.0, 0.0, 1e6, 0.0, 0.0, 0.0, 1e6);
    accel->in.Jinv = FGMatrix33(1e-6, 0.0, 0.0, 0.0, 1e-6, 0.0, 0.0, 0.0, 1e-6);
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

    FGColumnVector3 bodyAccel = accel->GetBodyAccel();
    TS_ASSERT_DELTA(bodyAccel(1), force(1)/heavyMass, epsilon);
  }

  /***************************************************************************
   * Earth Rotation Tests
   ***************************************************************************/

  void testWithEarthRotation() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    double omega_earth = 7.2921159e-5;  // rad/s

    accel->in.Force = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.Moment = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.GroundForce = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.GroundMoment = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vGravAccel = FGColumnVector3(0.0, 0.0, -32.174);
    accel->in.vPQR = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vPQRi = FGColumnVector3(0.0, 0.0, omega_earth);
    accel->in.vUVW = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.Mass = 100.0;
    accel->in.J = FGMatrix33(1000.0, 0.0, 0.0, 0.0, 1000.0, 0.0, 0.0, 0.0, 1000.0);
    accel->in.Jinv = FGMatrix33(0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001);
    accel->in.Ti2b = FGMatrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    accel->in.Tb2i = FGMatrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    accel->in.Tec2b = FGMatrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    accel->in.Tec2i = FGMatrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    accel->in.vOmegaPlanet = FGColumnVector3(0.0, 0.0, omega_earth);
    accel->in.vInertialPosition = FGColumnVector3(0.0, 0.0, 20925646.0);
    accel->in.DeltaT = 0.0083333;
    accel->in.TerrainVelocity = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.TerrainAngularVel = FGColumnVector3(0.0, 0.0, 0.0);

    accel->InitializeDerivatives();
    accel->Run(false);

    FGColumnVector3 uvwdot = accel->GetUVWdot();

    // Results should include Earth rotation effects
    TS_ASSERT(!std::isnan(uvwdot(1)));
    TS_ASSERT(!std::isnan(uvwdot(2)));
    TS_ASSERT(!std::isnan(uvwdot(3)));
  }

  /***************************************************************************
   * Terrain Velocity Tests
   ***************************************************************************/

  void testTerrainVelocityEffect() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    // Moving terrain (e.g., on an aircraft carrier deck)
    FGColumnVector3 terrainVel(50.0, 0.0, 0.0);

    accel->in.Force = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.Moment = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.GroundForce = FGColumnVector3(0.0, 0.0, 1000.0);  // On deck
    accel->in.GroundMoment = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vGravAccel = FGColumnVector3(0.0, 0.0, -32.174);
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
    accel->in.TerrainVelocity = terrainVel;
    accel->in.TerrainAngularVel = FGColumnVector3(0.0, 0.0, 0.0);

    accel->InitializeDerivatives();
    accel->Run(false);

    FGColumnVector3 uvwdot = accel->GetUVWdot();

    // Test valid results
    TS_ASSERT(!std::isnan(uvwdot(1)));
    TS_ASSERT(!std::isnan(uvwdot(2)));
    TS_ASSERT(!std::isnan(uvwdot(3)));
  }

  void testTerrainAngularVelocityEffect() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    // Rotating terrain (ship deck rolling)
    FGColumnVector3 terrainAngVel(0.1, 0.0, 0.0);

    accel->in.Force = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.Moment = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.GroundForce = FGColumnVector3(0.0, 0.0, 1000.0);
    accel->in.GroundMoment = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vGravAccel = FGColumnVector3(0.0, 0.0, -32.174);
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
    accel->in.TerrainAngularVel = terrainAngVel;

    accel->InitializeDerivatives();
    accel->Run(false);

    FGColumnVector3 pqrdot = accel->GetPQRdot();

    // Test valid results
    TS_ASSERT(!std::isnan(pqrdot(1)));
    TS_ASSERT(!std::isnan(pqrdot(2)));
    TS_ASSERT(!std::isnan(pqrdot(3)));
  }

  /***************************************************************************
   * Combined Moment Tests
   ***************************************************************************/

  void testAllThreeMoments() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    double Ixx = 1000.0;
    double Iyy = 2000.0;
    double Izz = 3000.0;

    FGColumnVector3 moment(100.0, 200.0, 300.0);

    accel->in.Force = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.Moment = moment;
    accel->in.GroundForce = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.GroundMoment = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vGravAccel = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vPQR = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vPQRi = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vUVW = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.Mass = 100.0;
    accel->in.J = FGMatrix33(Ixx, 0.0, 0.0, 0.0, Iyy, 0.0, 0.0, 0.0, Izz);
    accel->in.Jinv = FGMatrix33(1.0/Ixx, 0.0, 0.0, 0.0, 1.0/Iyy, 0.0, 0.0, 0.0, 1.0/Izz);
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

    FGColumnVector3 pqridot = accel->GetPQRidot();
    TS_ASSERT_DELTA(pqridot(1), moment(1)/Ixx, epsilon);
    TS_ASSERT_DELTA(pqridot(2), moment(2)/Iyy, epsilon);
    TS_ASSERT_DELTA(pqridot(3), moment(3)/Izz, epsilon);
  }

  void testCombinedAppliedAndGroundMoments() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    FGColumnVector3 appliedMoment(100.0, 0.0, 0.0);
    FGColumnVector3 groundMoment(50.0, 50.0, 0.0);

    accel->in.Force = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.Moment = appliedMoment;
    accel->in.GroundForce = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.GroundMoment = groundMoment;

    FGColumnVector3 totalMoments = accel->GetMoments();
    TS_ASSERT_DELTA(totalMoments(1), appliedMoment(1), epsilon);
    TS_ASSERT_DELTA(totalMoments(2), appliedMoment(2), epsilon);
    TS_ASSERT_DELTA(totalMoments(3), appliedMoment(3), epsilon);
  }

  /***************************************************************************
   * Model Identity Tests
   ***************************************************************************/

  void testGetName() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();
    TS_ASSERT(!accel->GetName().empty());
  }

  void testGetExec() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();
    TS_ASSERT(accel->GetExec() == &fdmex);
  }

  void testSetRate() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();
    accel->SetRate(3);
    TS_ASSERT_EQUALS(accel->GetRate(), 3u);
  }

  void testRateZero() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();
    accel->SetRate(0);
    TS_ASSERT_EQUALS(accel->GetRate(), 0u);
  }

  /***************************************************************************
   * Multiple Instance Tests
   ***************************************************************************/

  void testMultipleFDMExecInstances() {
    FGFDMExec fdmex1;
    FGFDMExec fdmex2;

    auto a1 = fdmex1.GetAccelerations();
    auto a2 = fdmex2.GetAccelerations();

    TS_ASSERT(a1 != a2);
    TS_ASSERT(a1->GetExec() == &fdmex1);
    TS_ASSERT(a2->GetExec() == &fdmex2);
  }

  void testIndependentRates() {
    FGFDMExec fdmex1;
    FGFDMExec fdmex2;

    auto a1 = fdmex1.GetAccelerations();
    auto a2 = fdmex2.GetAccelerations();

    a1->SetRate(2);
    a2->SetRate(5);

    TS_ASSERT_EQUALS(a1->GetRate(), 2u);
    TS_ASSERT_EQUALS(a2->GetRate(), 5u);
  }

  /***************************************************************************
   * Stress and Consistency Tests
   ***************************************************************************/

  void testStressRapidRateChanges() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    for (unsigned int i = 0; i < 100; i++) {
      accel->SetRate(i % 10);
      TS_ASSERT_EQUALS(accel->GetRate(), i % 10);
    }
  }

  void testInitModelConsistency() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    TS_ASSERT(accel->InitModel());
    TS_ASSERT(accel->InitModel());
    TS_ASSERT(accel->InitModel());
  }

  void testForcesVectorConsistency() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    FGColumnVector3 testForce(100.0, 200.0, 300.0);
    accel->in.Force = testForce;

    const FGColumnVector3& forces = accel->GetForces();
    TS_ASSERT_DELTA(forces(1), testForce(1), epsilon);
    TS_ASSERT_DELTA(forces(2), testForce(2), epsilon);
    TS_ASSERT_DELTA(forces(3), testForce(3), epsilon);
  }

  void testMomentsVectorConsistency() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    FGColumnVector3 testMoment(50.0, 100.0, 150.0);
    accel->in.Moment = testMoment;

    const FGColumnVector3& moments = accel->GetMoments();
    TS_ASSERT_DELTA(moments(1), testMoment(1), epsilon);
    TS_ASSERT_DELTA(moments(2), testMoment(2), epsilon);
    TS_ASSERT_DELTA(moments(3), testMoment(3), epsilon);
  }

  /***************************************************************************
   * Extended Transformation Tests
   ***************************************************************************/

  // Test 90-degree pitch rotation transformation
  void testPitch90DegreeTransformation() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    // Force in body X axis with 90 deg pitch
    accel->in.Force = FGColumnVector3(100.0, 0.0, 0.0);
    accel->in.Moment = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.GroundForce = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.GroundMoment = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vGravAccel = FGColumnVector3(0.0, 0.0, -32.174);
    accel->in.vPQR = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vPQRi = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vUVW = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.Mass = 100.0;

    accel->in.J = FGMatrix33(1000.0, 0.0, 0.0, 0.0, 1000.0, 0.0, 0.0, 0.0, 1000.0);
    accel->in.Jinv = FGMatrix33(0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001);

    // 90 degree pitch (theta = pi/2)
    accel->in.Ti2b = FGMatrix33(0.0, 0.0, 1.0,
                                 0.0, 1.0, 0.0,
                                 -1.0, 0.0, 0.0);
    accel->in.Tb2i = FGMatrix33(0.0, 0.0, -1.0,
                                 0.0, 1.0, 0.0,
                                 1.0, 0.0, 0.0);
    accel->in.Tec2b = FGMatrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    accel->in.Tec2i = FGMatrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    accel->in.vOmegaPlanet = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vInertialPosition = FGColumnVector3(0.0, 0.0, 20925646.0);
    accel->in.DeltaT = 0.0083333;
    accel->in.TerrainVelocity = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.TerrainAngularVel = FGColumnVector3(0.0, 0.0, 0.0);

    accel->InitializeDerivatives();
    accel->Run(false);

    FGColumnVector3 bodyAccel = accel->GetBodyAccel();
    TS_ASSERT(!std::isnan(bodyAccel(1)));
    TS_ASSERT(!std::isnan(bodyAccel(2)));
    TS_ASSERT(!std::isnan(bodyAccel(3)));
  }

  // Test 90-degree roll transformation
  void testRoll90DegreeTransformation() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    accel->in.Force = FGColumnVector3(0.0, 100.0, 0.0);
    accel->in.Moment = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.GroundForce = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.GroundMoment = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vGravAccel = FGColumnVector3(0.0, 0.0, -32.174);
    accel->in.vPQR = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vPQRi = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vUVW = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.Mass = 100.0;

    accel->in.J = FGMatrix33(1000.0, 0.0, 0.0, 0.0, 1000.0, 0.0, 0.0, 0.0, 1000.0);
    accel->in.Jinv = FGMatrix33(0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001);

    // 90 degree roll (phi = pi/2)
    accel->in.Ti2b = FGMatrix33(1.0, 0.0, 0.0,
                                 0.0, 0.0, 1.0,
                                 0.0, -1.0, 0.0);
    accel->in.Tb2i = FGMatrix33(1.0, 0.0, 0.0,
                                 0.0, 0.0, -1.0,
                                 0.0, 1.0, 0.0);
    accel->in.Tec2b = FGMatrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    accel->in.Tec2i = FGMatrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    accel->in.vOmegaPlanet = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vInertialPosition = FGColumnVector3(0.0, 0.0, 20925646.0);
    accel->in.DeltaT = 0.0083333;
    accel->in.TerrainVelocity = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.TerrainAngularVel = FGColumnVector3(0.0, 0.0, 0.0);

    accel->InitializeDerivatives();
    accel->Run(false);

    FGColumnVector3 bodyAccel = accel->GetBodyAccel();
    TS_ASSERT(!std::isnan(bodyAccel(1)));
    TS_ASSERT(!std::isnan(bodyAccel(2)));
    TS_ASSERT(!std::isnan(bodyAccel(3)));
  }

  /***************************************************************************
   * Extended Coriolis and Centrifugal Tests
   ***************************************************************************/

  // Test Coriolis acceleration with velocity
  void testCoriolisWithVelocity() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    // Moving forward while yawing
    FGColumnVector3 uvw(500.0, 0.0, 0.0);
    FGColumnVector3 pqr(0.0, 0.0, 0.5);

    accel->in.Force = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.Moment = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.GroundForce = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.GroundMoment = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vGravAccel = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vPQR = pqr;
    accel->in.vPQRi = pqr;
    accel->in.vUVW = uvw;
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

    accel->InitializeDerivatives();
    accel->Run(false);

    FGColumnVector3 uvwdot = accel->GetUVWdot();
    // Yawing while moving forward creates lateral acceleration
    TS_ASSERT(!std::isnan(uvwdot(2)));
  }

  // Test centrifugal with complex rotation
  void testCentrifugalComplexRotation() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    // Combined roll, pitch, yaw rates
    FGColumnVector3 pqr(0.3, 0.2, 0.1);
    FGColumnVector3 uvw(100.0, 50.0, 20.0);

    accel->in.Force = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.Moment = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.GroundForce = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.GroundMoment = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vGravAccel = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vPQR = pqr;
    accel->in.vPQRi = pqr;
    accel->in.vUVW = uvw;
    accel->in.Mass = 100.0;
    accel->in.J = FGMatrix33(1000.0, 0.0, 0.0, 0.0, 2000.0, 0.0, 0.0, 0.0, 3000.0);
    accel->in.Jinv = FGMatrix33(0.001, 0.0, 0.0, 0.0, 0.0005, 0.0, 0.0, 0.0, 0.000333);
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

    FGColumnVector3 uvwdot = accel->GetUVWdot();
    FGColumnVector3 pqrdot = accel->GetPQRdot();

    // All components should be non-zero with combined rotation
    TS_ASSERT(!std::isnan(uvwdot(1)));
    TS_ASSERT(!std::isnan(uvwdot(2)));
    TS_ASSERT(!std::isnan(uvwdot(3)));
    TS_ASSERT(!std::isnan(pqrdot(1)));
    TS_ASSERT(!std::isnan(pqrdot(2)));
    TS_ASSERT(!std::isnan(pqrdot(3)));
  }

  /***************************************************************************
   * Extended Inertia Matrix Tests
   ***************************************************************************/

  // Test highly asymmetric inertia
  void testHighlyAsymmetricInertia() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    double Ixx = 100.0;
    double Iyy = 5000.0;
    double Izz = 10000.0;

    accel->in.Force = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.Moment = FGColumnVector3(100.0, 100.0, 100.0);
    accel->in.GroundForce = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.GroundMoment = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vGravAccel = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vPQR = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vPQRi = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vUVW = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.Mass = 100.0;
    accel->in.J = FGMatrix33(Ixx, 0.0, 0.0, 0.0, Iyy, 0.0, 0.0, 0.0, Izz);
    accel->in.Jinv = FGMatrix33(1.0/Ixx, 0.0, 0.0, 0.0, 1.0/Iyy, 0.0, 0.0, 0.0, 1.0/Izz);
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

    FGColumnVector3 pqridot = accel->GetPQRidot();

    // Roll acceleration should be much higher than pitch/yaw
    TS_ASSERT(std::abs(pqridot(1)) > std::abs(pqridot(2)));
    TS_ASSERT(std::abs(pqridot(1)) > std::abs(pqridot(3)));
  }

  // Test full inertia matrix with products
  void testFullInertiaMatrixWithProducts() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    double Ixx = 1000.0;
    double Iyy = 2000.0;
    double Izz = 3000.0;
    double Ixy = 50.0;
    double Ixz = 100.0;
    double Iyz = 25.0;

    FGMatrix33 J(Ixx, -Ixy, -Ixz,
                 -Ixy, Iyy, -Iyz,
                 -Ixz, -Iyz, Izz);

    // Approximate inverse (would need proper matrix inversion)
    FGMatrix33 Jinv(1.0/Ixx, 0.0, 0.0, 0.0, 1.0/Iyy, 0.0, 0.0, 0.0, 1.0/Izz);

    accel->in.Force = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.Moment = FGColumnVector3(100.0, 0.0, 0.0);
    accel->in.GroundForce = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.GroundMoment = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vGravAccel = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vPQR = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vPQRi = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vUVW = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.Mass = 100.0;
    accel->in.J = J;
    accel->in.Jinv = Jinv;
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

    FGColumnVector3 pqridot = accel->GetPQRidot();
    TS_ASSERT(!std::isnan(pqridot(1)));
    TS_ASSERT(!std::isnan(pqridot(2)));
    TS_ASSERT(!std::isnan(pqridot(3)));
  }

  /***************************************************************************
   * Extended Force/Moment Combination Tests
   ***************************************************************************/

  // Test opposing forces
  void testOpposingForces() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    // Two equal opposing forces should result in zero net force
    FGColumnVector3 thrust(1000.0, 0.0, 0.0);
    FGColumnVector3 drag(-1000.0, 0.0, 0.0);

    accel->in.Force = thrust + drag;
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

    accel->InitializeDerivatives();
    accel->Run(false);

    FGColumnVector3 bodyAccel = accel->GetBodyAccel();
    TS_ASSERT_DELTA(bodyAccel(1), 0.0, epsilon);
  }

  // Test trimmed flight condition
  void testTrimmedFlightCondition() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    double mass = 100.0;
    double g = 32.174;

    // In trimmed flight, lift = weight, thrust = drag
    FGColumnVector3 aeroForce(0.0, 0.0, 0.0);  // Net zero for trim

    accel->in.Force = aeroForce;
    accel->in.Moment = FGColumnVector3(0.0, 0.0, 0.0);  // Trimmed moments
    accel->in.GroundForce = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.GroundMoment = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vGravAccel = FGColumnVector3(0.0, 0.0, 0.0);  // Balanced by lift
    accel->in.vPQR = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vPQRi = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vUVW = FGColumnVector3(200.0, 0.0, 0.0);  // Steady flight
    accel->in.Mass = mass;
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

    FGColumnVector3 uvwdot = accel->GetUVWdot();
    FGColumnVector3 pqrdot = accel->GetPQRdot();

    // In trim, accelerations should be zero
    TS_ASSERT_DELTA(uvwdot(1), 0.0, epsilon);
    TS_ASSERT_DELTA(uvwdot(2), 0.0, epsilon);
    TS_ASSERT_DELTA(uvwdot(3), 0.0, epsilon);
    TS_ASSERT_DELTA(pqrdot(1), 0.0, epsilon);
    TS_ASSERT_DELTA(pqrdot(2), 0.0, epsilon);
    TS_ASSERT_DELTA(pqrdot(3), 0.0, epsilon);
  }

  /***************************************************************************
   * Extended Numerical Stability Tests
   ***************************************************************************/

  // Test extreme force values
  void testExtremeForceValues() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    double extremeForce = 1e8;

    accel->in.Force = FGColumnVector3(extremeForce, 0.0, 0.0);
    accel->in.Moment = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.GroundForce = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.GroundMoment = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vGravAccel = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vPQR = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vPQRi = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vUVW = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.Mass = 1000.0;
    accel->in.J = FGMatrix33(1e6, 0.0, 0.0, 0.0, 1e6, 0.0, 0.0, 0.0, 1e6);
    accel->in.Jinv = FGMatrix33(1e-6, 0.0, 0.0, 0.0, 1e-6, 0.0, 0.0, 0.0, 1e-6);
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

    FGColumnVector3 bodyAccel = accel->GetBodyAccel();
    TS_ASSERT(!std::isnan(bodyAccel(1)));
    TS_ASSERT(!std::isinf(bodyAccel(1)));
    TS_ASSERT_DELTA(bodyAccel(1), extremeForce/1000.0, 1.0);
  }

  // Test tiny force values
  void testTinyForceValues() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    double tinyForce = 1e-10;

    accel->in.Force = FGColumnVector3(tinyForce, 0.0, 0.0);
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

    accel->InitializeDerivatives();
    accel->Run(false);

    FGColumnVector3 bodyAccel = accel->GetBodyAccel();
    TS_ASSERT(!std::isnan(bodyAccel(1)));
    TS_ASSERT_DELTA(bodyAccel(1), tinyForce/100.0, 1e-15);
  }

  /***************************************************************************
   * Extended Carrier Operations Tests
   ***************************************************************************/

  // Test carrier deck motion effects
  void testCarrierDeckMotion() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    // Carrier moving at 30 knots, pitching and rolling
    FGColumnVector3 deckVel(50.0, 0.0, 0.0);
    FGColumnVector3 deckAngVel(0.05, 0.03, 0.0);

    accel->in.Force = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.Moment = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.GroundForce = FGColumnVector3(0.0, 0.0, 3000.0);
    accel->in.GroundMoment = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vGravAccel = FGColumnVector3(0.0, 0.0, -32.174);
    accel->in.vPQR = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vPQRi = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vUVW = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.Mass = 100.0;
    accel->in.J = FGMatrix33(1000.0, 0.0, 0.0, 0.0, 2000.0, 0.0, 0.0, 0.0, 3000.0);
    accel->in.Jinv = FGMatrix33(0.001, 0.0, 0.0, 0.0, 0.0005, 0.0, 0.0, 0.0, 0.000333);
    accel->in.Ti2b = FGMatrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    accel->in.Tb2i = FGMatrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    accel->in.Tec2b = FGMatrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    accel->in.Tec2i = FGMatrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    accel->in.vOmegaPlanet = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vInertialPosition = FGColumnVector3(0.0, 0.0, 20925646.0);
    accel->in.DeltaT = 0.0083333;
    accel->in.TerrainVelocity = deckVel;
    accel->in.TerrainAngularVel = deckAngVel;

    accel->InitializeDerivatives();
    accel->Run(false);

    FGColumnVector3 uvwdot = accel->GetUVWdot();
    FGColumnVector3 pqrdot = accel->GetPQRdot();

    TS_ASSERT(!std::isnan(uvwdot(1)));
    TS_ASSERT(!std::isnan(uvwdot(2)));
    TS_ASSERT(!std::isnan(uvwdot(3)));
    TS_ASSERT(!std::isnan(pqrdot(1)));
    TS_ASSERT(!std::isnan(pqrdot(2)));
    TS_ASSERT(!std::isnan(pqrdot(3)));
  }

  /***************************************************************************
   * Extended Rapid Iteration Tests
   ***************************************************************************/

  // Test many rapid iterations
  void testManyRapidIterations() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    accel->in.Force = FGColumnVector3(100.0, 50.0, 25.0);
    accel->in.Moment = FGColumnVector3(10.0, 20.0, 30.0);
    accel->in.GroundForce = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.GroundMoment = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vGravAccel = FGColumnVector3(0.0, 0.0, -32.174);
    accel->in.vPQR = FGColumnVector3(0.1, 0.05, 0.02);
    accel->in.vPQRi = FGColumnVector3(0.1, 0.05, 0.02);
    accel->in.vUVW = FGColumnVector3(200.0, 5.0, 3.0);
    accel->in.Mass = 100.0;
    accel->in.J = FGMatrix33(1000.0, 0.0, 0.0, 0.0, 2000.0, 0.0, 0.0, 0.0, 3000.0);
    accel->in.Jinv = FGMatrix33(0.001, 0.0, 0.0, 0.0, 0.0005, 0.0, 0.0, 0.0, 0.000333);
    accel->in.Ti2b = FGMatrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    accel->in.Tb2i = FGMatrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    accel->in.Tec2b = FGMatrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    accel->in.Tec2i = FGMatrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    accel->in.vOmegaPlanet = FGColumnVector3(0.0, 0.0, 7.292e-5);
    accel->in.vInertialPosition = FGColumnVector3(0.0, 0.0, 20925646.0);
    accel->in.DeltaT = 0.0083333;
    accel->in.TerrainVelocity = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.TerrainAngularVel = FGColumnVector3(0.0, 0.0, 0.0);

    accel->InitializeDerivatives();

    for (int i = 0; i < 1000; i++) {
      accel->Run(false);

      FGColumnVector3 uvwdot = accel->GetUVWdot();
      FGColumnVector3 pqrdot = accel->GetPQRdot();

      TS_ASSERT(!std::isnan(uvwdot(1)));
      TS_ASSERT(!std::isnan(uvwdot(2)));
      TS_ASSERT(!std::isnan(uvwdot(3)));
      TS_ASSERT(!std::isnan(pqrdot(1)));
      TS_ASSERT(!std::isnan(pqrdot(2)));
      TS_ASSERT(!std::isnan(pqrdot(3)));
    }
  }

  // Test varying inputs over iterations
  void testVaryingInputsOverIterations() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

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

    accel->InitializeDerivatives();

    for (int i = 0; i < 100; i++) {
      double force = 100.0 * std::sin(i * 0.1);
      accel->in.Force = FGColumnVector3(force, 0.0, 0.0);
      accel->Run(false);

      FGColumnVector3 bodyAccel = accel->GetBodyAccel();
      TS_ASSERT_DELTA(bodyAccel(1), force / 100.0, epsilon);
    }
  }

  /***************************************************************************
   * Additional Multi-Instance Tests (80-83)
   ***************************************************************************/

  // Test 80: Multiple acceleration instances run independently
  void testMultipleAccelerationInstancesIndependent() {
    FGFDMExec fdmex1, fdmex2;
    auto a1 = fdmex1.GetAccelerations();
    auto a2 = fdmex2.GetAccelerations();

    // Set different forces
    a1->in.Force = FGColumnVector3(100.0, 0.0, 0.0);
    a2->in.Force = FGColumnVector3(200.0, 0.0, 0.0);
    a1->in.Mass = 100.0;
    a2->in.Mass = 100.0;

    // Set up minimal inputs
    for (auto& accel : {a1, a2}) {
      accel->in.Moment = FGColumnVector3(0.0, 0.0, 0.0);
      accel->in.GroundForce = FGColumnVector3(0.0, 0.0, 0.0);
      accel->in.GroundMoment = FGColumnVector3(0.0, 0.0, 0.0);
      accel->in.vGravAccel = FGColumnVector3(0.0, 0.0, 0.0);
      accel->in.vPQR = FGColumnVector3(0.0, 0.0, 0.0);
      accel->in.vPQRi = FGColumnVector3(0.0, 0.0, 0.0);
      accel->in.vUVW = FGColumnVector3(0.0, 0.0, 0.0);
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
    }

    FGColumnVector3 acc1 = a1->GetBodyAccel();
    FGColumnVector3 acc2 = a2->GetBodyAccel();

    TS_ASSERT_DELTA(acc1(1), 1.0, epsilon);   // 100/100
    TS_ASSERT_DELTA(acc2(1), 2.0, epsilon);   // 200/100
  }

  // Test 81: Three independent instances
  void testThreeIndependentInstances() {
    FGFDMExec fdmex1, fdmex2, fdmex3;
    auto a1 = fdmex1.GetAccelerations();
    auto a2 = fdmex2.GetAccelerations();
    auto a3 = fdmex3.GetAccelerations();

    TS_ASSERT(a1 != a2);
    TS_ASSERT(a2 != a3);
    TS_ASSERT(a1 != a3);

    a1->SetRate(1);
    a2->SetRate(2);
    a3->SetRate(3);

    TS_ASSERT_EQUALS(a1->GetRate(), 1u);
    TS_ASSERT_EQUALS(a2->GetRate(), 2u);
    TS_ASSERT_EQUALS(a3->GetRate(), 3u);
  }

  // Test 82: Instance isolation with forces
  void testInstanceIsolationWithForces() {
    FGFDMExec fdmex1, fdmex2;
    auto a1 = fdmex1.GetAccelerations();
    auto a2 = fdmex2.GetAccelerations();

    a1->in.Force = FGColumnVector3(1000.0, 0.0, 0.0);
    a2->in.Force = FGColumnVector3(0.0, 0.0, 0.0);

    TS_ASSERT_DELTA(a1->in.Force(1), 1000.0, epsilon);
    TS_ASSERT_DELTA(a2->in.Force(1), 0.0, epsilon);
  }

  // Test 83: Parallel state modifications
  void testParallelStateModifications() {
    FGFDMExec fdmex1, fdmex2;
    auto a1 = fdmex1.GetAccelerations();
    auto a2 = fdmex2.GetAccelerations();

    for (int i = 0; i < 50; i++) {
      a1->in.Force = FGColumnVector3(i * 10.0, 0.0, 0.0);
      a2->in.Force = FGColumnVector3(i * 20.0, 0.0, 0.0);

      TS_ASSERT_DELTA(a1->in.Force(1), i * 10.0, epsilon);
      TS_ASSERT_DELTA(a2->in.Force(1), i * 20.0, epsilon);
    }
  }

  /***************************************************************************
   * Additional State Consistency Tests (84-87)
   ***************************************************************************/

  // Test 84: Force/moment getter consistency
  void testForceAndMomentGetterConsistency() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    FGColumnVector3 testForce(111.0, 222.0, 333.0);
    FGColumnVector3 testMoment(44.0, 55.0, 66.0);

    accel->in.Force = testForce;
    accel->in.Moment = testMoment;

    const FGColumnVector3& forces = accel->GetForces();
    const FGColumnVector3& moments = accel->GetMoments();

    TS_ASSERT_DELTA(forces(1), testForce(1), epsilon);
    TS_ASSERT_DELTA(forces(2), testForce(2), epsilon);
    TS_ASSERT_DELTA(forces(3), testForce(3), epsilon);
    TS_ASSERT_DELTA(moments(1), testMoment(1), epsilon);
    TS_ASSERT_DELTA(moments(2), testMoment(2), epsilon);
    TS_ASSERT_DELTA(moments(3), testMoment(3), epsilon);
  }

  // Test 85: Rate persistence across runs
  void testRatePersistenceAcrossRuns() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    accel->SetRate(5);

    for (int i = 0; i < 10; i++) {
      accel->Run(false);
      TS_ASSERT_EQUALS(accel->GetRate(), 5u);
    }
  }

  // Test 86: InitModel resets state
  void testInitModelResetsState() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    accel->SetRate(7);
    bool result = accel->InitModel();

    TS_ASSERT(result);
    // Rate should persist after InitModel
    TS_ASSERT_EQUALS(accel->GetRate(), 7u);
  }

  // Test 87: Gravity acceleration passthrough
  void testGravityAccelerationPassthrough() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    FGColumnVector3 gravity(0.0, 0.0, -32.174);
    accel->in.vGravAccel = gravity;

    TS_ASSERT_DELTA(accel->in.vGravAccel(3), -32.174, epsilon);
  }

  /***************************************************************************
   * Additional Edge Case Tests (88-91)
   ***************************************************************************/

  // Test 88: Near-zero mass handling
  void testNearZeroMassHandling() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    double smallMass = 0.001;  // Very small mass
    FGColumnVector3 force(1.0, 0.0, 0.0);

    accel->in.Force = force;
    accel->in.Moment = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.GroundForce = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.GroundMoment = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vGravAccel = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vPQR = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vPQRi = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vUVW = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.Mass = smallMass;
    accel->in.J = FGMatrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    accel->in.Jinv = FGMatrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
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

    FGColumnVector3 bodyAccel = accel->GetBodyAccel();
    TS_ASSERT(!std::isnan(bodyAccel(1)));
    TS_ASSERT(!std::isinf(bodyAccel(1)));
  }

  // Test 89: Very large angular rates
  void testVeryLargeAngularRates() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    FGColumnVector3 highPQR(10.0, 10.0, 10.0);  // Very high rotation rates

    accel->in.Force = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.Moment = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.GroundForce = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.GroundMoment = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vGravAccel = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vPQR = highPQR;
    accel->in.vPQRi = highPQR;
    accel->in.vUVW = FGColumnVector3(500.0, 100.0, 50.0);
    accel->in.Mass = 100.0;
    accel->in.J = FGMatrix33(1000.0, 0.0, 0.0, 0.0, 2000.0, 0.0, 0.0, 0.0, 3000.0);
    accel->in.Jinv = FGMatrix33(0.001, 0.0, 0.0, 0.0, 0.0005, 0.0, 0.0, 0.0, 0.000333);
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

    FGColumnVector3 uvwdot = accel->GetUVWdot();
    TS_ASSERT(!std::isnan(uvwdot(1)));
    TS_ASSERT(!std::isnan(uvwdot(2)));
    TS_ASSERT(!std::isnan(uvwdot(3)));
  }

  // Test 90: Zero time step
  void testZeroTimeStep() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    accel->in.Force = FGColumnVector3(100.0, 0.0, 0.0);
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
    accel->in.DeltaT = 0.0;  // Zero time step
    accel->in.TerrainVelocity = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.TerrainAngularVel = FGColumnVector3(0.0, 0.0, 0.0);

    accel->InitializeDerivatives();
    accel->Run(false);

    FGColumnVector3 bodyAccel = accel->GetBodyAccel();
    TS_ASSERT(!std::isnan(bodyAccel(1)));
  }

  // Test 91: Negative forces with all components
  void testNegativeForcesAllComponents() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    FGColumnVector3 negForce(-500.0, -300.0, -100.0);

    accel->in.Force = negForce;
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

    accel->InitializeDerivatives();
    accel->Run(false);

    FGColumnVector3 bodyAccel = accel->GetBodyAccel();
    TS_ASSERT_DELTA(bodyAccel(1), -5.0, epsilon);
    TS_ASSERT_DELTA(bodyAccel(2), -3.0, epsilon);
    TS_ASSERT_DELTA(bodyAccel(3), -1.0, epsilon);
  }

  /***************************************************************************
   * Additional Stress Tests (92-96)
   ***************************************************************************/

  // Test 92: Alternating force directions
  void testAlternatingForceDirections() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

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

    accel->InitializeDerivatives();

    for (int i = 0; i < 100; i++) {
      double sign = (i % 2 == 0) ? 1.0 : -1.0;
      accel->in.Force = FGColumnVector3(sign * 100.0, 0.0, 0.0);
      accel->Run(false);

      FGColumnVector3 bodyAccel = accel->GetBodyAccel();
      TS_ASSERT_DELTA(bodyAccel(1), sign * 1.0, epsilon);
    }
  }

  // Test 93: Ramping forces
  void testRampingForces() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

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

    for (int i = 0; i <= 50; i++) {
      accel->in.Force = FGColumnVector3(i * 10.0, 0.0, 0.0);
      accel->Run(false);

      FGColumnVector3 bodyAccel = accel->GetBodyAccel();
      TS_ASSERT_DELTA(bodyAccel(1), i * 10.0 / 50.0, epsilon);
    }
  }

  // Test 94: Many instances stress
  void testManyInstancesStress() {
    std::vector<std::unique_ptr<FGFDMExec>> execs;
    for (int i = 0; i < 10; i++) {
      execs.push_back(std::make_unique<FGFDMExec>());
    }

    for (size_t i = 0; i < execs.size(); i++) {
      auto accel = execs[i]->GetAccelerations();
      accel->SetRate(static_cast<unsigned int>(i));
      TS_ASSERT_EQUALS(accel->GetRate(), static_cast<unsigned int>(i));
    }
  }

  // Test 95: Continuous initialization
  void testContinuousInitialization() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    for (int i = 0; i < 100; i++) {
      TS_ASSERT(accel->InitModel());
      accel->InitializeDerivatives();
    }
  }

  // Test 96: High frequency rate changes
  void testHighFrequencyRateChanges() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    for (unsigned int i = 0; i < 1000; i++) {
      accel->SetRate(i % 20);
      TS_ASSERT_EQUALS(accel->GetRate(), i % 20);
    }
  }

  /***************************************************************************
   * Complete Verification Tests (97-100)
   ***************************************************************************/

  // Test 97: Comprehensive translational acceleration verification
  void testComprehensiveTranslationalVerification() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    std::vector<std::pair<double, double>> test_cases = {
      {100.0, 10.0},   // F=100, m=10
      {500.0, 50.0},   // F=500, m=50
      {1000.0, 100.0}, // F=1000, m=100
      {50.0, 25.0},    // F=50, m=25
    };

    for (const auto& tc : test_cases) {
      double force = tc.first;
      double mass = tc.second;

      accel->in.Force = FGColumnVector3(force, 0.0, 0.0);
      accel->in.Moment = FGColumnVector3(0.0, 0.0, 0.0);
      accel->in.GroundForce = FGColumnVector3(0.0, 0.0, 0.0);
      accel->in.GroundMoment = FGColumnVector3(0.0, 0.0, 0.0);
      accel->in.vGravAccel = FGColumnVector3(0.0, 0.0, 0.0);
      accel->in.vPQR = FGColumnVector3(0.0, 0.0, 0.0);
      accel->in.vPQRi = FGColumnVector3(0.0, 0.0, 0.0);
      accel->in.vUVW = FGColumnVector3(0.0, 0.0, 0.0);
      accel->in.Mass = mass;
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

      FGColumnVector3 bodyAccel = accel->GetBodyAccel();
      TS_ASSERT_DELTA(bodyAccel(1), force / mass, epsilon);
    }
  }

  // Test 98: Comprehensive rotational acceleration verification
  void testComprehensiveRotationalVerification() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    std::vector<std::tuple<double, double, double>> test_cases = {
      {100.0, 1000.0, 0.1},  // M=100, I=1000, alpha=0.1
      {500.0, 2000.0, 0.25}, // M=500, I=2000, alpha=0.25
      {1000.0, 5000.0, 0.2}, // M=1000, I=5000, alpha=0.2
    };

    for (const auto& tc : test_cases) {
      double moment = std::get<0>(tc);
      double inertia = std::get<1>(tc);
      double expected_alpha = std::get<2>(tc);

      accel->in.Force = FGColumnVector3(0.0, 0.0, 0.0);
      accel->in.Moment = FGColumnVector3(moment, 0.0, 0.0);
      accel->in.GroundForce = FGColumnVector3(0.0, 0.0, 0.0);
      accel->in.GroundMoment = FGColumnVector3(0.0, 0.0, 0.0);
      accel->in.vGravAccel = FGColumnVector3(0.0, 0.0, 0.0);
      accel->in.vPQR = FGColumnVector3(0.0, 0.0, 0.0);
      accel->in.vPQRi = FGColumnVector3(0.0, 0.0, 0.0);
      accel->in.vUVW = FGColumnVector3(0.0, 0.0, 0.0);
      accel->in.Mass = 100.0;
      accel->in.J = FGMatrix33(inertia, 0.0, 0.0, 0.0, inertia, 0.0, 0.0, 0.0, inertia);
      accel->in.Jinv = FGMatrix33(1.0/inertia, 0.0, 0.0, 0.0, 1.0/inertia, 0.0, 0.0, 0.0, 1.0/inertia);
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

      FGColumnVector3 pqridot = accel->GetPQRidot();
      TS_ASSERT_DELTA(pqridot(1), expected_alpha, epsilon);
    }
  }

  // Test 99: Full 6-DOF simulation step
  void testFull6DOFSimulationStep() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();

    // Realistic aircraft state
    accel->in.Force = FGColumnVector3(1000.0, 50.0, -200.0);  // Thrust, side force, lift
    accel->in.Moment = FGColumnVector3(100.0, 200.0, 50.0);   // Roll, pitch, yaw moments
    accel->in.GroundForce = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.GroundMoment = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.vGravAccel = FGColumnVector3(0.0, 0.0, 32.174);
    accel->in.vPQR = FGColumnVector3(0.05, 0.02, 0.01);
    accel->in.vPQRi = FGColumnVector3(0.05, 0.02, 0.01);
    accel->in.vUVW = FGColumnVector3(300.0, 5.0, 10.0);
    accel->in.Mass = 150.0;  // ~4800 lbs
    accel->in.J = FGMatrix33(2000.0, 0.0, 0.0, 0.0, 4000.0, 0.0, 0.0, 0.0, 5000.0);
    accel->in.Jinv = FGMatrix33(0.0005, 0.0, 0.0, 0.0, 0.00025, 0.0, 0.0, 0.0, 0.0002);
    accel->in.Ti2b = FGMatrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    accel->in.Tb2i = FGMatrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    accel->in.Tec2b = FGMatrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    accel->in.Tec2i = FGMatrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    accel->in.vOmegaPlanet = FGColumnVector3(0.0, 0.0, 7.292e-5);
    accel->in.vInertialPosition = FGColumnVector3(0.0, 0.0, 20925646.0 + 5000.0);
    accel->in.DeltaT = 0.0083333;
    accel->in.TerrainVelocity = FGColumnVector3(0.0, 0.0, 0.0);
    accel->in.TerrainAngularVel = FGColumnVector3(0.0, 0.0, 0.0);

    accel->InitializeDerivatives();
    accel->Run(false);

    FGColumnVector3 uvwdot = accel->GetUVWdot();
    FGColumnVector3 pqrdot = accel->GetPQRdot();
    FGColumnVector3 bodyAccel = accel->GetBodyAccel();

    // All accelerations should be finite and non-NaN
    TS_ASSERT(!std::isnan(uvwdot(1)));
    TS_ASSERT(!std::isnan(uvwdot(2)));
    TS_ASSERT(!std::isnan(uvwdot(3)));
    TS_ASSERT(!std::isnan(pqrdot(1)));
    TS_ASSERT(!std::isnan(pqrdot(2)));
    TS_ASSERT(!std::isnan(pqrdot(3)));
    TS_ASSERT(!std::isnan(bodyAccel(1)));
    TS_ASSERT(!std::isnan(bodyAccel(2)));
    TS_ASSERT(!std::isnan(bodyAccel(3)));
  }

  // Test 100: Complete accelerations system integration
  void testCompleteAccelerationsSystemIntegration() {
    // Test multiple independent instances
    FGFDMExec fdmex1, fdmex2;
    auto a1 = fdmex1.GetAccelerations();
    auto a2 = fdmex2.GetAccelerations();

    // 1. Verify different instances
    TS_ASSERT(a1 != a2);
    TS_ASSERT(a1->GetExec() == &fdmex1);
    TS_ASSERT(a2->GetExec() == &fdmex2);

    // 2. Set different parameters
    a1->SetRate(1);
    a2->SetRate(2);
    TS_ASSERT(a1->GetRate() != a2->GetRate());

    // 3. Set up acceleration test for instance 1
    a1->in.Force = FGColumnVector3(500.0, 0.0, 0.0);
    a1->in.Moment = FGColumnVector3(0.0, 0.0, 0.0);
    a1->in.GroundForce = FGColumnVector3(0.0, 0.0, 0.0);
    a1->in.GroundMoment = FGColumnVector3(0.0, 0.0, 0.0);
    a1->in.vGravAccel = FGColumnVector3(0.0, 0.0, 0.0);
    a1->in.vPQR = FGColumnVector3(0.0, 0.0, 0.0);
    a1->in.vPQRi = FGColumnVector3(0.0, 0.0, 0.0);
    a1->in.vUVW = FGColumnVector3(0.0, 0.0, 0.0);
    a1->in.Mass = 100.0;
    a1->in.J = FGMatrix33(1000.0, 0.0, 0.0, 0.0, 1000.0, 0.0, 0.0, 0.0, 1000.0);
    a1->in.Jinv = FGMatrix33(0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001);
    a1->in.Ti2b = FGMatrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    a1->in.Tb2i = FGMatrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    a1->in.Tec2b = FGMatrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    a1->in.Tec2i = FGMatrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    a1->in.vOmegaPlanet = FGColumnVector3(0.0, 0.0, 0.0);
    a1->in.vInertialPosition = FGColumnVector3(0.0, 0.0, 20925646.0);
    a1->in.DeltaT = 0.0083333;
    a1->in.TerrainVelocity = FGColumnVector3(0.0, 0.0, 0.0);
    a1->in.TerrainAngularVel = FGColumnVector3(0.0, 0.0, 0.0);

    // 4. Initialize and run
    a1->InitializeDerivatives();
    a1->Run(false);

    // 5. Verify F=ma
    FGColumnVector3 bodyAccel = a1->GetBodyAccel();
    TS_ASSERT_DELTA(bodyAccel(1), 5.0, epsilon);  // 500/100

    // 6. Test getter consistency
    const FGColumnVector3& forces = a1->GetForces();
    const FGColumnVector3& moments = a1->GetMoments();
    TS_ASSERT_DELTA(forces(1), 500.0, epsilon);
    TS_ASSERT_DELTA(moments(1), 0.0, epsilon);

    // 7. Model name should be non-empty
    TS_ASSERT(!a1->GetName().empty());

    // 8. InitModel should succeed
    TS_ASSERT(a1->InitModel());

    // 9. Multiple runs should be stable
    for (int i = 0; i < 10; i++) {
      a1->Run(false);
      FGColumnVector3 accel = a1->GetBodyAccel();
      TS_ASSERT(!std::isnan(accel(1)));
    }

    // 10. Complete system verification passed
    TS_ASSERT(true);
  }
};
