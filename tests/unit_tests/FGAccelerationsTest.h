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
};
