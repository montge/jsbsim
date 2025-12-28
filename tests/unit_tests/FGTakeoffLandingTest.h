/*******************************************************************************
 * FGTakeoffLandingTest.h - Unit tests for takeoff and landing performance
 *
 * Tests comprehensive takeoff and landing performance calculations including:
 * - Takeoff ground roll distance and rotation speeds
 * - Liftoff speeds and climb gradient
 * - Balanced field length and engine-out performance
 * - Landing approach speeds and flare height
 * - Landing ground roll and stopping distances
 * - Rejected takeoff distance
 * - Obstacle clearance calculations
 * - Runway slope, wind, and density altitude effects
 *
 * Copyright (c) JSBSim Development Team
 * Licensed under LGPL
 ******************************************************************************/

#include <limits>
#include <cmath>
#include <cxxtest/TestSuite.h>
#include "TestUtilities.h"

const double epsilon = 100. * std::numeric_limits<double>::epsilon();
constexpr double radtodeg = 180. / M_PI;
constexpr double degtoman = M_PI / 180.;

using namespace JSBSim;
using namespace JSBSimTest;

class FGTakeoffLandingTest : public CxxTest::TestSuite
{
public:
  // Physical constants
  static constexpr double g = Constants::G_FTPS2;  // 32.174 ft/s^2
  static constexpr double rho0 = Constants::SEA_LEVEL_DENSITY_SLUGFT3;  // 0.002377 slugs/ft^3

  //===========================================================================
  // 1. TAKEOFF GROUND ROLL DISTANCE TESTS (4 tests)
  //===========================================================================

  void testGroundRollBasic() {
    // Ground roll distance: s = V^2 / (2*a)
    // where a = (T - D - mu*W) / mass
    double v_liftoff = 150.0;  // ft/s
    double acceleration = 8.0;  // ft/s^2

    double ground_roll = (v_liftoff * v_liftoff) / (2.0 * acceleration);

    TS_ASSERT_DELTA(ground_roll, 1406.25, 1.0);
  }

  void testGroundRollWithForces() {
    // Calculate acceleration from forces: a = F/m
    double thrust = 6000.0;     // lbs
    double drag = 800.0;        // lbs during ground roll
    double weight = 12000.0;    // lbs
    double mu_rolling = 0.02;   // Rolling friction coefficient
    double friction = mu_rolling * weight;

    double mass = weight / g;   // slugs
    double net_force = thrust - drag - friction;
    double acceleration = net_force / mass;

    double v_liftoff = 150.0;
    double ground_roll = (v_liftoff * v_liftoff) / (2.0 * acceleration);

    TS_ASSERT(acceleration > 0.0);
    TS_ASSERT(ground_roll > 0.0);
    // Actual calculation gives ~13.3
    TS_ASSERT_DELTA(acceleration, 13.3, 0.2);
  }

  void testGroundRollSimplified() {
    // Simplified equation: s = 1.44 * W^2 / (g * rho * S * CL_max * T)
    double weight = 15000.0;
    double wing_area = 300.0;  // ft^2
    double cl_max = 1.8;       // With flaps
    double thrust = 6000.0;

    double distance = 1.44 * weight * weight / (g * rho0 * wing_area * cl_max * thrust);

    TS_ASSERT(distance > 0.0);
    // 1.44 * 225e6 / (32.174 * 0.002377 * 300 * 1.8 * 6000) ≈ 1308
    TS_ASSERT_DELTA(distance, 1308.0, 5.0);  // ft
  }

  void testGroundRollHeavyWeight() {
    // Heavier aircraft requires longer ground roll
    double v_liftoff = 160.0;
    double thrust = 8000.0;
    double drag = 1000.0;
    double weight_light = 12000.0;
    double weight_heavy = 16000.0;
    double mu = 0.02;

    double mass_light = weight_light / g;
    double mass_heavy = weight_heavy / g;

    double accel_light = (thrust - drag - mu * weight_light) / mass_light;
    double accel_heavy = (thrust - drag - mu * weight_heavy) / mass_heavy;

    double dist_light = (v_liftoff * v_liftoff) / (2.0 * accel_light);
    double dist_heavy = (v_liftoff * v_liftoff) / (2.0 * accel_heavy);

    TS_ASSERT(dist_heavy > dist_light);
  }

  //===========================================================================
  // 2. ROTATION SPEED (Vr) TESTS (4 tests)
  //===========================================================================

  void testRotationSpeedBasic() {
    // Vr typically 1.05 to 1.1 * Vstall
    double v_stall = 120.0;  // ft/s
    double vr_factor = 1.05;

    double v_rotation = v_stall * vr_factor;

    TS_ASSERT_DELTA(v_rotation, 126.0, 0.1);
    TS_ASSERT(v_rotation > v_stall);
  }

  void testRotationSpeedFromStall() {
    // Calculate Vstall, then Vr
    double weight = 12000.0;
    double wing_area = 200.0;
    double cl_max = 1.8;  // Flaps down

    double v_stall = sqrt(2.0 * weight / (rho0 * wing_area * cl_max));
    double v_rotation = 1.1 * v_stall;

    TS_ASSERT(v_stall > 0.0);
    TS_ASSERT(v_rotation > v_stall);
    // sqrt(30000 / 1.069) ≈ 167.5
    TS_ASSERT_DELTA(v_stall, 167.5, 2.0);
    TS_ASSERT_DELTA(v_rotation, 184.2, 2.0);
  }

  void testRotationSpeedWeightEffect() {
    // Heavier weight requires higher Vr
    double wing_area = 250.0;
    double cl_max = 1.6;
    double weight_light = 10000.0;
    double weight_heavy = 15000.0;

    double v_stall_light = sqrt(2.0 * weight_light / (rho0 * wing_area * cl_max));
    double v_stall_heavy = sqrt(2.0 * weight_heavy / (rho0 * wing_area * cl_max));

    double vr_light = 1.05 * v_stall_light;
    double vr_heavy = 1.05 * v_stall_heavy;

    TS_ASSERT(vr_heavy > vr_light);
  }

  void testRotationSpeedMinimum() {
    // Vr must provide sufficient lift for rotation
    // At Vr, L = CL * 0.5 * rho * V^2 * S should approach W
    double v_rotation = 130.0;
    double wing_area = 200.0;
    double cl_rotation = 0.9;  // Below CL_max

    double lift_at_vr = cl_rotation * 0.5 * rho0 * v_rotation * v_rotation * wing_area;
    double weight = 10000.0;

    // Lift at rotation vs weight
    double lift_ratio = lift_at_vr / weight;
    TS_ASSERT(lift_ratio > 0.0);
    TS_ASSERT(lift_ratio < 2.0);  // Reasonable range
  }

  //===========================================================================
  // 3. LIFTOFF SPEED (Vlof) TESTS (4 tests)
  //===========================================================================

  void testLiftoffSpeedBasic() {
    // Vlof typically 1.1 to 1.15 * Vstall
    double v_stall = 120.0;
    double vlof_factor = 1.15;

    double v_liftoff = v_stall * vlof_factor;

    TS_ASSERT_DELTA(v_liftoff, 138.0, 0.1);
    TS_ASSERT(v_liftoff > v_stall);
  }

  void testLiftoffSpeedLiftEqualsWeight() {
    // At liftoff, L = W
    double weight = 12000.0;
    double wing_area = 250.0;
    double cl_liftoff = 1.5;  // Near CL_max

    // V_liftoff from L = 0.5 * rho * V^2 * S * CL
    double v_liftoff = sqrt(2.0 * weight / (rho0 * wing_area * cl_liftoff));

    TS_ASSERT(v_liftoff > 0.0);
    TS_ASSERT_DELTA(v_liftoff, 164.6, 2.0);
  }

  void testLiftoffSpeedVsRotation() {
    // Vlof is slightly higher than Vr
    double v_rotation = 130.0;
    double v_liftoff = 138.0;

    TS_ASSERT(v_liftoff > v_rotation);
    double delta_v = v_liftoff - v_rotation;
    TS_ASSERT_DELTA(delta_v, 8.0, 1.0);
  }

  void testLiftoffSpeedDensityEffect() {
    // Lower density requires higher liftoff speed
    double weight = 12000.0;
    double wing_area = 250.0;
    double cl = 1.5;

    double rho_sea_level = rho0;
    double rho_5000ft = 0.002048;  // slugs/ft^3 at 5000 ft

    double v_liftoff_sl = sqrt(2.0 * weight / (rho_sea_level * wing_area * cl));
    double v_liftoff_5k = sqrt(2.0 * weight / (rho_5000ft * wing_area * cl));

    TS_ASSERT(v_liftoff_5k > v_liftoff_sl);
  }

  //===========================================================================
  // 4. TAKEOFF CLIMB GRADIENT TESTS (4 tests)
  //===========================================================================

  void testClimbGradientBasic() {
    // Climb gradient = (T - D) / W
    double thrust = 6000.0;
    double drag = 2000.0;
    double weight = 12000.0;

    double gradient = (thrust - drag) / weight;
    double gradient_percent = gradient * 100.0;

    TS_ASSERT_DELTA(gradient, 0.3333, 0.001);
    TS_ASSERT_DELTA(gradient_percent, 33.33, 0.1);
  }

  void testClimbGradientAngle() {
    // Climb angle gamma = arcsin(gradient)
    double thrust = 7000.0;
    double drag = 3000.0;
    double weight = 10000.0;

    double gradient = (thrust - drag) / weight;
    double gamma_rad = asin(gradient);
    double gamma_deg = gamma_rad * radtodeg;

    TS_ASSERT_DELTA(gradient, 0.4, 0.001);
    TS_ASSERT_DELTA(gamma_deg, 23.58, 0.1);
  }

  void testClimbGradientMinimumRequired() {
    // FAR Part 25 requires minimum climb gradient for certification
    // Typically 2.4% for twins with one engine out
    double gradient_required = 0.024;

    double weight = 15000.0;
    double drag = 2000.0;

    // Required thrust to meet gradient
    double thrust_required = weight * gradient_required + drag;

    TS_ASSERT_DELTA(thrust_required, 2360.0, 1.0);
  }

  void testClimbGradientScreenHeight() {
    // Reaching 50 ft screen height
    double gradient = 0.20;  // 20% climb gradient
    double horizontal_distance = 1500.0;  // ft

    double height_gained = horizontal_distance * gradient;

    TS_ASSERT_DELTA(height_gained, 300.0, 1.0);
    TS_ASSERT(height_gained > 50.0);  // Clears 50 ft obstacle
  }

  //===========================================================================
  // 5. BALANCED FIELD LENGTH TESTS (4 tests)
  //===========================================================================

  void testBalancedFieldLengthDefinition() {
    // BFL: where accelerate-stop distance equals accelerate-go distance
    double accelerate_stop = 3500.0;  // ft
    double accelerate_go = 3500.0;    // ft

    TS_ASSERT_DELTA(accelerate_stop, accelerate_go, 1.0);
  }

  void testBalancedFieldLengthV1() {
    // V1 (decision speed) at balanced field condition
    // Below V1: abort, above V1: continue takeoff
    double v_stall = 120.0;
    double v_liftoff = 140.0;
    double v1 = 125.0;  // Between stall and liftoff

    TS_ASSERT(v1 > v_stall);
    TS_ASSERT(v1 < v_liftoff);
  }

  void testBalancedFieldLengthAccelerateStop() {
    // Accelerate-stop distance: accelerate to V1, then brake to stop
    double v1 = 130.0;  // ft/s
    double accel_rate = 10.0;  // ft/s^2
    double decel_rate = 8.0;   // ft/s^2 with brakes

    double accel_distance = (v1 * v1) / (2.0 * accel_rate);
    double stop_distance = (v1 * v1) / (2.0 * decel_rate);
    double total_distance = accel_distance + stop_distance;

    TS_ASSERT_DELTA(accel_distance, 845.0, 1.0);
    TS_ASSERT_DELTA(stop_distance, 1056.25, 1.0);
    TS_ASSERT_DELTA(total_distance, 1901.25, 2.0);
  }

  void testBalancedFieldLengthAccelerateGo() {
    // Accelerate-go distance: continue to liftoff after V1
    double v1 = 130.0;
    double v_liftoff = 150.0;
    double accel_rate = 10.0;
    double climb_gradient = 0.10;

    double accel_to_v1 = (v1 * v1) / (2.0 * accel_rate);
    double accel_v1_to_vlof = (v_liftoff * v_liftoff - v1 * v1) / (2.0 * accel_rate);
    double airborne_to_35ft = 35.0 / climb_gradient;  // Horizontal distance to clear 35 ft

    double total_distance = accel_to_v1 + accel_v1_to_vlof + airborne_to_35ft;

    TS_ASSERT(total_distance > accel_to_v1);
    TS_ASSERT(total_distance > 0.0);
  }

  //===========================================================================
  // 6. ENGINE-OUT TAKEOFF PERFORMANCE TESTS (4 tests)
  //===========================================================================

  void testEngineOutThrustReduction() {
    // Twin-engine aircraft loses 50% thrust
    double total_thrust = 10000.0;
    double thrust_one_engine = total_thrust / 2.0;

    TS_ASSERT_DELTA(thrust_one_engine, 5000.0, 0.1);
  }

  void testEngineOutClimbGradient() {
    // Reduced climb gradient with one engine out
    double thrust_all = 12000.0;
    double thrust_oei = 6000.0;  // One engine inoperative
    double drag = 2500.0;
    double weight = 20000.0;

    double gradient_all = (thrust_all - drag) / weight;
    double gradient_oei = (thrust_oei - drag) / weight;

    TS_ASSERT(gradient_oei < gradient_all);
    TS_ASSERT_DELTA(gradient_all, 0.475, 0.001);
    TS_ASSERT_DELTA(gradient_oei, 0.175, 0.001);
  }

  void testEngineOutV1Calculation() {
    // V1 must ensure safe continued takeoff or stop
    // V1 <= Vr and V1 >= Vmcg (minimum control speed ground)
    double v_mcg = 110.0;  // Minimum control speed
    double v_r = 140.0;    // Rotation speed
    double v1 = 125.0;     // Decision speed

    TS_ASSERT(v1 >= v_mcg);
    TS_ASSERT(v1 <= v_r);
  }

  void testEngineOutTakeoffDistance() {
    // Engine-out increases takeoff distance
    double v_liftoff = 150.0;
    double accel_all_engines = 12.0;   // ft/s^2
    double accel_oei = 7.0;            // ft/s^2

    double distance_normal = (v_liftoff * v_liftoff) / (2.0 * accel_all_engines);
    double distance_oei = (v_liftoff * v_liftoff) / (2.0 * accel_oei);

    TS_ASSERT(distance_oei > distance_normal);
    TS_ASSERT_DELTA(distance_normal, 937.5, 1.0);
    TS_ASSERT_DELTA(distance_oei, 1607.1, 1.0);
  }

  //===========================================================================
  // 7. LANDING APPROACH SPEED (Vref) TESTS (4 tests)
  //===========================================================================

  void testApproachSpeedBasic() {
    // Vref typically 1.3 * Vstall in landing configuration
    double v_stall_landing = 100.0;  // ft/s with flaps
    double vref_factor = 1.3;

    double v_ref = v_stall_landing * vref_factor;

    TS_ASSERT_DELTA(v_ref, 130.0, 0.1);
    TS_ASSERT(v_ref > v_stall_landing);
  }

  void testApproachSpeedFromWeight() {
    // Calculate Vref from landing weight
    double weight = 10000.0;
    double wing_area = 200.0;
    double cl_max_landing = 2.0;  // Full flaps

    double v_stall = sqrt(2.0 * weight / (rho0 * wing_area * cl_max_landing));
    double v_ref = 1.3 * v_stall;

    TS_ASSERT_DELTA(v_stall, 145.1, 2.0);
    TS_ASSERT_DELTA(v_ref, 188.6, 2.0);
  }

  void testApproachSpeedWindCorrection() {
    // Add half the gust to Vref
    double v_ref_basic = 130.0;
    double headwind_steady = 20.0;   // kts
    double gust = 10.0;              // kts

    double wind_correction = gust / 2.0;
    double v_ref_corrected = v_ref_basic + wind_correction;

    TS_ASSERT_DELTA(v_ref_corrected, 135.0, 0.1);
    TS_ASSERT(v_ref_corrected > v_ref_basic);
  }

  void testApproachSpeedSafetyMargin() {
    // Vref provides margin above stall
    double v_stall = 110.0;
    double v_ref = 143.0;

    double margin = v_ref / v_stall;
    TS_ASSERT_DELTA(margin, 1.3, 0.01);

    double margin_percent = (margin - 1.0) * 100.0;
    TS_ASSERT_DELTA(margin_percent, 30.0, 1.0);
  }

  //===========================================================================
  // 8. LANDING FLARE HEIGHT TESTS (3 tests)
  //===========================================================================

  void testFlareHeightTypical() {
    // Flare typically initiated at 20-50 ft AGL
    double flare_height = 30.0;  // ft

    TS_ASSERT(flare_height >= 20.0);
    TS_ASSERT(flare_height <= 50.0);
  }

  void testFlareHeightVelocityEffect() {
    // Higher approach speed requires higher flare initiation
    double v_approach_slow = 120.0;   // ft/s
    double v_approach_fast = 160.0;   // ft/s

    double flare_height_slow = 25.0;
    double flare_height_fast = 40.0;

    TS_ASSERT(flare_height_fast > flare_height_slow);
  }

  void testFlareTransitionDistance() {
    // Horizontal distance during flare maneuver
    double v_approach = 140.0;  // ft/s
    double flare_duration = 3.0;  // seconds

    double flare_distance = v_approach * flare_duration;

    TS_ASSERT_DELTA(flare_distance, 420.0, 1.0);
  }

  //===========================================================================
  // 9. LANDING GROUND ROLL DISTANCE TESTS (4 tests)
  //===========================================================================

  void testLandingGroundRollBasic() {
    // Ground roll: s = V^2 / (2*a)
    double v_touchdown = 130.0;  // ft/s
    double deceleration = 10.0;  // ft/s^2

    double ground_roll = (v_touchdown * v_touchdown) / (2.0 * deceleration);

    TS_ASSERT_DELTA(ground_roll, 845.0, 1.0);
  }

  void testLandingGroundRollWithDrag() {
    // Deceleration from aerodynamic drag
    double v_touchdown = 140.0;
    double drag = 1500.0;       // lbs
    double weight = 12000.0;
    double mu_rolling = 0.4;    // Braking friction

    double mass = weight / g;
    double friction_force = mu_rolling * weight;
    double decel = (drag + friction_force) / mass;

    double ground_roll = (v_touchdown * v_touchdown) / (2.0 * decel);

    TS_ASSERT(decel > 0.0);
    TS_ASSERT(ground_roll > 0.0);
  }

  void testLandingGroundRollSpoilers() {
    // Spoilers increase drag and reduce lift
    double v_touchdown = 130.0;
    double decel_no_spoilers = 8.0;   // ft/s^2
    double decel_with_spoilers = 12.0;  // ft/s^2

    double distance_no_spoilers = (v_touchdown * v_touchdown) / (2.0 * decel_no_spoilers);
    double distance_with_spoilers = (v_touchdown * v_touchdown) / (2.0 * decel_with_spoilers);

    TS_ASSERT(distance_with_spoilers < distance_no_spoilers);
  }

  void testLandingGroundRollSimplified() {
    // Simplified landing distance equation
    double v_approach = 130.0;  // ft/s (1.3 * Vstall)
    double v_stall = 100.0;

    // Landing distance approximately: s = 5 * (V_approach / 1.3)^2 / g
    double distance = 5.0 * (v_stall * v_stall) / g;

    TS_ASSERT(distance > 0.0);
    TS_ASSERT_DELTA(distance, 1553.4, 5.0);
  }

  //===========================================================================
  // 10. STOPPING DISTANCE WITH BRAKES TESTS (4 tests)
  //===========================================================================

  void testBrakingDeceleration() {
    // Maximum braking deceleration limited by friction
    double mu_braking = 0.5;  // Dry runway

    double max_decel = mu_braking * g;

    TS_ASSERT_DELTA(max_decel, 16.09, 0.1);  // ft/s^2
  }

  void testStoppingDistanceMaxBrakes() {
    // Stopping from landing speed with maximum braking
    double v_initial = 140.0;
    double mu_max = 0.6;
    double decel = mu_max * g;

    double distance = (v_initial * v_initial) / (2.0 * decel);

    TS_ASSERT_DELTA(decel, 19.3, 0.1);
    TS_ASSERT_DELTA(distance, 507.8, 2.0);
  }

  void testStoppingDistanceReverseThrust() {
    // Reverse thrust adds to braking
    double v_initial = 150.0;
    double weight = 15000.0;
    double reverse_thrust = 2000.0;  // lbs
    double brake_friction = 0.4;

    double mass = weight / g;
    double brake_force = brake_friction * weight;
    double total_decel = (brake_force + reverse_thrust) / mass;

    double distance = (v_initial * v_initial) / (2.0 * total_decel);

    TS_ASSERT(total_decel > brake_friction * g);
    TS_ASSERT(distance > 0.0);
  }

  void testStoppingDistanceTimeToStop() {
    // Time required to stop
    double v_initial = 130.0;
    double deceleration = 12.0;

    double time_to_stop = v_initial / deceleration;
    double distance = (v_initial * v_initial) / (2.0 * deceleration);

    TS_ASSERT_DELTA(time_to_stop, 10.83, 0.1);  // seconds
    TS_ASSERT_DELTA(distance, 704.2, 1.0);
  }

  //===========================================================================
  // 11. REJECTED TAKEOFF DISTANCE TESTS (4 tests)
  //===========================================================================

  void testRejectedTakeoffFromV1() {
    // RTO from decision speed V1
    double v1 = 120.0;
    double decel_rate = 10.0;  // ft/s^2 with brakes

    double distance = (v1 * v1) / (2.0 * decel_rate);

    TS_ASSERT_DELTA(distance, 720.0, 1.0);
  }

  void testRejectedTakeoffRecognitionDelay() {
    // Include pilot recognition and reaction time
    double v1 = 130.0;
    double reaction_time = 1.5;  // seconds
    double decel_rate = 12.0;

    double distance_during_reaction = v1 * reaction_time;
    double distance_braking = (v1 * v1) / (2.0 * decel_rate);
    double total_distance = distance_during_reaction + distance_braking;

    TS_ASSERT_DELTA(distance_during_reaction, 195.0, 1.0);
    TS_ASSERT_DELTA(total_distance, 899.2, 2.0);
  }

  void testRejectedTakeoffMaximumEffort() {
    // Maximum braking with spoilers and reverse thrust
    double v1 = 140.0;
    double weight = 20000.0;
    double mu_braking = 0.5;
    double reverse_thrust = 3000.0;

    double mass = weight / g;
    double brake_force = mu_braking * weight;
    double total_decel = (brake_force + reverse_thrust) / mass;

    double distance = (v1 * v1) / (2.0 * total_decel);

    TS_ASSERT(total_decel > mu_braking * g);
    TS_ASSERT(distance < (v1 * v1) / (2.0 * mu_braking * g));
  }

  void testRejectedTakeoffSafetyMargin() {
    // RTO distance must be less than runway remaining at V1
    double runway_length = 8000.0;
    double distance_to_v1 = 3000.0;
    double rto_distance = 3500.0;
    double safety_margin = 500.0;

    double runway_remaining = runway_length - distance_to_v1;
    double required_length = rto_distance + safety_margin;

    TS_ASSERT(runway_remaining >= required_length);
  }

  //===========================================================================
  // 12. OBSTACLE CLEARANCE TESTS (4 tests)
  //===========================================================================

  void testObstacleClearanceTakeoff() {
    // Clear 50 ft obstacle for transport category
    double obstacle_height = 50.0;  // ft
    double climb_gradient = 0.05;   // 5%

    double horizontal_distance = obstacle_height / climb_gradient;

    TS_ASSERT_DELTA(horizontal_distance, 1000.0, 1.0);
  }

  void testObstacleClearanceLanding() {
    // Clear 50 ft obstacle on approach
    double obstacle_height = 50.0;
    double approach_angle = 3.0 * degtoman;  // 3 degree glideslope

    double horizontal_distance = obstacle_height / tan(approach_angle);

    TS_ASSERT_DELTA(horizontal_distance, 954.9, 5.0);
  }

  void testObstacleClearanceClimbAngle() {
    // Climb angle required to clear obstacle
    double obstacle_height = 35.0;   // ft (FAR Part 23)
    double horizontal_distance = 1500.0;

    double climb_angle = atan(obstacle_height / horizontal_distance);
    double climb_angle_deg = climb_angle * radtodeg;

    TS_ASSERT_DELTA(climb_angle_deg, 1.34, 0.1);
  }

  void testObstacleClearanceWithMargin() {
    // Include safety margin above obstacle
    double obstacle_height = 50.0;
    double safety_margin = 35.0;  // Additional clearance
    double climb_gradient = 0.08;

    double total_height = obstacle_height + safety_margin;
    double horizontal_distance = total_height / climb_gradient;

    TS_ASSERT_DELTA(total_height, 85.0, 0.1);
    TS_ASSERT_DELTA(horizontal_distance, 1062.5, 2.0);
  }

  //===========================================================================
  // 13. RUNWAY SLOPE EFFECTS TESTS (4 tests)
  //===========================================================================

  void testRunwaySlopeAcceleration() {
    // Upslope reduces acceleration, downslope increases
    double slope = 0.01;  // 1% upslope
    double accel_level = 10.0;  // ft/s^2

    double accel_upslope = accel_level - g * slope;
    double accel_downslope = accel_level + g * slope;

    TS_ASSERT(accel_upslope < accel_level);
    TS_ASSERT(accel_downslope > accel_level);
    TS_ASSERT_DELTA(accel_upslope, 9.68, 0.01);
    TS_ASSERT_DELTA(accel_downslope, 10.32, 0.01);
  }

  void testRunwaySlopeTakeoffDistance() {
    // Upslope increases takeoff distance
    double v_liftoff = 150.0;
    double accel_level = 10.0;
    double slope = 0.01;

    double accel_upslope = accel_level - g * slope;
    double distance_level = (v_liftoff * v_liftoff) / (2.0 * accel_level);
    double distance_upslope = (v_liftoff * v_liftoff) / (2.0 * accel_upslope);

    TS_ASSERT(distance_upslope > distance_level);
  }

  void testRunwaySlopeLandingDistance() {
    // Downslope increases landing distance
    double v_touchdown = 130.0;
    double decel_level = 12.0;
    double slope = 0.015;  // 1.5% downslope

    double decel_downslope = decel_level - g * slope;
    double distance_level = (v_touchdown * v_touchdown) / (2.0 * decel_level);
    double distance_downslope = (v_touchdown * v_touchdown) / (2.0 * decel_downslope);

    TS_ASSERT(distance_downslope > distance_level);
  }

  void testRunwaySlopeEffect() {
    // 2% slope effect on distances
    double slope = 0.02;
    double base_distance = 2000.0;

    // Rule of thumb: each 1% slope changes distance by ~10%
    double slope_factor = 1.0 + (slope * 10.0);
    double adjusted_distance = base_distance * slope_factor;

    TS_ASSERT_DELTA(slope_factor, 1.2, 0.01);
    TS_ASSERT_DELTA(adjusted_distance, 2400.0, 1.0);
  }

  //===========================================================================
  // 14. WIND EFFECTS ON TAKEOFF/LANDING TESTS (5 tests)
  //===========================================================================

  void testHeadwindTakeoffDistance() {
    // Headwind reduces ground roll
    double v_liftoff_tas = 150.0;  // True airspeed
    double headwind = 20.0;        // ft/s
    double acceleration = 10.0;

    double v_liftoff_gs = v_liftoff_tas - headwind;  // Groundspeed
    double distance = (v_liftoff_gs * v_liftoff_gs) / (2.0 * acceleration);

    TS_ASSERT(distance < (v_liftoff_tas * v_liftoff_tas) / (2.0 * acceleration));
    TS_ASSERT_DELTA(distance, 845.0, 1.0);
  }

  void testTailwindTakeoffDistance() {
    // Tailwind increases ground roll
    double v_liftoff_tas = 150.0;
    double tailwind = 15.0;
    double acceleration = 10.0;

    double v_liftoff_gs = v_liftoff_tas + tailwind;
    double distance = (v_liftoff_gs * v_liftoff_gs) / (2.0 * acceleration);

    TS_ASSERT(distance > (v_liftoff_tas * v_liftoff_tas) / (2.0 * acceleration));
    TS_ASSERT_DELTA(distance, 1361.3, 2.0);
  }

  void testHeadwindLandingDistance() {
    // Headwind reduces landing distance
    double v_approach_tas = 140.0;
    double headwind = 25.0;
    double deceleration = 12.0;

    double v_touchdown_gs = v_approach_tas - headwind;
    double distance = (v_touchdown_gs * v_touchdown_gs) / (2.0 * deceleration);

    TS_ASSERT(distance < (v_approach_tas * v_approach_tas) / (2.0 * deceleration));
    TS_ASSERT_DELTA(distance, 551.0, 2.0);
  }

  void testCrosswindComponent() {
    // Crosswind component affects runway selection
    double wind_speed = 20.0;         // kts
    double wind_angle = 45.0 * degtoman;

    double crosswind = wind_speed * sin(wind_angle);
    double headwind = wind_speed * cos(wind_angle);

    TS_ASSERT_DELTA(crosswind, 14.14, 0.1);
    TS_ASSERT_DELTA(headwind, 14.14, 0.1);
  }

  void testWindEffectFactor() {
    // Wind correction factor for distance
    double no_wind_distance = 2000.0;
    double headwind_kts = 15.0;

    // Approximate: 10% reduction per 10 kts headwind
    double reduction_factor = 1.0 - (headwind_kts / 10.0) * 0.10;
    double corrected_distance = no_wind_distance * reduction_factor;

    TS_ASSERT(corrected_distance < no_wind_distance);
    TS_ASSERT_DELTA(reduction_factor, 0.85, 0.01);
  }

  //===========================================================================
  // 15. DENSITY ALTITUDE EFFECTS TESTS (5 tests)
  //===========================================================================

  void testDensityAltitudeBasic() {
    // Higher density altitude degrades performance
    double pressure_altitude = 5000.0;  // ft
    double temp_celsius = 30.0;         // Hot day
    double temp_standard = 5.1;         // Standard temp at 5000 ft

    double temp_deviation = temp_celsius - temp_standard;
    double density_altitude = pressure_altitude + 120.0 * temp_deviation;

    TS_ASSERT(density_altitude > pressure_altitude);
    TS_ASSERT_DELTA(density_altitude, 7988.0, 5.0);
  }

  void testDensityAltitudeTakeoffDistance() {
    // Sea level vs high density altitude
    double v_liftoff = 150.0;
    double accel_sea_level = 10.0;
    double accel_high_da = 7.0;  // Reduced thrust at altitude

    double distance_sl = (v_liftoff * v_liftoff) / (2.0 * accel_sea_level);
    double distance_high = (v_liftoff * v_liftoff) / (2.0 * accel_high_da);

    TS_ASSERT(distance_high > distance_sl);
    double ratio = distance_high / distance_sl;
    TS_ASSERT_DELTA(ratio, 1.43, 0.01);
  }

  void testDensityRatioEffect() {
    // Density ratio affects lift and thrust
    double rho_altitude = 0.00206;  // slugs/ft^3 at 5000 ft
    double density_ratio = rho_altitude / rho0;

    TS_ASSERT(density_ratio < 1.0);
    TS_ASSERT_DELTA(density_ratio, 0.867, 0.01);

    // Thrust reduction approximately follows density ratio
    double thrust_sl = 5000.0;
    double thrust_altitude = thrust_sl * density_ratio;
    TS_ASSERT_DELTA(thrust_altitude, 4335.0, 10.0);
  }

  void testDensityAltitudeLiftoffSpeed() {
    // Higher density altitude requires higher TAS for same IAS
    double weight = 12000.0;
    double wing_area = 250.0;
    double cl = 1.5;

    double v_liftoff_sl = sqrt(2.0 * weight / (rho0 * wing_area * cl));
    double rho_5000 = 0.00206;
    double v_liftoff_5k = sqrt(2.0 * weight / (rho_5000 * wing_area * cl));

    TS_ASSERT(v_liftoff_5k > v_liftoff_sl);
  }

  void testDensityAltitudeClimbRate() {
    // Reduced climb rate at high density altitude
    double power_sl = 200.0;      // HP
    double power_5000 = 170.0;    // HP (reduced)
    double weight = 2500.0;       // lbs

    // ROC proportional to excess power
    double roc_sl = (power_sl * 33000.0) / weight;  // ft/min
    double roc_5000 = (power_5000 * 33000.0) / weight;

    TS_ASSERT(roc_5000 < roc_sl);
    TS_ASSERT_DELTA(roc_sl, 2640.0, 5.0);
    TS_ASSERT_DELTA(roc_5000, 2244.0, 5.0);
  }

  //===========================================================================
  // ADDITIONAL COMPREHENSIVE TESTS
  //===========================================================================

  void testV2Speed() {
    // V2: Takeoff safety speed (minimum climb speed)
    // Typically 1.2 * Vstall for transport category
    double v_stall = 120.0;
    double v2 = 1.2 * v_stall;

    TS_ASSERT_DELTA(v2, 144.0, 0.1);
    TS_ASSERT(v2 > v_stall);
  }

  void testVMCSpeed() {
    // VMC: Minimum control speed (rudder authority limit)
    double v_stall = 110.0;
    double v_mc = 95.0;  // Typically 0.85 - 0.95 * Vstall

    TS_ASSERT(v_mc < v_stall);
    double ratio = v_mc / v_stall;
    TS_ASSERT(ratio > 0.80);
    TS_ASSERT(ratio < 1.0);
  }

  void testTouchdownSinkRate() {
    // Acceptable touchdown sink rate
    double sink_rate_normal = 3.0;  // ft/s (180 ft/min)
    double sink_rate_firm = 6.0;    // ft/s (360 ft/min)
    double sink_rate_hard = 10.0;   // ft/s (600 ft/min) - hard landing

    TS_ASSERT(sink_rate_normal < sink_rate_firm);
    TS_ASSERT(sink_rate_firm < sink_rate_hard);
  }

  void testTakeoffSafetySpeed() {
    // Speed at 35 ft screen height
    double v2 = 145.0;  // ft/s
    // Should maintain at least V2 after clearing obstacle
    TS_ASSERT(v2 > 0.0);
    TS_ASSERT(v2 > 120.0);  // Well above stall
  }

  void testLandingFlareDeceleration() {
    // Vertical deceleration during flare
    double sink_rate_approach = 10.0;  // ft/s (600 ft/min)
    double sink_rate_touchdown = 3.0;  // ft/s (180 ft/min)
    double flare_time = 3.0;           // seconds

    double vertical_decel = (sink_rate_approach - sink_rate_touchdown) / flare_time;

    TS_ASSERT_DELTA(vertical_decel, 2.33, 0.1);  // ft/s^2
  }

  void testAccelerateStopDistance() {
    // Total distance for rejected takeoff
    double v1 = 125.0;
    double reaction_time = 2.0;
    double decel = 10.0;

    double accel_distance = (v1 * v1) / (2.0 * 8.0);  // Accelerate at 8 ft/s^2
    double reaction_distance = v1 * reaction_time;
    double braking_distance = (v1 * v1) / (2.0 * decel);
    double total = accel_distance + reaction_distance + braking_distance;

    TS_ASSERT(total > accel_distance);
    TS_ASSERT(total > braking_distance);
  }

  void testLandingDistanceAvailable() {
    // LDA (Landing Distance Available) vs required
    double required_distance = 3500.0;  // ft
    double available_distance = 6000.0;  // ft (sufficient runway)
    double safety_factor = 1.67;         // FAR 25 (dry runway)

    double required_with_margin = required_distance * safety_factor;

    // 6000 >= 3500 * 1.67 = 5845
    TS_ASSERT(available_distance >= required_with_margin);
  }

  void testTakeoffDistanceAvailable() {
    // TODA (Takeoff Distance Available) vs required
    double required_distance = 4000.0;
    double runway_length = 6000.0;
    double clearway = 500.0;

    double toda = runway_length + clearway;

    TS_ASSERT(toda >= required_distance);
    TS_ASSERT_DELTA(toda, 6500.0, 1.0);
  }

  void testWetRunwayEffect() {
    // Wet runway increases landing distance
    double dry_distance = 2000.0;
    double wet_factor = 1.15;  // 15% increase

    double wet_distance = dry_distance * wet_factor;

    TS_ASSERT(wet_distance > dry_distance);
    TS_ASSERT_DELTA(wet_distance, 2300.0, 1.0);
  }

  void testContaminatedRunwayBraking() {
    // Reduced braking coefficient on contaminated runway
    double mu_dry = 0.5;
    double mu_wet = 0.3;
    double mu_ice = 0.1;

    TS_ASSERT(mu_dry > mu_wet);
    TS_ASSERT(mu_wet > mu_ice);
    TS_ASSERT(mu_ice < 0.2);  // Very poor braking
  }

  //===========================================================================
  // 16. SHORT FIELD OPERATIONS TESTS (4 tests)
  //===========================================================================

  void testShortFieldTakeoffTechnique() {
    // Short field: use max thrust before brake release
    double static_thrust = 6500.0;  // lbs at full power
    double rolling_thrust = 6000.0; // lbs once moving

    // Static thrust higher due to ram air effect absent
    TS_ASSERT(static_thrust > rolling_thrust);
  }

  void testShortFieldObstacleClearance() {
    // Clear 50 ft obstacle in minimum distance
    double climb_gradient = 0.12;  // 12% climb angle
    double obstacle_height = 50.0;

    double horizontal_distance = obstacle_height / climb_gradient;

    TS_ASSERT_DELTA(horizontal_distance, 416.7, 1.0);
  }

  void testShortFieldLandingSpotTarget() {
    // Touch down at precise point
    double threshold_crossing_height = 50.0;
    double approach_angle = 3.5 * degtoman;  // Steeper than normal

    double distance_from_threshold = threshold_crossing_height / tan(approach_angle);

    TS_ASSERT(distance_from_threshold < 900.0);  // Short distance
    TS_ASSERT_DELTA(distance_from_threshold, 817.7, 5.0);
  }

  void testShortFieldBrakingEffort() {
    // Maximum braking immediately after touchdown
    double v_touchdown = 120.0;
    double mu_max = 0.6;
    double decel = mu_max * g;

    double distance = (v_touchdown * v_touchdown) / (2.0 * decel);

    TS_ASSERT_DELTA(decel, 19.3, 0.1);
    TS_ASSERT_DELTA(distance, 372.9, 2.0);
  }

  //===========================================================================
  // 17. SOFT FIELD OPERATIONS TESTS (4 tests)
  //===========================================================================

  void testSoftFieldRollingResistance() {
    // Increased rolling friction on grass/dirt
    double mu_hard = 0.02;
    double mu_grass = 0.05;
    double mu_soft = 0.10;

    TS_ASSERT(mu_grass > mu_hard);
    TS_ASSERT(mu_soft > mu_grass);
  }

  void testSoftFieldTakeoffDistance() {
    // Soft field increases ground roll
    double v_liftoff = 140.0;
    double thrust = 5000.0;
    double drag = 600.0;
    double weight = 10000.0;
    double mu_hard = 0.02;
    double mu_soft = 0.08;

    double mass = weight / g;
    double accel_hard = (thrust - drag - mu_hard * weight) / mass;
    double accel_soft = (thrust - drag - mu_soft * weight) / mass;

    double distance_hard = (v_liftoff * v_liftoff) / (2.0 * accel_hard);
    double distance_soft = (v_liftoff * v_liftoff) / (2.0 * accel_soft);

    TS_ASSERT(distance_soft > distance_hard);
  }

  void testSoftFieldLiftoffTechnique() {
    // Lift off at minimum speed to reduce ground contact
    double v_stall = 100.0;
    double v_liftoff_normal = 1.15 * v_stall;
    double v_liftoff_soft = 1.05 * v_stall;  // Earlier rotation

    TS_ASSERT(v_liftoff_soft < v_liftoff_normal);
  }

  void testSoftFieldGroundEffect() {
    // Use ground effect to accelerate
    double height_agl = 5.0;  // ft
    double span = 30.0;       // ft wingspan
    double ground_effect_factor = 1.0 - (height_agl / span);

    TS_ASSERT(ground_effect_factor < 1.0);
    TS_ASSERT_DELTA(ground_effect_factor, 0.833, 0.01);
  }

  //===========================================================================
  // 18. GO-AROUND / MISSED APPROACH TESTS (4 tests)
  //===========================================================================

  void testGoAroundDecisionHeight() {
    // Decision altitude for missed approach
    double decision_height = 200.0;  // ft AGL
    double current_altitude = 180.0;
    double runway_in_sight = false;

    bool execute_go_around = (current_altitude <= decision_height) && !runway_in_sight;

    TS_ASSERT(execute_go_around);
  }

  void testGoAroundThrustApplication() {
    // Full thrust for go-around
    double approach_thrust = 0.5;  // 50% power
    double go_around_thrust = 1.0;  // TOGA

    TS_ASSERT(go_around_thrust > approach_thrust);
    TS_ASSERT_DELTA(go_around_thrust, 1.0, 0.01);
  }

  void testGoAroundClimbGradient() {
    // Minimum climb gradient for go-around
    double thrust = 8000.0;
    double drag_dirty = 3500.0;  // Gear and flaps down initially
    double weight = 15000.0;

    double gradient = (thrust - drag_dirty) / weight;

    TS_ASSERT(gradient > 0.0);
    TS_ASSERT_DELTA(gradient, 0.30, 0.01);  // 30%
  }

  void testGoAroundConfiguration() {
    // Clean up configuration sequence
    double flap_retract_speed = 160.0;  // kts
    double gear_retract_speed = 140.0;  // kts
    double current_speed = 150.0;

    bool retract_gear = (current_speed >= gear_retract_speed);
    bool retract_flaps = (current_speed >= flap_retract_speed);

    TS_ASSERT(retract_gear);
    TS_ASSERT(!retract_flaps);
  }

  //===========================================================================
  // 19. AUTOBRAKE SETTINGS TESTS (4 tests)
  //===========================================================================

  void testAutobrakeLow() {
    // Low autobrake setting
    double decel_low = 4.0;  // ft/s^2 (~0.12g)
    double v_touchdown = 130.0;

    double distance = (v_touchdown * v_touchdown) / (2.0 * decel_low);

    TS_ASSERT_DELTA(decel_low / g, 0.124, 0.01);
    TS_ASSERT_DELTA(distance, 2112.5, 2.0);
  }

  void testAutobrakeMedium() {
    // Medium autobrake setting
    double decel_med = 8.0;  // ft/s^2 (~0.25g)
    double v_touchdown = 130.0;

    double distance = (v_touchdown * v_touchdown) / (2.0 * decel_med);

    TS_ASSERT_DELTA(decel_med / g, 0.249, 0.01);
    TS_ASSERT_DELTA(distance, 1056.25, 2.0);
  }

  void testAutobrakeMax() {
    // Maximum autobrake setting
    double decel_max = 14.0;  // ft/s^2 (~0.44g)
    double v_touchdown = 130.0;

    double distance = (v_touchdown * v_touchdown) / (2.0 * decel_max);

    TS_ASSERT_DELTA(decel_max / g, 0.435, 0.01);
    TS_ASSERT_DELTA(distance, 603.6, 2.0);
  }

  void testAutobrakeRTO() {
    // RTO (Rejected Takeoff) autobrake mode
    double decel_rto = 16.0;  // ft/s^2 (~0.5g)
    double v_abort = 140.0;

    double distance = (v_abort * v_abort) / (2.0 * decel_rto);

    TS_ASSERT_DELTA(decel_rto / g, 0.497, 0.01);
    TS_ASSERT_DELTA(distance, 612.5, 2.0);
  }

  //===========================================================================
  // 20. CROSSWIND LANDING TESTS (4 tests)
  //===========================================================================

  void testCrosswindLimit() {
    // Maximum demonstrated crosswind
    double max_crosswind = 25.0;  // kts
    double current_crosswind = 20.0;

    bool within_limits = (current_crosswind <= max_crosswind);

    TS_ASSERT(within_limits);
  }

  void testCrosswindCrabAngle() {
    // Calculate crab angle for crosswind
    double groundspeed = 120.0;  // kts
    double crosswind = 15.0;     // kts

    double crab_angle = atan(crosswind / groundspeed) * radtodeg;

    TS_ASSERT_DELTA(crab_angle, 7.13, 0.1);
  }

  void testCrosswindDecrab() {
    // Rudder required to decrab before touchdown
    double crab_angle = 10.0;  // degrees
    double rudder_authority = 25.0;  // degrees

    bool sufficient_rudder = (rudder_authority > crab_angle);

    TS_ASSERT(sufficient_rudder);
  }

  void testCrosswindWingLow() {
    // Bank angle for wing-low crosswind approach
    double crosswind = 15.0;  // kts
    double approach_speed = 130.0;  // kts

    // Bank angle approximately proportional to crosswind/speed ratio
    double bank_angle = atan(crosswind / (2.0 * approach_speed)) * radtodeg;

    TS_ASSERT(bank_angle < 10.0);  // Small bank angle
    TS_ASSERT(bank_angle > 0.0);
  }

  //===========================================================================
  // 21. RUNWAY CONTAMINATION TESTS (4 tests)
  //===========================================================================

  void testStandingWaterEffect() {
    // Standing water causes hydroplaning risk
    double water_depth_mm = 3.0;  // mm
    double hydroplane_threshold = 2.5;  // mm

    bool hydroplane_risk = (water_depth_mm >= hydroplane_threshold);

    TS_ASSERT(hydroplane_risk);
  }

  void testHydroplaneSpeed() {
    // Speed at which hydroplaning occurs
    double tire_pressure = 200.0;  // psi

    // Hydroplane speed (kts) = 9 * sqrt(tire_pressure)
    double hydroplane_speed = 9.0 * sqrt(tire_pressure);

    TS_ASSERT_DELTA(hydroplane_speed, 127.3, 1.0);  // kts
  }

  void testSlushDragEffect() {
    // Slush/water increases drag during takeoff roll
    double normal_drag = 800.0;
    double slush_depth_in = 0.5;
    double slush_drag_factor = 1.0 + slush_depth_in * 0.5;

    double total_drag = normal_drag * slush_drag_factor;

    TS_ASSERT(total_drag > normal_drag);
    TS_ASSERT_DELTA(total_drag, 1000.0, 10.0);
  }

  void testSnowCoveredRunway() {
    // Snow reduces braking effectiveness
    double mu_dry = 0.5;
    double mu_compact_snow = 0.2;
    double mu_loose_snow = 0.15;

    TS_ASSERT(mu_compact_snow < mu_dry);
    TS_ASSERT(mu_loose_snow < mu_compact_snow);
  }

  //===========================================================================
  // 22. PERFORMANCE PLANNING FACTORS TESTS (4 tests)
  //===========================================================================

  void testSafetyFactorTakeoff() {
    // FAR 25 requires 15% safety margin
    double actual_distance = 3000.0;
    double safety_factor = 1.15;

    double required_runway = actual_distance * safety_factor;

    TS_ASSERT_DELTA(required_runway, 3450.0, 1.0);
  }

  void testSafetyFactorLandingDry() {
    // FAR 25 landing distance factor (dry runway)
    double actual_landing = 2500.0;
    double safety_factor = 1.67;  // 60% margin

    double required_runway = actual_landing * safety_factor;

    TS_ASSERT_DELTA(required_runway, 4175.0, 5.0);
  }

  void testSafetyFactorLandingWet() {
    // FAR 25 landing distance factor (wet runway)
    double actual_landing = 2500.0;
    double safety_factor_dry = 1.67;
    double wet_multiplier = 1.15;

    double required_runway = actual_landing * safety_factor_dry * wet_multiplier;

    TS_ASSERT_DELTA(required_runway, 4801.3, 5.0);
  }

  void testWeightCorrectionFactor() {
    // Weight correction for performance charts
    double ref_weight = 12000.0;
    double actual_weight = 14000.0;

    double weight_factor = actual_weight / ref_weight;
    double distance_factor = weight_factor * weight_factor;  // Distance ~ W^2

    TS_ASSERT_DELTA(weight_factor, 1.167, 0.01);
    TS_ASSERT_DELTA(distance_factor, 1.361, 0.01);
  }
};
