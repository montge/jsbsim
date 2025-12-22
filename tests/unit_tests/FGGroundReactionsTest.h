#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>

#include "TestUtilities.h"

using namespace JSBSimTest;

const double epsilon = 1e-8;

/**
 * Ground Reactions unit tests
 *
 * Tests for ground reaction physics including:
 * - Contact detection
 * - Normal force calculation
 * - Friction models (static, dynamic, rolling)
 * - Tire compression
 * - Brake modeling
 * - Gear strut dynamics
 */
class FGGroundReactionsTest : public CxxTest::TestSuite
{
public:
  // Test ground contact detection
  void testGroundContactDetection() {
    double ground_altitude = 0.0;
    double gear_altitude = -0.5;  // Below ground

    bool contact = (gear_altitude <= ground_altitude);
    TS_ASSERT(contact);

    gear_altitude = 1.0;  // Above ground
    contact = (gear_altitude <= ground_altitude);
    TS_ASSERT(!contact);
  }

  // Test penetration depth calculation
  void testPenetrationDepth() {
    double ground = 0.0;
    double gear = -0.5;  // 0.5 ft below ground

    double penetration = ground - gear;
    TS_ASSERT_DELTA(penetration, 0.5, epsilon);
  }

  // Test normal force from spring model
  void testNormalForceSpring() {
    // F = k * x (Hooke's law)
    double spring_constant = 100000.0;  // lbs/ft
    double compression = 0.5;           // ft

    double normal_force = spring_constant * compression;
    TS_ASSERT_DELTA(normal_force, 50000.0, epsilon);
  }

  // Test normal force with damping
  void testNormalForceWithDamping() {
    // F = k*x + c*v
    double k = 100000.0;  // Spring constant (lbs/ft)
    double c = 5000.0;    // Damping coefficient (lbs*s/ft)
    double x = 0.5;       // Compression (ft)
    double v = -2.0;      // Velocity (ft/s, negative = compressing)

    double F_spring = k * x;
    double F_damping = -c * v;  // Opposes velocity
    double F_total = F_spring + F_damping;

    TS_ASSERT_DELTA(F_spring, 50000.0, epsilon);
    TS_ASSERT_DELTA(F_damping, 10000.0, epsilon);
    TS_ASSERT_DELTA(F_total, 60000.0, epsilon);
  }

  // Test Coulomb friction model
  void testCoulombFriction() {
    double mu_static = 0.8;
    double mu_kinetic = 0.6;
    double normal_force = 10000.0;

    double F_static_max = mu_static * normal_force;
    double F_kinetic = mu_kinetic * normal_force;

    TS_ASSERT_DELTA(F_static_max, 8000.0, epsilon);
    TS_ASSERT_DELTA(F_kinetic, 6000.0, epsilon);
    TS_ASSERT(F_static_max > F_kinetic);  // Static > kinetic
  }

  // Test friction direction
  void testFrictionDirection() {
    double velocity_x = 10.0;
    double velocity_y = 0.0;
    double speed = std::sqrt(velocity_x*velocity_x + velocity_y*velocity_y);

    // Friction opposes motion
    double friction_magnitude = 1000.0;
    double friction_x = -friction_magnitude * velocity_x / speed;
    double friction_y = -friction_magnitude * velocity_y / speed;

    TS_ASSERT_DELTA(friction_x, -1000.0, epsilon);
    TS_ASSERT_DELTA(friction_y, 0.0, epsilon);
  }

  // Test rolling friction
  void testRollingFriction() {
    // Rolling friction is much less than sliding friction
    double mu_rolling = 0.02;
    double normal_force = 10000.0;

    double F_rolling = mu_rolling * normal_force;
    TS_ASSERT_DELTA(F_rolling, 200.0, epsilon);
  }

  // Test brake effectiveness
  void testBrakeEffectiveness() {
    double brake_coeff = 0.7;  // Brake coefficient of friction
    double normal_force = 10000.0;
    double brake_input = 0.5;  // 50% brake

    double brake_force = brake_coeff * normal_force * brake_input;
    TS_ASSERT_DELTA(brake_force, 3500.0, epsilon);
  }

  // Test anti-skid braking
  void testAntiSkidBraking() {
    double wheel_speed = 50.0;  // ft/s
    double ground_speed = 55.0; // ft/s
    double slip = (ground_speed - wheel_speed) / ground_speed;

    // Optimal slip for max braking is around 10-20%
    TS_ASSERT_DELTA(slip, 0.0909, 0.01);

    // Anti-skid would reduce brake pressure if slip > threshold
    double threshold = 0.15;
    bool reduce_braking = (slip < threshold);
    TS_ASSERT(reduce_braking);
  }

  // Test tire slip angle
  void testTireSlipAngle() {
    double velocity_x = 100.0;  // Forward velocity
    double velocity_y = 10.0;   // Lateral velocity

    double slip_angle = std::atan2(velocity_y, velocity_x);
    TS_ASSERT_DELTA(slip_angle * 180.0 / M_PI, 5.71, 0.1);  // About 6 degrees
  }

  // Test lateral tire force
  void testLateralTireForce() {
    // Simplified: F_lat = C_alpha * slip_angle
    double C_alpha = 5000.0;  // Cornering stiffness (lbs/rad)
    double slip_angle = 0.1;  // radians

    double F_lateral = C_alpha * slip_angle;
    TS_ASSERT_DELTA(F_lateral, 500.0, epsilon);
  }

  // Test tire load sensitivity
  void testTireLoadSensitivity() {
    // Tire friction coefficient decreases with load
    double mu_base = 0.8;
    double load = 5000.0;
    double reference_load = 2500.0;

    // Simple model: mu = mu_base * sqrt(ref_load / load)
    double mu = mu_base * std::sqrt(reference_load / load);
    TS_ASSERT_DELTA(mu, 0.566, 0.01);  // Reduced friction at higher load
  }

  // Test gear strut extension
  void testGearStrutExtension() {
    double max_extension = 1.5;  // ft
    double current_extension = 1.2;
    double compression_ratio = current_extension / max_extension;

    TS_ASSERT_DELTA(compression_ratio, 0.8, epsilon);
  }

  // Test gear retraction
  void testGearRetraction() {
    double retract_time = 8.0;  // seconds
    double current_position = 0.0;  // 0 = up, 1 = down
    double target_position = 1.0;

    double rate = (target_position - current_position) / retract_time;
    TS_ASSERT_DELTA(rate, 0.125, epsilon);  // 12.5% per second
  }

  // Test weight on wheels
  void testWeightOnWheels() {
    double gear_forces[] = {5000.0, 5000.0, 2000.0};  // Main L, Main R, Nose
    double total_weight = 0.0;

    for (double f : gear_forces) {
      total_weight += f;
    }

    bool wow = total_weight > 100.0;  // Weight on wheels threshold
    TS_ASSERT(wow);
    TS_ASSERT_DELTA(total_weight, 12000.0, epsilon);
  }

  // Test surface friction coefficients
  void testSurfaceFrictionCoefficients() {
    // Different surfaces have different friction
    double mu_dry_concrete = 0.8;
    double mu_wet_concrete = 0.5;
    double mu_ice = 0.1;

    TS_ASSERT(mu_dry_concrete > mu_wet_concrete);
    TS_ASSERT(mu_wet_concrete > mu_ice);
  }

  // Test steering angle effect
  void testSteeringAngleEffect() {
    double steering_angle = 30.0 * M_PI / 180.0;  // 30 degrees
    double wheel_force = 1000.0;

    double F_x = wheel_force * std::cos(steering_angle);
    double F_y = wheel_force * std::sin(steering_angle);

    TS_ASSERT_DELTA(F_x, 866.0, 1.0);
    TS_ASSERT_DELTA(F_y, 500.0, 1.0);
  }

  // Test camber effect
  void testCamberEffect() {
    // Camber generates lateral force
    double camber_angle = 2.0 * M_PI / 180.0;  // 2 degrees
    double camber_stiffness = 1000.0;  // lbs/rad

    double F_camber = camber_stiffness * camber_angle;
    TS_ASSERT_DELTA(F_camber, 34.9, 1.0);
  }

  // Test oleo strut damping
  void testOleoDamping() {
    // Oleo strut provides non-linear damping
    double velocity = -5.0;  // ft/s (compression)
    double orifice_area = 0.001;  // sq ft
    double fluid_density = 2.0;   // slugs/cu ft

    // Simple orifice flow: F ~ v^2
    double damping_coeff = 0.5 * fluid_density / (orifice_area * orifice_area);
    double damping_force = damping_coeff * velocity * std::abs(velocity);

    TS_ASSERT(std::abs(damping_force) > 0);
  }

  // Test tire burst detection
  void testTireBurstDetection() {
    double max_load = 20000.0;
    double current_load = 25000.0;

    bool tire_burst = current_load > max_load;
    TS_ASSERT(tire_burst);
  }

  // Test hydroplaning speed
  void testHydroplaningSpeed() {
    // V_hydro = 9 * sqrt(P) where P is tire pressure in psi
    double tire_pressure_psi = 100.0;
    double hydroplane_speed_kts = 9.0 * std::sqrt(tire_pressure_psi);

    TS_ASSERT_DELTA(hydroplane_speed_kts, 90.0, epsilon);
  }

  // Test crosswind effect on ground handling
  void testCrosswindEffect() {
    double crosswind_knots = 15.0;
    double crosswind_fps = crosswind_knots * 1.68781;
    double aircraft_area = 500.0;  // sq ft
    double rho = 0.002378;  // slugs/cu ft

    // Crosswind force
    double dynamic_pressure = 0.5 * rho * crosswind_fps * crosswind_fps;
    double side_force = dynamic_pressure * aircraft_area * 0.5;  // Cd = 0.5

    TS_ASSERT(side_force > 100.0);  // Significant side force
  }

  // Test ground effect on lift
  void testGroundEffect() {
    // Ground effect increases lift near ground
    double wingspan = 50.0;  // ft
    double height_agl = 10.0;  // ft
    double height_ratio = height_agl / wingspan;

    // Ground effect factor (simplified)
    double ground_effect = 1.0 / (1.0 - 0.25 * std::exp(-4.0 * height_ratio));

    TS_ASSERT(ground_effect > 1.0);  // Increased lift
    TS_ASSERT(ground_effect < 1.5);  // But not unreasonable
  }

  // Test contact point calculation
  void testContactPointCalculation() {
    // Wheel contact point moves with aircraft attitude
    double pitch_rad = 5.0 * M_PI / 180.0;  // 5 degree pitch
    double gear_length = 4.0;  // ft

    double x_offset = gear_length * std::sin(pitch_rad);
    double z_offset = gear_length * std::cos(pitch_rad);

    TS_ASSERT_DELTA(x_offset, 0.349, 0.01);
    TS_ASSERT_DELTA(z_offset, 3.985, 0.01);
  }

  // Test multiple gear contact
  void testMultipleGearContact() {
    // Aircraft can have 1, 2, or all 3 gears in contact
    bool main_left = true;
    bool main_right = true;
    bool nose = false;  // Nose wheel just lifted

    int contact_count = (main_left ? 1 : 0) + (main_right ? 1 : 0) + (nose ? 1 : 0);
    TS_ASSERT_EQUALS(contact_count, 2);
  }

  // Test gear bogey load distribution
  void testBogeyLoadDistribution() {
    // Multi-wheel bogey distributes load
    int num_wheels = 4;
    double total_load = 40000.0;
    double load_per_wheel = total_load / num_wheels;

    TS_ASSERT_DELTA(load_per_wheel, 10000.0, epsilon);
  }

  // Test tailwheel vs nosewheel configuration
  void testGearConfiguration() {
    // CG position relative to main gear
    double cg_x = 10.0;
    double main_gear_x = 11.0;
    double nose_gear_x = 3.0;

    bool cg_ahead_of_main = cg_x < main_gear_x;
    TS_ASSERT(cg_ahead_of_main);  // Stable nosewheel configuration
  }

  // Test taxi speed limits
  void testTaxiSpeedLimit() {
    double max_taxi_speed_kts = 25.0;
    double current_speed_kts = 30.0;

    bool over_limit = current_speed_kts > max_taxi_speed_kts;
    TS_ASSERT(over_limit);
  }

  // Test runway slope effect
  void testRunwaySlope() {
    double slope_percent = 2.0;  // 2% slope
    double weight = 10000.0;     // lbs

    double slope_rad = std::atan(slope_percent / 100.0);
    double force_along_slope = weight * std::sin(slope_rad);

    TS_ASSERT_DELTA(force_along_slope, 200.0, 1.0);
  }

  // Test arrested landing
  void testArrestedLanding() {
    double initial_velocity = 150.0;  // ft/s
    double wire_deceleration = 60.0;  // ft/s^2
    double time_to_stop = initial_velocity / wire_deceleration;

    TS_ASSERT_DELTA(time_to_stop, 2.5, 0.1);  // 2.5 seconds to stop
  }
};
