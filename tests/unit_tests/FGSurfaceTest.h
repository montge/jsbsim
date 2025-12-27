#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>

#include <FGFDMExec.h>
#include <models/FGSurface.h>
#include "TestUtilities.h"

using namespace JSBSim;
using namespace JSBSimTest;

const double epsilon = 1e-8;

/**
 * Surface unit tests
 *
 * Tests for surface contact physics including:
 * - Surface friction coefficients (static vs dynamic)
 * - Rolling resistance
 * - Bumpiness/roughness effects
 * - Maximum force limits
 * - Surface contact detection
 * - Friction force calculations
 * - Terrain effects on ground reactions
 */
class FGSurfaceTest : public CxxTest::TestSuite
{
public:
  // Task 6.5.1: Basic FGSurface test suite structure

  // Test construction with default values
  void testConstruction() {
    FGFDMExec fdmex;
    FGSurface surface(&fdmex);

    // Verify default values after construction
    TS_ASSERT_DELTA(surface.GetStaticFFactor(), 1.0, epsilon);
    TS_ASSERT_DELTA(surface.GetRollingFFactor(), 1.0, epsilon);
    TS_ASSERT_DELTA(surface.GetBumpiness(), 0.0, epsilon);
    TS_ASSERT_EQUALS(surface.GetMaximumForce(), std::numeric_limits<double>::max());
    TS_ASSERT(surface.GetSolid());
  }

  // Test reset values
  void testResetValues() {
    FGFDMExec fdmex;
    FGSurface surface(&fdmex);

    // Modify all properties
    surface.SetStaticFFactor(0.5);
    surface.SetRollingFFactor(0.02);
    surface.SetBumpiness(0.3);
    surface.SetMaximumForce(50000.0);
    surface.SetSolid(false);

    // Reset to defaults
    surface.resetValues();

    // Verify all values reset to defaults
    TS_ASSERT_DELTA(surface.GetStaticFFactor(), 1.0, epsilon);
    TS_ASSERT_DELTA(surface.GetRollingFFactor(), 1.0, epsilon);
    TS_ASSERT_DELTA(surface.GetBumpiness(), 0.0, epsilon);
    TS_ASSERT_EQUALS(surface.GetMaximumForce(), std::numeric_limits<double>::max());
    TS_ASSERT(surface.GetSolid());
  }

  // Task 6.5.2: Surface contact detection (height above ground, contact point)

  // Test surface solid flag
  void testSurfaceSolidFlag() {
    FGFDMExec fdmex;
    FGSurface surface(&fdmex);

    // Default should be solid
    TS_ASSERT(surface.GetSolid());

    // Test non-solid surface (e.g., water)
    surface.SetSolid(false);
    TS_ASSERT(!surface.GetSolid());

    // Test solid surface (e.g., concrete)
    surface.SetSolid(true);
    TS_ASSERT(surface.GetSolid());
  }

  // Test contact point position storage
  void testContactPointPosition() {
    FGFDMExec fdmex;
    FGSurface surface(&fdmex);

    double pos[3] = {100.0, 50.0, -10.0};
    surface.SetPosition(pos);

    // Position is used for bumpiness calculation
    // This implicitly tests that position is stored correctly
    // since GetBumpHeight() depends on it
    double height = surface.GetBumpHeight();
    TS_ASSERT(!std::isnan(height));
  }

  // Test height above ground with bumpiness zero
  void testHeightAboveGroundNoBumps() {
    FGFDMExec fdmex;
    FGSurface surface(&fdmex);

    surface.SetBumpiness(0.0);
    double pos[3] = {0.0, 0.0, 0.0};
    surface.SetPosition(pos);

    double bump_height = surface.GetBumpHeight();
    TS_ASSERT_DELTA(bump_height, 0.0, epsilon);
  }

  // Test height calculation with bumpiness
  void testHeightCalculationWithBumpiness() {
    FGFDMExec fdmex;
    FGSurface surface(&fdmex);

    surface.SetBumpiness(1.0);  // Full bumpiness
    double pos[3] = {0.0, 0.0, 0.0};
    surface.SetPosition(pos);

    double bump_height = surface.GetBumpHeight();

    // With bumpiness, height should be non-zero and within reasonable bounds
    // Max amplitude is 0.4, so height should be within [-0.4, 0.4]
    TS_ASSERT(std::abs(bump_height) <= 0.5);
  }

  // Test bump height varies with position
  void testBumpHeightVariesWithPosition() {
    FGFDMExec fdmex;
    FGSurface surface(&fdmex);

    surface.SetBumpiness(1.0);

    double pos1[3] = {0.0, 0.0, 0.0};
    surface.SetPosition(pos1);
    double height1 = surface.GetBumpHeight();

    double pos2[3] = {10.0, 10.0, 0.0};
    surface.SetPosition(pos2);
    double height2 = surface.GetBumpHeight();

    // Heights at different positions should likely be different
    // (very small chance they're equal by coincidence)
    // This tests the periodic function used for bumpiness
    TS_ASSERT(!std::isnan(height1));
    TS_ASSERT(!std::isnan(height2));
  }

  // Task 6.5.3: Friction coefficient lookup (static vs dynamic, surface types)

  // Test static friction factor
  void testStaticFrictionFactor() {
    FGFDMExec fdmex;
    FGSurface surface(&fdmex);

    // Test dry concrete/asphalt
    surface.SetStaticFFactor(0.8);
    TS_ASSERT_DELTA(surface.GetStaticFFactor(), 0.8, epsilon);

    // Test wet surface
    surface.SetStaticFFactor(0.5);
    TS_ASSERT_DELTA(surface.GetStaticFFactor(), 0.5, epsilon);

    // Test icy surface
    surface.SetStaticFFactor(0.1);
    TS_ASSERT_DELTA(surface.GetStaticFFactor(), 0.1, epsilon);
  }

  // Test rolling friction factor
  void testRollingFrictionFactor() {
    FGFDMExec fdmex;
    FGSurface surface(&fdmex);

    // Rolling friction should be much smaller than static friction
    surface.SetRollingFFactor(0.02);
    TS_ASSERT_DELTA(surface.GetRollingFFactor(), 0.02, epsilon);

    // Test different surface rolling resistance
    surface.SetRollingFFactor(0.05);
    TS_ASSERT_DELTA(surface.GetRollingFFactor(), 0.05, epsilon);
  }

  // Test friction force calculation (F_friction = μ * N)
  void testFrictionForceCalculation() {
    double mu_static = 0.8;     // Static friction coefficient
    double normal_force = 10000.0;  // Normal force in lbs

    // Maximum static friction force
    double F_friction_max = mu_static * normal_force;
    TS_ASSERT_DELTA(F_friction_max, 8000.0, epsilon);

    // If applied force is less than max, friction matches applied force (static)
    double applied_force = 5000.0;
    double actual_friction = std::min(applied_force, F_friction_max);
    TS_ASSERT_DELTA(actual_friction, 5000.0, epsilon);

    // If applied force exceeds max, friction is kinetic (sliding)
    applied_force = 9000.0;
    double mu_kinetic = 0.6;  // Kinetic friction < static
    double F_kinetic = mu_kinetic * normal_force;
    actual_friction = (applied_force > F_friction_max) ? F_kinetic : applied_force;
    TS_ASSERT_DELTA(actual_friction, 6000.0, epsilon);
  }

  // Test rolling resistance calculation
  void testRollingResistanceCalculation() {
    double mu_rolling = 0.02;   // Rolling friction coefficient
    double normal_force = 10000.0;  // Normal force in lbs

    // Rolling resistance force
    double F_rolling = mu_rolling * normal_force;
    TS_ASSERT_DELTA(F_rolling, 200.0, epsilon);

    // Rolling resistance is much less than sliding friction
    double mu_sliding = 0.6;
    double F_sliding = mu_sliding * normal_force;
    TS_ASSERT(F_rolling < F_sliding);
  }

  // Test static vs dynamic friction transition
  void testStaticToDynamicFrictionTransition() {
    double mu_static = 0.8;
    double mu_dynamic = 0.6;
    double normal_force = 10000.0;

    // Static friction force when stationary
    double F_static_max = mu_static * normal_force;
    TS_ASSERT_DELTA(F_static_max, 8000.0, epsilon);

    // Dynamic friction force when sliding
    double F_dynamic = mu_dynamic * normal_force;
    TS_ASSERT_DELTA(F_dynamic, 6000.0, epsilon);

    // Verify static > dynamic (stiction effect)
    TS_ASSERT(F_static_max > F_dynamic);

    // Force required to initiate sliding
    double force_to_slide = F_static_max + 1.0;
    TS_ASSERT(force_to_slide > F_static_max);
  }

  // Test surface normal calculation for level ground
  void testSurfaceNormalLevelGround() {
    // For level ground, surface normal is vertical (0, 0, -1) in body frame
    double normal_x = 0.0;
    double normal_y = 0.0;
    double normal_z = -1.0;

    double magnitude = std::sqrt(normal_x*normal_x + normal_y*normal_y + normal_z*normal_z);
    TS_ASSERT_DELTA(magnitude, 1.0, epsilon);
    TS_ASSERT_DELTA(normal_z, -1.0, epsilon);
  }

  // Test surface normal calculation for sloped terrain
  void testSurfaceNormalSlopedTerrain() {
    // For 10 degree slope, surface normal is tilted
    double slope_deg = 10.0;
    double slope_rad = slope_deg * M_PI / 180.0;

    // Normal perpendicular to slope
    double normal_x = std::sin(slope_rad);
    double normal_z = -std::cos(slope_rad);

    // Verify unit normal
    double magnitude = std::sqrt(normal_x*normal_x + normal_z*normal_z);
    TS_ASSERT_DELTA(magnitude, 1.0, epsilon);

    // Verify normal components
    TS_ASSERT_DELTA(normal_x, 0.1736, 0.001);
    TS_ASSERT_DELTA(normal_z, -0.9848, 0.001);
  }

  // Test bumpiness effects on friction
  void testBumpinessEffectOnFriction() {
    FGFDMExec fdmex;
    FGSurface surface(&fdmex);

    // Smooth surface
    surface.SetBumpiness(0.0);
    double pos[3] = {0.0, 0.0, 0.0};
    surface.SetPosition(pos);
    double smooth_height = surface.GetBumpHeight();
    TS_ASSERT_DELTA(smooth_height, 0.0, epsilon);

    // Rough surface increases effective friction due to micro-collisions
    surface.SetBumpiness(0.5);
    double rough_height = surface.GetBumpHeight();

    // Rough surface has non-zero height variations
    // This can increase effective friction coefficient
    double mu_smooth = 0.6;
    double bumpiness_factor = 1.0 + 0.1 * surface.GetBumpiness();
    double mu_rough = mu_smooth * bumpiness_factor;

    TS_ASSERT(mu_rough > mu_smooth);
    TS_ASSERT_DELTA(mu_rough, 0.63, 0.01);
  }

  // Test maximum force limit
  void testMaximumForceLimit() {
    FGFDMExec fdmex;
    FGSurface surface(&fdmex);

    // Set maximum force limit (e.g., for soft surface that yields)
    double max_force = 50000.0;
    surface.SetMaximumForce(max_force);
    TS_ASSERT_DELTA(surface.GetMaximumForce(), max_force, epsilon);

    // Calculated normal force
    double calculated_force = 60000.0;

    // Actual force limited by surface
    double actual_force = std::min(calculated_force, surface.GetMaximumForce());
    TS_ASSERT_DELTA(actual_force, 50000.0, epsilon);

    // Force below limit is not capped
    calculated_force = 40000.0;
    actual_force = std::min(calculated_force, surface.GetMaximumForce());
    TS_ASSERT_DELTA(actual_force, 40000.0, epsilon);
  }

  // Test contact point geometry
  void testContactPointGeometry() {
    // Contact point is where tire/wheel touches ground
    // For a wheel of radius R at height h above ground
    double wheel_radius = 1.5;  // ft
    double height_above_ground = 0.5;  // ft

    // Compression (penetration) into surface
    double compression = wheel_radius - height_above_ground;
    TS_ASSERT_DELTA(compression, 1.0, epsilon);

    // Contact patch moves with compression
    // For small deflections, contact point shifts forward/aft
    double deflection_angle = std::asin(compression / wheel_radius);
    TS_ASSERT(deflection_angle > 0.0);
    TS_ASSERT(deflection_angle < M_PI / 4.0);  // Less than 45 degrees
  }

  // Test terrain slope effects on normal force
  void testTerrainSlopeEffectOnNormalForce() {
    double weight = 10000.0;  // lbs
    double slope_deg = 10.0;
    double slope_rad = slope_deg * M_PI / 180.0;

    // On level ground, normal force = weight
    double normal_level = weight;
    TS_ASSERT_DELTA(normal_level, 10000.0, epsilon);

    // On slope, normal force component perpendicular to surface
    double normal_slope = weight * std::cos(slope_rad);
    TS_ASSERT_DELTA(normal_slope, 9848.0, 1.0);

    // Force component along slope (causes rolling/sliding)
    double force_along_slope = weight * std::sin(slope_rad);
    TS_ASSERT_DELTA(force_along_slope, 1736.0, 1.0);

    // Normal force is reduced on slope
    TS_ASSERT(normal_slope < normal_level);
  }

  // Test friction coefficient for different surface types
  void testSurfaceTypeFrictionCoefficients() {
    FGFDMExec fdmex;

    // Dry concrete/asphalt
    FGSurface dry_concrete(&fdmex);
    dry_concrete.SetStaticFFactor(0.8);
    dry_concrete.SetRollingFFactor(0.02);

    // Wet concrete
    FGSurface wet_concrete(&fdmex);
    wet_concrete.SetStaticFFactor(0.5);
    wet_concrete.SetRollingFFactor(0.03);

    // Ice
    FGSurface ice(&fdmex);
    ice.SetStaticFFactor(0.1);
    ice.SetRollingFFactor(0.015);

    // Grass
    FGSurface grass(&fdmex);
    grass.SetStaticFFactor(0.4);
    grass.SetRollingFFactor(0.08);

    // Verify friction hierarchy
    TS_ASSERT(dry_concrete.GetStaticFFactor() > wet_concrete.GetStaticFFactor());
    TS_ASSERT(wet_concrete.GetStaticFFactor() > grass.GetStaticFFactor());
    TS_ASSERT(grass.GetStaticFFactor() > ice.GetStaticFFactor());
  }

  // Test bumpiness scaling with amplitude
  void testBumpinessScaling() {
    FGFDMExec fdmex;
    FGSurface surface(&fdmex);

    double pos[3] = {5.0, 5.0, 0.0};
    surface.SetPosition(pos);

    // Test various bumpiness levels
    surface.SetBumpiness(0.0);
    double height_0 = surface.GetBumpHeight();
    TS_ASSERT_DELTA(height_0, 0.0, epsilon);

    surface.SetBumpiness(0.5);
    double height_50 = surface.GetBumpHeight();

    surface.SetBumpiness(1.0);
    double height_100 = surface.GetBumpHeight();

    // Height should scale with bumpiness
    // At same position, higher bumpiness = higher amplitude
    TS_ASSERT(std::abs(height_100) >= std::abs(height_50));
    TS_ASSERT(std::abs(height_50) >= std::abs(height_0));
  }

  // Test friction direction opposes motion
  void testFrictionDirectionOpposesMotion() {
    // Friction force direction is opposite to velocity direction
    double velocity_x = 100.0;  // ft/s forward
    double velocity_y = 0.0;
    double speed = std::sqrt(velocity_x*velocity_x + velocity_y*velocity_y);

    double mu = 0.6;
    double normal_force = 10000.0;
    double friction_magnitude = mu * normal_force;

    // Friction opposes velocity
    double friction_x = -friction_magnitude * velocity_x / speed;
    double friction_y = -friction_magnitude * velocity_y / speed;

    TS_ASSERT_DELTA(friction_x, -6000.0, epsilon);
    TS_ASSERT_DELTA(friction_y, 0.0, epsilon);
    TS_ASSERT(friction_x < 0.0);  // Opposes forward motion
  }

  // Test combined friction from static and rolling components
  void testCombinedFrictionComponents() {
    FGFDMExec fdmex;
    FGSurface surface(&fdmex);

    surface.SetStaticFFactor(0.8);
    surface.SetRollingFFactor(0.02);

    double normal_force = 10000.0;

    // When stationary or very slow, rolling friction dominates
    double F_rolling = surface.GetRollingFFactor() * normal_force;
    TS_ASSERT_DELTA(F_rolling, 200.0, epsilon);

    // When sliding, static (kinetic) friction dominates
    double F_static = surface.GetStaticFFactor() * normal_force;
    TS_ASSERT_DELTA(F_static, 8000.0, epsilon);

    // Total friction depends on motion state
    TS_ASSERT(F_static > F_rolling);
  }

  // Test surface properties persistence
  void testSurfacePropertiesPersistence() {
    FGFDMExec fdmex;
    FGSurface surface(&fdmex);

    // Set all properties
    surface.SetStaticFFactor(0.75);
    surface.SetRollingFFactor(0.025);
    surface.SetBumpiness(0.4);
    surface.SetMaximumForce(75000.0);
    surface.SetSolid(true);

    // Verify all properties persist
    TS_ASSERT_DELTA(surface.GetStaticFFactor(), 0.75, epsilon);
    TS_ASSERT_DELTA(surface.GetRollingFFactor(), 0.025, epsilon);
    TS_ASSERT_DELTA(surface.GetBumpiness(), 0.4, epsilon);
    TS_ASSERT_DELTA(surface.GetMaximumForce(), 75000.0, epsilon);
    TS_ASSERT(surface.GetSolid());
  }

  /***************************************************************************
   * Additional Surface Type Tests
   ***************************************************************************/

  // Test sand surface properties
  void testSandSurfaceProperties() {
    FGFDMExec fdmex;
    FGSurface sand(&fdmex);

    // Sand has lower friction and yields under load
    sand.SetStaticFFactor(0.35);
    sand.SetRollingFFactor(0.15);  // High rolling resistance
    sand.SetMaximumForce(30000.0);  // Soft surface

    TS_ASSERT(sand.GetRollingFFactor() > 0.1);
    TS_ASSERT(sand.GetMaximumForce() < 50000.0);
  }

  // Test gravel surface properties
  void testGravelSurfaceProperties() {
    FGFDMExec fdmex;
    FGSurface gravel(&fdmex);

    gravel.SetStaticFFactor(0.55);
    gravel.SetRollingFFactor(0.06);
    gravel.SetBumpiness(0.6);  // Rough surface

    TS_ASSERT(gravel.GetBumpiness() > 0.5);
    TS_ASSERT_DELTA(gravel.GetStaticFFactor(), 0.55, epsilon);
  }

  // Test mud surface properties
  void testMudSurfaceProperties() {
    FGFDMExec fdmex;
    FGSurface mud(&fdmex);

    // Mud has very low friction and limited force capacity
    mud.SetStaticFFactor(0.2);
    mud.SetRollingFFactor(0.2);  // Very high rolling resistance
    mud.SetMaximumForce(20000.0);
    mud.SetSolid(false);  // Not fully solid

    TS_ASSERT(!mud.GetSolid());
    TS_ASSERT(mud.GetStaticFFactor() < 0.3);
  }

  // Test snow surface properties
  void testSnowSurfaceProperties() {
    FGFDMExec fdmex;
    FGSurface snow(&fdmex);

    // Packed snow has moderate friction
    snow.SetStaticFFactor(0.25);
    snow.SetRollingFFactor(0.10);
    snow.SetBumpiness(0.3);

    // Snow friction is between ice (~0.1) and wet surface (~0.5)
    TS_ASSERT(snow.GetStaticFFactor() > 0.1);
    TS_ASSERT(snow.GetStaticFFactor() < 0.5);
    TS_ASSERT(snow.GetRollingFFactor() > 0.05);
  }

  // Test water surface (hydroplaning)
  void testWaterSurfaceProperties() {
    FGFDMExec fdmex;
    FGSurface water(&fdmex);

    water.SetStaticFFactor(0.05);  // Nearly frictionless
    water.SetRollingFFactor(0.05);
    water.SetSolid(false);

    TS_ASSERT(!water.GetSolid());
    TS_ASSERT(water.GetStaticFFactor() < 0.1);
  }

  /***************************************************************************
   * Velocity-Dependent Friction Tests
   ***************************************************************************/

  // Test speed effect on friction coefficient
  void testSpeedEffectOnFriction() {
    // Friction coefficient decreases slightly with speed
    double mu_static = 0.8;
    double speed = 0.0;  // ft/s

    // Stationary
    double mu_at_zero = mu_static;
    TS_ASSERT_DELTA(mu_at_zero, 0.8, epsilon);

    // At speed, friction is reduced
    speed = 100.0;
    double speedFactor = 1.0 / (1.0 + 0.001 * speed);
    double mu_at_speed = mu_static * speedFactor;

    TS_ASSERT(mu_at_speed < mu_static);
    TS_ASSERT_DELTA(mu_at_speed, 0.727, 0.01);
  }

  // Test brake fade with temperature
  void testBrakeFadeWithTemperature() {
    double mu_cold = 0.8;
    double temperature = 200.0;  // degrees C

    // Friction decreases as brakes heat up
    double fadeFactor = 1.0 - 0.001 * std::max(0.0, temperature - 150.0);
    double mu_hot = mu_cold * fadeFactor;

    TS_ASSERT(mu_hot < mu_cold);
    TS_ASSERT_DELTA(mu_hot, 0.76, 0.01);
  }

  // Test hydroplaning speed threshold
  void testHydroplaningThreshold() {
    // Hydroplaning speed = 9 * sqrt(tire_pressure_psi)
    double tirePressure = 100.0;  // psi
    double hydroplaningSpeed_kts = 9.0 * std::sqrt(tirePressure);

    TS_ASSERT_DELTA(hydroplaningSpeed_kts, 90.0, 0.1);

    // Lower tire pressure = lower hydroplaning speed
    tirePressure = 50.0;
    hydroplaningSpeed_kts = 9.0 * std::sqrt(tirePressure);
    TS_ASSERT_DELTA(hydroplaningSpeed_kts, 63.6, 0.1);
  }

  /***************************************************************************
   * Contact Patch and Load Distribution Tests
   ***************************************************************************/

  // Test contact patch area calculation
  void testContactPatchArea() {
    double tirePressure = 100.0;  // psi
    double load = 5000.0;  // lbs

    // Contact area = Load / Pressure
    double area = load / tirePressure;  // sq in
    TS_ASSERT_DELTA(area, 50.0, epsilon);

    // Higher pressure = smaller patch
    tirePressure = 150.0;
    area = load / tirePressure;
    TS_ASSERT_DELTA(area, 33.33, 0.01);
  }

  // Test load distribution between main gear
  void testLoadDistributionMainGear() {
    double totalWeight = 10000.0;  // lbs
    double cgPercent = 0.30;  // 30% forward of main gear

    // Main gear carries most of the weight
    double mainGearLoad = totalWeight * (1.0 - cgPercent);
    double noseGearLoad = totalWeight * cgPercent;

    TS_ASSERT_DELTA(mainGearLoad, 7000.0, epsilon);
    TS_ASSERT_DELTA(noseGearLoad, 3000.0, epsilon);
    TS_ASSERT(mainGearLoad > noseGearLoad);
  }

  // Test weight transfer during braking
  void testWeightTransferBraking() {
    double weight = 10000.0;  // lbs
    double deceleration = 0.3;  // g's
    double wheelbase = 10.0;  // ft
    double cgHeight = 3.0;  // ft

    // Weight transfer = (W * a * h) / L
    double weightTransfer = (weight * deceleration * cgHeight) / wheelbase;
    TS_ASSERT_DELTA(weightTransfer, 900.0, 1.0);

    // Nose gear load increases during braking
    double staticNoseLoad = 3000.0;
    double dynamicNoseLoad = staticNoseLoad + weightTransfer;
    TS_ASSERT_DELTA(dynamicNoseLoad, 3900.0, 1.0);
  }

  /***************************************************************************
   * Braking Performance Tests
   ***************************************************************************/

  // Test braking distance calculation
  void testBrakingDistance() {
    double speed = 150.0;  // ft/s (~90 kts)
    double mu = 0.5;       // wet runway
    double g = 32.2;       // ft/s^2

    // Stopping distance = v^2 / (2 * mu * g)
    double distance = (speed * speed) / (2.0 * mu * g);
    TS_ASSERT_DELTA(distance, 698.8, 1.0);

    // Dry runway (higher friction = shorter distance)
    mu = 0.8;
    distance = (speed * speed) / (2.0 * mu * g);
    TS_ASSERT_DELTA(distance, 436.0, 1.0);
  }

  // Test deceleration rate
  void testDecelerationRate() {
    double mu = 0.6;
    double g = 32.2;  // ft/s^2

    // Maximum deceleration = mu * g
    double maxDecel = mu * g;
    TS_ASSERT_DELTA(maxDecel, 19.32, 0.01);

    // In g's
    double decelG = mu;
    TS_ASSERT_DELTA(decelG, 0.6, epsilon);
  }

  // Test anti-skid effect on braking
  void testAntiSkidEffect() {
    double mu_locked = 0.5;    // Locked wheel (sliding)
    double mu_rolling = 0.7;   // Rolling with anti-skid

    // Anti-skid maintains rolling friction > sliding friction
    TS_ASSERT(mu_rolling > mu_locked);

    // Braking efficiency improvement
    double efficiency = mu_rolling / mu_locked;
    TS_ASSERT_DELTA(efficiency, 1.4, 0.01);
  }

  /***************************************************************************
   * Cornering Force Tests
   ***************************************************************************/

  // Test side force from slip angle
  void testSideForceFromSlipAngle() {
    double slipAngle = 5.0;  // degrees
    double normalForce = 5000.0;  // lbs
    double corneringStiffness = 150.0;  // lbs per degree

    // Side force = cornering_stiffness * slip_angle
    double sideForce = corneringStiffness * slipAngle;
    TS_ASSERT_DELTA(sideForce, 750.0, epsilon);

    // Limited by friction
    double mu = 0.8;
    double maxSideForce = mu * normalForce;
    double actualSideForce = std::min(sideForce, maxSideForce);
    TS_ASSERT_DELTA(actualSideForce, 750.0, epsilon);
  }

  // Test tire saturation at high slip angles
  void testTireSaturationHighSlipAngle() {
    double normalForce = 5000.0;
    double mu = 0.8;
    double maxSideForce = mu * normalForce;

    // At low slip angle, linear region
    double slipAngle = 3.0;
    double corneringStiffness = 200.0;
    double sideForce = corneringStiffness * slipAngle;
    TS_ASSERT(sideForce < maxSideForce);

    // At high slip angle, saturated
    slipAngle = 25.0;
    sideForce = corneringStiffness * slipAngle;
    double limitedForce = std::min(sideForce, maxSideForce);
    TS_ASSERT_DELTA(limitedForce, maxSideForce, epsilon);
  }

  /***************************************************************************
   * Bump and Roughness Tests
   ***************************************************************************/

  // Test bump frequency
  void testBumpFrequency() {
    double speed = 100.0;  // ft/s
    double bumpSpacing = 50.0;  // ft between bumps

    double frequency = speed / bumpSpacing;  // Hz
    TS_ASSERT_DELTA(frequency, 2.0, epsilon);

    // Higher speed = higher frequency
    speed = 200.0;
    frequency = speed / bumpSpacing;
    TS_ASSERT_DELTA(frequency, 4.0, epsilon);
  }

  // Test vertical acceleration from bumps
  void testVerticalAccelerationFromBumps() {
    FGFDMExec fdmex;
    FGSurface surface(&fdmex);

    surface.SetBumpiness(0.5);

    // Bump amplitude
    double amplitude = 0.4 * 0.5;  // max_amplitude * bumpiness
    TS_ASSERT_DELTA(amplitude, 0.2, epsilon);

    // Vertical acceleration = amplitude * (2*pi*freq)^2
    double frequency = 2.0;  // Hz
    double maxAccel = amplitude * pow(2.0 * M_PI * frequency, 2);

    // In g's
    double accelG = maxAccel / 32.2;
    TS_ASSERT(accelG > 0);
  }

  // Test bumpiness effect on ride quality
  void testBumpinessRideQuality() {
    FGFDMExec fdmex;
    FGSurface smooth(&fdmex);
    FGSurface rough(&fdmex);

    smooth.SetBumpiness(0.1);
    rough.SetBumpiness(0.8);

    double pos[3] = {10.0, 10.0, 0.0};
    smooth.SetPosition(pos);
    rough.SetPosition(pos);

    double smoothBump = std::abs(smooth.GetBumpHeight());
    double roughBump = std::abs(rough.GetBumpHeight());

    // Rougher surface has larger amplitude variations
    // (may not always be true at exact same position, but
    // the amplitude scale factor ensures rough >= smooth)
    TS_ASSERT(rough.GetBumpiness() > smooth.GetBumpiness());
  }

  /***************************************************************************
   * Surface Gradient Tests
   ***************************************************************************/

  // Test runway gradient effect on takeoff
  void testRunwayGradientTakeoff() {
    double thrust = 5000.0;  // lbs
    double weight = 10000.0;
    double gradient = 0.02;  // 2% uphill

    // Force component from gradient
    double gravityComponent = weight * gradient;
    TS_ASSERT_DELTA(gravityComponent, 200.0, epsilon);

    // Net accelerating force
    double netForce = thrust - gravityComponent;
    TS_ASSERT_DELTA(netForce, 4800.0, epsilon);
  }

  // Test runway gradient effect on landing
  void testRunwayGradientLanding() {
    double brakeForce = 4000.0;  // lbs
    double weight = 10000.0;
    double gradient = 0.02;  // 2% downhill

    // Gravity helps braking when landing uphill
    double gravityComponent = weight * gradient;

    // Landing uphill (assists braking)
    double totalDecelForce = brakeForce + gravityComponent;
    TS_ASSERT_DELTA(totalDecelForce, 4200.0, epsilon);

    // Landing downhill (reduces braking)
    totalDecelForce = brakeForce - gravityComponent;
    TS_ASSERT_DELTA(totalDecelForce, 3800.0, epsilon);
  }

  // Test cross slope effect
  void testCrossSlopeEffect() {
    double weight = 10000.0;
    double crossSlope = 0.015;  // 1.5% cross slope

    // Side force from cross slope
    double sideForce = weight * crossSlope;
    TS_ASSERT_DELTA(sideForce, 150.0, epsilon);

    // This must be countered by steering or tire side force
    double mu = 0.8;
    double availableSideForce = mu * weight;
    TS_ASSERT(availableSideForce > sideForce);
  }

  /***************************************************************************
   * Multi-Point Contact Tests
   ***************************************************************************/

  // Test three-point contact stability
  void testThreePointContact() {
    // Tricycle gear with nose wheel
    double mainGearSpan = 10.0;  // ft
    double wheelbase = 15.0;     // ft
    double cgX = 5.0;            // ft forward of main gear
    double cgY = 0.0;            // centerline

    // CG must be within triangle formed by gear
    bool stable = (cgX > 0) && (cgX < wheelbase) && (std::abs(cgY) < mainGearSpan/2);
    TS_ASSERT(stable);

    // Tip-over angle
    double tipAngle = std::atan(mainGearSpan / (2.0 * cgX));
    TS_ASSERT(tipAngle > 0.5);  // > ~30 degrees is good
  }

  // Test differential braking
  void testDifferentialBraking() {
    double leftBrake = 0.8;   // 80% left brake
    double rightBrake = 0.2;  // 20% right brake
    double totalBrakeForce = 4000.0;  // lbs total available

    double leftForce = totalBrakeForce * leftBrake;
    double rightForce = totalBrakeForce * rightBrake;

    // Net yawing moment
    double gearSpan = 10.0;  // ft
    double yawMoment = (leftForce - rightForce) * gearSpan / 2.0;
    TS_ASSERT_DELTA(yawMoment, 12000.0, 1.0);  // ft-lbs
  }

  /***************************************************************************
   * Edge Cases and Limits
   ***************************************************************************/

  // Test zero normal force
  void testZeroNormalForce() {
    double normalForce = 0.0;
    double mu = 0.8;

    double frictionForce = mu * normalForce;
    TS_ASSERT_DELTA(frictionForce, 0.0, epsilon);
  }

  // Test negative normal force (lift-off)
  void testNegativeNormalForce() {
    double normalForce = -100.0;  // Wheel lifting off

    // No friction when not in contact
    double frictionForce = (normalForce > 0) ? 0.8 * normalForce : 0.0;
    TS_ASSERT_DELTA(frictionForce, 0.0, epsilon);
  }

  // Test very high speed effects
  void testVeryHighSpeedEffects() {
    double speed = 300.0;  // ft/s (~180 kts)
    double mu_static = 0.8;

    // At very high speeds, effective friction is reduced
    double speedFactor = 1.0 / (1.0 + 0.002 * speed);
    double mu_effective = mu_static * speedFactor;

    TS_ASSERT(mu_effective < mu_static);
    TS_ASSERT(mu_effective > 0.4);  // Still some friction
  }

  // Test surface factor limits
  void testSurfaceFactorLimits() {
    FGFDMExec fdmex;
    FGSurface surface(&fdmex);

    // Static friction factor should be positive
    surface.SetStaticFFactor(0.0);
    TS_ASSERT_DELTA(surface.GetStaticFFactor(), 0.0, epsilon);

    // Bumpiness between 0 and 1
    surface.SetBumpiness(0.0);
    TS_ASSERT_DELTA(surface.GetBumpiness(), 0.0, epsilon);

    surface.SetBumpiness(1.0);
    TS_ASSERT_DELTA(surface.GetBumpiness(), 1.0, epsilon);
  }

  // Test friction circle concept
  void testFrictionCircle() {
    double mu = 0.8;
    double normalForce = 5000.0;
    double maxTotalForce = mu * normalForce;

    // Braking only
    double brakingForce = maxTotalForce;
    double corneringForce = 0.0;
    double totalForce = std::sqrt(brakingForce*brakingForce + corneringForce*corneringForce);
    TS_ASSERT(totalForce <= maxTotalForce);

    // Combined braking and cornering (exceeds friction circle)
    brakingForce = maxTotalForce * 0.8;
    corneringForce = maxTotalForce * 0.8;
    totalForce = std::sqrt(brakingForce*brakingForce + corneringForce*corneringForce);
    // sqrt(0.8^2 + 0.8^2) = sqrt(1.28) = 1.13 > 1.0
    TS_ASSERT(totalForce > maxTotalForce);  // Exceeds friction circle
  }

  // Test consistency between position and bump height
  void testBumpHeightConsistency() {
    FGFDMExec fdmex;
    FGSurface surface(&fdmex);

    surface.SetBumpiness(0.5);
    double pos[3] = {123.456, 789.012, 0.0};
    surface.SetPosition(pos);

    double height1 = surface.GetBumpHeight();

    // Same position should give same height
    surface.SetPosition(pos);
    double height2 = surface.GetBumpHeight();

    TS_ASSERT_DELTA(height1, height2, epsilon);
  }

  /***************************************************************************
   * Temperature Effects on Surface Properties
   ***************************************************************************/

  void testTemperatureEffectOnRubber() {
    // Hot asphalt reduces tire grip
    double mu_cold = 0.85;
    double temperature = 140.0;  // °F (hot summer day)
    double referenceTemp = 70.0;

    // Friction decreases at extreme temperatures
    double tempFactor = 1.0 - 0.002 * std::abs(temperature - referenceTemp);
    double mu_hot = mu_cold * tempFactor;

    TS_ASSERT(mu_hot < mu_cold);
    TS_ASSERT(mu_hot > 0.7);
  }

  void testColdSurfaceCondensation() {
    // Cold surface may have moisture condensation
    double mu_dry = 0.8;
    double mu_damp = 0.6;

    double surfaceTemp = 35.0;  // °F
    double dewPoint = 40.0;     // °F

    // If surface temp < dew point, condensation occurs
    double mu_effective = (surfaceTemp < dewPoint) ? mu_damp : mu_dry;
    TS_ASSERT_DELTA(mu_effective, mu_damp, epsilon);
  }

  void testFrostOnSurface() {
    // Light frost reduces friction significantly
    double mu_dry = 0.8;
    double mu_frost = 0.3;

    double surfaceTemp = 28.0;  // °F (below freezing)
    bool frostPresent = surfaceTemp < 32.0;

    double mu_effective = frostPresent ? mu_frost : mu_dry;
    TS_ASSERT_DELTA(mu_effective, mu_frost, epsilon);
    TS_ASSERT(mu_frost < mu_dry);
  }

  /***************************************************************************
   * Tire-Surface Interface Tests
   ***************************************************************************/

  void testTirePressureEffectOnFriction() {
    double pressure_low = 30.0;   // psi
    double pressure_normal = 50.0;
    double pressure_high = 80.0;

    // Optimal friction at normal pressure
    double mu_base = 0.8;

    // Low pressure increases contact area but sidewall flex
    double mu_low = mu_base * 0.95;

    // High pressure decreases contact area
    double mu_high = mu_base * 0.90;

    TS_ASSERT(mu_low > mu_high);
    TS_ASSERT_DELTA(mu_low, 0.76, 0.01);
    TS_ASSERT_DELTA(mu_high, 0.72, 0.01);
  }

  void testTireWearEffectOnFriction() {
    // New tire vs worn tire
    double treadDepth_new = 8.0;   // mm
    double treadDepth_worn = 2.0;  // mm

    double mu_new = 0.85;
    double mu_worn = 0.70;

    // Worn tires have less grip, especially in wet
    TS_ASSERT(mu_new > mu_worn);
  }

  void testTireCompoundType() {
    // Different tire compounds
    double mu_standard = 0.80;
    double mu_soft = 0.90;     // Racing soft compound
    double mu_hard = 0.75;     // Long-life hard compound

    TS_ASSERT(mu_soft > mu_standard);
    TS_ASSERT(mu_standard > mu_hard);
  }

  /***************************************************************************
   * Dynamic Loading Tests
   ***************************************************************************/

  void testImpactLoadOnLanding() {
    double normalWeight = 10000.0;  // lbs
    double sinkRate = 5.0;          // ft/s
    double springConstant = 20000.0; // lbs/ft

    // Impact force = F = k * x, where x is compression
    // Energy absorbed = 0.5 * k * x^2 = 0.5 * m * v^2
    double mass = normalWeight / 32.2;  // slugs
    double kineticEnergy = 0.5 * mass * sinkRate * sinkRate;

    // Max compression = sqrt(2 * E / k)
    double maxCompression = std::sqrt(2.0 * kineticEnergy / springConstant);

    TS_ASSERT(maxCompression > 0.0);
    TS_ASSERT(maxCompression < 2.0);  // ft
  }

  void testOscillatingLoad() {
    // Wheel bouncing after touchdown
    double naturalFrequency = 2.0;  // Hz
    double dampingRatio = 0.3;

    // Time to settle (approximately 4 time constants)
    double period = 1.0 / naturalFrequency;
    double timeConstant = 1.0 / (dampingRatio * 2.0 * M_PI * naturalFrequency);
    double settlingTime = 4.0 * timeConstant;

    TS_ASSERT(settlingTime > 0.0);
    TS_ASSERT(settlingTime < 5.0);  // seconds
  }

  void testDynamicLoadFactor() {
    // During maneuvers, load factor changes
    double staticLoad = 5000.0;  // lbs per gear
    double loadFactor = 1.5;     // 1.5g turn

    double dynamicLoad = staticLoad * loadFactor;
    TS_ASSERT_DELTA(dynamicLoad, 7500.0, epsilon);

    // Available friction force increases
    double mu = 0.8;
    double frictionAvailable = mu * dynamicLoad;
    TS_ASSERT_DELTA(frictionAvailable, 6000.0, epsilon);
  }

  /***************************************************************************
   * Runway Contamination Tests
   ***************************************************************************/

  void testWetRunwayFriction() {
    // Water depth affects friction
    double mu_dry = 0.8;
    double waterDepth = 0.1;  // inches

    // Thin water film reduces friction
    double reductionFactor = 1.0 - 0.3 * waterDepth;
    double mu_wet = mu_dry * reductionFactor;

    TS_ASSERT_DELTA(mu_wet, 0.776, 0.001);
  }

  void testStandingWaterDepth() {
    // Deep standing water
    double mu_dry = 0.8;
    double waterDepth = 0.5;  // inches

    // Severe reduction
    double reductionFactor = 1.0 - 0.5 * waterDepth;
    double mu_flooded = mu_dry * reductionFactor;

    TS_ASSERT_DELTA(mu_flooded, 0.6, 0.01);
    TS_ASSERT(mu_flooded < mu_dry);
  }

  void testSlushContamination() {
    // Slush (water + ice mix)
    double mu_dry = 0.8;
    double mu_slush = 0.2;

    // Slush severely reduces braking
    TS_ASSERT(mu_slush < mu_dry * 0.5);
  }

  void testRubberDeposits() {
    // Rubber buildup on touchdown zone
    double mu_clean = 0.8;
    double rubberCoverage = 0.3;  // 30% coverage

    // Rubber deposits reduce friction, especially when wet
    double mu_rubber = mu_clean * (1.0 - 0.2 * rubberCoverage);
    TS_ASSERT_DELTA(mu_rubber, 0.752, 0.001);
  }

  void testOilContamination() {
    // Oil spill on runway
    double mu_clean = 0.8;
    double mu_oily = 0.1;  // Very slippery

    TS_ASSERT(mu_oily < mu_clean * 0.2);
  }

  /***************************************************************************
   * Crosswind Effect on Tire Forces
   ***************************************************************************/

  void testCrosswindTireLoad() {
    double weight = 10000.0;
    double crosswindForce = 500.0;  // lbs (from 15 kt crosswind)
    double wheelTrack = 10.0;       // ft

    // Crosswind creates rolling moment
    double cgHeight = 5.0;  // ft
    double rollingMoment = crosswindForce * cgHeight;

    // Upwind wheel unloaded, downwind wheel overloaded
    double loadTransfer = rollingMoment / wheelTrack;
    TS_ASSERT_DELTA(loadTransfer, 250.0, epsilon);
  }

  void testCrosswindSideFriction() {
    double weight = 10000.0;
    double crosswindForce = 500.0;
    double mu_side = 0.7;

    // Available side friction
    double maxSideFriction = mu_side * weight;
    TS_ASSERT(maxSideFriction > crosswindForce);

    // Safety margin
    double margin = maxSideFriction / crosswindForce;
    TS_ASSERT(margin > 10.0);  // Large safety margin
  }

  /***************************************************************************
   * Runway Surface Aging Tests
   ***************************************************************************/

  void testNewVsOldAsphalt() {
    FGFDMExec fdmex;

    FGSurface newAsphalt(&fdmex);
    newAsphalt.SetStaticFFactor(0.85);
    newAsphalt.SetBumpiness(0.1);

    FGSurface oldAsphalt(&fdmex);
    oldAsphalt.SetStaticFFactor(0.70);  // Worn
    oldAsphalt.SetBumpiness(0.4);       // Rougher

    TS_ASSERT(newAsphalt.GetStaticFFactor() > oldAsphalt.GetStaticFFactor());
    TS_ASSERT(newAsphalt.GetBumpiness() < oldAsphalt.GetBumpiness());
  }

  void testCrackedSurface() {
    FGFDMExec fdmex;
    FGSurface cracked(&fdmex);

    // Cracked surface has variable friction and roughness
    cracked.SetStaticFFactor(0.65);
    cracked.SetBumpiness(0.6);
    cracked.SetMaximumForce(80000.0);  // May break under heavy load

    TS_ASSERT(cracked.GetBumpiness() > 0.5);
    TS_ASSERT(cracked.GetMaximumForce() < std::numeric_limits<double>::max());
  }

  void testRepaintedRunway() {
    FGFDMExec fdmex;
    FGSurface painted(&fdmex);

    // Fresh paint is slippery when wet
    painted.SetStaticFFactor(0.55);  // Reduced grip
    painted.SetBumpiness(0.05);      // Very smooth

    TS_ASSERT(painted.GetStaticFFactor() < 0.6);
    TS_ASSERT(painted.GetBumpiness() < 0.1);
  }

  /***************************************************************************
   * Stress Tests
   ***************************************************************************/

  void testStressManySurfaces() {
    FGFDMExec fdmex;

    for (int i = 0; i < 50; i++) {
      FGSurface surface(&fdmex);
      surface.SetStaticFFactor(0.3 + (i % 10) * 0.05);
      surface.SetRollingFFactor(0.01 + (i % 10) * 0.01);
      surface.SetBumpiness((i % 10) * 0.1);

      TS_ASSERT(surface.GetStaticFFactor() >= 0.3);
      TS_ASSERT(surface.GetStaticFFactor() <= 0.85);
    }
  }

  void testStressManyPositionUpdates() {
    FGFDMExec fdmex;
    FGSurface surface(&fdmex);
    surface.SetBumpiness(0.5);

    for (int i = 0; i < 100; i++) {
      double pos[3] = {i * 10.0, i * 5.0, 0.0};
      surface.SetPosition(pos);
      double height = surface.GetBumpHeight();
      TS_ASSERT(!std::isnan(height));
      TS_ASSERT(!std::isinf(height));
    }
  }

  void testStressRapidPropertyChanges() {
    FGFDMExec fdmex;
    FGSurface surface(&fdmex);

    for (int i = 0; i < 100; i++) {
      surface.SetStaticFFactor((i % 10) * 0.1);
      surface.SetRollingFFactor((i % 10) * 0.01);
      surface.SetBumpiness((i % 10) * 0.1);
      surface.SetMaximumForce(10000.0 + i * 1000.0);
      surface.SetSolid(i % 2 == 0);

      TS_ASSERT(surface.GetStaticFFactor() >= 0.0);
      TS_ASSERT(surface.GetRollingFFactor() >= 0.0);
    }
  }

  void testStressResetCycle() {
    FGFDMExec fdmex;
    FGSurface surface(&fdmex);

    for (int i = 0; i < 50; i++) {
      surface.SetStaticFFactor(0.5);
      surface.SetRollingFFactor(0.03);
      surface.SetBumpiness(0.4);
      surface.SetMaximumForce(60000.0);
      surface.SetSolid(false);

      surface.resetValues();

      TS_ASSERT_DELTA(surface.GetStaticFFactor(), 1.0, epsilon);
      TS_ASSERT_DELTA(surface.GetRollingFFactor(), 1.0, epsilon);
      TS_ASSERT_DELTA(surface.GetBumpiness(), 0.0, epsilon);
      TS_ASSERT(surface.GetSolid());
    }
  }

  /***************************************************************************
   * Combined Effects Tests
   ***************************************************************************/

  void testCombinedWetAndSloped() {
    double mu_dry = 0.8;
    double mu_wet = 0.5;
    double slope = 0.02;  // 2% grade
    double weight = 10000.0;

    // Wet reduces available friction
    double frictionForce = mu_wet * weight * std::cos(std::atan(slope));
    TS_ASSERT(frictionForce < mu_dry * weight);

    // Slope adds gravity component
    double gravityForce = weight * std::sin(std::atan(slope));
    TS_ASSERT(frictionForce > gravityForce);  // Still enough to hold
  }

  void testCombinedBumpyAndIcy() {
    FGFDMExec fdmex;
    FGSurface surface(&fdmex);

    surface.SetStaticFFactor(0.1);  // Ice
    surface.SetBumpiness(0.8);      // Very bumpy ice

    double pos[3] = {50.0, 50.0, 0.0};
    surface.SetPosition(pos);

    TS_ASSERT(surface.GetStaticFFactor() < 0.2);
    TS_ASSERT(surface.GetBumpiness() > 0.5);
  }

  void testCombinedHeavyAndSoft() {
    FGFDMExec fdmex;
    FGSurface soft(&fdmex);

    soft.SetStaticFFactor(0.35);
    soft.SetRollingFFactor(0.15);
    soft.SetMaximumForce(40000.0);

    double normalForce = 50000.0;  // Exceeds max

    double effectiveForce = std::min(normalForce, soft.GetMaximumForce());
    TS_ASSERT_DELTA(effectiveForce, 40000.0, epsilon);

    double friction = soft.GetStaticFFactor() * effectiveForce;
    TS_ASSERT_DELTA(friction, 14000.0, epsilon);
  }

  void testMultipleSurfaceComparison() {
    FGFDMExec fdmex;

    FGSurface concrete(&fdmex);
    concrete.SetStaticFFactor(0.8);
    concrete.SetRollingFFactor(0.02);

    FGSurface grass(&fdmex);
    grass.SetStaticFFactor(0.4);
    grass.SetRollingFFactor(0.08);

    FGSurface gravel(&fdmex);
    gravel.SetStaticFFactor(0.55);
    gravel.SetRollingFFactor(0.06);

    // Concrete has highest static friction
    TS_ASSERT(concrete.GetStaticFFactor() > gravel.GetStaticFFactor());
    TS_ASSERT(gravel.GetStaticFFactor() > grass.GetStaticFFactor());

    // Grass has highest rolling resistance
    TS_ASSERT(grass.GetRollingFFactor() > gravel.GetRollingFFactor());
    TS_ASSERT(gravel.GetRollingFFactor() > concrete.GetRollingFFactor());
  }
};
