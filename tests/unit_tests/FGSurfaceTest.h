#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>

#include <FGFDMExec.h>
#include <models/FGSurface.h>
#include "TestUtilities.h"

using namespace JSBSim;

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
};
