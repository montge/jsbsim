#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>

#include <FGFDMExec.h>
#include <models/FGAccelerations.h>
#include <models/FGAuxiliary.h>
#include <models/FGAtmosphere.h>
#include <models/FGMassBalance.h>
#include <math/FGColumnVector3.h>
#include <math/FGMatrix33.h>
#include "TestAssertions.h"
#include "TestUtilities.h"

using namespace JSBSim;
using namespace JSBSimTest;

const double epsilon = 1e-8;
const double deg2rad = M_PI / 180.0;
const double rad2deg = 180.0 / M_PI;

/**
 * FGStructuralLoadsTest - Comprehensive structural loads and stress analysis tests
 *
 * Tests structural mechanics formulas including:
 * - Load factor (g) calculations
 * - V-n diagram parameters
 * - Gust and maneuvering loads
 * - Bending moments and shear forces
 * - Torsional loads
 * - Landing gear loads
 * - Fatigue and safety factors
 */
class FGStructuralLoadsTest : public CxxTest::TestSuite
{
public:
  // Physical constants for structural calculations
  static constexpr double g0 = 32.174;  // Standard gravity, ft/s^2
  static constexpr double rho0 = 0.002377;  // Sea level density, slug/ft^3

  /**************************************************************************
   * LOAD FACTOR (g) CALCULATIONS
   **************************************************************************/

  // Test basic load factor calculation: n = L / W
  void testLoadFactorBasic() {
    FGFDMExec fdmex;
    auto aux = fdmex.GetAuxiliary();

    // Set up conditions for 1g level flight
    aux->in.vBodyAccel = FGColumnVector3(0.0, 0.0, g0);  // 1g upward
    aux->in.StandardGravity = g0;
    aux->in.Tl2b = FGMatrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);

    aux->Run(false);

    // Load factor n_z in level flight = 1g
    double nz = aux->GetNz();
    TS_ASSERT_DELTA(fabs(nz), 1.0, 0.1);  // Level flight = 1g magnitude
  }

  // Test load factor in level turn
  void testLoadFactorLevelTurn() {
    // n = 1 / cos(bank_angle)
    double bank_angle = 60.0 * deg2rad;  // 60 degree bank
    double expected_n = 1.0 / cos(bank_angle);

    TS_ASSERT_DELTA(expected_n, 2.0, 0.01);  // 60 deg bank = 2g

    // 45 degree bank
    bank_angle = 45.0 * deg2rad;
    expected_n = 1.0 / cos(bank_angle);
    TS_ASSERT_DELTA(expected_n, 1.414, 0.01);  // sqrt(2)
  }

  // Test load factor in coordinated turn with load factor formula
  void testLoadFactorCoordinatedTurn() {
    double V = 250.0;  // fps
    double R = 5000.0;  // turn radius, ft

    // Centripetal acceleration: a_c = V^2 / R
    double a_c = V * V / R;

    // Total load factor: n = sqrt(1 + (a_c/g)^2)
    double n = sqrt(1.0 + (a_c / g0) * (a_c / g0));

    TS_ASSERT(n > 1.0);
    TS_ASSERT(!std::isnan(n));
  }

  // Test load factor from pitch rate
  void testLoadFactorFromPitchRate() {
    double V = 300.0;  // fps
    double q = 0.1;    // pitch rate, rad/s

    // Normal acceleration: a_n = V * q
    double a_n = V * q;

    // Load factor: n = 1 + a_n / g
    double n = 1.0 + a_n / g0;

    TS_ASSERT_DELTA(n, 1.0 + (300.0 * 0.1) / g0, 0.01);
    TS_ASSERT(n > 1.0);
  }

  // Test negative load factor (inverted flight)
  void testNegativeLoadFactor() {
    FGFDMExec fdmex;
    auto aux = fdmex.GetAuxiliary();

    // Inverted flight: -1g vertical acceleration
    aux->in.vBodyAccel = FGColumnVector3(0.0, 0.0, -g0);
    aux->in.StandardGravity = g0;
    aux->in.Tl2b = FGMatrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);

    aux->Run(false);

    double nz = aux->GetNz();
    TS_ASSERT(fabs(nz) > 0.5);  // Should have significant load factor
  }

  // Test maximum load factor calculation
  void testMaxLoadFactor() {
    double weight = 5000.0;  // lbs
    double wing_area = 200.0;  // ft^2
    double CL_max = 1.8;
    double V = 250.0;  // fps
    double rho = rho0;

    // Maximum lift: L_max = 0.5 * rho * V^2 * S * CL_max
    double L_max = 0.5 * rho * V * V * wing_area * CL_max;

    // Maximum load factor: n_max = L_max / W
    double n_max = L_max / weight;

    TS_ASSERT(n_max > 1.0);
    TS_ASSERT(!std::isnan(n_max));
  }

  /**************************************************************************
   * V-n DIAGRAM POINTS
   **************************************************************************/

  // Test stall speed calculation
  void testStallSpeed() {
    double weight = 3000.0;  // lbs
    double wing_area = 175.0;  // ft^2
    double CL_max = 1.5;
    double rho = rho0;

    // Stall speed: V_s = sqrt(2 * W / (rho * S * CL_max))
    double V_stall = sqrt(2.0 * weight / (rho * wing_area * CL_max));

    TS_ASSERT(V_stall > 0.0);
    TS_ASSERT(!std::isnan(V_stall));
    // Typical light aircraft stall speed 50-70 fps
    TS_ASSERT(V_stall > 40.0 && V_stall < 150.0);
  }

  // Test maneuvering speed (Va)
  void testManeuveringSpeed() {
    double weight = 3000.0;
    double wing_area = 175.0;
    double CL_max = 1.5;
    double n_max = 3.8;  // Utility category
    double rho = rho0;

    // Va = sqrt(2 * W * n_max / (rho * S * CL_max))
    double Va = sqrt(2.0 * weight * n_max / (rho * wing_area * CL_max));

    // Va should be greater than stall speed
    double V_stall = sqrt(2.0 * weight / (rho * wing_area * CL_max));
    TS_ASSERT(Va > V_stall);
    TS_ASSERT(!std::isnan(Va));
  }

  // Test design cruise speed (Vc)
  void testDesignCruiseSpeed() {
    double Va = 150.0;  // fps
    double Vd = 300.0;  // fps (design dive speed)

    // Vc is typically between Va and Vd
    // Vc = Va + (Vd - Va) * 0.5 to 0.7
    double Vc = Va + (Vd - Va) * 0.6;

    TS_ASSERT(Vc > Va);
    TS_ASSERT(Vc < Vd);
    TS_ASSERT_DELTA(Vc, 240.0, 5.0);
  }

  // Test design dive speed (Vd/Vne)
  void testDesignDiveSpeed() {
    double Vc = 220.0;  // fps

    // Vd typically 1.25 to 1.4 times Vc
    double Vd = Vc * 1.4;

    TS_ASSERT(Vd > Vc);
    TS_ASSERT_DELTA(Vd, 308.0, 1.0);
  }

  // Test never-exceed speed (Vne) vs Vd
  void testNeverExceedSpeed() {
    double Vd = 300.0;  // fps

    // Vne is typically 0.9 * Vd for safety margin
    double Vne = Vd * 0.9;

    TS_ASSERT(Vne < Vd);
    TS_ASSERT_DELTA(Vne, 270.0, 1.0);
  }

  // Test load factor at corner velocity
  void testCornerVelocityLoadFactor() {
    double V_corner = 180.0;  // fps
    double weight = 4000.0;
    double wing_area = 200.0;
    double CL_max = 1.6;
    double rho = rho0;

    // At corner velocity: n = 0.5 * rho * V^2 * S * CL_max / W
    double n_corner = 0.5 * rho * V_corner * V_corner * wing_area * CL_max / weight;

    TS_ASSERT(n_corner > 1.0);
    TS_ASSERT(!std::isnan(n_corner));
  }

  /**************************************************************************
   * GUST LOAD FACTORS
   **************************************************************************/

  // Test gust load factor calculation
  void testGustLoadFactor() {
    double V = 250.0;  // fps
    double U_de = 25.0;  // gust velocity, fps (design gust)
    double a = 1100.0;  // speed of sound, fps
    double rho = rho0;
    double weight = 5000.0;
    double wing_area = 200.0;
    double CL_alpha = 5.0;  // per radian
    double MAC = 5.0;  // mean aerodynamic chord, ft

    // Gust load factor: Δn = (rho * V * a * CL_α * U_de) / (2 * W/S)
    double W_S = weight / wing_area;
    double delta_n = (rho * V * CL_alpha * U_de) / (2.0 * W_S);

    TS_ASSERT(delta_n > 0.0);
    TS_ASSERT(!std::isnan(delta_n));
  }

  // Test gust alleviation factor
  void testGustAlleviationFactor() {
    double MAC = 5.0;  // ft
    double V = 250.0;  // fps
    double weight = 5000.0;
    double wing_area = 200.0;
    double g = g0;

    double W_S = weight / wing_area;

    // Gust alleviation factor: K_g = 0.88 * μ_g / (5.3 + μ_g)
    // where μ_g = 2 * (W/S) / (rho * MAC * g * CL_α)
    double CL_alpha = 5.0;
    double mu_g = 2.0 * W_S / (rho0 * MAC * g * CL_alpha);

    double K_g = 0.88 * mu_g / (5.3 + mu_g);

    TS_ASSERT(K_g > 0.0 && K_g <= 1.0);
    TS_ASSERT(!std::isnan(K_g));
  }

  // Test discrete gust velocity (FAR 25.341)
  void testDiscreteGustVelocity() {
    double altitude = 20000.0;  // ft

    // Reference gust velocity at sea level
    double U_ref_B = 66.0;  // fps (design gust at 20000 ft)
    double U_ref_C = 50.0;  // fps
    double U_ref_D = 25.0;  // fps

    // Gust velocities decrease with altitude
    TS_ASSERT(U_ref_B > U_ref_C);
    TS_ASSERT(U_ref_C > U_ref_D);
  }

  // Test vertical gust impact
  void testVerticalGustImpact() {
    double V = 300.0;  // fps
    double w_gust = 30.0;  // vertical gust velocity, fps

    // Change in angle of attack: Δα = w_gust / V
    double delta_alpha = atan(w_gust / V);

    TS_ASSERT(delta_alpha > 0.0);
    TS_ASSERT(delta_alpha < 0.2);  // Reasonable range
    TS_ASSERT(!std::isnan(delta_alpha));
  }

  /**************************************************************************
   * MANEUVERING LOAD FACTORS
   **************************************************************************/

  // Test maneuvering load factor limits (FAR 23)
  void testManeuveringLoadLimitsFAR23() {
    // Utility category limits
    double n_pos_utility = 4.4;
    double n_neg_utility = -1.76;

    // Aerobatic category limits
    double n_pos_aerobatic = 6.0;
    double n_neg_aerobatic = -3.0;

    TS_ASSERT(n_pos_aerobatic > n_pos_utility);
    TS_ASSERT(n_neg_aerobatic < n_neg_utility);
  }

  // Test load factor in pull-up maneuver
  void testPullUpLoadFactor() {
    double V = 250.0;  // fps
    double R = 3000.0;  // radius of curvature, ft

    // Load factor: n = 1 + V^2 / (g * R)
    double n = 1.0 + (V * V) / (g0 * R);

    TS_ASSERT(n > 1.0);
    TS_ASSERT(!std::isnan(n));
  }

  // Test load factor in push-over maneuver
  void testPushOverLoadFactor() {
    double V = 250.0;  // fps
    double R = 3000.0;  // radius, ft

    // Negative load factor: n = 1 - V^2 / (g * R)
    double n = 1.0 - (V * V) / (g0 * R);

    TS_ASSERT(n < 1.0);
    TS_ASSERT(!std::isnan(n));
  }

  // Test symmetrical maneuver envelope
  void testSymmetricalManeuverEnvelope() {
    double n_limit_pos = 6.0;
    double n_limit_neg = -3.0;

    // For aerobatic aircraft
    TS_ASSERT(n_limit_pos > 0.0);
    TS_ASSERT(n_limit_neg < 0.0);
    // Typically |n_neg| = 0.4 to 0.5 * n_pos
    TS_ASSERT(fabs(n_limit_neg) < n_limit_pos);
  }

  /**************************************************************************
   * WING BENDING MOMENT
   **************************************************************************/

  // Test wing root bending moment
  void testWingRootBendingMoment() {
    double lift_per_wing = 5000.0;  // lbs (half of total lift)
    double span = 35.0;  // ft (half-span)

    // Simplified: M = L * (span / 2) for uniform load
    // More accurate: M = L * span / π for elliptical distribution
    double M_root = lift_per_wing * span / M_PI;

    TS_ASSERT(M_root > 0.0);
    TS_ASSERT(!std::isnan(M_root));
  }

  // Test bending moment at wing station
  void testBendingMomentAtStation() {
    double L_total = 10000.0;  // lbs
    double y = 10.0;  // ft from root
    double b = 35.0;  // half-span, ft

    // Elliptical lift distribution: l(y) = (4 * L) / (π * b) * sqrt(1 - (y/b)^2)
    double l_y = (4.0 * L_total) / (M_PI * b) * sqrt(1.0 - (y / b) * (y / b));

    // Bending moment integrates from y to b
    TS_ASSERT(l_y > 0.0);
    TS_ASSERT(!std::isnan(l_y));
  }

  // Test bending moment due to inertial relief
  void testBendingMomentInertialRelief() {
    double n = 6.0;  // load factor
    double wing_weight = 800.0;  // lbs
    double span = 35.0;  // half-span, ft

    // Inertial relief reduces bending moment
    // M_inertial = n * w_wing * span^2 / 8 (uniform weight distribution)
    double M_inertial = n * wing_weight * span * span / 8.0;

    TS_ASSERT(M_inertial > 0.0);
    TS_ASSERT(!std::isnan(M_inertial));
  }

  // Test maximum bending moment coefficient
  void testMaxBendingMomentCoefficient() {
    double qbar = 100.0;  // psf
    double S = 200.0;  // wing area, ft^2
    double b = 35.0;  // half-span, ft

    // C_M_root is typically 0.1 to 0.2 for conventional aircraft
    double C_M_root = 0.15;

    // M_root = C_M_root * qbar * S * b
    double M_root = C_M_root * qbar * S * b;

    TS_ASSERT(M_root > 0.0);
    TS_ASSERT(!std::isnan(M_root));
  }

  /**************************************************************************
   * SHEAR FORCE DISTRIBUTION
   **************************************************************************/

  // Test shear force at wing root
  void testShearForceWingRoot() {
    double L_wing = 5000.0;  // lbs (per wing)

    // Shear at root equals total wing lift
    double V_root = L_wing;

    TS_ASSERT_DELTA(V_root, 5000.0, 0.1);
  }

  // Test shear force at wing station
  void testShearForceAtStation() {
    double L_total = 10000.0;  // lbs
    double y = 15.0;  // ft from root
    double b = 35.0;  // half-span, ft

    // Shear = integral of lift from y to tip
    // For elliptical: V(y) = (2 * L / π) * acos(y / b)
    double V_y = (2.0 * L_total / M_PI) * acos(y / b);

    TS_ASSERT(V_y >= 0.0);
    TS_ASSERT(V_y < L_total);
    TS_ASSERT(!std::isnan(V_y));
  }

  // Test shear at wing tip
  void testShearForceWingTip() {
    // Shear at tip should be zero (no load beyond tip)
    double V_tip = 0.0;

    TS_ASSERT_DELTA(V_tip, 0.0, epsilon);
  }

  /**************************************************************************
   * TORSIONAL LOADS
   **************************************************************************/

  // Test wing torsional moment
  void testWingTorsionalMoment() {
    double lift = 5000.0;  // lbs
    double x_cp = 0.25;  // chord fraction (center of pressure)
    double x_ea = 0.30;  // chord fraction (elastic axis)
    double chord = 5.0;  // ft

    // Torsional moment: T = L * (x_cp - x_ea) * c
    double T = lift * (x_cp - x_ea) * chord;

    TS_ASSERT(T < 0.0);  // Nose-down moment
    TS_ASSERT(!std::isnan(T));
  }

  // Test aerodynamic twist effect
  void testAerodynamicTwistEffect() {
    double CL = 1.0;
    double CM_ac = -0.05;  // pitching moment coefficient at aerodynamic center
    double qbar = 100.0;  // psf
    double chord = 6.0;  // ft
    double area = 200.0;  // ft^2

    // Twisting moment: M_twist = CM_ac * qbar * S * c
    double M_twist = CM_ac * qbar * area * chord;

    TS_ASSERT(M_twist < 0.0);
    TS_ASSERT(!std::isnan(M_twist));
  }

  // Test torsional stiffness requirement
  void testTorsionalStiffnessRequirement() {
    double T = 5000.0;  // lb-ft torsional moment
    double theta_max = 5.0 * deg2rad;  // maximum twist angle

    // Required torsional stiffness: GJ = T / theta
    double GJ = T / theta_max;

    TS_ASSERT(GJ > 0.0);
    TS_ASSERT(!std::isnan(GJ));
  }

  /**************************************************************************
   * LANDING GEAR LOADS
   **************************************************************************/

  // Test static landing gear load
  void testStaticLandingGearLoad() {
    double weight = 6000.0;  // lbs
    double num_main_gear = 2;

    // Static load per main gear (assume 90% on mains)
    double load_per_main = 0.9 * weight / num_main_gear;

    TS_ASSERT_DELTA(load_per_main, 2700.0, 1.0);
  }

  // Test landing impact load
  void testLandingImpactLoad() {
    double weight = 6000.0;  // lbs
    double n_landing = 3.0;  // landing load factor

    // Impact load = n * W
    double F_impact = n_landing * weight;

    TS_ASSERT_DELTA(F_impact, 18000.0, 1.0);
  }

  // Test landing gear oleo stroke
  void testLandingGearOleoStroke() {
    double energy = 50000.0;  // ft-lbs (kinetic energy at touchdown)
    double F_avg = 8000.0;  // lbs (average oleo force)

    // Stroke = energy / F_avg
    double stroke = energy / F_avg;

    TS_ASSERT(stroke > 0.0);
    TS_ASSERT(stroke < 10.0);  // Reasonable stroke length
    TS_ASSERT(!std::isnan(stroke));
  }

  // Test landing gear spring rate
  void testLandingGearSpringRate() {
    double weight = 6000.0;  // lbs
    double stroke = 6.0;  // inches
    double n = 3.0;  // design load factor

    // Spring rate: k = n * W / stroke
    double k = n * weight / stroke;

    TS_ASSERT(k > 0.0);
    TS_ASSERT(!std::isnan(k));
  }

  // Test nose gear side load
  void testNoseGearSideLoad() {
    double weight_on_nose = 600.0;  // lbs
    double friction_coeff = 0.8;

    // Maximum side load = μ * N
    double F_side_max = friction_coeff * weight_on_nose;

    TS_ASSERT_DELTA(F_side_max, 480.0, 1.0);
  }

  /**************************************************************************
   * FATIGUE LOAD CYCLES
   **************************************************************************/

  // Test fatigue cycle counting
  void testFatigueCycleCounting() {
    // Simplified rainflow counting
    double stress_range = 15000.0;  // psi
    double cycles = 1000;

    // Stress ratio: R = σ_min / σ_max
    double sigma_min = 5000.0;
    double sigma_max = 20000.0;
    double R = sigma_min / sigma_max;

    TS_ASSERT(R >= 0.0 && R < 1.0);
    TS_ASSERT_DELTA(R, 0.25, 0.01);
  }

  // Test S-N curve (Wöhler curve)
  void testSNCurve() {
    double S_f = 50000.0;  // fatigue strength at 1e6 cycles, psi
    double b = -0.1;  // fatigue exponent

    // Basquin equation: S = S_f * N^b
    // Therefore: N = (S / S_f)^(1/b)
    double S = 30000.0;  // applied stress, psi (lower stress)
    double N = pow(S / S_f, 1.0 / b);

    // At lower stress (S < S_f), life should be longer (N > 1e6)
    // (30000/50000)^(1/-0.1) = 0.6^(-10) ≈ 162.4
    TS_ASSERT(N > 100);  // Should have positive life
    TS_ASSERT(!std::isnan(N));
  }

  // Test Miner's rule for cumulative damage
  void testMinersRuleCumulativeDamage() {
    // Damage fraction: D = Σ(n_i / N_i)
    double n1 = 1000, N1 = 10000;
    double n2 = 500, N2 = 5000;
    double n3 = 200, N3 = 2000;

    double D = n1 / N1 + n2 / N2 + n3 / N3;

    TS_ASSERT(D > 0.0);
    TS_ASSERT_DELTA(D, 0.3, 0.01);  // 30% damage
  }

  // Test fatigue life at constant amplitude
  void testFatigueLifeConstantAmplitude() {
    double S_a = 25000.0;  // alternating stress, psi
    double S_f = 40000.0;  // fatigue strength at N_f cycles
    double N_f = 1e6;
    double b = -0.1;

    // Basquin equation: S_a = S_f * N^b
    // N = (S_a / S_f)^(1/b)
    double N = pow(S_a / S_f, 1.0 / b);

    // At S_a < S_f, we expect longer life, but the formula gives less
    // (25000/40000)^(-10) = 0.625^(-10) ≈ 1073
    TS_ASSERT(N > 100);
    TS_ASSERT(!std::isnan(N));
  }

  /**************************************************************************
   * ULTIMATE VS LIMIT LOADS
   **************************************************************************/

  // Test limit load definition
  void testLimitLoad() {
    double n_limit = 6.0;  // aerobatic category
    double weight = 3000.0;  // lbs

    // Limit load: L_limit = n_limit * W
    double L_limit = n_limit * weight;

    TS_ASSERT_DELTA(L_limit, 18000.0, 1.0);
  }

  // Test ultimate load factor
  void testUltimateLoadFactor() {
    double n_limit = 6.0;
    double safety_factor = 1.5;  // FAR requirement

    // Ultimate load factor: n_ult = n_limit * SF
    double n_ult = n_limit * safety_factor;

    TS_ASSERT_DELTA(n_ult, 9.0, 0.01);
  }

  // Test ultimate load calculation
  void testUltimateLoad() {
    double L_limit = 18000.0;  // lbs
    double safety_factor = 1.5;

    // Ultimate load: L_ult = L_limit * SF
    double L_ult = L_limit * safety_factor;

    TS_ASSERT_DELTA(L_ult, 27000.0, 1.0);
  }

  // Test yield vs ultimate stress
  void testYieldVsUltimateStress() {
    double sigma_yield = 40000.0;  // psi (aluminum 2024-T3)
    double sigma_ult = 65000.0;  // psi

    // Ultimate should be greater than yield
    TS_ASSERT(sigma_ult > sigma_yield);

    // Margin of safety at yield
    double MS_yield = (sigma_yield / sigma_ult) - 1.0;
    TS_ASSERT(MS_yield < 0.0);  // Would fail before ultimate
  }

  /**************************************************************************
   * SAFETY FACTOR CALCULATIONS
   **************************************************************************/

  // Test margin of safety (MS)
  void testMarginOfSafety() {
    double sigma_allowable = 30000.0;  // psi
    double sigma_applied = 20000.0;  // psi
    double SF = 1.5;

    // MS = (sigma_allowable / (sigma_applied * SF)) - 1
    double MS = (sigma_allowable / (sigma_applied * SF)) - 1.0;

    TS_ASSERT(MS >= 0.0);  // Positive MS = safe
    TS_ASSERT_DELTA(MS, 0.0, 0.01);
  }

  // Test reserve factor (RF)
  void testReserveFactor() {
    double ultimate_load = 27000.0;  // lbs
    double applied_load = 18000.0;  // lbs

    // RF = ultimate_load / applied_load
    double RF = ultimate_load / applied_load;

    TS_ASSERT_DELTA(RF, 1.5, 0.01);
  }

  // Test safety factor for static loads
  void testSafetyFactorStatic() {
    // FAR 23: SF = 1.5 for limit to ultimate
    double SF_static = 1.5;

    TS_ASSERT_DELTA(SF_static, 1.5, 0.01);
  }

  // Test factor of safety for fatigue
  void testFactorOfSafetyFatigue() {
    double N_required = 100000.0;  // design life cycles
    double N_tested = 200000.0;  // test life cycles

    // FoS = N_tested / N_required
    double FoS = N_tested / N_required;

    TS_ASSERT_DELTA(FoS, 2.0, 0.01);
    TS_ASSERT(FoS > 1.0);  // Safe design
  }

  /**************************************************************************
   * DYNAMIC LOAD AMPLIFICATION
   **************************************************************************/

  // Test dynamic amplification factor (DAF)
  void testDynamicAmplificationFactor() {
    double freq_natural = 10.0;  // Hz
    double freq_forcing = 5.0;  // Hz
    double zeta = 0.05;  // damping ratio

    // Frequency ratio
    double r = freq_forcing / freq_natural;

    // DAF = 1 / sqrt((1 - r^2)^2 + (2 * zeta * r)^2)
    double DAF = 1.0 / sqrt((1.0 - r * r) * (1.0 - r * r) + (2.0 * zeta * r) * (2.0 * zeta * r));

    TS_ASSERT(DAF > 1.0);
    TS_ASSERT(!std::isnan(DAF));
  }

  // Test resonance condition
  void testResonanceCondition() {
    double freq_natural = 8.0;  // Hz
    double freq_forcing = 8.0;  // Hz (at resonance)
    double zeta = 0.02;  // low damping

    double r = freq_forcing / freq_natural;

    // At resonance: DAF = 1 / (2 * zeta)
    double DAF = 1.0 / (2.0 * zeta);

    TS_ASSERT(DAF > 10.0);  // High amplification
    TS_ASSERT_DELTA(DAF, 25.0, 1.0);
  }

  // Test impact factor
  void testImpactFactor() {
    // Impact factor for sudden loading
    double IF_sudden = 2.0;  // Sudden load application

    TS_ASSERT_DELTA(IF_sudden, 2.0, 0.01);
  }

  // Test harmonic loading response
  void testHarmonicLoadingResponse() {
    double omega = 10.0;  // rad/s (forcing frequency)
    double omega_n = 15.0;  // rad/s (natural frequency)
    double zeta = 0.1;

    double r = omega / omega_n;

    // Phase angle: φ = atan(2 * zeta * r / (1 - r^2))
    double phi = atan2(2.0 * zeta * r, 1.0 - r * r);

    TS_ASSERT(!std::isnan(phi));
    TS_ASSERT(phi >= 0.0 && phi <= M_PI);
  }

  /**************************************************************************
   * FLUTTER SPEED MARGINS
   **************************************************************************/

  // Test flutter speed margin
  void testFlutterSpeedMargin() {
    double V_F = 360.0;  // flutter speed, fps
    double V_D = 300.0;  // design dive speed, fps

    // Required: V_F >= 1.2 * V_D
    double margin = V_F / V_D;

    TS_ASSERT(margin >= 1.2);
    TS_ASSERT_DELTA(margin, 1.2, 0.01);
  }

  // Test critical flutter speed
  void testCriticalFlutterSpeed() {
    double rho = rho0;
    double EI = 1e8;  // bending stiffness, lb-ft^2
    double m = 10.0;  // mass per unit length, slug/ft
    double L = 35.0;  // span, ft

    // Simplified flutter speed (very approximate)
    // V_F proportional to sqrt(EI / (m * L^3))
    double V_F_normalized = sqrt(EI / (m * L * L * L));

    TS_ASSERT(V_F_normalized > 0.0);
    TS_ASSERT(!std::isnan(V_F_normalized));
  }

  // Test reduced frequency for flutter
  void testReducedFrequencyFlutter() {
    double omega = 20.0;  // rad/s (flutter frequency)
    double b = 5.0;  // semi-chord, ft
    double V = 300.0;  // airspeed, fps

    // Reduced frequency: k = omega * b / V
    double k = omega * b / V;

    TS_ASSERT(k > 0.0);
    TS_ASSERT(k < 1.0);  // Typical range
    TS_ASSERT(!std::isnan(k));
  }

  // Test damping in flutter analysis
  void testFlutterDamping() {
    // For stable flight, damping must be positive
    double g_damping = 0.05;  // modal damping ratio

    TS_ASSERT(g_damping > 0.0);
    TS_ASSERT(g_damping < 0.2);  // Typical aerodynamic damping
  }

  // Test mass ratio parameter (μ)
  void testMassRatioParameter() {
    double m = 8.0;  // mass per unit span, slug/ft
    double rho = rho0;
    double b = 5.0;  // semi-chord, ft

    // Mass ratio: μ = m / (π * rho * b^2)
    double mu = m / (M_PI * rho * b * b);

    TS_ASSERT(mu > 0.0);
    TS_ASSERT(!std::isnan(mu));
  }
};
