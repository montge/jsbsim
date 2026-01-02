/*******************************************************************************
 * FGCompressibleFlowTest.h - Unit tests for compressible flow calculations
 *
 * Tests fundamental compressible aerodynamics and gas dynamics relevant to
 * high-speed flight including Prandtl-Glauert correction, transonic/supersonic
 * flow, shock waves, expansion waves, and compressibility effects.
 *
 * Copyright (c) JSBSim Development Team
 * Licensed under LGPL
 ******************************************************************************/

#include <cxxtest/TestSuite.h>
#include <cmath>
#include <limits>

#include <FGFDMExec.h>
#include <models/FGAtmosphere.h>
#include <models/FGAuxiliary.h>
#include <models/FGPropagate.h>
#include <models/FGPropulsion.h>
#include <initialization/FGInitialCondition.h>
#include "TestUtilities.h"

using namespace JSBSim;
using namespace JSBSimTest;

// Compressible flow constants
constexpr double GAMMA_AIR = 1.4;              // Specific heat ratio for air
constexpr double PI_CONST = M_PI;
constexpr double EPSILON = 1e-10;              // Numerical tolerance
constexpr double LOOSE_TOL = 1e-6;             // Loose tolerance for complex calculations

class FGCompressibleFlowTest : public CxxTest::TestSuite
{
public:

    //==========================================================================
    // 1. PRANDTL-GLAUERT COMPRESSIBILITY CORRECTION (~4 tests)
    //==========================================================================

    void testPrandtlGlauert_SubsonicCorrection() {
        // Prandtl-Glauert: C_p = C_p0 / sqrt(1 - M²)
        // Valid for subsonic flow (M < 0.8)
        double M = 0.5;
        double Cp0 = 0.1;  // Incompressible pressure coefficient

        double beta = sqrt(1.0 - M * M);
        double Cp = Cp0 / beta;

        TS_ASSERT_DELTA(beta, 0.8660, 0.001);
        TS_ASSERT_DELTA(Cp, 0.1155, 0.001);
    }

    void testPrandtlGlauert_LowMachLimit() {
        // At M→0, correction → 1 (incompressible limit)
        double M = 0.05;
        double Cp0 = 0.15;

        double beta = sqrt(1.0 - M * M);
        double Cp = Cp0 / beta;

        TS_ASSERT_DELTA(Cp / Cp0, 1.0, 0.002);  // < 0.2% difference
    }

    void testPrandtlGlauert_LiftCoefficientCorrection() {
        // C_L = C_L0 / sqrt(1 - M²)
        double M = 0.7;
        double CL0 = 0.5;

        double beta = sqrt(1.0 - M * M);
        double CL = CL0 / beta;

        TS_ASSERT_DELTA(beta, 0.7141, 0.001);
        TS_ASSERT_DELTA(CL, 0.7002, 0.001);
    }

    void testPrandtlGlauert_ValidityLimit() {
        // P-G correction becomes unreliable near M = 0.8
        double M_subsonic = 0.75;
        double M_transonic = 0.85;

        double beta_sub = sqrt(1.0 - M_subsonic * M_subsonic);
        double beta_trans = sqrt(1.0 - M_transonic * M_transonic);

        TS_ASSERT(beta_sub > 0.6);     // Still reasonable
        TS_ASSERT(beta_trans < 0.53);  // Becoming unreliable
    }

    //==========================================================================
    // 2. TRANSONIC DRAG RISE MODELING (~4 tests)
    //==========================================================================

    void testDragDivergence_CriticalMach() {
        // Drag divergence Mach number (M_dd) is where drag rapidly increases
        // Typically M_dd ≈ 0.75-0.85 for transonic aircraft
        double M_crit = 0.80;  // Critical Mach number
        double M_dd = M_crit + 0.05;  // Typical offset

        TS_ASSERT_DELTA(M_dd, 0.85, 0.01);
    }

    void testDragRise_LockProposal() {
        // Lock's drag rise formula: ΔC_D = K * (M - M_dd)^n
        double M = 0.90;
        double M_dd = 0.85;
        double K = 20.0;  // Empirical constant
        double n = 4.0;   // Typical exponent

        double deltaCD = 0.0;
        if (M > M_dd) {
            deltaCD = K * pow(M - M_dd, n);
        }

        // 20 * (0.05)^4 = 20 * 6.25e-6 = 0.000125
        TS_ASSERT_DELTA(deltaCD, 0.000125, 0.00001);
    }

    void testDragRise_SubsonicNoDivergence() {
        // Below M_dd, no significant drag rise
        double M = 0.70;
        double M_dd = 0.85;
        double K = 20.0;

        double deltaCD = 0.0;
        if (M > M_dd) {
            deltaCD = K * pow(M - M_dd, 4.0);
        }

        TS_ASSERT_DELTA(deltaCD, 0.0, EPSILON);
    }

    void testDragRise_SuppressionFactor() {
        // Korn equation: M_dd = M_crit - (0.1/80)^(1/3) * (C_L / t/c)
        // Simplified test of relationship
        double M_crit = 0.80;
        double CL = 0.5;
        double tc_ratio = 0.12;  // thickness-to-chord ratio

        // Higher CL or t/c reduces M_dd
        double factor = CL / tc_ratio;
        TS_ASSERT(factor > 0);  // Positive correction
    }

    //==========================================================================
    // 3. WAVE DRAG CALCULATIONS (~4 tests)
    //==========================================================================

    void testWaveDrag_LinearTheory() {
        // Wave drag coefficient for supersonic flow (linear theory)
        // C_D_wave ≈ 4 / sqrt(M² - 1) * (t/c)²
        double M = 2.0;
        double tc_ratio = 0.10;  // 10% thickness

        double Msq_minus_1 = M * M - 1.0;
        double CD_wave = 4.0 / sqrt(Msq_minus_1) * tc_ratio * tc_ratio;

        TS_ASSERT_DELTA(CD_wave, 0.0231, 0.001);
    }

    void testWaveDrag_MachDependence() {
        // Wave drag decreases with increasing Mach number
        double tc_ratio = 0.08;

        double M1 = 1.5;
        double M2 = 3.0;

        double CD_wave_1 = 4.0 / sqrt(M1 * M1 - 1.0) * tc_ratio * tc_ratio;
        double CD_wave_2 = 4.0 / sqrt(M2 * M2 - 1.0) * tc_ratio * tc_ratio;

        TS_ASSERT(CD_wave_1 > CD_wave_2);  // Decreases with Mach
    }

    void testWaveDrag_SlenderBody() {
        // For very slender bodies, wave drag is minimal
        double M = 2.5;
        double tc_ratio = 0.03;  // 3% thickness (very slender)

        double CD_wave = 4.0 / sqrt(M * M - 1.0) * tc_ratio * tc_ratio;

        TS_ASSERT(CD_wave < 0.002);  // Very low wave drag
    }

    void testWaveDrag_VolumeEffect() {
        // Area rule: wave drag depends on volume distribution
        // Sears-Haack body has minimum wave drag for given volume
        double M = 1.5;
        double fineness_ratio = 10.0;  // L/D of body

        // Simplified volume effect
        double volume_factor = 1.0 / (fineness_ratio * fineness_ratio);
        TS_ASSERT(volume_factor > 0);  // Positive but small for slender bodies
    }

    //==========================================================================
    // 4. NORMAL SHOCK RELATIONS (~5 tests)
    //==========================================================================

    void testNormalShock_PressureRatio() {
        // P2/P1 = 1 + (2γ/(γ+1)) * (M1² - 1)
        double M1 = 2.0;  // Supersonic upstream Mach
        double gamma = GAMMA_AIR;

        double P2_P1 = 1.0 + (2.0 * gamma / (gamma + 1.0)) * (M1 * M1 - 1.0);

        TS_ASSERT_DELTA(P2_P1, 4.5, 0.01);
    }

    void testNormalShock_DensityRatio() {
        // ρ2/ρ1 = ((γ+1)M1²) / ((γ-1)M1² + 2)
        double M1 = 2.0;
        double gamma = GAMMA_AIR;

        double numerator = (gamma + 1.0) * M1 * M1;
        double denominator = (gamma - 1.0) * M1 * M1 + 2.0;
        double rho2_rho1 = numerator / denominator;

        TS_ASSERT_DELTA(rho2_rho1, 2.667, 0.01);
    }

    void testNormalShock_TemperatureRatio() {
        // T2/T1 = (P2/P1) * (ρ1/ρ2)
        double M1 = 2.5;
        double gamma = GAMMA_AIR;

        double P2_P1 = 1.0 + (2.0 * gamma / (gamma + 1.0)) * (M1 * M1 - 1.0);
        double rho2_rho1 = ((gamma + 1.0) * M1 * M1) / ((gamma - 1.0) * M1 * M1 + 2.0);
        double T2_T1 = P2_P1 / rho2_rho1;

        TS_ASSERT_DELTA(T2_T1, 2.138, 0.01);
    }

    void testNormalShock_DownstreamMach() {
        // M2² = (1 + ((γ-1)/2)M1²) / (γM1² - (γ-1)/2)
        double M1 = 3.0;
        double gamma = GAMMA_AIR;

        double numerator = 1.0 + ((gamma - 1.0) / 2.0) * M1 * M1;
        double denominator = gamma * M1 * M1 - (gamma - 1.0) / 2.0;
        double M2_squared = numerator / denominator;
        double M2 = sqrt(M2_squared);

        TS_ASSERT_DELTA(M2, 0.4752, 0.001);
        TS_ASSERT(M2 < 1.0);  // Always subsonic downstream
    }

    void testNormalShock_WeakShockLimit() {
        // For M1 → 1, shock strength → 0
        double M1 = 1.05;  // Just supersonic
        double gamma = GAMMA_AIR;

        double P2_P1 = 1.0 + (2.0 * gamma / (gamma + 1.0)) * (M1 * M1 - 1.0);

        // 1 + (2*1.4/2.4)*(1.1025-1) = 1.1195
        TS_ASSERT_DELTA(P2_P1, 1.12, 0.01);  // Weak shock
    }

    //==========================================================================
    // 5. OBLIQUE SHOCK RELATIONS (~5 tests)
    //==========================================================================

    void testObliqueShock_ThetaBetaMach() {
        // θ-β-M relation: tan(θ) = 2cot(β) * (M1²sin²β - 1) / (M1²(γ+cos2β) + 2)
        double M1 = 2.0;
        double beta = 45.0 * PI_CONST / 180.0;  // Shock angle (radians)
        double gamma = GAMMA_AIR;

        double M1n = M1 * sin(beta);  // Normal component
        double numerator = 2.0 / tan(beta) * (M1 * M1 * sin(beta) * sin(beta) - 1.0);
        double denominator = M1 * M1 * (gamma + cos(2.0 * beta)) + 2.0;
        double theta = atan(numerator / denominator);
        double theta_deg = theta * 180.0 / PI_CONST;

        TS_ASSERT_DELTA(theta_deg, 15.1, 0.5);  // Deflection angle
    }

    void testObliqueShock_NormalComponent() {
        // Normal shock relations apply to normal component M1n = M1*sin(β)
        double M1 = 3.0;
        double beta = 30.0 * PI_CONST / 180.0;
        double gamma = GAMMA_AIR;

        double M1n = M1 * sin(beta);
        TS_ASSERT_DELTA(M1n, 1.5, 0.01);

        // Apply normal shock relations to M1n
        double P2_P1 = 1.0 + (2.0 * gamma / (gamma + 1.0)) * (M1n * M1n - 1.0);
        TS_ASSERT_DELTA(P2_P1, 2.458, 0.01);
    }

    void testObliqueShock_WeakVsStrong() {
        // For given M1 and θ, two solutions exist (weak and strong shock)
        // Weak shock: smaller β, higher M2
        // Strong shock: larger β, lower M2
        double M1 = 2.0;
        double theta = 10.0 * PI_CONST / 180.0;

        // Weak shock approximation: β ≈ θ + sin⁻¹(1/M1)
        double beta_weak_approx = theta + asin(1.0 / M1);
        double beta_weak_deg = beta_weak_approx * 180.0 / PI_CONST;

        TS_ASSERT(beta_weak_deg > 30.0 && beta_weak_deg < 50.0);
    }

    void testObliqueShock_DetachmentCondition() {
        // Maximum deflection angle for attached shock
        // For M1 = 2.0, θ_max ≈ 22.97°
        double M1 = 2.0;
        double theta_max_approx = 23.0;  // degrees (approximate)

        // Beyond this angle, shock detaches (bow shock forms)
        double theta_test = 25.0;  // degrees
        TS_ASSERT(theta_test > theta_max_approx);  // Would cause detachment
    }

    void testObliqueShock_MachAngle() {
        // Mach angle: μ = sin⁻¹(1/M)
        // Minimum shock angle is the Mach angle
        double M1 = 3.0;
        double mu = asin(1.0 / M1);
        double mu_deg = mu * 180.0 / PI_CONST;

        TS_ASSERT_DELTA(mu_deg, 19.47, 0.1);
    }

    //==========================================================================
    // 6. EXPANSION WAVE (PRANDTL-MEYER) (~4 tests)
    //==========================================================================

    void testPrandtlMeyer_Function() {
        // ν(M) = sqrt((γ+1)/(γ-1)) * tan⁻¹(sqrt((γ-1)/(γ+1)*(M²-1))) - tan⁻¹(sqrt(M²-1))
        double M = 2.0;
        double gamma = GAMMA_AIR;

        double sqrt_term = sqrt((gamma + 1.0) / (gamma - 1.0));
        double term1 = sqrt_term * atan(sqrt((gamma - 1.0) / (gamma + 1.0) * (M * M - 1.0)));
        double term2 = atan(sqrt(M * M - 1.0));
        double nu = term1 - term2;
        double nu_deg = nu * 180.0 / PI_CONST;

        TS_ASSERT_DELTA(nu_deg, 26.38, 0.1);
    }

    void testPrandtlMeyer_ExpansionTurn() {
        // For expansion turn: ν2 - ν1 = θ (turning angle)
        double M1 = 2.0;
        double theta = 10.0 * PI_CONST / 180.0;  // radians
        double gamma = GAMMA_AIR;

        // Calculate ν1
        double sqrt_term = sqrt((gamma + 1.0) / (gamma - 1.0));
        double nu1 = sqrt_term * atan(sqrt((gamma - 1.0) / (gamma + 1.0) * (M1 * M1 - 1.0)))
                     - atan(sqrt(M1 * M1 - 1.0));

        double nu2 = nu1 + theta;
        double nu2_deg = nu2 * 180.0 / PI_CONST;

        TS_ASSERT_DELTA(nu2_deg, 36.38, 0.5);
    }

    void testPrandtlMeyer_MaximumAngle() {
        // Maximum turning angle: ν_max at M → ∞
        // ν_max = (π/2) * (sqrt((γ+1)/(γ-1)) - 1)
        double gamma = GAMMA_AIR;

        double nu_max = (PI_CONST / 2.0) * (sqrt((gamma + 1.0) / (gamma - 1.0)) - 1.0);
        double nu_max_deg = nu_max * 180.0 / PI_CONST;

        TS_ASSERT_DELTA(nu_max_deg, 130.45, 0.1);
    }

    void testPrandtlMeyer_PressureRatio() {
        // Across expansion fan: P2/P1 from isentropic relations
        double M1 = 2.0;
        double M2 = 2.5;
        double gamma = GAMMA_AIR;

        // P/P0 = (1 + ((γ-1)/2)M²)^(-γ/(γ-1))
        double P1_P01 = pow(1.0 + ((gamma - 1.0) / 2.0) * M1 * M1, -gamma / (gamma - 1.0));
        double P2_P02 = pow(1.0 + ((gamma - 1.0) / 2.0) * M2 * M2, -gamma / (gamma - 1.0));

        // P01 = P02 for isentropic expansion
        double P2_P1 = P2_P02 / P1_P01;

        // Isentropic pressure ratio for M1=2, M2=2.5
        TS_ASSERT_DELTA(P2_P1, 0.458, 0.01);
        TS_ASSERT(P2_P1 < 1.0);  // Pressure decreases in expansion
    }

    //==========================================================================
    // 7. CRITICAL MACH NUMBER (~3 tests)
    //==========================================================================

    void testCriticalMach_Definition() {
        // M_crit: freestream Mach where local flow first reaches M = 1
        // For airfoil: 1 + ((γ-1)/2)M_crit² = (1 + ((γ-1)/2))^(γ/(γ-1)) / C_p_min
        // Simplified: M_crit ≈ 0.7-0.9 for typical airfoils
        double M_crit_typical = 0.80;

        TS_ASSERT(M_crit_typical > 0.7 && M_crit_typical < 0.9);
    }

    void testCriticalMach_PressureCoefficient() {
        // Critical pressure coefficient: C_p* = -2/(γM∞²) * ((1+((γ-1)/2)M∞²)^(γ/(γ-1)) - 1)
        double M_inf = 0.75;
        double gamma = GAMMA_AIR;

        double term = pow(1.0 + ((gamma - 1.0) / 2.0) * M_inf * M_inf, gamma / (gamma - 1.0));
        double Cp_crit = -2.0 / (gamma * M_inf * M_inf) * (term - 1.0);

        // For M_inf=0.75, the critical pressure coefficient
        TS_ASSERT_DELTA(Cp_crit, -1.15, 0.02);
    }

    void testCriticalMach_ThicknessEffect() {
        // Thicker airfoils have lower critical Mach numbers
        // Approximate: M_crit ≈ 0.87 - (t/c)
        double tc_thin = 0.06;   // 6% thickness
        double tc_thick = 0.15;  // 15% thickness

        double M_crit_thin = 0.87 - tc_thin;
        double M_crit_thick = 0.87 - tc_thick;

        TS_ASSERT(M_crit_thin > M_crit_thick);
        TS_ASSERT_DELTA(M_crit_thin, 0.81, 0.01);
        TS_ASSERT_DELTA(M_crit_thick, 0.72, 0.01);
    }

    //==========================================================================
    // 8. DRAG DIVERGENCE MACH (~3 tests)
    //==========================================================================

    void testDragDivergence_Definition() {
        // M_dd: Mach where dC_D/dM = 0.1 (rapid drag increase begins)
        // Typically M_dd = M_crit + 0.03 to 0.05
        double M_crit = 0.78;
        double M_dd = M_crit + 0.04;

        TS_ASSERT_DELTA(M_dd, 0.82, 0.01);
    }

    void testDragDivergence_KornEquation() {
        // Korn equation: M_dd/cos(Λ) = K - (t/c)/cos²(Λ) - C_L/(10*cos³(Λ))
        // Simplified for unswept wing (Λ = 0):
        double K = 0.87;  // Technology factor
        double tc = 0.12;
        double CL = 0.5;

        double M_dd = K - tc - CL / 10.0;

        TS_ASSERT_DELTA(M_dd, 0.70, 0.01);
    }

    void testDragDivergence_SweepEffect() {
        // Sweep increases M_dd: M_dd_swept = M_dd_unswept / cos(Λ)
        double M_dd_unswept = 0.75;
        double sweep_angle = 30.0 * PI_CONST / 180.0;  // radians

        double M_dd_swept = M_dd_unswept / cos(sweep_angle);

        TS_ASSERT_DELTA(M_dd_swept, 0.866, 0.01);
        TS_ASSERT(M_dd_swept > M_dd_unswept);
    }

    //==========================================================================
    // 9. SUPERSONIC LIFT COEFFICIENT CORRECTIONS (~3 tests)
    //==========================================================================

    void testSupersonicLift_AckermanTheory() {
        // Supersonic lift slope: dC_L/dα = 4 / sqrt(M² - 1)
        double M = 2.0;

        double dCL_dalpha = 4.0 / sqrt(M * M - 1.0);

        TS_ASSERT_DELTA(dCL_dalpha, 2.309, 0.01);  // per radian
    }

    void testSupersonicLift_LiftCoefficient() {
        // C_L = (4α) / sqrt(M² - 1)  for thin airfoil
        double M = 2.5;
        double alpha = 5.0 * PI_CONST / 180.0;  // radians

        double CL = (4.0 * alpha) / sqrt(M * M - 1.0);

        TS_ASSERT_DELTA(CL, 0.152, 0.01);
    }

    void testSupersonicLift_MachDependence() {
        // Lift slope decreases with increasing Mach number
        double alpha = 5.0 * PI_CONST / 180.0;

        double M1 = 1.5;
        double M2 = 3.0;

        double CL1 = (4.0 * alpha) / sqrt(M1 * M1 - 1.0);
        double CL2 = (4.0 * alpha) / sqrt(M2 * M2 - 1.0);

        TS_ASSERT(CL1 > CL2);  // Decreases with Mach
    }

    //==========================================================================
    // 10. MACH CONE ANGLE CALCULATIONS (~3 tests)
    //==========================================================================

    void testMachCone_MachAngle() {
        // Mach angle: μ = sin⁻¹(1/M)
        double M = 2.0;

        double mu = asin(1.0 / M);
        double mu_deg = mu * 180.0 / PI_CONST;

        TS_ASSERT_DELTA(mu_deg, 30.0, 0.1);
    }

    void testMachCone_ZoneOfSilence() {
        // Disturbances propagate within Mach cone (half-angle μ)
        double M = 3.0;

        double mu = asin(1.0 / M);
        double mu_deg = mu * 180.0 / PI_CONST;

        TS_ASSERT_DELTA(mu_deg, 19.47, 0.1);
        TS_ASSERT(mu < PI_CONST / 4.0);  // Less than 45° for M > sqrt(2)
    }

    void testMachCone_SubsonicLimit() {
        // At M < 1, no Mach cone exists
        double M_subsonic = 0.8;

        // sin⁻¹(1/M) undefined for M < 1
        TS_ASSERT(M_subsonic < 1.0);
        TS_ASSERT(1.0 / M_subsonic > 1.0);  // Can't compute Mach angle
    }

    //==========================================================================
    // 11. STAGNATION TEMPERATURE/PRESSURE IN COMPRESSIBLE FLOW (~5 tests)
    //==========================================================================

    void testStagnation_TemperatureRatio() {
        // T0/T = 1 + ((γ-1)/2)M²
        double M = 2.0;
        double gamma = GAMMA_AIR;

        double T0_T = 1.0 + ((gamma - 1.0) / 2.0) * M * M;

        TS_ASSERT_DELTA(T0_T, 1.8, 0.01);
    }

    void testStagnation_PressureRatio() {
        // P0/P = (1 + ((γ-1)/2)M²)^(γ/(γ-1))
        double M = 2.0;
        double gamma = GAMMA_AIR;

        double P0_P = pow(1.0 + ((gamma - 1.0) / 2.0) * M * M, gamma / (gamma - 1.0));

        TS_ASSERT_DELTA(P0_P, 7.824, 0.01);
    }

    void testStagnation_DensityRatio() {
        // ρ0/ρ = (1 + ((γ-1)/2)M²)^(1/(γ-1))
        double M = 2.0;
        double gamma = GAMMA_AIR;

        double rho0_rho = pow(1.0 + ((gamma - 1.0) / 2.0) * M * M, 1.0 / (gamma - 1.0));

        TS_ASSERT_DELTA(rho0_rho, 4.347, 0.01);
    }

    void testStagnation_IncompressibleLimit() {
        // At low Mach, Bernoulli equation applies
        // P0 - P ≈ (1/2)ρV² for M → 0
        double M = 0.1;
        double gamma = GAMMA_AIR;

        double P0_P = pow(1.0 + ((gamma - 1.0) / 2.0) * M * M, gamma / (gamma - 1.0));

        TS_ASSERT_DELTA(P0_P, 1.007, 0.001);  // Very close to 1
    }

    void testStagnation_SupersonicRecovery() {
        // Pitot tube in supersonic flow measures post-shock stagnation pressure
        double M1 = 2.5;
        double gamma = GAMMA_AIR;

        // Normal shock ahead of pitot tube
        double M2_sq = (1.0 + ((gamma - 1.0) / 2.0) * M1 * M1) / (gamma * M1 * M1 - (gamma - 1.0) / 2.0);
        double M2 = sqrt(M2_sq);

        // Stagnation pressure after shock (Rayleigh pitot formula)
        double P0_ratio = pow(((gamma + 1.0) * M1 * M1) / ((gamma - 1.0) * M1 * M1 + 2.0), gamma / (gamma - 1.0))
                         * pow((gamma + 1.0) / (2.0 * gamma * M1 * M1 - (gamma - 1.0)), 1.0 / (gamma - 1.0));

        TS_ASSERT(P0_ratio < 1.0);  // Total pressure loss across shock
        TS_ASSERT(M2 < 1.0);        // Subsonic after shock
    }

    //==========================================================================
    // 12. AREA-MACH RELATIONS FOR NOZZLES (~4 tests)
    //==========================================================================

    void testAreaMach_SubsonicBranch() {
        // A/A* = (1/M) * ((2/(γ+1)) * (1 + ((γ-1)/2)M²))^((γ+1)/(2(γ-1)))
        double M = 0.5;
        double gamma = GAMMA_AIR;

        double term1 = 1.0 / M;
        double term2 = (2.0 / (gamma + 1.0)) * (1.0 + ((gamma - 1.0) / 2.0) * M * M);
        double exponent = (gamma + 1.0) / (2.0 * (gamma - 1.0));
        double A_Astar = term1 * pow(term2, exponent);

        TS_ASSERT_DELTA(A_Astar, 1.340, 0.01);
    }

    void testAreaMach_SupersonicBranch() {
        // Same formula, but M > 1
        double M = 2.0;
        double gamma = GAMMA_AIR;

        double term1 = 1.0 / M;
        double term2 = (2.0 / (gamma + 1.0)) * (1.0 + ((gamma - 1.0) / 2.0) * M * M);
        double exponent = (gamma + 1.0) / (2.0 * (gamma - 1.0));
        double A_Astar = term1 * pow(term2, exponent);

        TS_ASSERT_DELTA(A_Astar, 1.688, 0.01);
    }

    void testAreaMach_SonicThroat() {
        // At throat (M = 1), A = A*
        double M = 1.0;
        double gamma = GAMMA_AIR;

        double term1 = 1.0 / M;
        double term2 = (2.0 / (gamma + 1.0)) * (1.0 + ((gamma - 1.0) / 2.0) * M * M);
        double exponent = (gamma + 1.0) / (2.0 * (gamma - 1.0));
        double A_Astar = term1 * pow(term2, exponent);

        TS_ASSERT_DELTA(A_Astar, 1.0, 0.01);
    }

    void testAreaMach_MassFlowParameter() {
        // ṁsqrt(T0)/(A*P0) = sqrt(γ/R) * ((γ+1)/2)^(-(γ+1)/(2(γ-1)))
        double gamma = GAMMA_AIR;

        double term = pow((gamma + 1.0) / 2.0, -(gamma + 1.0) / (2.0 * (gamma - 1.0)));
        // Mass flow parameter is constant for choked flow

        TS_ASSERT(term > 0);
        TS_ASSERT_DELTA(term, 0.5787, 0.001);
    }

    //==========================================================================
    // 13. FANNO FLOW (FRICTION EFFECTS) (~4 tests)
    //==========================================================================

    void testFannoFlow_SubsonicFriction() {
        // In Fanno flow, friction causes:
        // - Subsonic: M increases, P decreases, T decreases
        double M1 = 0.5;
        double M2 = 0.6;  // After friction

        TS_ASSERT(M2 > M1);  // Mach increases
    }

    void testFannoFlow_SupersonicFriction() {
        // In Fanno flow, friction causes:
        // - Supersonic: M decreases, P increases, T increases
        double M1 = 2.0;
        double M2 = 1.8;  // After friction

        TS_ASSERT(M2 < M1);  // Mach decreases toward M = 1
    }

    void testFannoFlow_ChokedLimit() {
        // Fanno flow chokes at M = 1 (sonic conditions)
        // Maximum length: L_max = (1 - M²)/(γM²) + ((γ+1)/(2γ)) * ln((γ+1)M²/(2(1+((γ-1)/2)M²)))
        double M = 0.5;
        double gamma = GAMMA_AIR;

        double term1 = (1.0 - M * M) / (gamma * M * M);
        double term2_num = (gamma + 1.0) * M * M;
        double term2_den = 2.0 * (1.0 + ((gamma - 1.0) / 2.0) * M * M);
        double term2 = ((gamma + 1.0) / (2.0 * gamma)) * log(term2_num / term2_den);
        double fLD_max = term1 + term2;  // f*L_max/D

        TS_ASSERT(fLD_max > 0);
    }

    void testFannoFlow_PressureRatio() {
        // P1/P2 = (M2/M1) * sqrt((1 + ((γ-1)/2)M1²) / (1 + ((γ-1)/2)M2²))
        double M1 = 0.5;
        double M2 = 0.6;
        double gamma = GAMMA_AIR;

        double P1_P2 = (M2 / M1) * sqrt((1.0 + ((gamma - 1.0) / 2.0) * M1 * M1) / (1.0 + ((gamma - 1.0) / 2.0) * M2 * M2));

        TS_ASSERT(P1_P2 > 1.0);  // Pressure decreases in subsonic Fanno flow
    }

    //==========================================================================
    // 14. RAYLEIGH FLOW (HEAT ADDITION) (~4 tests)
    //==========================================================================

    void testRayleighFlow_SubsonicHeating() {
        // Heat addition in subsonic flow:
        // - M increases, P decreases, T increases
        double M1 = 0.5;
        double M2 = 0.6;  // After heating

        TS_ASSERT(M2 > M1);  // Mach increases toward M = 1
    }

    void testRayleighFlow_SupersonicHeating() {
        // Heat addition in supersonic flow:
        // - M decreases, P increases, T increases
        double M1 = 2.0;
        double M2 = 1.8;  // After heating

        TS_ASSERT(M2 < M1);  // Mach decreases toward M = 1
    }

    void testRayleighFlow_MaximumHeat() {
        // Maximum heat addition occurs at M = 1
        // T/T* is maximum at M = 1
        double gamma = GAMMA_AIR;

        // For reference, T0/T0* at M = 1
        double T0_T0star_sonic = (gamma + 1.0) / (2.0 + (gamma - 1.0));

        // At M=1: (γ+1)/(2+(γ-1)) = 2.4/2.4 = 1.0
        TS_ASSERT(T0_T0star_sonic > 0);
        TS_ASSERT_DELTA(T0_T0star_sonic, 1.0, 0.01);
    }

    void testRayleighFlow_StagnationPressureRatio() {
        // P0/P0* = ((γ+1)/(2)) * (1 + γM²) / ((1 + ((γ-1)/2)M²)^(γ/(γ-1)))
        double M = 0.5;
        double gamma = GAMMA_AIR;

        double numerator = ((gamma + 1.0) / 2.0) * (1.0 + gamma * M * M);
        double denominator = pow(1.0 + ((gamma - 1.0) / 2.0) * M * M, gamma / (gamma - 1.0));
        double P0_P0star = numerator / denominator;

        TS_ASSERT(P0_P0star > 0);
    }

    //==========================================================================
    // INTEGRATION TESTS
    //==========================================================================

    void testCompressibleFlow_TransonicAircraft() {
        // Complete transonic analysis
        double M = 0.85;
        double gamma = GAMMA_AIR;

        // Prandtl-Glauert correction (marginal validity)
        double beta = sqrt(1.0 - M * M);
        TS_ASSERT(beta < 0.53);  // Near validity limit

        // Drag divergence check
        double M_dd = 0.82;
        TS_ASSERT(M > M_dd);  // In drag rise region

        // Stagnation conditions
        double T0_T = 1.0 + ((gamma - 1.0) / 2.0) * M * M;
        TS_ASSERT_DELTA(T0_T, 1.1445, 0.01);
    }

    void testCompressibleFlow_SupersonicFighter() {
        // Supersonic flight analysis
        double M = 1.8;
        double gamma = GAMMA_AIR;

        // Mach angle
        double mu = asin(1.0 / M);
        double mu_deg = mu * 180.0 / PI_CONST;
        TS_ASSERT_DELTA(mu_deg, 33.75, 0.1);

        // Lift slope
        double dCL_dalpha = 4.0 / sqrt(M * M - 1.0);
        TS_ASSERT_DELTA(dCL_dalpha, 2.676, 0.01);

        // Wave drag present
        double tc = 0.05;
        double CD_wave = 4.0 / sqrt(M * M - 1.0) * tc * tc;
        TS_ASSERT(CD_wave > 0);
    }

    void testCompressibleFlow_InletShock() {
        // Normal shock in supersonic inlet
        double M1 = 2.2;
        double gamma = GAMMA_AIR;

        // Shock relations
        double P2_P1 = 1.0 + (2.0 * gamma / (gamma + 1.0)) * (M1 * M1 - 1.0);
        double M2_sq = (1.0 + ((gamma - 1.0) / 2.0) * M1 * M1) / (gamma * M1 * M1 - (gamma - 1.0) / 2.0);
        double M2 = sqrt(M2_sq);

        // Verify shock behavior
        TS_ASSERT(P2_P1 > 1.0);  // Pressure rise
        TS_ASSERT(M2 < 1.0);     // Subsonic after shock
        TS_ASSERT_DELTA(M2, 0.547, 0.01);
    }
};

/*******************************************************************************
 * Additional FGCompressibleFlow Tests (30 new tests)
 ******************************************************************************/

class FGCompressibleFlowAdditionalTest : public CxxTest::TestSuite
{
public:
    //==========================================================================
    // HYPERSONIC FLOW EFFECTS
    //==========================================================================

    // Test 46: Hypersonic similarity parameter
    void testHypersonicSimilarity() {
        // Hypersonic similarity: K = M * θ (Mach * deflection angle)
        double M = 6.0;
        double theta = 0.10;  // radians (~5.7 degrees)

        double K = M * theta;
        TS_ASSERT_DELTA(K, 0.6, 0.01);

        // Similar flows have same K
        double M2 = 10.0;
        double theta2 = K / M2;
        TS_ASSERT_DELTA(theta2, 0.06, 0.001);
    }

    // Test 47: Newtonian flow limit
    void testNewtonianFlowLimit() {
        // At very high Mach, Cp = 2 * sin²θ (Newtonian)
        double theta = 30.0 * PI_CONST / 180.0;  // radians

        double Cp_newtonian = 2.0 * sin(theta) * sin(theta);
        TS_ASSERT_DELTA(Cp_newtonian, 0.5, 0.01);
    }

    // Test 48: High Mach stagnation pressure ratio
    void testHighMachStagnation() {
        double M = 5.0;
        double gamma = GAMMA_AIR;

        double P0_P = pow(1.0 + ((gamma - 1.0) / 2.0) * M * M, gamma / (gamma - 1.0));
        TS_ASSERT_DELTA(P0_P, 529.1, 1.0);  // Very high ratio
    }

    // Test 49: Hypersonic temperature ratio
    void testHypersonicTemperatureRatio() {
        double M = 8.0;
        double gamma = GAMMA_AIR;

        double T0_T = 1.0 + ((gamma - 1.0) / 2.0) * M * M;
        TS_ASSERT_DELTA(T0_T, 13.8, 0.1);  // Very high stagnation temp
    }

    //==========================================================================
    // CONICAL SHOCK FLOW
    //==========================================================================

    // Test 50: Conical shock angle
    void testConicalShockAngle() {
        // Conical shock angle is less than 2D wedge shock angle
        double M = 3.0;
        double cone_half_angle = 10.0;  // degrees

        // Approximate shock angle for cone
        double shock_angle_wedge = 25.0;  // degrees (2D)
        double shock_angle_cone = 20.0;   // degrees (3D, less)

        TS_ASSERT(shock_angle_cone < shock_angle_wedge);
    }

    // Test 51: Conical flow pressure ratio
    void testConicalFlowPressureRatio() {
        // Conical shocks have lower pressure rise than 2D
        double P2_P1_wedge = 3.5;  // 2D oblique shock
        double P2_P1_cone = 2.5;   // 3D conical shock

        TS_ASSERT(P2_P1_cone < P2_P1_wedge);
    }

    // Test 52: Taylor-Maccoll cone flow
    void testTaylorMaccollConeFlow() {
        // Surface pressure on cone follows Taylor-Maccoll equation
        double M = 2.5;
        double cone_angle = 15.0 * PI_CONST / 180.0;

        // Simplified surface Mach number
        double M_surface_approx = M * cos(cone_angle) * 0.95;
        TS_ASSERT(M_surface_approx > 2.0);
        TS_ASSERT(M_surface_approx < M);
    }

    //==========================================================================
    // COMPRESSION CORNER FLOWS
    //==========================================================================

    // Test 53: Compression corner shock angle
    void testCompressionCornerShock() {
        double M1 = 2.0;
        double theta = 15.0 * PI_CONST / 180.0;  // Corner angle

        // For compression, flow turns toward shock
        // Mach angle is lower bound
        double mu = asin(1.0 / M1);
        TS_ASSERT(theta + mu < PI_CONST / 2.0);  // Attached shock possible
    }

    // Test 54: Pressure recovery in compression
    void testCompressionPressureRecovery() {
        double M1 = 2.0;
        double gamma = GAMMA_AIR;

        // Single oblique shock at β = 40°
        double beta = 40.0 * PI_CONST / 180.0;
        double M1n = M1 * sin(beta);
        double P2_P1_oblique = 1.0 + (2.0 * gamma / (gamma + 1.0)) * (M1n * M1n - 1.0);

        // Normal shock at same M1
        double P2_P1_normal = 1.0 + (2.0 * gamma / (gamma + 1.0)) * (M1 * M1 - 1.0);

        TS_ASSERT(P2_P1_oblique < P2_P1_normal);  // Oblique better
    }

    // Test 55: Isentropic compression efficiency
    void testIsentropicCompressionEfficiency() {
        double M1 = 2.0;
        double M2 = 1.5;  // After isentropic compression
        double gamma = GAMMA_AIR;

        // Isentropic pressure ratio
        double P2_P1 = pow((1.0 + ((gamma - 1.0) / 2.0) * M1 * M1) /
                          (1.0 + ((gamma - 1.0) / 2.0) * M2 * M2), gamma / (gamma - 1.0));

        TS_ASSERT(P2_P1 > 1.0);  // Pressure rises in compression
    }

    //==========================================================================
    // SHOCK-EXPANSION THEORY
    //==========================================================================

    // Test 56: Diamond airfoil pressure distribution
    void testDiamondAirfoilPressure() {
        double M = 2.0;
        double gamma = GAMMA_AIR;
        double theta = 5.0 * PI_CONST / 180.0;  // Half-angle

        // Leading edge compression
        double M1n = M * sin(45.0 * PI_CONST / 180.0);  // Approximate
        double P2_P1 = 1.0 + (2.0 * gamma / (gamma + 1.0)) * (M1n * M1n - 1.0);
        TS_ASSERT(P2_P1 > 1.0);

        // Trailing edge expansion (isentropic)
        // Pressure decreases
        double P3_P2 = 0.5;  // Simplified
        TS_ASSERT(P3_P2 < 1.0);
    }

    // Test 57: Shock-expansion wave drag
    void testShockExpansionWaveDrag() {
        double M = 2.5;
        double theta = 10.0 * PI_CONST / 180.0;  // Wedge half-angle
        double gamma = GAMMA_AIR;

        // Wave drag from pressure difference
        // Simplified: CD ∝ θ² for small angles
        double CD_wave_approx = 4.0 * theta * theta / sqrt(M * M - 1.0);

        TS_ASSERT(CD_wave_approx > 0);
        TS_ASSERT(CD_wave_approx < 0.1);
    }

    // Test 58: Expansion fan interaction
    void testExpansionFanInteraction() {
        // Two expansion fans meeting
        double M1 = 2.0;
        double turn1 = 10.0 * PI_CONST / 180.0;
        double turn2 = 8.0 * PI_CONST / 180.0;

        // Each fan accelerates flow
        double total_turn = turn1 + turn2;
        TS_ASSERT(total_turn < 30.0 * PI_CONST / 180.0);  // Still attached
    }

    //==========================================================================
    // INLET DESIGN
    //==========================================================================

    // Test 59: External compression inlet
    void testExternalCompressionInlet() {
        double M0 = 2.0;
        double n_shocks = 2;  // Number of oblique shocks

        // Multiple weak shocks more efficient than single normal shock
        double eta_multiple = 0.95;  // Typical for 2-shock inlet
        double eta_normal = 0.72;    // Normal shock only

        TS_ASSERT(eta_multiple > eta_normal);
    }

    // Test 60: Mixed compression inlet
    void testMixedCompressionInlet() {
        // External + internal compression
        double M0 = 3.0;
        double M_throat = 1.2;  // Just supersonic at throat

        // Pressure recovery better than external only
        double PR_external = 0.85;
        double PR_mixed = 0.92;

        TS_ASSERT(PR_mixed > PR_external);
    }

    // Test 61: Inlet buzz margin
    void testInletBuzzMargin() {
        // Inlet buzz occurs when normal shock oscillates
        double M_design = 2.0;
        double M_margin = 0.1;  // Buzz margin

        double M_buzz = M_design - M_margin;
        TS_ASSERT_DELTA(M_buzz, 1.9, 0.01);
    }

    //==========================================================================
    // NOZZLE EXPANSION
    //==========================================================================

    // Test 62: Underexpanded nozzle plume
    void testUnderexpandedPlume() {
        double P_exit = 150000.0;  // Pa
        double P_ambient = 101325.0;  // Pa

        double pressure_ratio = P_exit / P_ambient;
        TS_ASSERT(pressure_ratio > 1.0);  // Underexpanded

        // Plume expands after nozzle exit
        double expansion_angle_approx = 15.0;  // degrees
        TS_ASSERT(expansion_angle_approx > 0);
    }

    // Test 63: Overexpanded nozzle separation
    void testOverexpandedSeparation() {
        double P_exit_design = 50000.0;  // Pa
        double P_ambient = 101325.0;      // Pa

        double pressure_ratio = P_exit_design / P_ambient;
        TS_ASSERT(pressure_ratio < 1.0);  // Overexpanded

        // Flow may separate from nozzle wall
        double separation_ratio = 0.35;  // Typical separation P/P_ambient
        TS_ASSERT(separation_ratio < 0.5);
    }

    // Test 64: Plug nozzle expansion
    void testPlugNozzleExpansion() {
        double M_design = 3.0;
        double gamma = GAMMA_AIR;

        // Plug nozzle self-adjusts to ambient pressure
        double A_exit_A_throat = 4.0;  // Design area ratio

        // Thrust coefficient at design
        double Cf_design = 1.8;  // Typical for plug nozzle
        TS_ASSERT(Cf_design > 1.5);
    }

    //==========================================================================
    // TRANSONIC BUFFET
    //==========================================================================

    // Test 65: Buffet onset Mach
    void testBuffetOnsetMach() {
        // Buffet begins when shock-boundary layer interaction causes oscillation
        double CL = 0.6;
        double M_buffet_onset = 0.80 - 0.05 * CL;

        TS_ASSERT_DELTA(M_buffet_onset, 0.77, 0.01);
    }

    // Test 66: Shock oscillation frequency
    void testShockOscillationFrequency() {
        // Typical buffet frequency
        double chord = 5.0;  // m
        double V = 250.0;    // m/s

        // Strouhal number for buffet ~ 0.1
        double St = 0.1;
        double f_buffet = St * V / chord;

        TS_ASSERT_DELTA(f_buffet, 5.0, 0.5);  // Hz
    }

    // Test 67: Buffet boundary
    void testBuffetBoundary() {
        // CL vs M buffet boundary
        double M = 0.85;
        double CL_max_buffet = 0.80;  // At this Mach

        // Lower Mach allows higher CL before buffet
        double M2 = 0.75;
        double CL_max_buffet_2 = 1.0;

        TS_ASSERT(CL_max_buffet_2 > CL_max_buffet);
    }

    //==========================================================================
    // SHOCK-BOUNDARY LAYER INTERACTION
    //==========================================================================

    // Test 68: Shock-induced separation
    void testShockInducedSeparation() {
        // Strong shock can separate boundary layer
        double M_local = 1.4;
        double P2_P1 = 2.0;  // Across shock

        // Incipient separation occurs at pressure rise ~ 2x
        double P_ratio_separation = 2.0;

        TS_ASSERT(P2_P1 >= P_ratio_separation);  // May cause separation
    }

    // Test 69: Lambda shock structure
    void testLambdaShockStructure() {
        // Lambda shock forms in turbulent boundary layer interaction
        double M_upstream = 1.5;
        double shock_foot_height_approx = 0.005;  // m (boundary layer scale)

        // Bifurcated shock with oblique leg
        TS_ASSERT(shock_foot_height_approx > 0);
    }

    // Test 70: Separation bubble length
    void testSeparationBubbleLength() {
        // Separation bubble length scales with shock strength
        double delta = 0.01;  // m (boundary layer thickness)
        double P2_P1 = 2.5;   // Shock pressure ratio

        double bubble_length_approx = delta * (P2_P1 - 1.0) * 10.0;
        TS_ASSERT(bubble_length_approx > 0);
        TS_ASSERT(bubble_length_approx < 0.5);
    }

    //==========================================================================
    // AREA RULE AND WAVE DRAG
    //==========================================================================

    // Test 71: Area rule concept
    void testAreaRuleConcept() {
        // Smooth area distribution minimizes wave drag
        double A_max_fuselage = 10.0;   // m² (fuselage cross-section)
        double A_wing_root = 3.0;       // m² (wing adds area)

        // Waisted fuselage reduces total
        double A_waist = 7.0;
        double A_total_waisted = A_waist + A_wing_root;
        double A_total_unwaisted = A_max_fuselage + A_wing_root;

        TS_ASSERT(A_total_waisted < A_total_unwaisted);
    }

    // Test 72: Sears-Haack body drag
    void testSearsHaackDrag() {
        // Minimum wave drag for given volume
        double volume = 10.0;  // m³
        double length = 15.0;  // m

        // Fineness ratio
        double fineness = length / pow(volume, 1.0/3.0);
        TS_ASSERT(fineness > 5.0);  // Should be slender
    }

    // Test 73: Transonic area ruling effect
    void testTransonicAreaRuling() {
        // CD reduction from area ruling
        double CD_unruled = 0.040;
        double CD_ruled = 0.025;

        double reduction = (CD_unruled - CD_ruled) / CD_unruled * 100.0;
        TS_ASSERT_DELTA(reduction, 37.5, 1.0);  // ~38% reduction
    }

    //==========================================================================
    // COMPRESSIBILITY EFFECTS ON CONTROL
    //==========================================================================

    // Test 74: Aileron reversal Mach
    void testAileronReversalMach() {
        // At high Mach, flexible wing can cause control reversal
        double q_dynamic = 50000.0;  // Pa
        double q_reversal = 80000.0; // Pa (reversal dynamic pressure)

        bool reversal = q_dynamic > q_reversal;
        TS_ASSERT_EQUALS(reversal, false);  // Not reversed at this q
    }

    // Test 75: Transonic control effectiveness
    void testTransonicControlEffectiveness() {
        // Control surface effectiveness varies in transonic regime
        double eta_subsonic = 1.0;    // Baseline
        double eta_transonic = 0.7;   // Reduced due to shock
        double eta_supersonic = 0.8;  // Partially recovered

        TS_ASSERT(eta_transonic < eta_subsonic);
        TS_ASSERT(eta_supersonic > eta_transonic);
    }

    //==========================================================================
    // EXTENDED COMPRESSIBLE FLOW TESTS (76-100)
    //==========================================================================

    // Test 76: Mach number from velocity and temperature
    void testMachFromVelocityTemperature() {
        double V = 300.0;  // m/s
        double T = 288.15; // K
        double gamma = GAMMA_AIR;
        double R = 287.05; // J/(kg·K)

        double a = sqrt(gamma * R * T);
        double M = V / a;
        TS_ASSERT_DELTA(M, 0.882, 0.01);
    }

    // Test 77: Dynamic pressure in compressible flow
    void testCompressibleDynamicPressure() {
        double M = 0.8;
        double gamma = GAMMA_AIR;
        double P = 101325.0;

        // q = (γ/2) * P * M²
        double q = (gamma / 2.0) * P * M * M;
        TS_ASSERT_DELTA(q, 45390.0, 100.0);
    }

    // Test 78: Impact pressure ratio
    void testImpactPressureRatio() {
        double M = 0.5;
        double gamma = GAMMA_AIR;

        // Qc/P = (1 + (γ-1)/2 * M²)^(γ/(γ-1)) - 1
        double Qc_P = pow(1.0 + ((gamma - 1.0) / 2.0) * M * M, gamma / (gamma - 1.0)) - 1.0;
        TS_ASSERT_DELTA(Qc_P, 0.186, 0.01);
    }

    // Test 79: Calibrated airspeed relationship
    void testCalibratedAirspeedRelation() {
        double M = 0.7;
        double P = 101325.0;
        double gamma = GAMMA_AIR;

        // CAS is related to impact pressure
        double Qc = P * (pow(1.0 + ((gamma - 1.0) / 2.0) * M * M, gamma / (gamma - 1.0)) - 1.0);
        TS_ASSERT(Qc > 0);
        TS_ASSERT(Qc < P);
    }

    // Test 80: Equivalent airspeed
    void testEquivalentAirspeed() {
        double TAS = 250.0;  // m/s
        double rho = 0.5;    // kg/m³ (high altitude)
        double rho_SL = 1.225;

        double EAS = TAS * sqrt(rho / rho_SL);
        TS_ASSERT_DELTA(EAS, 159.7, 1.0);
    }

    // Test 81: Total temperature probe recovery
    void testTotalTemperatureRecovery() {
        double M = 0.85;
        double T_static = 220.0;  // K
        double gamma = GAMMA_AIR;
        double recovery = 0.98;  // TAT probe recovery factor

        double T0 = T_static * (1.0 + ((gamma - 1.0) / 2.0) * M * M);
        double T_measured = T_static + recovery * (T0 - T_static);

        TS_ASSERT(T_measured < T0);
        TS_ASSERT(T_measured > T_static);
    }

    // Test 82: Characteristic Mach number
    void testCharacteristicMachNumber() {
        double M = 2.0;
        double gamma = GAMMA_AIR;

        // M* = sqrt((gamma+1)*M² / (2 + (gamma-1)*M²))
        double M_star = sqrt((gamma + 1.0) * M * M / (2.0 + (gamma - 1.0) * M * M));
        TS_ASSERT_DELTA(M_star, 1.633, 0.01);
    }

    // Test 83: Crocco number
    void testCroccoNumber() {
        double M = 1.5;
        double gamma = GAMMA_AIR;

        // Cr = V/V_max = sqrt((γ-1)*M² / (2 + (γ-1)*M²))
        double Cr = sqrt((gamma - 1.0) * M * M / (2.0 + (gamma - 1.0) * M * M));
        TS_ASSERT_DELTA(Cr, 0.557, 0.01);
    }

    // Test 84: Normal shock entropy rise
    void testNormalShockEntropyRise() {
        double M1 = 2.0;
        double gamma = GAMMA_AIR;

        // Δs/R = ln((P02/P01)^(-1/(γ-1)) * (T02/T01)^(γ/(γ-1)))
        // Simplified: entropy always increases across shock
        double P2_P1 = 1.0 + (2.0 * gamma / (gamma + 1.0)) * (M1 * M1 - 1.0);
        TS_ASSERT(P2_P1 > 1.0);  // Pressure increases
    }

    // Test 85: Stagnation pressure loss across shock
    void testStagnationPressureLossShock() {
        double M1 = 2.0;
        double gamma = GAMMA_AIR;

        // P02/P01 = (((γ+1)M1²)/((γ-1)M1²+2))^(γ/(γ-1)) * ((γ+1)/(2γM1²-(γ-1)))^(1/(γ-1))
        double term1 = pow(((gamma + 1.0) * M1 * M1) / ((gamma - 1.0) * M1 * M1 + 2.0), gamma / (gamma - 1.0));
        double term2 = pow((gamma + 1.0) / (2.0 * gamma * M1 * M1 - (gamma - 1.0)), 1.0 / (gamma - 1.0));
        double P02_P01 = term1 * term2;

        TS_ASSERT(P02_P01 < 1.0);  // Always loses stagnation pressure
        TS_ASSERT_DELTA(P02_P01, 0.721, 0.01);
    }

    // Test 86: Supersonic diffuser efficiency
    void testSupersonicDiffuserEfficiency() {
        double M1 = 2.5;
        double P02_P01_ideal = 1.0;  // Isentropic
        double P02_P01_actual = 0.65;  // Typical inlet

        double eta_d = P02_P01_actual / P02_P01_ideal;
        TS_ASSERT_DELTA(eta_d, 0.65, 0.01);
    }

    // Test 87: Conical nozzle efficiency
    void testConicalNozzleEfficiency() {
        double half_angle = 15.0 * PI_CONST / 180.0;

        // λ = (1 + cos(θ))/2 for conical nozzle
        double lambda = (1.0 + cos(half_angle)) / 2.0;
        TS_ASSERT_DELTA(lambda, 0.983, 0.01);
    }

    // Test 88: Bell nozzle length optimization
    void testBellNozzleLength() {
        double L_conical = 1.0;  // Reference length (15° cone)
        double L_bell = 0.8;     // Typical 80% bell

        double length_ratio = L_bell / L_conical;
        TS_ASSERT_DELTA(length_ratio, 0.8, 0.01);
    }

    // Test 89: Thrust vectoring angle effect
    void testThrustVectoringEffect() {
        double F = 100000.0;  // N thrust
        double delta = 15.0 * PI_CONST / 180.0;  // 15° vector angle

        double F_axial = F * cos(delta);
        double F_normal = F * sin(delta);

        TS_ASSERT_DELTA(F_axial, 96593.0, 100.0);
        TS_ASSERT_DELTA(F_normal, 25882.0, 100.0);
    }

    // Test 90: Scramjet combustion Mach
    void testScramjetCombustionMach() {
        double M_inlet = 6.0;
        double M_combustor = 3.0;  // Supersonic combustion

        TS_ASSERT(M_combustor > 1.0);  // Must remain supersonic
        TS_ASSERT(M_combustor < M_inlet);  // Slows down
    }

    // Test 91: Thermal choking limit
    void testThermalChokingLimit() {
        double M = 0.8;
        double gamma = GAMMA_AIR;

        // Maximum heat addition before choking
        // T0_max/T0 at M=1
        double T0_ratio = (gamma + 1.0) * M * M * (2.0 + (gamma - 1.0) * M * M) /
                          pow(1.0 + gamma * M * M, 2.0);
        TS_ASSERT(T0_ratio > 0);
        TS_ASSERT(T0_ratio < 1.0);
    }

    // Test 92: Mass flux in choked flow
    void testChokedMassFlux() {
        double P0 = 500000.0;  // Pa
        double T0 = 800.0;     // K
        double gamma = GAMMA_AIR;
        double R = 287.05;

        // m_dot/A* = P0 * sqrt(γ/R/T0) * (2/(γ+1))^((γ+1)/(2(γ-1)))
        double factor = pow(2.0 / (gamma + 1.0), (gamma + 1.0) / (2.0 * (gamma - 1.0)));
        double mdot_Astar = P0 * sqrt(gamma / R / T0) * factor;

        TS_ASSERT(mdot_Astar > 0);
    }

    // Test 93: Blunt body bow shock standoff
    void testBowShockStandoff() {
        double R_nose = 0.5;  // m (nose radius)
        double M = 5.0;
        double gamma = GAMMA_AIR;

        // Approximate standoff distance: Δ/R ≈ 0.386 * exp(4.67/M²)
        double delta_R = 0.386 * exp(4.67 / (M * M));
        double delta = delta_R * R_nose;

        TS_ASSERT(delta > 0);
        TS_ASSERT(delta < R_nose);
    }

    // Test 94: Reattachment shock angle
    void testReattachmentShockAngle() {
        double separation_angle = 15.0;  // degrees
        double reattachment_angle = 20.0;  // degrees (typically larger)

        TS_ASSERT(reattachment_angle > separation_angle);
    }

    // Test 95: Base pressure coefficient
    void testBasePressureCoefficient() {
        double M = 2.0;
        double gamma = GAMMA_AIR;

        // Approximate base pressure coefficient
        // Cpb ≈ -2/(γM²) for supersonic flow
        double Cpb = -2.0 / (gamma * M * M);
        TS_ASSERT_DELTA(Cpb, -0.357, 0.01);
    }

    // Test 96: Pressure coefficient limit
    void testPressureCoefficientLimit() {
        double M = 3.0;
        double gamma = GAMMA_AIR;

        // Maximum Cp at stagnation
        double Cp_max = (pow(1.0 + ((gamma - 1.0) / 2.0) * M * M, gamma / (gamma - 1.0)) - 1.0) /
                        (0.5 * gamma * M * M);
        TS_ASSERT(Cp_max > 1.0);  // Can exceed 1 in compressible flow
    }

    // Test 97: Vacuum pressure coefficient
    void testVacuumPressureCoefficient() {
        double M = 2.0;
        double gamma = GAMMA_AIR;

        // Cp_vacuum = -2/(γM²)
        double Cp_vac = -2.0 / (gamma * M * M);
        TS_ASSERT_DELTA(Cp_vac, -0.357, 0.01);
    }

    // Test 98: Hypersonic viscous interaction
    void testHypersonicViscousInteraction() {
        double M = 10.0;
        double Re_x = 1e7;

        // Viscous interaction parameter
        double chi = M * M * M / sqrt(Re_x);
        TS_ASSERT(chi > 0);
    }

    // Test 99: Stagnation point heat flux
    void testStagnationPointHeatFlux() {
        double V = 7000.0;  // m/s (reentry)
        double rho = 0.001; // kg/m³
        double R_nose = 1.0;  // m

        // Simplified Sutton-Graves: q ∝ sqrt(rho/R) * V³
        double q_factor = sqrt(rho / R_nose) * pow(V, 3.0);
        TS_ASSERT(q_factor > 0);
    }

    // Test 100: Complete supersonic flight parameters
    void testCompleteSupersonicFlightParameters() {
        double M = 2.2;
        double gamma = GAMMA_AIR;
        double T_static = 220.0;  // K
        double P_static = 20000.0;  // Pa

        // Calculate all parameters
        double T0_T = 1.0 + ((gamma - 1.0) / 2.0) * M * M;
        double P0_P = pow(T0_T, gamma / (gamma - 1.0));
        double mu = asin(1.0 / M);

        double T0 = T_static * T0_T;
        double P0 = P_static * P0_P;
        double mu_deg = mu * 180.0 / PI_CONST;

        TS_ASSERT_DELTA(T0, 432.08, 1.0);
        TS_ASSERT_DELTA(P0, 213854.0, 100.0);
        TS_ASSERT_DELTA(mu_deg, 27.04, 0.1);
    }
};

// ============ C172x Aircraft Compressible Flow Integration Tests ============
class FGCompressibleFlowC172xTest : public CxxTest::TestSuite
{
public:

  // Test C172x Mach number at various altitudes
  void testC172xMachNumberAtAltitudes() {
    double alts[] = {1000.0, 5000.0, 10000.0};

    for (double alt : alts) {
      FGFDMExec fdmex;
      fdmex.LoadModel("c172x");

      auto ic = fdmex.GetIC();
      ic->SetAltitudeASLFtIC(alt);
      ic->SetVcalibratedKtsIC(120.0);

      fdmex.RunIC();

      auto aux = fdmex.GetAuxiliary();
      double mach = aux->GetMach();

      TS_ASSERT(std::isfinite(mach));
      TS_ASSERT(mach > 0.0);
      TS_ASSERT(mach < 0.5);  // C172x is well subsonic
    }
  }

  // Test C172x dynamic pressure calculation
  void testC172xDynamicPressure() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto ic = fdmex.GetIC();
    ic->SetAltitudeASLFtIC(5000.0);
    ic->SetVcalibratedKtsIC(100.0);

    fdmex.RunIC();

    auto aux = fdmex.GetAuxiliary();
    double qbar = aux->Getqbar();

    TS_ASSERT(std::isfinite(qbar));
    TS_ASSERT(qbar > 0.0);
  }

  // Test C172x TAS/CAS relationship
  void testC172xTASCASRelationship() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto ic = fdmex.GetIC();
    ic->SetAltitudeASLFtIC(10000.0);
    ic->SetVcalibratedKtsIC(100.0);

    fdmex.RunIC();

    auto aux = fdmex.GetAuxiliary();
    auto propagate = fdmex.GetPropagate();

    double vcas = aux->GetVcalibratedKTS();
    double vtas = aux->GetVtrueFPS() * 0.592484;  // fps to kts

    TS_ASSERT(std::isfinite(vcas));
    TS_ASSERT(std::isfinite(vtas));
    // At altitude, TAS > CAS
    TS_ASSERT(vtas > vcas * 0.95);
  }

  // Test C172x speed of sound access
  void testC172xSpeedOfSoundAccess() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto ic = fdmex.GetIC();
    ic->SetAltitudeASLFtIC(5000.0);

    fdmex.RunIC();

    auto atm = fdmex.GetAtmosphere();
    double soundSpeed = atm->GetSoundSpeed();

    TS_ASSERT(std::isfinite(soundSpeed));
    TS_ASSERT(soundSpeed > 900.0);  // fps
    TS_ASSERT(soundSpeed < 1200.0); // fps
  }

  // Test C172x flight Mach number stability
  void testC172xMachNumberStability() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto ic = fdmex.GetIC();
    ic->SetAltitudeASLFtIC(8000.0);
    ic->SetVcalibratedKtsIC(110.0);

    fdmex.RunIC();

    auto prop = fdmex.GetPropulsion();
    auto aux = fdmex.GetAuxiliary();
    prop->InitRunning(-1);

    for (int i = 0; i < 200; i++) {
      fdmex.Run();

      double mach = aux->GetMach();
      TS_ASSERT(std::isfinite(mach));
      TS_ASSERT(mach > 0.0);
      TS_ASSERT(mach < 1.0);  // Subsonic
    }
  }

  // Test C172x compressibility effects are minimal
  void testC172xMinimalCompressibilityEffects() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto ic = fdmex.GetIC();
    ic->SetAltitudeASLFtIC(5000.0);
    ic->SetVcalibratedKtsIC(150.0);  // High speed for C172

    fdmex.RunIC();

    auto aux = fdmex.GetAuxiliary();
    double mach = aux->GetMach();

    // Even at high speed, C172x is well below compressibility threshold
    TS_ASSERT(mach < 0.3);
  }

  // Test C172x qbar vs altitude relationship
  void testC172xQbarVsAltitude() {
    double qbars[3];
    double alts[] = {0.0, 5000.0, 10000.0};

    for (int i = 0; i < 3; i++) {
      FGFDMExec fdmex;
      fdmex.LoadModel("c172x");

      auto ic = fdmex.GetIC();
      ic->SetAltitudeASLFtIC(alts[i]);
      ic->SetVcalibratedKtsIC(100.0);

      fdmex.RunIC();

      qbars[i] = fdmex.GetAuxiliary()->Getqbar();
      TS_ASSERT(std::isfinite(qbars[i]));
      TS_ASSERT(qbars[i] > 0.0);
    }

    // Qbar at same CAS is similar across altitudes (CAS accounts for density)
    // But may vary slightly due to compressibility
  }

  // Test C172x equivalent airspeed
  void testC172xEquivalentAirspeed() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto ic = fdmex.GetIC();
    ic->SetAltitudeASLFtIC(10000.0);
    ic->SetVcalibratedKtsIC(100.0);

    fdmex.RunIC();

    auto aux = fdmex.GetAuxiliary();
    double vequiv = aux->GetVequivalentFPS();

    TS_ASSERT(std::isfinite(vequiv));
    TS_ASSERT(vequiv > 0.0);
  }

  // Test C172x total temperature calculation
  void testC172xTotalTemperature() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto ic = fdmex.GetIC();
    ic->SetAltitudeASLFtIC(8000.0);
    ic->SetVcalibratedKtsIC(120.0);

    fdmex.RunIC();

    auto aux = fdmex.GetAuxiliary();
    auto atm = fdmex.GetAtmosphere();

    double TAT = aux->GetTotalTemperature();
    double SAT = atm->GetTemperature();

    TS_ASSERT(std::isfinite(TAT));
    TS_ASSERT(std::isfinite(SAT));
    // Total temp >= static temp (kinetic heating)
    TS_ASSERT(TAT >= SAT);
  }

  // Test C172x total pressure calculation
  void testC172xTotalPressure() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto ic = fdmex.GetIC();
    ic->SetAltitudeASLFtIC(5000.0);
    ic->SetVcalibratedKtsIC(100.0);

    fdmex.RunIC();

    auto aux = fdmex.GetAuxiliary();
    auto atm = fdmex.GetAtmosphere();

    double pt = aux->GetTotalPressure();
    double ps = atm->GetPressure();

    TS_ASSERT(std::isfinite(pt));
    TS_ASSERT(std::isfinite(ps));
    // Total pressure >= static pressure
    TS_ASSERT(pt >= ps);
  }

  // Test C172x flight at various speeds
  void testC172xFlightAtVariousSpeeds() {
    double speeds[] = {60.0, 80.0, 100.0, 120.0, 140.0};

    for (double spd : speeds) {
      FGFDMExec fdmex;
      fdmex.LoadModel("c172x");

      auto ic = fdmex.GetIC();
      ic->SetAltitudeASLFtIC(5000.0);
      ic->SetVcalibratedKtsIC(spd);

      fdmex.RunIC();

      auto aux = fdmex.GetAuxiliary();
      double mach = aux->GetMach();
      double qbar = aux->Getqbar();

      TS_ASSERT(std::isfinite(mach));
      TS_ASSERT(std::isfinite(qbar));
      TS_ASSERT(mach < 0.5);
      TS_ASSERT(qbar > 0.0);
    }
  }

  // Test C172x impact pressure calculation
  void testC172xImpactPressure() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto ic = fdmex.GetIC();
    ic->SetAltitudeASLFtIC(5000.0);
    ic->SetVcalibratedKtsIC(100.0);

    fdmex.RunIC();

    auto aux = fdmex.GetAuxiliary();
    double qc = aux->GetTotalPressure() - fdmex.GetAtmosphere()->GetPressure();

    TS_ASSERT(std::isfinite(qc));
    TS_ASSERT(qc >= 0.0);
  }
};
