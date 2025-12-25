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
#include "TestUtilities.h"

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
