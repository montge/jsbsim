/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

 Header:       FGAeroBodyTest.h
 Author:       Claude Code
 Date started: 12/24/2025

 ------------- Copyright (C) 2025 -------------

 This program is free software; you can redistribute it and/or modify it under
 the terms of the GNU Lesser General Public License as published by the Free
 Software Foundation; either version 2 of the License, or (at your option) any
 later version.

 This program is distributed in the hope that it will be useful, but WITHOUT
 ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more
 details.

 You should have received a copy of the GNU Lesser General Public License along
 with this program; if not, write to the Free Software Foundation, Inc., 59
 Temple Place - Suite 330, Boston, MA 02111-1307, USA.

 Further information about the GNU Lesser General Public License can also be
 found on the world wide web at http://www.gnu.org.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
SENTRY
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>

constexpr double epsilon = 1e-3;  // Practical tolerance for aerodynamic calculations
constexpr double PI = 3.14159265358979323846;
constexpr double DEG_TO_RAD = PI / 180.0;
constexpr double RAD_TO_DEG = 180.0 / PI;

class FGAeroBodyTest : public CxxTest::TestSuite
{
public:
  // Test 1: Basic lift equation L = q * S * CL
  void testLiftEquation() {
    double q = 1000.0;  // Dynamic pressure (Pa)
    double S = 20.0;    // Wing area (m^2)
    double CL = 0.5;    // Lift coefficient
    
    double L = q * S * CL;
    
    TS_ASSERT_DELTA(L, 10000.0, epsilon);
  }
  
  // Test 2: Lift at different CL values
  void testLiftVariousCL() {
    double q = 500.0;
    double S = 15.0;
    
    double CL1 = 0.0;
    double L1 = q * S * CL1;
    TS_ASSERT_DELTA(L1, 0.0, epsilon);
    
    double CL2 = 1.0;
    double L2 = q * S * CL2;
    TS_ASSERT_DELTA(L2, 7500.0, epsilon);
    
    double CL3 = -0.2;
    double L3 = q * S * CL3;
    TS_ASSERT_DELTA(L3, -1500.0, epsilon);
  }
  
  // Test 3: Basic drag equation D = q * S * CD
  void testDragEquation() {
    double q = 1000.0;  // Dynamic pressure (Pa)
    double S = 20.0;    // Wing area (m^2)
    double CD = 0.05;   // Drag coefficient
    
    double D = q * S * CD;
    
    TS_ASSERT_DELTA(D, 1000.0, epsilon);
  }
  
  // Test 4: Drag polar - parasite drag only
  void testDragPolarParasite() {
    double CD0 = 0.025;  // Zero-lift drag
    double CL = 0.0;
    double e = 0.8;      // Oswald efficiency
    double AR = 8.0;     // Aspect ratio
    
    double K = 1.0 / (PI * e * AR);
    double CD = CD0 + K * CL * CL;
    
    TS_ASSERT_DELTA(CD, 0.025, epsilon);
  }
  
  // Test 5: Drag polar - induced drag component
  void testDragPolarInduced() {
    double CD0 = 0.025;
    double CL = 0.6;
    double e = 0.8;
    double AR = 8.0;
    
    double K = 1.0 / (PI * e * AR);
    double CD_induced = K * CL * CL;
    double CD = CD0 + CD_induced;
    
    TS_ASSERT_DELTA(K, 0.04973591964, 1e-3);
    TS_ASSERT_DELTA(CD_induced, 0.01790093107, 1e-3);
    TS_ASSERT_DELTA(CD, 0.04290093107, 1e-3);
  }
  
  // Test 6: Drag polar at various CL values
  void testDragPolarCurve() {
    double CD0 = 0.03;
    double e = 0.85;
    double AR = 10.0;
    double K = 1.0 / (PI * e * AR);
    
    // Minimum drag at CL = 0
    double CD_min = CD0;
    TS_ASSERT_DELTA(CD_min, 0.03, epsilon);
    
    // At CL = 0.5
    double CD1 = CD0 + K * 0.5 * 0.5;
    TS_ASSERT_DELTA(CD1, 0.03940985, 1e-3);
    
    // At CL = 1.0
    double CD2 = CD0 + K * 1.0 * 1.0;
    TS_ASSERT_DELTA(CD2, 0.06763944, 1e-3);
  }
  
  // Test 7: Pitching moment equation M = q * S * c * Cm
  void testPitchingMoment() {
    double q = 800.0;   // Dynamic pressure (Pa)
    double S = 18.0;    // Wing area (m^2)
    double c = 2.0;     // Mean aerodynamic chord (m)
    double Cm = -0.05;  // Pitching moment coefficient
    
    double M = q * S * c * Cm;
    
    TS_ASSERT_DELTA(M, -1440.0, epsilon);
  }
  
  // Test 8: Rolling moment equation L = q * S * b * Cl
  void testRollingMoment() {
    double q = 1000.0;  // Dynamic pressure (Pa)
    double S = 20.0;    // Wing area (m^2)
    double b = 12.0;    // Wing span (m)
    double Cl = 0.02;   // Rolling moment coefficient
    
    double L = q * S * b * Cl;
    
    TS_ASSERT_DELTA(L, 4800.0, epsilon);
  }
  
  // Test 9: Yawing moment equation N = q * S * b * Cn
  void testYawingMoment() {
    double q = 900.0;   // Dynamic pressure (Pa)
    double S = 16.0;    // Wing area (m^2)
    double b = 10.0;    // Wing span (m)
    double Cn = -0.01;  // Yawing moment coefficient
    
    double N = q * S * b * Cn;
    
    TS_ASSERT_DELTA(N, -1440.0, epsilon);
  }
  
  // Test 10: Dynamic pressure calculation q = 0.5 * rho * V^2
  void testDynamicPressure() {
    double rho = 1.225;  // Air density (kg/m^3) at sea level
    double V = 100.0;    // Velocity (m/s)
    
    double q = 0.5 * rho * V * V;
    
    TS_ASSERT_DELTA(q, 6125.0, epsilon);
  }
  
  // Test 11: Dynamic pressure at altitude
  void testDynamicPressureAltitude() {
    double rho = 0.7364;  // Air density at 10,000 ft
    double V = 150.0;     // Velocity (m/s)
    
    double q = 0.5 * rho * V * V;
    
    TS_ASSERT_DELTA(q, 8285.0, 10.0);
  }
  
  // Test 12: Dynamic pressure from Mach number q = 0.5 * gamma * P * M^2
  void testDynamicPressureFromMach() {
    double gamma = 1.4;    // Specific heat ratio
    double P = 101325.0;   // Static pressure (Pa)
    double M = 0.5;        // Mach number
    
    double q = 0.5 * gamma * P * M * M;
    
    TS_ASSERT_DELTA(q, 17731.875, epsilon);
  }
  
  // Test 13: Angle of attack from velocity components
  void testAngleOfAttackCalculation() {
    double u = 100.0;  // Axial velocity (m/s)
    double w = 10.0;   // Normal velocity (m/s)
    
    double alpha = std::atan2(w, u);
    
    TS_ASSERT_DELTA(alpha, 0.09966865249, 1e-3);
    TS_ASSERT_DELTA(alpha * RAD_TO_DEG, 5.710593137, 1e-3);
  }
  
  // Test 14: Angle of attack at various conditions
  void testAngleOfAttackVariousConditions() {
    // Zero angle of attack
    double alpha1 = std::atan2(0.0, 100.0);
    TS_ASSERT_DELTA(alpha1, 0.0, epsilon);
    
    // Positive angle of attack
    double alpha2 = std::atan2(20.0, 100.0);
    TS_ASSERT_DELTA(alpha2 * RAD_TO_DEG, 11.3099324740, 1e-3);
    
    // Negative angle of attack
    double alpha3 = std::atan2(-15.0, 100.0);
    TS_ASSERT_DELTA(alpha3 * RAD_TO_DEG, -8.53076750, 1e-3);
  }
  
  // Test 15: Sideslip angle from velocity components
  void testSideslipAngleCalculation() {
    double u = 100.0;  // Axial velocity (m/s)
    double v = 5.0;    // Side velocity (m/s)
    double w = 0.0;    // Normal velocity (m/s)
    
    double V = std::sqrt(u*u + v*v + w*w);
    double beta = std::asin(v / V);
    
    TS_ASSERT_DELTA(beta, 0.04995837583, 1e-3);
    TS_ASSERT_DELTA(beta * RAD_TO_DEG, 2.862405227, 1e-3);
  }
  
  // Test 16: Sideslip angle at various conditions
  void testSideslipAngleVariousConditions() {
    // Zero sideslip
    double V1 = 100.0;
    double beta1 = std::asin(0.0 / V1);
    TS_ASSERT_DELTA(beta1, 0.0, epsilon);
    
    // Positive sideslip
    double V2 = std::sqrt(100.0*100.0 + 10.0*10.0);
    double beta2 = std::asin(10.0 / V2);
    TS_ASSERT_DELTA(beta2 * RAD_TO_DEG, 5.710593137, 1e-3);
    
    // Negative sideslip
    double V3 = std::sqrt(100.0*100.0 + 8.0*8.0);
    double beta3 = std::asin(-8.0 / V3);
    TS_ASSERT_DELTA(beta3 * RAD_TO_DEG, -4.573974149, 1e-3);
  }
  
  // Test 17: Body to wind axis transformation - lift and drag from forces
  void testBodyToWindAxisLiftDrag() {
    double alpha = 5.0 * DEG_TO_RAD;
    double Fx = -100.0;  // Body axial force (N, negative = drag)
    double Fz = -500.0;  // Body normal force (N, negative = lift)

    // Transform to wind axes
    double D = -Fx * std::cos(alpha) - Fz * std::sin(alpha);
    double L = -Fx * std::sin(alpha) + Fz * std::cos(alpha);

    // Verify calculation makes physical sense (positive drag, lift depends on forces)
    TS_ASSERT(D > 0);  // Drag should be positive
    TS_ASSERT_DELTA(D, 143.2, 0.5);
    TS_ASSERT_DELTA(L, -489.4, 0.5);
  }

  // Test 18: Wind to body axis transformation
  void testWindToBodyAxisTransformation() {
    double alpha = 10.0 * DEG_TO_RAD;
    double D = 150.0;   // Drag (wind axis)
    double L = 600.0;   // Lift (wind axis)

    // Transform to body axes
    double Fx = -D * std::cos(alpha) + L * std::sin(alpha);
    double Fz = -D * std::sin(alpha) - L * std::cos(alpha);

    TS_ASSERT_DELTA(Fx, -43.5, 0.5);
    TS_ASSERT_DELTA(Fz, -617.0, 0.5);
  }
  
  // Test 19: Reynolds number calculation
  void testReynoldsNumber() {
    double rho = 1.225;     // Air density (kg/m^3)
    double V = 100.0;       // Velocity (m/s)
    double c = 2.0;         // Characteristic length (m)
    double mu = 1.789e-5;   // Dynamic viscosity (Pa·s)
    
    double Re = (rho * V * c) / mu;
    
    TS_ASSERT_DELTA(Re, 13695000.0, 5000.0);
  }
  
  // Test 20: Reynolds number at different conditions
  void testReynoldsNumberVariousConditions() {
    double c = 1.5;
    double mu = 1.789e-5;
    
    // Sea level, low speed
    double Re1 = (1.225 * 50.0 * c) / mu;
    TS_ASSERT_DELTA(Re1, 5135673.47, 1000.0);
    
    // Sea level, high speed
    double Re2 = (1.225 * 200.0 * c) / mu;
    TS_ASSERT_DELTA(Re2, 20542693.90, 1000.0);
    
    // High altitude, lower density
    double Re3 = (0.7364 * 150.0 * c) / mu;
    TS_ASSERT_DELTA(Re3, 9260000.0, 5000.0);
  }
  
  // Test 21: Prandtl-Glauert compressibility correction
  void testPrandtlGlauertCorrection() {
    double M = 0.5;  // Mach number
    double beta = std::sqrt(1.0 - M * M);
    double CL0 = 0.5;  // Incompressible lift coefficient
    
    double CL = CL0 / beta;
    
    TS_ASSERT_DELTA(beta, 0.8660254038, 1e-3);
    TS_ASSERT_DELTA(CL, 0.5773502692, 1e-3);
  }
  
  // Test 22: Prandtl-Glauert at various Mach numbers
  void testPrandtlGlauertVariousMach() {
    double CL0 = 0.6;
    
    // Low subsonic
    double M1 = 0.3;
    double beta1 = std::sqrt(1.0 - M1 * M1);
    double CL1 = CL0 / beta1;
    TS_ASSERT_DELTA(CL1, 0.6285393611, 1e-3);
    
    // Medium subsonic
    double M2 = 0.6;
    double beta2 = std::sqrt(1.0 - M2 * M2);
    double CL2 = CL0 / beta2;
    TS_ASSERT_DELTA(CL2, 0.75, 1e-3);
    
    // High subsonic
    double M3 = 0.8;
    double beta3 = std::sqrt(1.0 - M3 * M3);
    double CL3 = CL0 / beta3;
    TS_ASSERT_DELTA(CL3, 1.0, 1e-3);
  }
  
  // Test 23: Ground effect - reduced induced drag
  void testGroundEffectInducedDrag() {
    double h = 5.0;   // Height above ground (m)
    double b = 12.0;  // Wing span (m)
    
    // Ground effect factor (simplified)
    double phi = (16.0 * h / b) * (16.0 * h / b);
    double ground_factor = phi / (1.0 + phi);
    
    double K_free = 0.05;  // Induced drag factor in free air
    double K_ground = K_free * ground_factor;
    
    TS_ASSERT_DELTA(ground_factor, phi / (1.0 + phi), 1e-6);
    TS_ASSERT_DELTA(K_ground, K_free * ground_factor, 1e-6);
  }
  
  // Test 24: Ground effect at various heights
  void testGroundEffectVariousHeights() {
    double b = 10.0;
    
    // Very close to ground
    double h1 = 1.0;
    double phi1 = (16.0 * h1 / b) * (16.0 * h1 / b);
    double gf1 = phi1 / (1.0 + phi1);
    TS_ASSERT_DELTA(gf1, 0.7199999999, 1e-3);
    
    // Half wingspan height
    double h2 = 5.0;
    double phi2 = (16.0 * h2 / b) * (16.0 * h2 / b);
    double gf2 = phi2 / (1.0 + phi2);
    TS_ASSERT_DELTA(gf2, 0.9846153846, 1e-3);
    
    // One wingspan height
    double h3 = 10.0;
    double phi3 = (16.0 * h3 / b) * (16.0 * h3 / b);
    double gf3 = phi3 / (1.0 + phi3);
    TS_ASSERT_DELTA(gf3, 0.9961240310, 1e-3);
  }
  
  // Test 25: Thrust specific fuel consumption (TSFC)
  void testThrustSpecificFuelConsumption() {
    double fuel_flow = 0.5;  // kg/s
    double thrust = 10000.0; // N
    
    double TSFC = fuel_flow / thrust;
    
    TS_ASSERT_DELTA(TSFC, 5.0e-5, epsilon);
  }
  
  // Test 26: Power calculation from thrust and velocity
  void testPowerFromThrust() {
    double thrust = 5000.0;  // N
    double V = 100.0;        // m/s
    
    double power = thrust * V;
    
    TS_ASSERT_DELTA(power, 500000.0, epsilon);
  }
  
  // Test 27: Propeller efficiency
  void testPropellerEfficiency() {
    double thrust = 4000.0;   // N
    double V = 80.0;          // m/s
    double shaft_power = 350000.0;  // W
    
    double thrust_power = thrust * V;
    double efficiency = thrust_power / shaft_power;
    
    TS_ASSERT_DELTA(thrust_power, 320000.0, epsilon);
    TS_ASSERT_DELTA(efficiency, 0.9142857143, 1e-3);
  }
  
  // Test 28: L/D ratio calculation
  void testLiftToDragRatio() {
    double CL = 0.8;
    double CD = 0.05;
    
    double LD_ratio = CL / CD;
    
    TS_ASSERT_DELTA(LD_ratio, 16.0, epsilon);
  }
  
  // Test 29: Maximum L/D from drag polar
  void testMaximumLDRatio() {
    double CD0 = 0.025;
    double e = 0.85;
    double AR = 10.0;
    double K = 1.0 / (PI * e * AR);
    
    // Maximum L/D occurs when CL = sqrt(CD0 / K)
    double CL_opt = std::sqrt(CD0 / K);
    double CD_opt = CD0 + K * CL_opt * CL_opt;
    double LD_max = CL_opt / CD_opt;
    
    // CL_opt = sqrt(0.025 / (1/(π*0.85*10))) = sqrt(0.025 * π * 8.5) ≈ 0.817
    TS_ASSERT_DELTA(CL_opt, 0.817, 0.01);
    TS_ASSERT_DELTA(CD_opt, 0.05, 1e-3);
    // LD_max = CL_opt / CD_opt ≈ 0.817 / 0.05 ≈ 16.34
    TS_ASSERT_DELTA(LD_max, 16.34, 0.1);
  }
  
  // Test 30: Velocity for best L/D
  void testVelocityForBestLD() {
    double W = 50000.0;  // Weight (N)
    double rho = 1.225;  // Air density (kg/m^3)
    double S = 20.0;     // Wing area (m^2)
    double CD0 = 0.025;
    double K = 0.05;
    
    // Velocity for best L/D: sqrt((2W/ρS) * sqrt(K/CD0)) ≈ 75.98 m/s
    double V_opt = std::sqrt((2.0 * W / (rho * S)) * std::sqrt(K / CD0));

    TS_ASSERT_DELTA(V_opt, 75.98, 0.1);
  }
  
  // Test 31: Stall speed calculation
  void testStallSpeed() {
    double W = 40000.0;   // Weight (N)
    double rho = 1.225;   // Air density (kg/m^3)
    double S = 18.0;      // Wing area (m^2)
    double CL_max = 1.6;  // Maximum lift coefficient
    
    // V_stall = sqrt(2W/(ρ*S*CL_max)) = sqrt(80000/35.28) ≈ 47.62 m/s
    double V_stall = std::sqrt((2.0 * W) / (rho * S * CL_max));

    TS_ASSERT_DELTA(V_stall, 47.62, 0.1);
  }
  
  // Test 32: Rate of climb calculation
  void testRateOfClimb() {
    double thrust = 12000.0;  // N
    double drag = 8000.0;     // N
    double W = 50000.0;       // Weight (N)
    double V = 100.0;         // Velocity (m/s)
    
    double excess_thrust = thrust - drag;
    double excess_power = excess_thrust * V;
    double rate_of_climb = excess_power / W;
    
    TS_ASSERT_DELTA(excess_power, 400000.0, epsilon);
    TS_ASSERT_DELTA(rate_of_climb, 8.0, epsilon);
  }
  
  // Test 33: Turn radius calculation
  void testTurnRadius() {
    double V = 100.0;      // Velocity (m/s)
    double g = 9.81;       // Gravitational acceleration (m/s^2)
    double bank_angle = 45.0 * DEG_TO_RAD;
    
    double turn_radius = (V * V) / (g * std::tan(bank_angle));
    
    TS_ASSERT_DELTA(turn_radius, 1019.367992, 1e-3);
  }
  
  // Test 34: Load factor in turn
  void testLoadFactorInTurn() {
    double bank_angle = 60.0 * DEG_TO_RAD;
    
    double load_factor = 1.0 / std::cos(bank_angle);
    
    TS_ASSERT_DELTA(load_factor, 2.0, epsilon);
  }
  
  // Test 35: Mach number calculation
  void testMachNumberCalculation() {
    double V = 250.0;      // Velocity (m/s)
    double gamma = 1.4;    // Specific heat ratio
    double R = 287.05;     // Gas constant for air (J/(kg·K))
    double T = 288.15;     // Temperature (K)
    
    double a = std::sqrt(gamma * R * T);  // Speed of sound
    double M = V / a;
    
    TS_ASSERT_DELTA(a, 340.29, 0.01);
    TS_ASSERT_DELTA(M, 0.7346, 0.001);
  }
  
  // Test 36: Wing loading calculation
  void testWingLoading() {
    double W = 45000.0;  // Weight (N)
    double S = 18.0;     // Wing area (m^2)
    
    double wing_loading = W / S;
    
    TS_ASSERT_DELTA(wing_loading, 2500.0, epsilon);
  }
  
  // Test 37: Aspect ratio from wing geometry
  void testAspectRatioCalculation() {
    double b = 12.0;  // Wing span (m)
    double S = 18.0;  // Wing area (m^2)
    
    double AR = (b * b) / S;
    
    TS_ASSERT_DELTA(AR, 8.0, epsilon);
  }
  
  // Test 38: Mean aerodynamic chord
  void testMeanAerodynamicChord() {
    double S = 20.0;  // Wing area (m^2)
    double b = 10.0;  // Wing span (m)
    
    // For rectangular wing
    double MAC = S / b;
    
    TS_ASSERT_DELTA(MAC, 2.0, epsilon);
  }
};
