#include <limits>
#include <cxxtest/TestSuite.h>

#include <FGFDMExec.h>
#include <models/FGAtmosphere.h>
#include <models/FGAuxiliary.h>

const double epsilon = 100. * std::numeric_limits<double>::epsilon();
constexpr double radtodeg = 180. / M_PI;

using namespace JSBSim;

class DummyAtmosphere : public FGAtmosphere
{
public:
  DummyAtmosphere(FGFDMExec* fdm) : FGAtmosphere(fdm) {}

  // Getters for the protected members
  static constexpr double GetR(void) { return Reng0; }
};

constexpr double R = DummyAtmosphere::GetR();

class FGAuxiliaryTest : public CxxTest::TestSuite
{
public:
  static constexpr double gama = FGAtmosphere::SHRatio;

  FGFDMExec fdmex;
  std::shared_ptr<FGAtmosphere> atm;

  FGAuxiliaryTest() {
    auto aux = fdmex.GetAuxiliary();
    atm = fdmex.GetAtmosphere();
    atm->InitModel();
    fdmex.GetPropertyManager()->Unbind(aux);
  }

  void testPitotTotalPressure() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    // Ambient conditions far upstream (i.e. upstream the normal schock
    // in supersonic flight)
    double p1 = atm->GetPressureSL();
    double t1 = atm->GetTemperatureSL();
    double rho1 = atm->GetDensitySL();
    constexpr double Cp = gama*R/(gama-1.0);

    // Based on formulas from Modern Compressible Flow (3rd edition)
    // - John D. Anderson
    for(double M1=0; M1<3.0; M1+=0.25) {
      double a1 = sqrt(gama*R*t1);
      double u1 = M1*a1;
      // Total temperature
      double T0 = t1+u1*u1/(2.0*Cp);
      // Compute conditions downstream (at the pitot tube)
      double u2 = u1;
      if (M1 >= 1.0){
        // Assess the normal shock effect knowing that a_star=u1*u2
        double a_star = sqrt((a1*a1/(gama-1.0)+0.5*u1*u1)*2*(gama-1.0)/(gama+1.0)); // equation (3.32) p.81
        u2 = a_star*a_star/u1;// equation (3.47) p.89
      }
      double t2 = T0-u2*u2/(2*Cp);
      double P2 = aux.PitotTotalPressure(M1, p1);
      double p2 = P2*pow(t2/T0, gama/(gama-1.0));
      double rho2 = p2/(R*t2);

      // mass conservation
      TS_ASSERT_DELTA(rho1*u1, rho2*u2, epsilon);
      // momentum conservation
      TS_ASSERT_DELTA(p1+rho1*u1*u1, p2+rho2*u2*u2, 1000.*epsilon);
      // energy conservation
#ifdef __arm64__
      TS_ASSERT_DELTA((Cp*t1+0.5*u1*u1)/(Cp*t2+0.5*u2*u2), 1.0, epsilon);
#else
      TS_ASSERT_DELTA(Cp*t1+0.5*u1*u1, Cp*t2+0.5*u2*u2, epsilon);
#endif
    }

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testMachFromImpactPressure() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    // Ambient conditions far upstream (i.e. upstream the normal schock
    // in supersonic flight)
    double p1 = atm->GetPressureSL();
    double t1 = atm->GetTemperatureSL();
    double rho1 = atm->GetDensitySL();
    constexpr double Cp = gama*R/(gama-1.0);

    // Based on formulas from Modern Compressible Flow (3rd edition)
    // - John D. Anderson
    for(double M1=0; M1<3.0; M1+=0.25) {
      double a1 = sqrt(gama*R*t1);
      double u1 = M1*a1;
      // Total temperature
      double T0 = t1+u1*u1/(2.0*Cp);
      // Compute conditions downstream (at the pitot tube)
      double u2 = u1;
      if (M1 >= 1.0) {
        // Assess the normal shock effect knowing that a_star=u1*u2
        double a_star = sqrt((a1*a1/(gama-1.0)+0.5*u1*u1)*2*(gama-1.0)/(gama+1.0)); // equation (3.32) p.81
        u2 = a_star*a_star/u1;// equation (3.47) p.89
      }
      double t2 = T0-u2*u2/(2*Cp);
      double rho2 = M1 == 0.0 ? rho1 : rho1*u1/u2;
      double p2 = rho2*R*t2;
      double P2 = p2*pow(T0/t2, gama/(gama-1.0));
      double mach1 = aux.MachFromImpactPressure(P2-p1, p1);
      double a2 = sqrt(gama*R*t2);
      double M2 = u2/a2;
      double mach2 = aux.MachFromImpactPressure(P2-p2, p2);

      // mass conservation
      TS_ASSERT_DELTA(rho1*u1, rho2*u2, epsilon);
      // momentum conservation
      TS_ASSERT_DELTA(p1+rho1*u1*u1, p2+rho2*u2*u2, 1000.*epsilon);
      // energy conservation
#ifdef __arm64__
      TS_ASSERT_DELTA((Cp*t1+0.5*u1*u1)/(Cp*t2+0.5*u2*u2), 1.0, epsilon);
#else
      TS_ASSERT_DELTA(Cp*t1+0.5*u1*u1, Cp*t2+0.5*u2*u2, epsilon);
#endif
      // Check the Mach computations
      TS_ASSERT_DELTA(mach1, M1, 1e-7);
      TS_ASSERT_DELTA(mach2, M2, 1e-7);
    }

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testCASConversion() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.StdDaySLsoundspeed = atm->StdDaySLsoundspeed;
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    // Ambient conditions far upstream (i.e. upstream the normal schock
    // in supersonic flight)
    double t1 = atm->GetTemperatureSL();
    double p1 = atm->GetPressureSL();

    TS_ASSERT_DELTA(aux.VcalibratedFromMach(0.0, p1), 0.0, epsilon);
    TS_ASSERT_DELTA(aux.MachFromVcalibrated(0.0, p1), 0.0, epsilon);

    // Check that VCAS match the true airspeed at sea level
    for(double M1=0.1; M1<3.0; M1+=0.25) {
      double u1 = M1*sqrt(gama*R*t1);
      TS_ASSERT_DELTA(aux.VcalibratedFromMach(M1, p1)/u1, 1.0, 1e-7);
      TS_ASSERT_DELTA(aux.MachFromVcalibrated(u1, p1)/M1, 1.0, 1e-7);
    }

    // Check the VCAS computation at an altitude of 1000 ft
    double asl = atm->GetSoundSpeedSL();
    p1 = atm->GetPressure(1000.);

    TS_ASSERT_DELTA(aux.VcalibratedFromMach(0.0, p1), 0.0, epsilon);
    TS_ASSERT_DELTA(aux.MachFromVcalibrated(0.0, p1), 0.0, epsilon);

    for(double M=0.1; M<3.0; M+=0.25) {
      double vcas = M*asl;
      double M1 = aux.MachFromVcalibrated(vcas, p1);
      TS_ASSERT_DELTA(aux.VcalibratedFromMach(M1, p1)/vcas, 1.0, 1e-7);
    }

    double psl = atm->GetPressureSL();
    t1 = atm->GetTemperature(1000.);
    double rho1 = atm->GetDensity(1000.);
    constexpr double Cp = gama*R/(gama-1.0);

    // Based on formulas from Modern Compressible Flow (3rd edition)
    // - John D. Anderson
    for(double M1=0.1; M1<3.0; M1+=0.25) {
      double a1 = sqrt(gama*R*t1);
      double u1 = M1*a1;
      // Total temperature
      double T0 = t1+u1*u1/(2.0*Cp);
      // Compute conditions downstream (at the pitot tube)
      double u2 = u1;
      if (M1 >= 1.0) {
        // Assess the normal shock effect knowing that a_star=u1*u2
        double a_star = sqrt((a1*a1/(gama-1.0)+0.5*u1*u1)*2*(gama-1.0)/(gama+1.0)); // equation (3.32) p.81
        u2 = a_star*a_star/u1;// equation (3.47) p.89
      }
      double t2 = T0-u2*u2/(2*Cp);
      double rho2 = M1 == 0.0 ? rho1 : rho1*u1/u2;
      double p2 = rho2*R*t2;
      double P2 = p2*pow(T0/t2, gama/(gama-1.0));
      double mach = aux.MachFromImpactPressure(P2-p1, psl);

      TS_ASSERT_DELTA(aux.VcalibratedFromMach(M1, p1)/(mach*asl), 1.0, 1e-8);
    }

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  // Test Mach number at zero velocity
  void testMachZeroVelocity() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    double p = atm->GetPressureSL();

    // Zero impact pressure should give zero Mach
    TS_ASSERT_DELTA(aux.MachFromImpactPressure(0.0, p), 0.0, epsilon);
    TS_ASSERT_DELTA(aux.VcalibratedFromMach(0.0, p), 0.0, epsilon);

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  // Test transonic region (Mach 0.8-1.2)
  void testTransonicMach() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;
    aux.in.StdDaySLsoundspeed = atm->StdDaySLsoundspeed;

    double p = atm->GetPressureSL();

    // Test transonic region - critical for compressibility effects
    for (double M = 0.8; M <= 1.2; M += 0.05) {
      double vcas = aux.VcalibratedFromMach(M, p);
      double M_back = aux.MachFromVcalibrated(vcas, p);
      TS_ASSERT_DELTA(M_back, M, 1e-6);
    }

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  // Test hypersonic Mach (M > 5)
  void testHypersonicMach() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;
    aux.in.StdDaySLsoundspeed = atm->StdDaySLsoundspeed;

    double p = atm->GetPressureSL();

    // Test hypersonic region
    for (double M = 5.0; M <= 10.0; M += 1.0) {
      double vcas = aux.VcalibratedFromMach(M, p);
      TS_ASSERT(vcas > 0.0);  // Should give valid result
      TS_ASSERT(!std::isnan(vcas));
      TS_ASSERT(!std::isinf(vcas));
    }

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  // Test dynamic pressure calculation (qbar = 0.5 * rho * V^2)
  void testDynamicPressure() {
    auto aux = fdmex.GetAuxiliary();

    // Set up inputs for dynamic pressure calculation
    double rho = atm->GetDensitySL();
    aux->in.Density = rho;

    // With zero velocity, qbar should be zero
    aux->in.vUVW = FGColumnVector3(0.0, 0.0, 0.0);
    aux->Run(false);
    TS_ASSERT_DELTA(aux->Getqbar(), 0.0, epsilon);
  }

  // Test qbar with significant velocity
  void testDynamicPressureWithVelocity() {
    auto aux = fdmex.GetAuxiliary();

    double rho = atm->GetDensitySL();
    double V = 500.0;  // 500 fps

    aux->in.Density = rho;
    aux->in.vUVW = FGColumnVector3(V, 0.0, 0.0);
    aux->in.SoundSpeed = atm->GetSoundSpeedSL();
    aux->in.Pressure = atm->GetPressureSL();
    aux->in.Temperature = atm->GetTemperatureSL();

    aux->Run(false);

    // Expected qbar = 0.5 * rho * V^2
    double expected_qbar = 0.5 * rho * V * V;
    TS_ASSERT_DELTA(aux->Getqbar(), expected_qbar, 1.0);  // Allow 1 psf tolerance
  }

  // Test angle of attack (alpha) at zero
  void testAlphaZero() {
    auto aux = fdmex.GetAuxiliary();

    // Pure forward flight - no angle of attack
    aux->in.vUVW = FGColumnVector3(500.0, 0.0, 0.0);
    aux->in.Density = atm->GetDensitySL();
    aux->in.SoundSpeed = atm->GetSoundSpeedSL();
    aux->in.Pressure = atm->GetPressureSL();
    aux->in.Temperature = atm->GetTemperatureSL();

    aux->Run(false);

    TS_ASSERT_DELTA(aux->Getalpha(), 0.0, 1e-10);
    TS_ASSERT_DELTA(aux->Getalpha(FGJSBBase::inDegrees), 0.0, 1e-10);
  }

  // Test sideslip angle (beta) at zero
  void testBetaZero() {
    auto aux = fdmex.GetAuxiliary();

    // Pure forward flight - no sideslip
    aux->in.vUVW = FGColumnVector3(500.0, 0.0, 0.0);
    aux->in.Density = atm->GetDensitySL();
    aux->in.SoundSpeed = atm->GetSoundSpeedSL();
    aux->in.Pressure = atm->GetPressureSL();
    aux->in.Temperature = atm->GetTemperatureSL();

    aux->Run(false);

    TS_ASSERT_DELTA(aux->Getbeta(), 0.0, 1e-10);
    TS_ASSERT_DELTA(aux->Getbeta(FGJSBBase::inDegrees), 0.0, 1e-10);
  }

  // Test true airspeed getter
  void testTrueAirspeed() {
    auto aux = fdmex.GetAuxiliary();

    double V = 300.0;  // fps
    aux->in.vUVW = FGColumnVector3(V, 0.0, 0.0);
    aux->in.Density = atm->GetDensitySL();
    aux->in.SoundSpeed = atm->GetSoundSpeedSL();
    aux->in.Pressure = atm->GetPressureSL();
    aux->in.Temperature = atm->GetTemperatureSL();

    aux->Run(false);

    TS_ASSERT_DELTA(aux->GetVtrueFPS(), V, epsilon);
    // Convert to knots: 1 fps = 0.592484 kts
    TS_ASSERT_DELTA(aux->GetVtrueKTS(), V * 0.592483801, 0.001);
  }

  // Test calibrated airspeed getters
  void testCalibratedAirspeedGetters() {
    auto aux = fdmex.GetAuxiliary();

    // At sea level standard day, VCAS should equal VTAS at low speeds
    double V = 200.0;  // fps
    aux->in.vUVW = FGColumnVector3(V, 0.0, 0.0);
    aux->in.Density = atm->GetDensitySL();
    aux->in.SoundSpeed = atm->GetSoundSpeedSL();
    aux->in.Pressure = atm->GetPressureSL();
    aux->in.Temperature = atm->GetTemperatureSL();
    aux->in.StdDaySLsoundspeed = atm->StdDaySLsoundspeed;

    aux->Run(false);

    double vcasFPS = aux->GetVcalibratedFPS();
    double vcasKTS = aux->GetVcalibratedKTS();

    TS_ASSERT(!std::isnan(vcasFPS));
    TS_ASSERT(!std::isnan(vcasKTS));
    // At sea level, VCAS should be close to VTAS
    TS_ASSERT_DELTA(vcasFPS, V, 5.0);  // Allow 5 fps tolerance
  }

  // Test equivalent airspeed getters
  void testEquivalentAirspeedGetters() {
    auto aux = fdmex.GetAuxiliary();

    double V = 300.0;  // fps
    aux->in.vUVW = FGColumnVector3(V, 0.0, 0.0);
    aux->in.Density = atm->GetDensitySL();
    aux->in.SoundSpeed = atm->GetSoundSpeedSL();
    aux->in.Pressure = atm->GetPressureSL();
    aux->in.Temperature = atm->GetTemperatureSL();

    aux->Run(false);

    double veasFPS = aux->GetVequivalentFPS();
    double veasKTS = aux->GetVequivalentKTS();

    TS_ASSERT(!std::isnan(veasFPS));
    TS_ASSERT(!std::isnan(veasKTS));
    // At sea level, VEAS should equal VTAS
    TS_ASSERT_DELTA(veasFPS, V, 1.0);
  }

  // Test total temperature
  void testTotalTemperature() {
    auto aux = fdmex.GetAuxiliary();

    double T = atm->GetTemperatureSL();  // Rankine
    double V = 500.0;  // fps
    double a = atm->GetSoundSpeedSL();
    double M = V / a;

    aux->in.vUVW = FGColumnVector3(V, 0.0, 0.0);
    aux->in.Density = atm->GetDensitySL();
    aux->in.SoundSpeed = a;
    aux->in.Pressure = atm->GetPressureSL();
    aux->in.Temperature = T;

    aux->Run(false);

    // Total temperature = T * (1 + 0.2 * M^2)
    double expectedTat = T * (1.0 + 0.2 * M * M);
    TS_ASSERT_DELTA(aux->GetTotalTemperature(), expectedTat, 1.0);  // 1 R tolerance
  }

  // Test pilot acceleration getters
  void testPilotAcceleration() {
    auto aux = fdmex.GetAuxiliary();

    // Set up some accelerations
    aux->in.vBodyAccel = FGColumnVector3(10.0, 5.0, 32.174);  // ~1g down
    aux->in.StandardGravity = 32.174;
    aux->in.ToEyePt = FGColumnVector3(0.0, 0.0, 0.0);
    aux->in.vPQR = FGColumnVector3(0.0, 0.0, 0.0);
    aux->in.vPQRidot = FGColumnVector3(0.0, 0.0, 0.0);

    aux->Run(false);

    // Get pilot accelerations
    FGColumnVector3 pilotAccel = aux->GetPilotAccel();
    FGColumnVector3 npilot = aux->GetNpilot();

    TS_ASSERT(!std::isnan(pilotAccel(1)));
    TS_ASSERT(!std::isnan(pilotAccel(2)));
    TS_ASSERT(!std::isnan(pilotAccel(3)));
    TS_ASSERT(!std::isnan(npilot(1)));
    TS_ASSERT(!std::isnan(npilot(2)));
    TS_ASSERT(!std::isnan(npilot(3)));
  }

  // Test CG acceleration getters (Nx, Ny, Nz)
  void testCGAcceleration() {
    auto aux = fdmex.GetAuxiliary();

    aux->in.vBodyAccel = FGColumnVector3(0.0, 0.0, 32.174);  // 1g down
    aux->in.StandardGravity = 32.174;
    aux->in.Tl2b = FGMatrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);

    aux->Run(false);

    double Nx = aux->GetNx();
    double Ny = aux->GetNy();
    double Nz = aux->GetNz();

    TS_ASSERT(!std::isnan(Nx));
    TS_ASSERT(!std::isnan(Ny));
    TS_ASSERT(!std::isnan(Nz));
  }

  // Test flight path angle (gamma)
  void testFlightPathAngle() {
    auto aux = fdmex.GetAuxiliary();

    // Level flight
    aux->in.vUVW = FGColumnVector3(500.0, 0.0, 0.0);
    aux->in.Tb2l = FGMatrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    aux->in.vVel = FGColumnVector3(500.0, 0.0, 0.0);  // NED velocity
    aux->in.Density = atm->GetDensitySL();
    aux->in.SoundSpeed = atm->GetSoundSpeedSL();

    aux->Run(false);

    double gamma = aux->GetGamma();
    double gammaDeg = aux->GetGamma(FGJSBBase::inDegrees);

    TS_ASSERT(!std::isnan(gamma));
    TS_ASSERT(!std::isnan(gammaDeg));
    // For level flight, gamma should be near zero
    TS_ASSERT_DELTA(gamma, 0.0, 0.1);  // Allow 0.1 rad tolerance
  }

  // Test Reynolds number
  void testReynoldsNumber() {
    auto aux = fdmex.GetAuxiliary();

    double V = 500.0;  // fps
    double chord = 5.0;  // ft
    double nu = 1.5e-4;  // kinematic viscosity (approx)

    aux->in.vUVW = FGColumnVector3(V, 0.0, 0.0);
    aux->in.Wingchord = chord;
    aux->in.KinematicViscosity = nu;
    aux->in.Density = atm->GetDensitySL();
    aux->in.SoundSpeed = atm->GetSoundSpeedSL();

    aux->Run(false);

    double Re = aux->GetReynoldsNumber();
    // Re = V * c / nu
    double expectedRe = V * chord / nu;
    TS_ASSERT_DELTA(Re, expectedRe, expectedRe * 0.01);  // 1% tolerance
  }

  // Test wind-to-body transformation matrix
  void testWindToBodyMatrix() {
    auto aux = fdmex.GetAuxiliary();

    aux->in.vUVW = FGColumnVector3(500.0, 0.0, 0.0);
    aux->in.Density = atm->GetDensitySL();
    aux->in.SoundSpeed = atm->GetSoundSpeedSL();

    aux->Run(false);

    const FGMatrix33& Tw2b = aux->GetTw2b();
    const FGMatrix33& Tb2w = aux->GetTb2w();

    // At zero alpha/beta, these should be identity
    TS_ASSERT_DELTA(Tw2b(1,1), 1.0, 1e-6);
    TS_ASSERT_DELTA(Tw2b(2,2), 1.0, 1e-6);
    TS_ASSERT_DELTA(Tw2b(3,3), 1.0, 1e-6);

    // Tw2b and Tb2w should be inverses
    FGMatrix33 product = Tw2b * Tb2w;
    TS_ASSERT_DELTA(product(1,1), 1.0, 1e-6);
    TS_ASSERT_DELTA(product(2,2), 1.0, 1e-6);
    TS_ASSERT_DELTA(product(3,3), 1.0, 1e-6);
  }

  // Test Mach at very low altitude (high pressure)
  void testMachLowAltitude() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;
    aux.in.StdDaySLsoundspeed = atm->StdDaySLsoundspeed;

    // Sea level pressure (high)
    double p = atm->GetPressureSL();

    for (double M = 0.1; M <= 2.0; M += 0.1) {
      double vcas = aux.VcalibratedFromMach(M, p);
      double M_back = aux.MachFromVcalibrated(vcas, p);
      TS_ASSERT_DELTA(M_back, M, 1e-6);
    }

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  // Test Mach at high altitude (low pressure)
  void testMachHighAltitude() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;
    aux.in.StdDaySLsoundspeed = atm->StdDaySLsoundspeed;

    // Pressure at 40,000 ft (approximately 393 psf)
    double p = atm->GetPressure(40000.);

    for (double M = 0.1; M <= 2.0; M += 0.1) {
      double vcas = aux.VcalibratedFromMach(M, p);
      double M_back = aux.MachFromVcalibrated(vcas, p);
      TS_ASSERT_DELTA(M_back, M, 1e-6);
    }

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  // Test height over bar calculations
  void testHeightOverBar() {
    auto aux = fdmex.GetAuxiliary();

    aux->in.Wingspan = 35.0;  // ft
    aux->in.DistanceAGL = 100.0;  // ft

    aux->Run(false);

    double hOverB = aux->GetHOverBCG();
    TS_ASSERT(!std::isnan(hOverB));
    // h/b = AGL / wingspan
    TS_ASSERT_DELTA(hOverB, 100.0 / 35.0, 0.1);
  }

  // Test Mach calculation at various speeds
  void testMachCalculation() {
    auto aux = fdmex.GetAuxiliary();

    double a = atm->GetSoundSpeedSL();  // Speed of sound at sea level

    // Test subsonic
    double V_subsonic = 0.5 * a;
    aux->in.vUVW = FGColumnVector3(V_subsonic, 0.0, 0.0);
    aux->in.SoundSpeed = a;
    aux->in.Density = atm->GetDensitySL();
    aux->in.Pressure = atm->GetPressureSL();
    aux->in.Temperature = atm->GetTemperatureSL();

    aux->Run(false);
    TS_ASSERT_DELTA(aux->GetMach(), 0.5, 1e-6);

    // Test sonic
    aux->in.vUVW = FGColumnVector3(a, 0.0, 0.0);
    aux->Run(false);
    TS_ASSERT_DELTA(aux->GetMach(), 1.0, 1e-6);

    // Test supersonic
    aux->in.vUVW = FGColumnVector3(2.0 * a, 0.0, 0.0);
    aux->Run(false);
    TS_ASSERT_DELTA(aux->GetMach(), 2.0, 1e-6);
  }

  // Test dynamic pressure (qbar) at various speeds
  void testQbarVariousSpeeds() {
    auto aux = fdmex.GetAuxiliary();
    double rho = atm->GetDensitySL();

    // Test at different speeds
    for (double V = 100.0; V <= 1000.0; V += 100.0) {
      aux->in.vUVW = FGColumnVector3(V, 0.0, 0.0);
      aux->in.Density = rho;
      aux->in.SoundSpeed = atm->GetSoundSpeedSL();
      aux->in.Pressure = atm->GetPressureSL();
      aux->in.Temperature = atm->GetTemperatureSL();

      aux->Run(false);

      double expected_qbar = 0.5 * rho * V * V;
      TS_ASSERT_DELTA(aux->Getqbar(), expected_qbar, 1.0);
    }
  }

  // Test angle of attack with non-zero vertical velocity
  void testAlphaWithVerticalVelocity() {
    auto aux = fdmex.GetAuxiliary();

    double u = 500.0;  // Forward velocity
    double w = 50.0;   // Downward velocity

    aux->in.vUVW = FGColumnVector3(u, 0.0, w);
    aux->in.Density = atm->GetDensitySL();
    aux->in.SoundSpeed = atm->GetSoundSpeedSL();
    aux->in.Pressure = atm->GetPressureSL();
    aux->in.Temperature = atm->GetTemperatureSL();

    aux->Run(false);

    double expected_alpha = atan2(w, u);
    TS_ASSERT_DELTA(aux->Getalpha(), expected_alpha, 1e-6);
    TS_ASSERT_DELTA(aux->Getalpha(FGJSBBase::inDegrees), expected_alpha * radtodeg, 1e-6);
  }

  // Test sideslip angle with lateral velocity
  void testBetaWithLateralVelocity() {
    auto aux = fdmex.GetAuxiliary();

    double u = 500.0;  // Forward velocity
    double v = 50.0;   // Lateral velocity

    aux->in.vUVW = FGColumnVector3(u, v, 0.0);
    aux->in.Density = atm->GetDensitySL();
    aux->in.SoundSpeed = atm->GetSoundSpeedSL();
    aux->in.Pressure = atm->GetPressureSL();
    aux->in.Temperature = atm->GetTemperatureSL();

    aux->Run(false);

    double expected_beta = atan2(v, u);
    TS_ASSERT_DELTA(aux->Getbeta(), expected_beta, 1e-6);
    TS_ASSERT_DELTA(aux->Getbeta(FGJSBBase::inDegrees), expected_beta * radtodeg, 1e-6);
  }

  // Test combined alpha and beta
  void testAlphaBetaCombined() {
    auto aux = fdmex.GetAuxiliary();

    double u = 400.0;
    double v = 40.0;
    double w = 80.0;

    aux->in.vUVW = FGColumnVector3(u, v, w);
    aux->in.Density = atm->GetDensitySL();
    aux->in.SoundSpeed = atm->GetSoundSpeedSL();
    aux->in.Pressure = atm->GetPressureSL();
    aux->in.Temperature = atm->GetTemperatureSL();

    aux->Run(false);

    double muw = u*u + w*w;
    double expected_alpha = atan2(w, u);
    double expected_beta = atan2(v, sqrt(muw));

    TS_ASSERT_DELTA(aux->Getalpha(), expected_alpha, 1e-6);
    TS_ASSERT_DELTA(aux->Getbeta(), expected_beta, 1e-6);
  }

  // Test total pressure at subsonic and supersonic speeds
  void testTotalPressureSubsonicSupersonic() {
    auto aux = fdmex.GetAuxiliary();

    double p = atm->GetPressureSL();
    double T = atm->GetTemperatureSL();
    double a = atm->GetSoundSpeedSL();

    // Subsonic - M = 0.5
    double V_sub = 0.5 * a;
    aux->in.vUVW = FGColumnVector3(V_sub, 0.0, 0.0);
    aux->in.Pressure = p;
    aux->in.Temperature = T;
    aux->in.SoundSpeed = a;
    aux->in.Density = atm->GetDensitySL();

    aux->Run(false);
    double pt_sub = aux->GetTotalPressure();
    TS_ASSERT(pt_sub > p);  // Total pressure should be greater than static

    // Supersonic - M = 2.0
    double V_sup = 2.0 * a;
    aux->in.vUVW = FGColumnVector3(V_sup, 0.0, 0.0);

    aux->Run(false);
    double pt_sup = aux->GetTotalPressure();
    TS_ASSERT(pt_sup > pt_sub);  // Supersonic total pressure should be higher
  }

  // Test total temperature at various Mach numbers
  void testTotalTemperatureVariousMach() {
    auto aux = fdmex.GetAuxiliary();

    double T = atm->GetTemperatureSL();
    double a = atm->GetSoundSpeedSL();
    double p = atm->GetPressureSL();
    double rho = atm->GetDensitySL();

    for (double M = 0.0; M <= 3.0; M += 0.5) {
      double V = M * a;
      aux->in.vUVW = FGColumnVector3(V, 0.0, 0.0);
      aux->in.Temperature = T;
      aux->in.SoundSpeed = a;
      aux->in.Pressure = p;
      aux->in.Density = rho;

      aux->Run(false);

      double expected_tat = T * (1.0 + 0.2 * M * M);
      TS_ASSERT_DELTA(aux->GetTotalTemperature(), expected_tat, 1.0);
    }
  }

  // Test pilot acceleration with rotation
  void testPilotAccelWithRotation() {
    auto aux = fdmex.GetAuxiliary();

    // Set up rotation
    aux->in.vPQR = FGColumnVector3(0.1, 0.05, 0.02);  // rad/s
    aux->in.vPQRidot = FGColumnVector3(0.01, 0.01, 0.01);  // rad/s^2
    aux->in.ToEyePt = FGColumnVector3(15.0, 0.0, -3.0);  // ft from CG to pilot
    aux->in.vBodyAccel = FGColumnVector3(5.0, 2.0, 32.174);
    aux->in.StandardGravity = 32.174;
    aux->in.vPQRi = aux->in.vPQR;  // Assume inertial = body for this test

    aux->Run(false);

    FGColumnVector3 pilotAccel = aux->GetPilotAccel();

    // Pilot acceleration should include centripetal and angular acceleration effects
    TS_ASSERT(!std::isnan(pilotAccel(1)));
    TS_ASSERT(!std::isnan(pilotAccel(2)));
    TS_ASSERT(!std::isnan(pilotAccel(3)));

    // Should be different from CG acceleration due to rotation
    TS_ASSERT(fabs(pilotAccel(1) - aux->in.vBodyAccel(1)) > 0.001 ||
              fabs(pilotAccel(2) - aux->in.vBodyAccel(2)) > 0.001 ||
              fabs(pilotAccel(3) - aux->in.vBodyAccel(3)) > 0.001);
  }

  // Test load factors Nx, Ny, Nz in level flight
  void testLoadFactorsLevelFlight() {
    auto aux = fdmex.GetAuxiliary();

    // Level flight at constant speed - only gravity acting
    aux->in.vBodyAccel = FGColumnVector3(0.0, 0.0, 32.174);  // 1g upward
    aux->in.StandardGravity = 32.174;
    aux->in.Tl2b = FGMatrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);

    aux->Run(false);

    // In level flight with 1g lift
    TS_ASSERT_DELTA(aux->GetNx(), 0.0, 0.01);
    TS_ASSERT_DELTA(aux->GetNy(), 0.0, 0.01);
    TS_ASSERT_DELTA(aux->GetNz(), -1.0, 0.01);  // -1g (note sign convention)
  }

  // Test load factors in accelerated flight
  void testLoadFactorsAccelerated() {
    auto aux = fdmex.GetAuxiliary();

    // Forward acceleration + gravity
    aux->in.vBodyAccel = FGColumnVector3(32.174, 0.0, 32.174);  // 1g forward, 1g up
    aux->in.StandardGravity = 32.174;
    aux->in.Tl2b = FGMatrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);

    aux->Run(false);

    TS_ASSERT_DELTA(aux->GetNx(), 1.0, 0.01);
    TS_ASSERT_DELTA(aux->GetNy(), 0.0, 0.01);
    TS_ASSERT_DELTA(aux->GetNz(), -1.0, 0.01);
  }

  // Test GetVcalibratedFPS and GetVcalibratedKTS consistency
  void testCalibratedAirspeedConversion() {
    auto aux = fdmex.GetAuxiliary();

    double V = 300.0;  // fps
    aux->in.vUVW = FGColumnVector3(V, 0.0, 0.0);
    aux->in.Density = atm->GetDensitySL();
    aux->in.SoundSpeed = atm->GetSoundSpeedSL();
    aux->in.Pressure = atm->GetPressureSL();
    aux->in.Temperature = atm->GetTemperatureSL();
    aux->in.StdDaySLsoundspeed = atm->StdDaySLsoundspeed;

    aux->Run(false);

    double vcas_fps = aux->GetVcalibratedFPS();
    double vcas_kts = aux->GetVcalibratedKTS();

    // Verify conversion: 1 fps = 0.592484 kts
    TS_ASSERT_DELTA(vcas_kts, vcas_fps * 0.592483801, 0.01);
  }

  // Test GetVequivalentFPS and GetVequivalentKTS consistency
  void testEquivalentAirspeedConversion() {
    auto aux = fdmex.GetAuxiliary();

    double V = 400.0;  // fps
    aux->in.vUVW = FGColumnVector3(V, 0.0, 0.0);
    aux->in.Density = atm->GetDensitySL();
    aux->in.SoundSpeed = atm->GetSoundSpeedSL();
    aux->in.Pressure = atm->GetPressureSL();
    aux->in.Temperature = atm->GetTemperatureSL();

    aux->Run(false);

    double veas_fps = aux->GetVequivalentFPS();
    double veas_kts = aux->GetVequivalentKTS();

    // Verify conversion: 1 fps = 0.592484 kts
    TS_ASSERT_DELTA(veas_kts, veas_fps * 0.592483801, 0.01);
  }

  // Test equivalent airspeed at altitude
  void testEquivalentAirspeedAtAltitude() {
    auto aux = fdmex.GetAuxiliary();

    // At 10,000 ft altitude
    double rho_alt = atm->GetDensity(10000.0);
    double rho_sl = atm->GetDensitySL();
    double V = 500.0;  // fps

    aux->in.vUVW = FGColumnVector3(V, 0.0, 0.0);
    aux->in.Density = rho_alt;
    aux->in.SoundSpeed = atm->GetSoundSpeed(10000.0);
    aux->in.Pressure = atm->GetPressure(10000.0);
    aux->in.Temperature = atm->GetTemperature(10000.0);

    aux->Run(false);

    // VEAS = VTAS * sqrt(rho / rho_sl)
    double expected_veas = V * sqrt(rho_alt / rho_sl);
    TS_ASSERT_DELTA(aux->GetVequivalentFPS(), expected_veas, 2.0);
  }

  // Test flight path angle in climb
  void testFlightPathAngleClimb() {
    auto aux = fdmex.GetAuxiliary();

    // Climbing flight
    aux->in.vUVW = FGColumnVector3(500.0, 0.0, -50.0);  // Climbing (negative w)
    aux->in.Tb2l = FGMatrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    aux->in.vVel = FGColumnVector3(500.0, 0.0, -50.0);  // NED velocity (negative down = up)
    aux->in.Density = atm->GetDensitySL();
    aux->in.SoundSpeed = atm->GetSoundSpeedSL();

    aux->Run(false);

    // Gamma should be positive for climb
    TS_ASSERT(aux->GetGamma() > 0.0);
    TS_ASSERT(aux->GetGamma(FGJSBBase::inDegrees) > 0.0);
  }

  // Test flight path angle in descent
  void testFlightPathAngleDescent() {
    auto aux = fdmex.GetAuxiliary();

    // Descending flight
    aux->in.vUVW = FGColumnVector3(500.0, 0.0, 0.0);
    aux->in.Tb2l = FGMatrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    aux->in.vVel = FGColumnVector3(500.0, 0.0, 50.0);  // NED velocity (positive down)
    aux->in.Density = atm->GetDensitySL();
    aux->in.SoundSpeed = atm->GetSoundSpeedSL();

    aux->Run(false);

    // Gamma should be negative for descent
    TS_ASSERT(aux->GetGamma() < 0.0);
    TS_ASSERT(aux->GetGamma(FGJSBBase::inDegrees) < 0.0);
  }

  // Test qbarUW (dynamic pressure in u-w plane)
  void testQbarUW() {
    auto aux = fdmex.GetAuxiliary();

    double rho = atm->GetDensitySL();
    double u = 400.0;
    double w = 100.0;

    aux->in.vUVW = FGColumnVector3(u, 0.0, w);
    aux->in.Density = rho;
    aux->in.SoundSpeed = atm->GetSoundSpeedSL();
    aux->in.Pressure = atm->GetPressureSL();
    aux->in.Temperature = atm->GetTemperatureSL();

    aux->Run(false);

    double expected_qbarUW = 0.5 * rho * (u*u + w*w);
    TS_ASSERT_DELTA(aux->GetqbarUW(), expected_qbarUW, 1.0);
  }

  // Test qbarUV (dynamic pressure in u-v plane)
  void testQbarUV() {
    auto aux = fdmex.GetAuxiliary();

    double rho = atm->GetDensitySL();
    double u = 400.0;
    double v = 80.0;

    aux->in.vUVW = FGColumnVector3(u, v, 0.0);
    aux->in.Density = rho;
    aux->in.SoundSpeed = atm->GetSoundSpeedSL();
    aux->in.Pressure = atm->GetPressureSL();
    aux->in.Temperature = atm->GetTemperatureSL();

    aux->Run(false);

    double expected_qbarUV = 0.5 * rho * (u*u + v*v);
    TS_ASSERT_DELTA(aux->GetqbarUV(), expected_qbarUV, 1.0);
  }

  // Test MachU (Mach number based on u component)
  void testMachU() {
    auto aux = fdmex.GetAuxiliary();

    double a = atm->GetSoundSpeedSL();
    double u = 0.8 * a;

    aux->in.vUVW = FGColumnVector3(u, 50.0, 30.0);  // Non-zero v and w
    aux->in.SoundSpeed = a;
    aux->in.Density = atm->GetDensitySL();
    aux->in.Pressure = atm->GetPressureSL();
    aux->in.Temperature = atm->GetTemperatureSL();

    aux->Run(false);

    double expected_MachU = u / a;
    TS_ASSERT_DELTA(aux->GetMachU(), expected_MachU, 1e-6);
  }

  // Test ground speed calculation
  void testGroundSpeed() {
    auto aux = fdmex.GetAuxiliary();

    double v_north = 300.0;
    double v_east = 400.0;

    aux->in.vVel = FGColumnVector3(v_north, v_east, -50.0);  // NED
    aux->in.vUVW = FGColumnVector3(500.0, 0.0, 0.0);
    aux->in.Density = atm->GetDensitySL();
    aux->in.SoundSpeed = atm->GetSoundSpeedSL();

    aux->Run(false);

    double expected_Vground = sqrt(v_north*v_north + v_east*v_east);
    TS_ASSERT_DELTA(aux->GetVground(), expected_Vground, 0.1);
  }

  // Test ground track angle
  void testGroundTrack() {
    auto aux = fdmex.GetAuxiliary();

    // Northeast heading
    aux->in.vVel = FGColumnVector3(300.0, 300.0, 0.0);  // NED - 45 degrees
    aux->in.vUVW = FGColumnVector3(400.0, 0.0, 0.0);
    aux->in.Density = atm->GetDensitySL();
    aux->in.SoundSpeed = atm->GetSoundSpeedSL();

    aux->Run(false);

    double psi_gt = aux->GetGroundTrack();

    // Should be 45 degrees (pi/4 radians)
    TS_ASSERT_DELTA(psi_gt, M_PI / 4.0, 0.01);
  }

  // Test wind-to-body transformation with non-zero alpha
  void testWindToBodyTransformWithAlpha() {
    auto aux = fdmex.GetAuxiliary();

    // Create angle of attack
    double u = 500.0;
    double w = 100.0;  // About 11 degrees alpha

    aux->in.vUVW = FGColumnVector3(u, 0.0, w);
    aux->in.Density = atm->GetDensitySL();
    aux->in.SoundSpeed = atm->GetSoundSpeedSL();

    aux->Run(false);

    const FGMatrix33& Tw2b = aux->GetTw2b();
    const FGMatrix33& Tb2w = aux->GetTb2w();

    // Verify they are inverses (Tw2b * Tb2w = I)
    FGMatrix33 product = Tw2b * Tb2w;
    TS_ASSERT_DELTA(product(1,1), 1.0, 1e-6);
    TS_ASSERT_DELTA(product(2,2), 1.0, 1e-6);
    TS_ASSERT_DELTA(product(3,3), 1.0, 1e-6);
    TS_ASSERT_DELTA(product(1,2), 0.0, 1e-6);
    TS_ASSERT_DELTA(product(1,3), 0.0, 1e-6);
    TS_ASSERT_DELTA(product(2,1), 0.0, 1e-6);
    TS_ASSERT_DELTA(product(2,3), 0.0, 1e-6);
    TS_ASSERT_DELTA(product(3,1), 0.0, 1e-6);
    TS_ASSERT_DELTA(product(3,2), 0.0, 1e-6);
  }

  // Test wind-to-body transformation with non-zero beta
  void testWindToBodyTransformWithBeta() {
    auto aux = fdmex.GetAuxiliary();

    // Create sideslip angle
    double u = 500.0;
    double v = 80.0;  // About 9 degrees beta

    aux->in.vUVW = FGColumnVector3(u, v, 0.0);
    aux->in.Density = atm->GetDensitySL();
    aux->in.SoundSpeed = atm->GetSoundSpeedSL();

    aux->Run(false);

    const FGMatrix33& Tw2b = aux->GetTw2b();
    const FGMatrix33& Tb2w = aux->GetTb2w();

    // Verify orthogonality (transpose = inverse)
    FGMatrix33 product = Tw2b * Tb2w;
    FGMatrix33 identity(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);

    for (int i = 1; i <= 3; i++) {
      for (int j = 1; j <= 3; j++) {
        TS_ASSERT_DELTA(product(i,j), identity(i,j), 1e-6);
      }
    }
  }

  // Test Reynolds number calculation
  void testReynoldsNumberVariousConditions() {
    auto aux = fdmex.GetAuxiliary();

    double chord = 6.0;  // ft
    double nu = 1.5e-4;  // ft^2/s (approximate at sea level)

    // Test at different velocities
    for (double V = 100.0; V <= 800.0; V += 100.0) {
      aux->in.vUVW = FGColumnVector3(V, 0.0, 0.0);
      aux->in.Wingchord = chord;
      aux->in.KinematicViscosity = nu;
      aux->in.Density = atm->GetDensitySL();
      aux->in.SoundSpeed = atm->GetSoundSpeedSL();

      aux->Run(false);

      double expected_Re = V * chord / nu;
      TS_ASSERT_DELTA(aux->GetReynoldsNumber(), expected_Re, expected_Re * 0.01);
    }
  }

  // Test GetMagBeta (magnitude of beta)
  void testMagBeta() {
    auto aux = fdmex.GetAuxiliary();

    // Positive beta
    aux->in.vUVW = FGColumnVector3(500.0, 50.0, 0.0);
    aux->in.Density = atm->GetDensitySL();
    aux->in.SoundSpeed = atm->GetSoundSpeedSL();

    aux->Run(false);
    double beta = aux->Getbeta();
    TS_ASSERT_DELTA(aux->GetMagBeta(), fabs(beta), 1e-10);

    // Negative beta
    aux->in.vUVW = FGColumnVector3(500.0, -50.0, 0.0);
    aux->Run(false);
    beta = aux->Getbeta();
    TS_ASSERT_DELTA(aux->GetMagBeta(), fabs(beta), 1e-10);
    TS_ASSERT(aux->GetMagBeta() >= 0.0);  // Magnitude is always positive
  }

  // Test alpha and beta dot (rates of change)
  void testAlphaBetaDot() {
    auto aux = fdmex.GetAuxiliary();

    // Set up conditions with changing velocities
    aux->in.vUVW = FGColumnVector3(500.0, 50.0, 80.0);
    aux->in.vUVWdot = FGColumnVector3(10.0, 5.0, 15.0);  // Accelerations
    aux->in.Density = atm->GetDensitySL();
    aux->in.SoundSpeed = atm->GetSoundSpeedSL();
    aux->in.Pressure = atm->GetPressureSL();
    aux->in.Temperature = atm->GetTemperatureSL();

    aux->Run(false);

    double adot = aux->Getadot();
    double bdot = aux->Getbdot();

    // Should be non-zero with acceleration
    TS_ASSERT(!std::isnan(adot));
    TS_ASSERT(!std::isnan(bdot));

    // Test unit conversion
    double adot_deg = aux->Getadot(FGJSBBase::inDegrees);
    double bdot_deg = aux->Getbdot(FGJSBBase::inDegrees);
    TS_ASSERT_DELTA(adot_deg, adot * radtodeg, 1e-6);
    TS_ASSERT_DELTA(bdot_deg, bdot * radtodeg, 1e-6);
  }

  // Test GetNcg (load factor vector at CG)
  void testNcgVector() {
    auto aux = fdmex.GetAuxiliary();

    aux->in.vBodyAccel = FGColumnVector3(16.087, 8.0435, 32.174);  // 0.5g, 0.25g, 1g
    aux->in.StandardGravity = 32.174;
    aux->in.Tl2b = FGMatrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);

    aux->Run(false);

    const FGColumnVector3& Ncg = aux->GetNcg();

    TS_ASSERT_DELTA(Ncg(1), 0.5, 0.01);
    TS_ASSERT_DELTA(Ncg(2), 0.25, 0.01);
    TS_ASSERT_DELTA(Ncg(3), 1.0, 0.01);

    // Test indexed getter
    TS_ASSERT_DELTA(aux->GetNcg(1), 0.5, 0.01);
    TS_ASSERT_DELTA(aux->GetNcg(2), 0.25, 0.01);
    TS_ASSERT_DELTA(aux->GetNcg(3), 1.0, 0.01);
  }

  // Test GetNpilot with indexed access
  void testNpilotIndexed() {
    auto aux = fdmex.GetAuxiliary();

    aux->in.vBodyAccel = FGColumnVector3(10.0, 5.0, 32.174);
    aux->in.StandardGravity = 32.174;
    aux->in.ToEyePt = FGColumnVector3(0.0, 0.0, 0.0);
    aux->in.vPQR = FGColumnVector3(0.0, 0.0, 0.0);
    aux->in.vPQRidot = FGColumnVector3(0.0, 0.0, 0.0);
    aux->in.vPQRi = FGColumnVector3(0.0, 0.0, 0.0);

    aux->Run(false);

    // Test both vector and indexed getters
    const FGColumnVector3& Npilot_vec = aux->GetNpilot();
    TS_ASSERT_DELTA(aux->GetNpilot(1), Npilot_vec(1), 1e-10);
    TS_ASSERT_DELTA(aux->GetNpilot(2), Npilot_vec(2), 1e-10);
    TS_ASSERT_DELTA(aux->GetNpilot(3), Npilot_vec(3), 1e-10);
  }

  // Test GetPilotAccel with indexed access
  void testPilotAccelIndexed() {
    auto aux = fdmex.GetAuxiliary();

    aux->in.vBodyAccel = FGColumnVector3(15.0, 8.0, 30.0);
    aux->in.StandardGravity = 32.174;
    aux->in.ToEyePt = FGColumnVector3(10.0, 0.0, -2.0);
    aux->in.vPQR = FGColumnVector3(0.05, 0.03, 0.01);
    aux->in.vPQRidot = FGColumnVector3(0.0, 0.0, 0.0);
    aux->in.vPQRi = aux->in.vPQR;

    aux->Run(false);

    // Test both vector and indexed getters
    const FGColumnVector3& pilotAccel_vec = aux->GetPilotAccel();
    TS_ASSERT_DELTA(aux->GetPilotAccel(1), pilotAccel_vec(1), 1e-10);
    TS_ASSERT_DELTA(aux->GetPilotAccel(2), pilotAccel_vec(2), 1e-10);
    TS_ASSERT_DELTA(aux->GetPilotAccel(3), pilotAccel_vec(3), 1e-10);
  }

  // Test GetAeroUVW vector and indexed access
  void testAeroUVW() {
    auto aux = fdmex.GetAuxiliary();

    // Body velocities with wind
    aux->in.vUVW = FGColumnVector3(500.0, 30.0, 50.0);
    aux->in.TotalWindNED = FGColumnVector3(10.0, 5.0, 0.0);  // Wind in NED
    aux->in.Tl2b = FGMatrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    aux->in.Density = atm->GetDensitySL();
    aux->in.SoundSpeed = atm->GetSoundSpeedSL();

    aux->Run(false);

    const FGColumnVector3& aeroUVW = aux->GetAeroUVW();
    TS_ASSERT_DELTA(aux->GetAeroUVW(1), aeroUVW(1), 1e-10);
    TS_ASSERT_DELTA(aux->GetAeroUVW(2), aeroUVW(2), 1e-10);
    TS_ASSERT_DELTA(aux->GetAeroUVW(3), aeroUVW(3), 1e-10);
  }

  // Test GetAeroPQR vector and indexed access
  void testAeroPQRIndexed() {
    auto aux = fdmex.GetAuxiliary();

    aux->in.vPQR = FGColumnVector3(0.1, 0.05, 0.02);
    aux->in.TurbPQR = FGColumnVector3(0.01, 0.005, 0.002);
    aux->in.vUVW = FGColumnVector3(500.0, 0.0, 0.0);
    aux->in.Density = atm->GetDensitySL();
    aux->in.SoundSpeed = atm->GetSoundSpeedSL();

    aux->Run(false);

    const FGColumnVector3& aeroPQR = aux->GetAeroPQR();
    TS_ASSERT_DELTA(aux->GetAeroPQR(1), aeroPQR(1), 1e-10);
    TS_ASSERT_DELTA(aux->GetAeroPQR(2), aeroPQR(2), 1e-10);
    TS_ASSERT_DELTA(aux->GetAeroPQR(3), aeroPQR(3), 1e-10);

    // Aero rotation should be body rotation minus turbulence
    TS_ASSERT_DELTA(aeroPQR(1), 0.09, 1e-6);
    TS_ASSERT_DELTA(aeroPQR(2), 0.045, 1e-6);
    TS_ASSERT_DELTA(aeroPQR(3), 0.018, 1e-6);
  }

  // Test GetEulerRates vector and indexed access
  void testEulerRatesIndexed() {
    auto aux = fdmex.GetAuxiliary();

    aux->in.vPQR = FGColumnVector3(0.1, 0.05, 0.02);
    aux->in.CosPhi = 1.0;
    aux->in.SinPhi = 0.0;
    aux->in.CosTht = 0.9848;  // cos(10 degrees)
    aux->in.SinTht = 0.1736;  // sin(10 degrees)
    aux->in.vUVW = FGColumnVector3(500.0, 0.0, 0.0);
    aux->in.Density = atm->GetDensitySL();
    aux->in.SoundSpeed = atm->GetSoundSpeedSL();

    aux->Run(false);

    const FGColumnVector3& eulerRates = aux->GetEulerRates();
    TS_ASSERT_DELTA(aux->GetEulerRates(1), eulerRates(1), 1e-10);
    TS_ASSERT_DELTA(aux->GetEulerRates(2), eulerRates(2), 1e-10);
    TS_ASSERT_DELTA(aux->GetEulerRates(3), eulerRates(3), 1e-10);
  }

  // Test GetVt (total velocity)
  void testGetVt() {
    auto aux = fdmex.GetAuxiliary();

    double u = 300.0;
    double v = 40.0;
    double w = 60.0;

    aux->in.vUVW = FGColumnVector3(u, v, w);
    aux->in.TotalWindNED = FGColumnVector3(0.0, 0.0, 0.0);  // No wind
    aux->in.Tl2b = FGMatrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    aux->in.Density = atm->GetDensitySL();
    aux->in.SoundSpeed = atm->GetSoundSpeedSL();
    aux->in.Pressure = atm->GetPressureSL();
    aux->in.Temperature = atm->GetTemperatureSL();

    aux->Run(false);

    double expected_Vt = sqrt(u*u + v*v + w*w);
    TS_ASSERT_DELTA(aux->GetVt(), expected_Vt, 0.01);
  }

  // Test HOverBMAC (height over span at MAC)
  void testHOverBMAC() {
    auto aux = fdmex.GetAuxiliary();

    aux->in.Wingspan = 40.0;
    aux->in.DistanceAGL = 120.0;
    aux->in.RPBody = FGColumnVector3(10.0, 0.0, -5.0);  // Reference point
    aux->in.Tb2l = FGMatrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);

    aux->Run(false);

    double hOverBMAC = aux->GetHOverBMAC();
    TS_ASSERT(!std::isnan(hOverBMAC));
    // Should account for vertical offset of reference point
    TS_ASSERT(fabs(hOverBMAC - aux->GetHOverBCG()) > 0.01);
  }

  // Test TAT in Celsius
  void testTATCelsius() {
    auto aux = fdmex.GetAuxiliary();

    double T = atm->GetTemperatureSL();  // Rankine
    double V = 500.0;
    double a = atm->GetSoundSpeedSL();

    aux->in.vUVW = FGColumnVector3(V, 0.0, 0.0);
    aux->in.Temperature = T;
    aux->in.SoundSpeed = a;
    aux->in.Density = atm->GetDensitySL();
    aux->in.Pressure = atm->GetPressureSL();

    aux->Run(false);

    double tat_r = aux->GetTotalTemperature();
    double tat_c = aux->GetTAT_C();

    // Convert Rankine to Celsius: C = (R - 491.67) * 5/9
    double expected_tat_c = (tat_r - 491.67) * 5.0 / 9.0;
    TS_ASSERT_DELTA(tat_c, expected_tat_c, 0.1);
  }

  // Test zero velocity edge case
  void testZeroVelocityEdgeCase() {
    auto aux = fdmex.GetAuxiliary();

    aux->in.vUVW = FGColumnVector3(0.0, 0.0, 0.0);
    aux->in.TotalWindNED = FGColumnVector3(0.0, 0.0, 0.0);  // No wind
    aux->in.Tl2b = FGMatrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    aux->in.Density = atm->GetDensitySL();
    aux->in.SoundSpeed = atm->GetSoundSpeedSL();
    aux->in.Pressure = atm->GetPressureSL();
    aux->in.Temperature = atm->GetTemperatureSL();

    aux->Run(false);

    // At zero velocity, many parameters should be zero
    TS_ASSERT_DELTA(aux->GetVt(), 0.0, 1e-10);
    TS_ASSERT_DELTA(aux->GetMach(), 0.0, 1e-10);
    TS_ASSERT_DELTA(aux->Getqbar(), 0.0, 1e-10);
    TS_ASSERT_DELTA(aux->Getalpha(), 0.0, 1e-10);
    TS_ASSERT_DELTA(aux->Getbeta(), 0.0, 1e-10);
  }

  // Test very low velocity edge case
  void testVeryLowVelocity() {
    auto aux = fdmex.GetAuxiliary();

    // Velocity just above threshold (0.001 fps)
    aux->in.vUVW = FGColumnVector3(0.002, 0.0, 0.0);
    aux->in.Density = atm->GetDensitySL();
    aux->in.SoundSpeed = atm->GetSoundSpeedSL();
    aux->in.Pressure = atm->GetPressureSL();
    aux->in.Temperature = atm->GetTemperatureSL();

    aux->Run(false);

    // Should have valid (non-NaN) results
    TS_ASSERT(!std::isnan(aux->GetMach()));
    TS_ASSERT(!std::isnan(aux->Getalpha()));
    TS_ASSERT(!std::isnan(aux->Getbeta()));
  }

  // ==================== Model Identity Tests ====================

  // Test model name
  void testGetName() {
    auto aux = fdmex.GetAuxiliary();
    std::string name = aux->GetName();
    TS_ASSERT(!name.empty());
  }

  // Test model FDMExec pointer
  void testGetExec() {
    auto aux = fdmex.GetAuxiliary();
    TS_ASSERT(aux->GetExec() == &fdmex);
  }

  // Test rate setting
  void testSetGetRate() {
    auto aux = fdmex.GetAuxiliary();
    int originalRate = aux->GetRate();

    aux->SetRate(2);
    TS_ASSERT_EQUALS(aux->GetRate(), 2);

    aux->SetRate(5);
    TS_ASSERT_EQUALS(aux->GetRate(), 5);

    aux->SetRate(originalRate);
    TS_ASSERT_EQUALS(aux->GetRate(), originalRate);
  }

  // Test InitModel
  void testInitModel() {
    FGFDMExec localFdm;
    auto aux = localFdm.GetAuxiliary();

    // Set some state
    aux->in.vUVW = FGColumnVector3(500.0, 50.0, 30.0);
    aux->in.Density = atm->GetDensitySL();
    aux->in.SoundSpeed = atm->GetSoundSpeedSL();
    aux->Run(false);

    // InitModel should succeed
    TS_ASSERT(aux->InitModel());
  }

  // ==================== Multiple Instance Tests ====================

  // Test multiple FDMExec instances have independent Auxiliary models
  void testMultipleFDMExecInstances() {
    FGFDMExec fdmex1;
    FGFDMExec fdmex2;

    auto aux1 = fdmex1.GetAuxiliary();
    auto aux2 = fdmex2.GetAuxiliary();

    TS_ASSERT(aux1 != aux2);
    TS_ASSERT(aux1->GetExec() == &fdmex1);
    TS_ASSERT(aux2->GetExec() == &fdmex2);

    // Set different inputs
    auto atm1 = fdmex1.GetAtmosphere();
    auto atm2 = fdmex2.GetAtmosphere();
    atm1->InitModel();
    atm2->InitModel();

    aux1->in.vUVW = FGColumnVector3(300.0, 0.0, 0.0);
    aux1->in.Density = atm1->GetDensitySL();
    aux1->in.SoundSpeed = atm1->GetSoundSpeedSL();
    aux1->in.Pressure = atm1->GetPressureSL();
    aux1->in.Temperature = atm1->GetTemperatureSL();

    aux2->in.vUVW = FGColumnVector3(600.0, 0.0, 0.0);
    aux2->in.Density = atm2->GetDensitySL();
    aux2->in.SoundSpeed = atm2->GetSoundSpeedSL();
    aux2->in.Pressure = atm2->GetPressureSL();
    aux2->in.Temperature = atm2->GetTemperatureSL();

    aux1->Run(false);
    aux2->Run(false);

    // Should have different Mach numbers
    TS_ASSERT(fabs(aux1->GetMach() - aux2->GetMach()) > 0.1);
  }

  // Test three independent instances
  void testThreeFDMExecInstances() {
    FGFDMExec fdm1, fdm2, fdm3;
    auto aux1 = fdm1.GetAuxiliary();
    auto aux2 = fdm2.GetAuxiliary();
    auto aux3 = fdm3.GetAuxiliary();

    // All should be distinct
    TS_ASSERT(aux1 != aux2);
    TS_ASSERT(aux2 != aux3);
    TS_ASSERT(aux1 != aux3);

    // Each should point to its own FDM
    TS_ASSERT(aux1->GetExec() == &fdm1);
    TS_ASSERT(aux2->GetExec() == &fdm2);
    TS_ASSERT(aux3->GetExec() == &fdm3);
  }

  // ==================== Altitude Variation Tests ====================

  // Test at troposphere (36000 ft)
  void testMachAtTroposphere() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;
    aux.in.StdDaySLsoundspeed = atm->StdDaySLsoundspeed;

    double p = atm->GetPressure(36000.0);

    for (double M = 0.5; M <= 1.5; M += 0.25) {
      double vcas = aux.VcalibratedFromMach(M, p);
      double M_back = aux.MachFromVcalibrated(vcas, p);
      TS_ASSERT_DELTA(M_back, M, 1e-6);
    }

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  // Test at high altitude (60000 ft)
  void testMachAtHighAltitude60K() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;
    aux.in.StdDaySLsoundspeed = atm->StdDaySLsoundspeed;

    double p = atm->GetPressure(60000.0);

    for (double M = 0.5; M <= 2.5; M += 0.5) {
      double vcas = aux.VcalibratedFromMach(M, p);
      double M_back = aux.MachFromVcalibrated(vcas, p);
      TS_ASSERT_DELTA(M_back, M, 1e-5);
    }

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  // Test equivalent airspeed at high altitude
  void testEquivalentAirspeedHighAltitude() {
    auto aux = fdmex.GetAuxiliary();

    // At 35,000 ft altitude
    double alt = 35000.0;
    double rho_alt = atm->GetDensity(alt);
    double rho_sl = atm->GetDensitySL();
    double V = 700.0;  // fps

    aux->in.vUVW = FGColumnVector3(V, 0.0, 0.0);
    aux->in.Density = rho_alt;
    aux->in.SoundSpeed = atm->GetSoundSpeed(alt);
    aux->in.Pressure = atm->GetPressure(alt);
    aux->in.Temperature = atm->GetTemperature(alt);

    aux->Run(false);

    // VEAS = VTAS * sqrt(rho / rho_sl)
    double expected_veas = V * sqrt(rho_alt / rho_sl);
    TS_ASSERT_DELTA(aux->GetVequivalentFPS(), expected_veas, 5.0);
  }

  // ==================== Additional Angle Tests ====================

  // Test large positive alpha
  void testLargePositiveAlpha() {
    auto aux = fdmex.GetAuxiliary();

    // About 30 degrees alpha
    double u = 400.0;
    double w = 230.9;  // tan(30°) * u

    aux->in.vUVW = FGColumnVector3(u, 0.0, w);
    aux->in.Density = atm->GetDensitySL();
    aux->in.SoundSpeed = atm->GetSoundSpeedSL();
    aux->in.Pressure = atm->GetPressureSL();
    aux->in.Temperature = atm->GetTemperatureSL();

    aux->Run(false);

    double expected_alpha = atan2(w, u);
    TS_ASSERT_DELTA(aux->Getalpha(), expected_alpha, 1e-6);
    TS_ASSERT_DELTA(aux->Getalpha(FGJSBBase::inDegrees), 30.0, 0.1);
  }

  // Test negative alpha
  void testNegativeAlpha() {
    auto aux = fdmex.GetAuxiliary();

    double u = 400.0;
    double w = -100.0;  // Negative w = negative alpha

    aux->in.vUVW = FGColumnVector3(u, 0.0, w);
    aux->in.Density = atm->GetDensitySL();
    aux->in.SoundSpeed = atm->GetSoundSpeedSL();
    aux->in.Pressure = atm->GetPressureSL();
    aux->in.Temperature = atm->GetTemperatureSL();

    aux->Run(false);

    TS_ASSERT(aux->Getalpha() < 0.0);
    double expected_alpha = atan2(w, u);
    TS_ASSERT_DELTA(aux->Getalpha(), expected_alpha, 1e-6);
  }

  // Test negative beta
  void testNegativeBeta() {
    auto aux = fdmex.GetAuxiliary();

    double u = 400.0;
    double v = -80.0;  // Negative v = negative beta

    aux->in.vUVW = FGColumnVector3(u, v, 0.0);
    aux->in.Density = atm->GetDensitySL();
    aux->in.SoundSpeed = atm->GetSoundSpeedSL();
    aux->in.Pressure = atm->GetPressureSL();
    aux->in.Temperature = atm->GetTemperatureSL();

    aux->Run(false);

    TS_ASSERT(aux->Getbeta() < 0.0);
    double expected_beta = atan2(v, u);
    TS_ASSERT_DELTA(aux->Getbeta(), expected_beta, 1e-6);
  }

  // Test large combined alpha and beta
  void testLargeAlphaBetaCombined() {
    auto aux = fdmex.GetAuxiliary();

    double u = 300.0;
    double v = 100.0;  // ~18 degrees beta
    double w = 150.0;  // ~26.5 degrees alpha

    aux->in.vUVW = FGColumnVector3(u, v, w);
    aux->in.Density = atm->GetDensitySL();
    aux->in.SoundSpeed = atm->GetSoundSpeedSL();
    aux->in.Pressure = atm->GetPressureSL();
    aux->in.Temperature = atm->GetTemperatureSL();

    aux->Run(false);

    double expected_alpha = atan2(w, u);
    double muw = u*u + w*w;
    double expected_beta = atan2(v, sqrt(muw));

    TS_ASSERT_DELTA(aux->Getalpha(), expected_alpha, 1e-6);
    TS_ASSERT_DELTA(aux->Getbeta(), expected_beta, 1e-6);
  }

  // ==================== Pressure Calculations ====================

  // Test pitot total pressure at Mach 0
  void testPitotTotalPressureMach0() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    double p = atm->GetPressureSL();

    // At Mach 0, total pressure equals static pressure
    double P0 = aux.PitotTotalPressure(0.0, p);
    TS_ASSERT_DELTA(P0, p, epsilon);

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  // Test pitot total pressure at Mach 1
  void testPitotTotalPressureSonic() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    double p = atm->GetPressureSL();

    // At Mach 1, ratio = (gama+1)/2)^(gama/(gama-1))
    double expected_ratio = pow((gama + 1.0) / 2.0, gama / (gama - 1.0));
    double P0 = aux.PitotTotalPressure(1.0, p);
    TS_ASSERT_DELTA(P0 / p, expected_ratio, 0.01);

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  // ==================== Output Consistency Tests ====================

  // Test multiple consecutive Run calls give consistent results
  void testConsecutiveRunCalls() {
    auto aux = fdmex.GetAuxiliary();

    aux->in.vUVW = FGColumnVector3(500.0, 30.0, 50.0);
    aux->in.Density = atm->GetDensitySL();
    aux->in.SoundSpeed = atm->GetSoundSpeedSL();
    aux->in.Pressure = atm->GetPressureSL();
    aux->in.Temperature = atm->GetTemperatureSL();

    aux->Run(false);
    double mach1 = aux->GetMach();
    double alpha1 = aux->Getalpha();
    double beta1 = aux->Getbeta();
    double qbar1 = aux->Getqbar();

    // Run again with same inputs
    aux->Run(false);
    double mach2 = aux->GetMach();
    double alpha2 = aux->Getalpha();
    double beta2 = aux->Getbeta();
    double qbar2 = aux->Getqbar();

    TS_ASSERT_DELTA(mach1, mach2, epsilon);
    TS_ASSERT_DELTA(alpha1, alpha2, epsilon);
    TS_ASSERT_DELTA(beta1, beta2, epsilon);
    TS_ASSERT_DELTA(qbar1, qbar2, epsilon);
  }

  // Test consistency across different velocity magnitudes with same ratios
  void testVelocityScalingConsistency() {
    auto aux = fdmex.GetAuxiliary();

    // First test: base velocity
    double scale1 = 1.0;
    aux->in.vUVW = FGColumnVector3(300.0 * scale1, 30.0 * scale1, 60.0 * scale1);
    aux->in.Density = atm->GetDensitySL();
    aux->in.SoundSpeed = atm->GetSoundSpeedSL();
    aux->in.Pressure = atm->GetPressureSL();
    aux->in.Temperature = atm->GetTemperatureSL();
    aux->Run(false);
    double alpha1 = aux->Getalpha();
    double beta1 = aux->Getbeta();

    // Second test: scaled velocity (alpha/beta should be same)
    double scale2 = 2.0;
    aux->in.vUVW = FGColumnVector3(300.0 * scale2, 30.0 * scale2, 60.0 * scale2);
    aux->Run(false);
    double alpha2 = aux->Getalpha();
    double beta2 = aux->Getbeta();

    // Alpha and beta depend on ratios, not magnitudes
    TS_ASSERT_DELTA(alpha1, alpha2, 1e-10);
    TS_ASSERT_DELTA(beta1, beta2, 1e-10);
  }

  // ==================== Stress Tests ====================

  // Test rapid state changes
  void testRapidStateChanges() {
    auto aux = fdmex.GetAuxiliary();

    for (int i = 0; i < 100; i++) {
      double V = 100.0 + i * 10.0;
      aux->in.vUVW = FGColumnVector3(V, V * 0.05, V * 0.1);
      aux->in.Density = atm->GetDensitySL();
      aux->in.SoundSpeed = atm->GetSoundSpeedSL();
      aux->in.Pressure = atm->GetPressureSL();
      aux->in.Temperature = atm->GetTemperatureSL();

      aux->Run(false);

      // All outputs should be valid
      TS_ASSERT(!std::isnan(aux->GetMach()));
      TS_ASSERT(!std::isnan(aux->Getalpha()));
      TS_ASSERT(!std::isnan(aux->Getbeta()));
      TS_ASSERT(!std::isnan(aux->Getqbar()));
      TS_ASSERT(!std::isinf(aux->GetMach()));
      TS_ASSERT(!std::isinf(aux->Getalpha()));
      TS_ASSERT(!std::isinf(aux->Getbeta()));
      TS_ASSERT(!std::isinf(aux->Getqbar()));
    }
  }

  // Test with varying density
  void testVaryingDensity() {
    auto aux = fdmex.GetAuxiliary();
    double V = 500.0;

    aux->in.vUVW = FGColumnVector3(V, 0.0, 0.0);
    aux->in.SoundSpeed = atm->GetSoundSpeedSL();
    aux->in.Pressure = atm->GetPressureSL();
    aux->in.Temperature = atm->GetTemperatureSL();

    // Test different altitudes (different densities)
    for (double alt = 0.0; alt <= 40000.0; alt += 5000.0) {
      double rho = atm->GetDensity(alt);
      aux->in.Density = rho;

      aux->Run(false);

      // qbar = 0.5 * rho * V^2
      double expected_qbar = 0.5 * rho * V * V;
      TS_ASSERT_DELTA(aux->Getqbar(), expected_qbar, 1.0);
    }
  }

  // Test alternating between high and low speed
  void testAlternatingSpeed() {
    auto aux = fdmex.GetAuxiliary();

    aux->in.Density = atm->GetDensitySL();
    aux->in.SoundSpeed = atm->GetSoundSpeedSL();
    aux->in.Pressure = atm->GetPressureSL();
    aux->in.Temperature = atm->GetTemperatureSL();

    for (int i = 0; i < 50; i++) {
      double V = (i % 2 == 0) ? 100.0 : 900.0;
      aux->in.vUVW = FGColumnVector3(V, 0.0, 0.0);

      aux->Run(false);

      double expected_mach = V / atm->GetSoundSpeedSL();
      TS_ASSERT_DELTA(aux->GetMach(), expected_mach, 1e-6);
    }
  }

  // ==================== Edge Case Tests ====================

  // Test backward flight (negative u)
  void testBackwardFlight() {
    auto aux = fdmex.GetAuxiliary();

    aux->in.vUVW = FGColumnVector3(-200.0, 0.0, 0.0);  // Flying backward
    aux->in.Density = atm->GetDensitySL();
    aux->in.SoundSpeed = atm->GetSoundSpeedSL();
    aux->in.Pressure = atm->GetPressureSL();
    aux->in.Temperature = atm->GetTemperatureSL();

    aux->Run(false);

    // Should still give valid results
    TS_ASSERT(!std::isnan(aux->GetMach()));
    TS_ASSERT(!std::isnan(aux->Getalpha()));
    TS_ASSERT(!std::isnan(aux->Getbeta()));
  }

  // Test sideways flight (large lateral velocity)
  void testSidewaysFlight() {
    auto aux = fdmex.GetAuxiliary();

    aux->in.vUVW = FGColumnVector3(100.0, 300.0, 0.0);  // Large sideslip
    aux->in.Density = atm->GetDensitySL();
    aux->in.SoundSpeed = atm->GetSoundSpeedSL();
    aux->in.Pressure = atm->GetPressureSL();
    aux->in.Temperature = atm->GetTemperatureSL();

    aux->Run(false);

    TS_ASSERT(!std::isnan(aux->GetMach()));
    TS_ASSERT(!std::isnan(aux->Getbeta()));
    // Large beta expected
    TS_ASSERT(fabs(aux->Getbeta()) > 0.5);  // More than ~30 degrees
  }

  // Test pure vertical flight (only w component)
  void testPureVerticalFlight() {
    auto aux = fdmex.GetAuxiliary();

    aux->in.vUVW = FGColumnVector3(1.0, 0.0, 500.0);  // Small u to avoid singularity
    aux->in.Density = atm->GetDensitySL();
    aux->in.SoundSpeed = atm->GetSoundSpeedSL();
    aux->in.Pressure = atm->GetPressureSL();
    aux->in.Temperature = atm->GetTemperatureSL();

    aux->Run(false);

    // Very large alpha expected (~89 degrees)
    TS_ASSERT(aux->Getalpha(FGJSBBase::inDegrees) > 80.0);
    TS_ASSERT(!std::isnan(aux->GetMach()));
  }

  // Test temperature at extreme Mach
  void testTotalTemperatureExtremeMach() {
    auto aux = fdmex.GetAuxiliary();

    double T = atm->GetTemperatureSL();
    double a = atm->GetSoundSpeedSL();
    double M = 4.0;  // Mach 4
    double V = M * a;

    aux->in.vUVW = FGColumnVector3(V, 0.0, 0.0);
    aux->in.Temperature = T;
    aux->in.SoundSpeed = a;
    aux->in.Density = atm->GetDensitySL();
    aux->in.Pressure = atm->GetPressureSL();

    aux->Run(false);

    // TAT = T * (1 + 0.2 * M^2) at high Mach
    double expected_tat = T * (1.0 + 0.2 * M * M);
    TS_ASSERT_DELTA(aux->GetTotalTemperature(), expected_tat, 5.0);  // Higher tolerance at extreme temps
  }

  // ==================== Unit Conversion Tests ====================

  // Test alpha degrees vs radians conversion
  void testAlphaUnitConversion() {
    auto aux = fdmex.GetAuxiliary();

    aux->in.vUVW = FGColumnVector3(400.0, 0.0, 100.0);
    aux->in.Density = atm->GetDensitySL();
    aux->in.SoundSpeed = atm->GetSoundSpeedSL();
    aux->in.Pressure = atm->GetPressureSL();
    aux->in.Temperature = atm->GetTemperatureSL();

    aux->Run(false);

    double alpha_rad = aux->Getalpha();
    double alpha_deg = aux->Getalpha(FGJSBBase::inDegrees);

    TS_ASSERT_DELTA(alpha_deg, alpha_rad * radtodeg, 1e-10);
  }

  // Test beta degrees vs radians conversion
  void testBetaUnitConversion() {
    auto aux = fdmex.GetAuxiliary();

    aux->in.vUVW = FGColumnVector3(400.0, 80.0, 0.0);
    aux->in.Density = atm->GetDensitySL();
    aux->in.SoundSpeed = atm->GetSoundSpeedSL();
    aux->in.Pressure = atm->GetPressureSL();
    aux->in.Temperature = atm->GetTemperatureSL();

    aux->Run(false);

    double beta_rad = aux->Getbeta();
    double beta_deg = aux->Getbeta(FGJSBBase::inDegrees);

    TS_ASSERT_DELTA(beta_deg, beta_rad * radtodeg, 1e-10);
  }

  // Test gamma degrees vs radians conversion
  void testGammaUnitConversion() {
    auto aux = fdmex.GetAuxiliary();

    aux->in.vUVW = FGColumnVector3(500.0, 0.0, 0.0);
    aux->in.Tb2l = FGMatrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    aux->in.vVel = FGColumnVector3(500.0, 0.0, -100.0);  // Climbing
    aux->in.Density = atm->GetDensitySL();
    aux->in.SoundSpeed = atm->GetSoundSpeedSL();

    aux->Run(false);

    double gamma_rad = aux->GetGamma();
    double gamma_deg = aux->GetGamma(FGJSBBase::inDegrees);

    TS_ASSERT_DELTA(gamma_deg, gamma_rad * radtodeg, 1e-10);
  }

  // Test FPS to KTS conversion for true airspeed
  void testTrueAirspeedFPSToKTS() {
    auto aux = fdmex.GetAuxiliary();

    double V = 500.0;  // fps
    aux->in.vUVW = FGColumnVector3(V, 0.0, 0.0);
    aux->in.Density = atm->GetDensitySL();
    aux->in.SoundSpeed = atm->GetSoundSpeedSL();
    aux->in.Pressure = atm->GetPressureSL();
    aux->in.Temperature = atm->GetTemperatureSL();

    aux->Run(false);

    double vtas_fps = aux->GetVtrueFPS();
    double vtas_kts = aux->GetVtrueKTS();

    // 1 fps = 0.592483801 kts
    TS_ASSERT_DELTA(vtas_kts, vtas_fps * 0.592483801, 0.01);
  }
};
