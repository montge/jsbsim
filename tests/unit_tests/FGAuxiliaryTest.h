#include <limits>
#include <cxxtest/TestSuite.h>

#include <FGFDMExec.h>
#include <models/FGAtmosphere.h>
#include <models/FGAuxiliary.h>

const double epsilon = 100. * std::numeric_limits<double>::epsilon();

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
};
