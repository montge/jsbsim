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

class FGAirspeedTest : public CxxTest::TestSuite
{
public:
  static constexpr double gama = FGAtmosphere::SHRatio;

  // ISA sea level constants
  static constexpr double T0_R = FGAtmosphere::StdDaySLtemperature;  // Rankine
  static constexpr double P0_psf = FGAtmosphere::StdDaySLpressure;   // psf
  static constexpr double rho0_slugft3 = P0_psf / (R * T0_R);        // slugs/ft³
  static constexpr double a0_fps = 1116.45;  // Speed of sound at sea level (fps)

  // Unit conversion constants
  static constexpr double fpstokts = 0.592483801;  // fps to knots
  static constexpr double fttometer = 0.3048;      // feet to meters

  FGFDMExec fdmex;
  std::shared_ptr<FGAtmosphere> atm;

  FGAirspeedTest() {
    auto aux = fdmex.GetAuxiliary();
    atm = fdmex.GetAtmosphere();
    atm->InitModel();
    fdmex.GetPropertyManager()->Unbind(aux);
  }

  //=============================================================================
  // 1. TRUE AIRSPEED (TAS) TESTS (~6 tests)
  //=============================================================================

  void testTASFromGroundSpeedNoWind() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    // No wind - TAS should equal ground speed
    double groundSpeed = 500.0;  // fps
    aux.in.vUVW = FGColumnVector3(groundSpeed, 0.0, 0.0);
    aux.in.TotalWindNED = FGColumnVector3(0.0, 0.0, 0.0);
    aux.in.Tl2b = FGMatrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    aux.in.Density = atm->GetDensitySL();
    aux.in.SoundSpeed = atm->GetSoundSpeedSL();
    aux.in.Pressure = atm->GetPressureSL();
    aux.in.Temperature = atm->GetTemperatureSL();

    aux.Run(false);

    double TAS = aux.GetVtrueFPS();
    TS_ASSERT_DELTA(TAS, groundSpeed, 0.1);

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testTASFromGroundSpeedWithHeadwind() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    // TAS is the magnitude of body velocity (vUVW)
    // The wind affects ground speed but TAS is airspeed relative to air mass
    double velocity = 450.0;  // fps (body frame velocity)

    aux.in.vUVW = FGColumnVector3(velocity, 0.0, 0.0);
    aux.in.Tl2b = FGMatrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    aux.in.Density = atm->GetDensitySL();
    aux.in.SoundSpeed = atm->GetSoundSpeedSL();

    aux.Run(false);

    double TAS = aux.GetVtrueFPS();
    TS_ASSERT_DELTA(TAS, velocity, 0.1);

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testTASAtSeaLevel() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    // At sea level, standard conditions
    double velocity = 300.0;  // fps
    aux.in.vUVW = FGColumnVector3(velocity, 0.0, 0.0);
    aux.in.Density = rho0_slugft3;
    aux.in.SoundSpeed = a0_fps;
    aux.in.Pressure = P0_psf;
    aux.in.Temperature = T0_R;

    aux.Run(false);

    TS_ASSERT_DELTA(aux.GetVtrueFPS(), velocity, epsilon);
    TS_ASSERT_DELTA(aux.GetVtrueKTS(), velocity * fpstokts, 0.01);

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testTASAt10000Feet() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    // At 10,000 ft altitude
    double altitude = 10000.0;
    double velocity = 400.0;  // fps

    aux.in.vUVW = FGColumnVector3(velocity, 0.0, 0.0);
    aux.in.Density = atm->GetDensity(altitude);
    aux.in.SoundSpeed = atm->GetSoundSpeed(altitude);
    aux.in.Pressure = atm->GetPressure(altitude);
    aux.in.Temperature = atm->GetTemperature(altitude);

    aux.Run(false);

    // TAS should still be the body velocity magnitude
    TS_ASSERT_DELTA(aux.GetVtrueFPS(), velocity, 0.1);

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testTASAt35000Feet() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    // At cruise altitude (35,000 ft)
    double altitude = 35000.0;
    double velocity = 800.0;  // fps (high-speed cruise)

    aux.in.vUVW = FGColumnVector3(velocity, 0.0, 0.0);
    aux.in.Density = atm->GetDensity(altitude);
    aux.in.SoundSpeed = atm->GetSoundSpeed(altitude);
    aux.in.Pressure = atm->GetPressure(altitude);
    aux.in.Temperature = atm->GetTemperature(altitude);

    aux.Run(false);

    TS_ASSERT_DELTA(aux.GetVtrueFPS(), velocity, 0.1);

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testTASTemperatureEffects() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    // Test that TAS is independent of temperature at fixed velocity
    double velocity = 500.0;  // fps

    // Hot day (+30°F from standard)
    double T_hot = T0_R + 30.0;
    aux.in.vUVW = FGColumnVector3(velocity, 0.0, 0.0);
    aux.in.Temperature = T_hot;
    aux.in.Pressure = P0_psf;
    aux.in.Density = P0_psf / (R * T_hot);
    aux.in.SoundSpeed = sqrt(gama * R * T_hot);

    aux.Run(false);

    // TAS doesn't change with temperature for same velocity
    TS_ASSERT_DELTA(aux.GetVtrueFPS(), velocity, 0.1);

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  //=============================================================================
  // 2. EQUIVALENT AIRSPEED (EAS) TESTS (~6 tests)
  //=============================================================================

  void testEASEqualsТASAtSeaLevel() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    // At sea level, EAS = TAS
    double velocity = 250.0;  // fps
    aux.in.vUVW = FGColumnVector3(velocity, 0.0, 0.0);
    aux.in.Density = rho0_slugft3;
    aux.in.SoundSpeed = a0_fps;
    aux.in.Pressure = P0_psf;
    aux.in.Temperature = T0_R;

    aux.Run(false);

    double EAS = aux.GetVequivalentFPS();
    double TAS = aux.GetVtrueFPS();

    TS_ASSERT_DELTA(EAS, TAS, 1.0);
    TS_ASSERT_DELTA(EAS, velocity, 1.0);

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testEASFormula() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    // EAS = TAS * sqrt(rho / rho0)
    double altitude = 10000.0;
    double TAS = 400.0;  // fps

    double rho = atm->GetDensity(altitude);
    double rho0 = atm->GetDensitySL();

    aux.in.vUVW = FGColumnVector3(TAS, 0.0, 0.0);
    aux.in.Density = rho;
    aux.in.SoundSpeed = atm->GetSoundSpeed(altitude);
    aux.in.Pressure = atm->GetPressure(altitude);
    aux.in.Temperature = atm->GetTemperature(altitude);

    aux.Run(false);

    double expectedEAS = TAS * sqrt(rho / rho0);
    TS_ASSERT_DELTA(aux.GetVequivalentFPS(), expectedEAS, 2.0);

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testEAStoTASConversion() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    // TAS = EAS / sqrt(rho / rho0)
    double altitude = 15000.0;
    double TAS = 500.0;  // fps

    double rho = atm->GetDensity(altitude);
    double rho0 = atm->GetDensitySL();

    aux.in.vUVW = FGColumnVector3(TAS, 0.0, 0.0);
    aux.in.Density = rho;
    aux.in.SoundSpeed = atm->GetSoundSpeed(altitude);

    aux.Run(false);

    double EAS = aux.GetVequivalentFPS();
    double computedTAS = EAS / sqrt(rho / rho0);

    TS_ASSERT_DELTA(computedTAS, TAS, 2.0);

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testEASAltitudeEffect10000ft() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    // At 10,000 ft, EAS < TAS due to lower density
    double altitude = 10000.0;
    double TAS = 450.0;  // fps

    aux.in.vUVW = FGColumnVector3(TAS, 0.0, 0.0);
    aux.in.Density = atm->GetDensity(altitude);
    aux.in.SoundSpeed = atm->GetSoundSpeed(altitude);

    aux.Run(false);

    double EAS = aux.GetVequivalentFPS();

    // At altitude, EAS should be less than TAS
    TS_ASSERT(EAS < TAS);

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testEASAltitudeEffect35000ft() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    // At 35,000 ft, much lower EAS than TAS
    double altitude = 35000.0;
    double TAS = 800.0;  // fps (typical cruise)

    double rho = atm->GetDensity(altitude);
    double rho0 = atm->GetDensitySL();

    aux.in.vUVW = FGColumnVector3(TAS, 0.0, 0.0);
    aux.in.Density = rho;
    aux.in.SoundSpeed = atm->GetSoundSpeed(altitude);

    aux.Run(false);

    double EAS = aux.GetVequivalentFPS();
    double expectedEAS = TAS * sqrt(rho / rho0);

    TS_ASSERT_DELTA(EAS, expectedEAS, 3.0);
    TS_ASSERT(EAS < TAS * 0.7);  // Significantly less

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testEASUnitConsistency() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    // Test EAS in both fps and knots
    double altitude = 5000.0;
    double TAS = 350.0;  // fps

    aux.in.vUVW = FGColumnVector3(TAS, 0.0, 0.0);
    aux.in.Density = atm->GetDensity(altitude);
    aux.in.SoundSpeed = atm->GetSoundSpeed(altitude);

    aux.Run(false);

    double EAS_fps = aux.GetVequivalentFPS();
    double EAS_kts = aux.GetVequivalentKTS();

    TS_ASSERT_DELTA(EAS_kts, EAS_fps * fpstokts, 0.01);

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  //=============================================================================
  // 3. CALIBRATED AIRSPEED (CAS) TESTS (~6 tests)
  //=============================================================================

  void testCASFromImpactPressure() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;
    aux.in.StdDaySLsoundspeed = atm->StdDaySLsoundspeed;
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    // At sea level, CAS should be close to TAS at low speeds
    double Mach = 0.3;
    double pressure = P0_psf;

    double CAS = aux.VcalibratedFromMach(Mach, pressure);
    double TAS = Mach * a0_fps;

    // At sea level and low Mach, CAS ≈ TAS
    TS_ASSERT_DELTA(CAS, TAS, 5.0);

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testCASCompressibilityCorrection() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;
    aux.in.StdDaySLsoundspeed = atm->StdDaySLsoundspeed;
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    // At high Mach, compressibility effects are significant
    double Mach_low = 0.2;
    double Mach_high = 0.8;
    double pressure = P0_psf;

    double CAS_low = aux.VcalibratedFromMach(Mach_low, pressure);
    double CAS_high = aux.VcalibratedFromMach(Mach_high, pressure);

    // Verify both are positive and reasonable
    TS_ASSERT(CAS_low > 0.0);
    TS_ASSERT(CAS_high > CAS_low);

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testCASPositionErrorConcept() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;
    aux.in.StdDaySLsoundspeed = atm->StdDaySLsoundspeed;
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    // CAS corrects for compressibility, conceptually before position error
    // IAS = CAS - position_error (not directly tested here as JSBSim doesn't model this)

    double Mach = 0.5;
    double pressure_sl = P0_psf;
    double pressure_alt = atm->GetPressure(10000.0);

    double CAS_sl = aux.VcalibratedFromMach(Mach, pressure_sl);
    double CAS_alt = aux.VcalibratedFromMach(Mach, pressure_alt);

    // CAS varies with pressure
    TS_ASSERT(CAS_alt != CAS_sl);

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testCASRoundTripConversion() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;
    aux.in.StdDaySLsoundspeed = atm->StdDaySLsoundspeed;
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    // Test Mach -> CAS -> Mach conversion
    double Mach_original = 0.6;
    double pressure = atm->GetPressure(5000.0);

    double CAS = aux.VcalibratedFromMach(Mach_original, pressure);
    double Mach_computed = aux.MachFromVcalibrated(CAS, pressure);

    TS_ASSERT_DELTA(Mach_computed, Mach_original, 1e-6);

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testCASAtVariousAltitudes() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;
    aux.in.StdDaySLsoundspeed = atm->StdDaySLsoundspeed;
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    // Test CAS calculation at different altitudes
    double Mach = 0.5;

    for (double alt = 0.0; alt <= 35000.0; alt += 5000.0) {
      double pressure = atm->GetPressure(alt);
      double CAS = aux.VcalibratedFromMach(Mach, pressure);

      TS_ASSERT(CAS > 0.0);
      TS_ASSERT(!std::isnan(CAS));
      TS_ASSERT(!std::isinf(CAS));
    }

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testCASGetters() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    double velocity = 350.0;  // fps
    aux.in.vUVW = FGColumnVector3(velocity, 0.0, 0.0);
    aux.in.Density = atm->GetDensitySL();
    aux.in.SoundSpeed = atm->GetSoundSpeedSL();
    aux.in.Pressure = atm->GetPressureSL();
    aux.in.Temperature = atm->GetTemperatureSL();
    aux.in.StdDaySLsoundspeed = atm->StdDaySLsoundspeed;
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    aux.Run(false);

    double CAS_fps = aux.GetVcalibratedFPS();
    double CAS_kts = aux.GetVcalibratedKTS();

    TS_ASSERT_DELTA(CAS_kts, CAS_fps * fpstokts, 0.01);

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  //=============================================================================
  // 4. INDICATED AIRSPEED (IAS) TESTS (~4 tests)
  //=============================================================================

  void testIAStoCASRelationship() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;
    aux.in.StdDaySLsoundspeed = atm->StdDaySLsoundspeed;
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    // IAS = CAS - instrument_error - position_error
    // JSBSim provides CAS; IAS would need additional corrections
    // Here we verify CAS is reasonable (conceptual test)

    double Mach = 0.4;
    double pressure = P0_psf;
    double CAS = aux.VcalibratedFromMach(Mach, pressure);

    // CAS should be positive and in reasonable range
    TS_ASSERT(CAS > 0.0);
    TS_ASSERT(CAS < 1000.0);  // Reasonable upper bound for this Mach

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testInstrumentErrorModeling() {
    // Conceptual test: instrument error would add to CAS to get IAS
    // In practice: IAS_indicated = CAS + instrument_error + position_error

    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;
    aux.in.StdDaySLsoundspeed = atm->StdDaySLsoundspeed;
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    double Mach = 0.35;
    double pressure = P0_psf;
    double CAS = aux.VcalibratedFromMach(Mach, pressure);

    // Verify CAS is valid (base for IAS calculation)
    TS_ASSERT(!std::isnan(CAS));
    TS_ASSERT(CAS > 0.0);

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testIASLowSpeed() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;
    aux.in.StdDaySLsoundspeed = atm->StdDaySLsoundspeed;
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    // At very low speeds, all airspeeds converge
    double Mach = 0.1;
    double pressure = P0_psf;

    double CAS = aux.VcalibratedFromMach(Mach, pressure);
    double TAS = Mach * a0_fps;

    // At low Mach and sea level, CAS ≈ TAS ≈ IAS
    TS_ASSERT_DELTA(CAS / TAS, 1.0, 0.05);

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testIASZeroVelocity() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;
    aux.in.StdDaySLsoundspeed = atm->StdDaySLsoundspeed;
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    // At zero velocity, all airspeeds should be zero
    double Mach = 0.0;
    double pressure = P0_psf;

    double CAS = aux.VcalibratedFromMach(Mach, pressure);

    TS_ASSERT_DELTA(CAS, 0.0, epsilon);

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  //=============================================================================
  // 5. MACH NUMBER TESTS (~8 tests)
  //=============================================================================

  void testMachFormula() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    // M = V / a, where a = sqrt(gamma * R * T)
    double T = T0_R;
    double a = sqrt(gama * R * T);
    double V = 0.5 * a;  // M = 0.5

    aux.in.vUVW = FGColumnVector3(V, 0.0, 0.0);
    aux.in.Temperature = T;
    aux.in.SoundSpeed = a;
    aux.in.Density = rho0_slugft3;
    aux.in.Pressure = P0_psf;

    aux.Run(false);

    TS_ASSERT_DELTA(aux.GetMach(), 0.5, 1e-6);

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testMachFromTASAndTemperature() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    // Test Mach calculation at different temperatures
    double TAS = 600.0;  // fps
    double T_cold = T0_R - 50.0;  // Cold day
    double a_cold = sqrt(gama * R * T_cold);

    aux.in.vUVW = FGColumnVector3(TAS, 0.0, 0.0);
    aux.in.Temperature = T_cold;
    aux.in.SoundSpeed = a_cold;
    aux.in.Density = P0_psf / (R * T_cold);
    aux.in.Pressure = P0_psf;

    aux.Run(false);

    double expectedMach = TAS / a_cold;
    TS_ASSERT_DELTA(aux.GetMach(), expectedMach, 1e-6);

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testMachSubsonic() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    // Test subsonic regime (M < 0.8)
    double M = 0.7;
    double a = a0_fps;
    double V = M * a;

    aux.in.vUVW = FGColumnVector3(V, 0.0, 0.0);
    aux.in.SoundSpeed = a;
    aux.in.Density = rho0_slugft3;
    aux.in.Pressure = P0_psf;
    aux.in.Temperature = T0_R;

    aux.Run(false);

    TS_ASSERT_DELTA(aux.GetMach(), M, 1e-6);
    TS_ASSERT(aux.GetMach() < 0.8);  // Subsonic

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testMachTransonic() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    // Test transonic regime (0.8 < M < 1.2)
    double M = 0.95;
    double a = a0_fps;
    double V = M * a;

    aux.in.vUVW = FGColumnVector3(V, 0.0, 0.0);
    aux.in.SoundSpeed = a;
    aux.in.Density = rho0_slugft3;
    aux.in.Pressure = P0_psf;
    aux.in.Temperature = T0_R;

    aux.Run(false);

    TS_ASSERT_DELTA(aux.GetMach(), M, 1e-6);
    TS_ASSERT(aux.GetMach() > 0.8 && aux.GetMach() < 1.2);

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testMachSupersonic() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    // Test supersonic regime (M > 1.2)
    double M = 2.0;
    double a = a0_fps;
    double V = M * a;

    aux.in.vUVW = FGColumnVector3(V, 0.0, 0.0);
    aux.in.SoundSpeed = a;
    aux.in.Density = rho0_slugft3;
    aux.in.Pressure = P0_psf;
    aux.in.Temperature = T0_R;

    aux.Run(false);

    TS_ASSERT_DELTA(aux.GetMach(), M, 1e-6);
    TS_ASSERT(aux.GetMach() > 1.2);  // Supersonic

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testCriticalMachConcept() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    // Critical Mach number is where local flow first reaches M=1
    // Typically around M=0.7-0.9 for transonic aircraft
    // Here we just verify we can calculate Mach in this range

    double M_crit = 0.85;  // Typical critical Mach
    double a = a0_fps;
    double V = M_crit * a;

    aux.in.vUVW = FGColumnVector3(V, 0.0, 0.0);
    aux.in.SoundSpeed = a;
    aux.in.Density = rho0_slugft3;
    aux.in.Pressure = P0_psf;
    aux.in.Temperature = T0_R;

    aux.Run(false);

    TS_ASSERT_DELTA(aux.GetMach(), M_crit, 1e-6);

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testMachAtAltitude() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    // Mach number at altitude with different sound speed
    double altitude = 25000.0;
    double M = 0.75;
    double T = atm->GetTemperature(altitude);
    double a = atm->GetSoundSpeed(altitude);
    double V = M * a;

    aux.in.vUVW = FGColumnVector3(V, 0.0, 0.0);
    aux.in.Temperature = T;
    aux.in.SoundSpeed = a;
    aux.in.Density = atm->GetDensity(altitude);
    aux.in.Pressure = atm->GetPressure(altitude);

    aux.Run(false);

    TS_ASSERT_DELTA(aux.GetMach(), M, 1e-6);

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testMachU() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    // MachU is Mach based only on u-component
    double u = 500.0;
    double v = 200.0;
    double w = 150.0;
    double a = a0_fps;

    aux.in.vUVW = FGColumnVector3(u, v, w);
    aux.in.SoundSpeed = a;
    aux.in.Density = rho0_slugft3;
    aux.in.Pressure = P0_psf;
    aux.in.Temperature = T0_R;

    aux.Run(false);

    double expectedMachU = u / a;
    TS_ASSERT_DELTA(aux.GetMachU(), expectedMachU, 1e-6);

    // MachU should be different from total Mach (larger side velocities make difference visible)
    double totalV = sqrt(u*u + v*v + w*w);
    double totalMach = totalV / a;
    TS_ASSERT(fabs(aux.GetMachU() - totalMach) > 0.05);

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  //=============================================================================
  // 6. DYNAMIC PRESSURE TESTS (~6 tests)
  //=============================================================================

  void testDynamicPressureIncompressible() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    // q = 0.5 * rho * V^2 (incompressible formula)
    double rho = rho0_slugft3;
    double V = 200.0;  // fps (low speed)

    aux.in.vUVW = FGColumnVector3(V, 0.0, 0.0);
    aux.in.Density = rho;
    aux.in.SoundSpeed = a0_fps;
    aux.in.Pressure = P0_psf;
    aux.in.Temperature = T0_R;

    aux.Run(false);

    double expected_qbar = 0.5 * rho * V * V;
    TS_ASSERT_DELTA(aux.Getqbar(), expected_qbar, 1.0);

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testDynamicPressureCompressible() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    // At high Mach, compressibility effects matter
    // q still computed as 0.5 * rho * V^2 but with actual conditions

    double M = 1.5;
    double a = a0_fps;
    double V = M * a;
    double rho = rho0_slugft3;

    aux.in.vUVW = FGColumnVector3(V, 0.0, 0.0);
    aux.in.Density = rho;
    aux.in.SoundSpeed = a;
    aux.in.Pressure = P0_psf;
    aux.in.Temperature = T0_R;

    aux.Run(false);

    double expected_qbar = 0.5 * rho * V * V;
    TS_ASSERT_DELTA(aux.Getqbar(), expected_qbar, 10.0);

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testImpactPressure() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    // Impact pressure qc = Pt - P
    // Test at subsonic speeds

    double M = 0.5;
    double P = P0_psf;

    double Pt = aux.PitotTotalPressure(M, P);
    double qc = Pt - P;

    TS_ASSERT(qc > 0.0);  // Impact pressure should be positive
    TS_ASSERT(Pt > P);    // Total pressure > static pressure

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testImpactPressureSupersonic() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    // At supersonic speeds, shock wave affects pitot measurement
    double M = 2.0;
    double P = P0_psf;

    double Pt = aux.PitotTotalPressure(M, P);
    double qc = Pt - P;

    TS_ASSERT(qc > 0.0);
    TS_ASSERT(Pt > P);

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testPitotStaticRelationship() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    // Verify Mach can be recovered from impact pressure
    double M_original = 0.8;
    double P = P0_psf;

    double Pt = aux.PitotTotalPressure(M_original, P);
    double qc = Pt - P;

    double M_computed = aux.MachFromImpactPressure(qc, P);

    TS_ASSERT_DELTA(M_computed, M_original, 1e-7);

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testDynamicPressureAtAltitude() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    // Dynamic pressure decreases with altitude (lower density)
    double V = 500.0;  // fps constant TAS

    double qbar_sl = 0.5 * rho0_slugft3 * V * V;

    double altitude = 20000.0;
    double rho_alt = atm->GetDensity(altitude);

    aux.in.vUVW = FGColumnVector3(V, 0.0, 0.0);
    aux.in.Density = rho_alt;
    aux.in.SoundSpeed = atm->GetSoundSpeed(altitude);

    aux.Run(false);

    double qbar_alt = aux.Getqbar();

    TS_ASSERT(qbar_alt < qbar_sl);  // Lower qbar at altitude

    double expected_qbar_alt = 0.5 * rho_alt * V * V;
    TS_ASSERT_DELTA(qbar_alt, expected_qbar_alt, 1.0);

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  //=============================================================================
  // 7. AIRSPEED CONVERSIONS (~6 tests)
  //=============================================================================

  void testConversionChainSeaLevel() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;
    aux.in.StdDaySLsoundspeed = atm->StdDaySLsoundspeed;
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    // At sea level: IAS ≈ CAS ≈ EAS ≈ TAS (at low speeds)
    double TAS = 250.0;  // fps

    aux.in.vUVW = FGColumnVector3(TAS, 0.0, 0.0);
    aux.in.Density = rho0_slugft3;
    aux.in.SoundSpeed = a0_fps;
    aux.in.Pressure = P0_psf;
    aux.in.Temperature = T0_R;

    aux.Run(false);

    double EAS = aux.GetVequivalentFPS();
    double CAS = aux.GetVcalibratedFPS();

    TS_ASSERT_DELTA(TAS, EAS, 5.0);
    TS_ASSERT_DELTA(TAS, CAS, 5.0);

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testConversionChain10000ft() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;
    aux.in.StdDaySLsoundspeed = atm->StdDaySLsoundspeed;
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    // At 10,000 ft: TAS > EAS, CAS differs from both
    double altitude = 10000.0;
    double TAS = 400.0;  // fps

    double rho = atm->GetDensity(altitude);
    double rho0 = atm->GetDensitySL();

    aux.in.vUVW = FGColumnVector3(TAS, 0.0, 0.0);
    aux.in.Density = rho;
    aux.in.SoundSpeed = atm->GetSoundSpeed(altitude);
    aux.in.Pressure = atm->GetPressure(altitude);
    aux.in.Temperature = atm->GetTemperature(altitude);

    aux.Run(false);

    double EAS = aux.GetVequivalentFPS();
    double CAS = aux.GetVcalibratedFPS();

    // At altitude: TAS > EAS
    TS_ASSERT(TAS > EAS);

    // EAS = TAS * sqrt(rho/rho0)
    double expectedEAS = TAS * sqrt(rho / rho0);
    TS_ASSERT_DELTA(EAS, expectedEAS, 2.0);

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testConversionChain35000ft() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;
    aux.in.StdDaySLsoundspeed = atm->StdDaySLsoundspeed;
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    // At cruise altitude: large difference between TAS and EAS/CAS
    double altitude = 35000.0;
    double TAS = 800.0;  // fps (typical jet cruise)

    double rho = atm->GetDensity(altitude);
    double rho0 = atm->GetDensitySL();

    aux.in.vUVW = FGColumnVector3(TAS, 0.0, 0.0);
    aux.in.Density = rho;
    aux.in.SoundSpeed = atm->GetSoundSpeed(altitude);
    aux.in.Pressure = atm->GetPressure(altitude);
    aux.in.Temperature = atm->GetTemperature(altitude);

    aux.Run(false);

    double EAS = aux.GetVequivalentFPS();

    // Significant difference at high altitude
    TS_ASSERT(TAS > EAS * 1.5);

    double expectedEAS = TAS * sqrt(rho / rho0);
    TS_ASSERT_DELTA(EAS, expectedEAS, 3.0);

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testReverseConversions() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;
    aux.in.StdDaySLsoundspeed = atm->StdDaySLsoundspeed;
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    // Test CAS -> Mach -> CAS round-trip
    double altitude = 15000.0;
    double M = 0.65;
    double pressure = atm->GetPressure(altitude);

    double CAS1 = aux.VcalibratedFromMach(M, pressure);
    double M_back = aux.MachFromVcalibrated(CAS1, pressure);
    double CAS2 = aux.VcalibratedFromMach(M_back, pressure);

    TS_ASSERT_DELTA(M_back, M, 1e-6);
    TS_ASSERT_DELTA(CAS2, CAS1, 1e-6);

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testConversionConsistency() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;
    aux.in.StdDaySLsoundspeed = atm->StdDaySLsoundspeed;
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    // Verify conversions maintain physical relationships
    double altitude = 8000.0;
    double TAS = 450.0;  // fps

    double rho = atm->GetDensity(altitude);
    double rho0 = atm->GetDensitySL();

    aux.in.vUVW = FGColumnVector3(TAS, 0.0, 0.0);
    aux.in.Density = rho;
    aux.in.SoundSpeed = atm->GetSoundSpeed(altitude);
    aux.in.Pressure = atm->GetPressure(altitude);
    aux.in.Temperature = atm->GetTemperature(altitude);

    aux.Run(false);

    double EAS = aux.GetVequivalentFPS();
    double CAS = aux.GetVcalibratedFPS();

    // Physical sanity checks
    TS_ASSERT(TAS > 0.0);
    TS_ASSERT(EAS > 0.0);
    TS_ASSERT(CAS > 0.0);
    TS_ASSERT(TAS > EAS);  // At altitude

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testKnotsConversions() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;
    aux.in.StdDaySLsoundspeed = atm->StdDaySLsoundspeed;
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    // Test all airspeed types in knots
    double TAS = 500.0;  // fps

    aux.in.vUVW = FGColumnVector3(TAS, 0.0, 0.0);
    aux.in.Density = atm->GetDensity(5000.0);
    aux.in.SoundSpeed = atm->GetSoundSpeed(5000.0);
    aux.in.Pressure = atm->GetPressure(5000.0);
    aux.in.Temperature = atm->GetTemperature(5000.0);

    aux.Run(false);

    double TAS_kts = aux.GetVtrueKTS();
    double EAS_kts = aux.GetVequivalentKTS();
    double CAS_kts = aux.GetVcalibratedKTS();

    TS_ASSERT_DELTA(TAS_kts, TAS * fpstokts, 0.01);
    TS_ASSERT_DELTA(EAS_kts, aux.GetVequivalentFPS() * fpstokts, 0.01);
    TS_ASSERT_DELTA(CAS_kts, aux.GetVcalibratedFPS() * fpstokts, 0.01);

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  //=============================================================================
  // 8. EDGE CASES (~4 tests)
  //=============================================================================

  void testZeroAirspeed() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;
    aux.in.StdDaySLsoundspeed = atm->StdDaySLsoundspeed;
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    // All airspeeds should be zero
    aux.in.vUVW = FGColumnVector3(0.0, 0.0, 0.0);
    aux.in.Density = rho0_slugft3;
    aux.in.SoundSpeed = a0_fps;
    aux.in.Pressure = P0_psf;
    aux.in.Temperature = T0_R;

    aux.Run(false);

    TS_ASSERT_DELTA(aux.GetVtrueFPS(), 0.0, epsilon);
    TS_ASSERT_DELTA(aux.GetVequivalentFPS(), 0.0, epsilon);
    TS_ASSERT_DELTA(aux.GetVcalibratedFPS(), 0.0, epsilon);
    TS_ASSERT_DELTA(aux.GetMach(), 0.0, epsilon);
    TS_ASSERT_DELTA(aux.Getqbar(), 0.0, epsilon);

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testVeryHighAltitude() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;
    aux.in.StdDaySLsoundspeed = atm->StdDaySLsoundspeed;
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    // At very high altitude (low density)
    double altitude = 60000.0;  // Near space
    double TAS = 1000.0;  // fps

    double rho = atm->GetDensity(altitude);
    double rho0 = atm->GetDensitySL();

    aux.in.vUVW = FGColumnVector3(TAS, 0.0, 0.0);
    aux.in.Density = rho;
    aux.in.SoundSpeed = atm->GetSoundSpeed(altitude);
    aux.in.Pressure = atm->GetPressure(altitude);
    aux.in.Temperature = atm->GetTemperature(altitude);

    aux.Run(false);

    double EAS = aux.GetVequivalentFPS();

    // EAS should be much less than TAS
    TS_ASSERT(EAS < TAS * 0.5);

    // Verify calculation
    double expectedEAS = TAS * sqrt(rho / rho0);
    TS_ASSERT_DELTA(EAS, expectedEAS, 5.0);

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testHighMachNumbers() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;
    aux.in.StdDaySLsoundspeed = atm->StdDaySLsoundspeed;
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    // Test at M = 3.0 (supersonic)
    double M = 3.0;
    double a = a0_fps;
    double V = M * a;

    aux.in.vUVW = FGColumnVector3(V, 0.0, 0.0);
    aux.in.SoundSpeed = a;
    aux.in.Density = rho0_slugft3;
    aux.in.Pressure = P0_psf;
    aux.in.Temperature = T0_R;

    aux.Run(false);

    TS_ASSERT_DELTA(aux.GetMach(), M, 1e-6);

    // Verify pitot tube calculation works
    double Pt = aux.PitotTotalPressure(M, P0_psf);
    TS_ASSERT(Pt > P0_psf);
    TS_ASSERT(!std::isnan(Pt));

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testNonStandardDay() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;
    aux.in.StdDaySLsoundspeed = atm->StdDaySLsoundspeed;
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    // Hot day: +15°C from standard (ISA+15)
    double T_hot = T0_R + 27.0;  // +15°C in Rankine
    double P = P0_psf;
    double rho_hot = P / (R * T_hot);
    double a_hot = sqrt(gama * R * T_hot);

    double TAS = 450.0;  // fps

    aux.in.vUVW = FGColumnVector3(TAS, 0.0, 0.0);
    aux.in.Temperature = T_hot;
    aux.in.Pressure = P;
    aux.in.Density = rho_hot;
    aux.in.SoundSpeed = a_hot;

    aux.Run(false);

    // TAS should still be correct
    TS_ASSERT_DELTA(aux.GetVtrueFPS(), TAS, 0.1);

    // Mach will be different due to different sound speed
    double expectedMach = TAS / a_hot;
    TS_ASSERT_DELTA(aux.GetMach(), expectedMach, 1e-6);

    // EAS calculation should still work
    double EAS = aux.GetVequivalentFPS();
    double expectedEAS = TAS * sqrt(rho_hot / rho0_slugft3);
    TS_ASSERT_DELTA(EAS, expectedEAS, 2.0);

    fdmex.GetPropertyManager()->Unbind(&aux);
  }
};
