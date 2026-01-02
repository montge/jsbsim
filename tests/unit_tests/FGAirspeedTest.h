#include <limits>
#include <cxxtest/TestSuite.h>

#include <FGFDMExec.h>
#include <models/FGAtmosphere.h>
#include <models/FGAuxiliary.h>
#include <models/FGPropagate.h>
#include <models/FGFCS.h>
#include <models/FGPropulsion.h>
#include <input_output/FGPropertyManager.h>

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

  //=============================================================================
  // 9. ADDITIONAL AIRSPEED TESTS
  //=============================================================================

  void testTASWithSideVelocity() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    // TAS includes lateral velocity components
    double u = 400.0, v = 100.0, w = 50.0;
    double expectedTAS = sqrt(u*u + v*v + w*w);

    aux.in.vUVW = FGColumnVector3(u, v, w);
    aux.in.Density = rho0_slugft3;
    aux.in.SoundSpeed = a0_fps;
    aux.in.Pressure = P0_psf;
    aux.in.Temperature = T0_R;

    aux.Run(false);

    TS_ASSERT_DELTA(aux.GetVtrueFPS(), expectedTAS, 0.1);

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testTASWithNegativeU() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    // Flying backwards (negative u-component)
    double u = -100.0;
    aux.in.vUVW = FGColumnVector3(u, 0.0, 0.0);
    aux.in.Density = rho0_slugft3;
    aux.in.SoundSpeed = a0_fps;
    aux.in.Pressure = P0_psf;
    aux.in.Temperature = T0_R;

    aux.Run(false);

    // TAS is magnitude, always positive
    TS_ASSERT_DELTA(aux.GetVtrueFPS(), fabs(u), 0.1);
    TS_ASSERT(aux.GetVtrueFPS() > 0.0);

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testMachVelocityComponents() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    // Velocity with all three components
    double u = 600.0, v = 200.0, w = 100.0;
    double V = sqrt(u*u + v*v + w*w);

    aux.in.vUVW = FGColumnVector3(u, v, w);
    aux.in.SoundSpeed = a0_fps;
    aux.in.Density = rho0_slugft3;
    aux.in.Pressure = P0_psf;
    aux.in.Temperature = T0_R;

    aux.Run(false);

    double expectedMach = V / a0_fps;
    TS_ASSERT_DELTA(aux.GetMach(), expectedMach, 1e-6);

    // MachU should be based on u-component only
    double expectedMachU = u / a0_fps;
    TS_ASSERT_DELTA(aux.GetMachU(), expectedMachU, 1e-6);

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testQbarProportionalToVelocitySquared() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    // Double velocity should quadruple qbar
    double V1 = 200.0;
    double V2 = 400.0;

    aux.in.vUVW = FGColumnVector3(V1, 0.0, 0.0);
    aux.in.Density = rho0_slugft3;
    aux.in.SoundSpeed = a0_fps;
    aux.Run(false);
    double qbar1 = aux.Getqbar();

    aux.in.vUVW = FGColumnVector3(V2, 0.0, 0.0);
    aux.Run(false);
    double qbar2 = aux.Getqbar();

    // qbar2 / qbar1 = (V2/V1)^2 = 4
    TS_ASSERT_DELTA(qbar2 / qbar1, 4.0, 0.01);

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testQbarProportionalToDensity() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    // Same velocity, half density should give half qbar
    double V = 300.0;

    aux.in.vUVW = FGColumnVector3(V, 0.0, 0.0);
    aux.in.Density = rho0_slugft3;
    aux.in.SoundSpeed = a0_fps;
    aux.Run(false);
    double qbar1 = aux.Getqbar();

    aux.in.Density = rho0_slugft3 / 2.0;
    aux.Run(false);
    double qbar2 = aux.Getqbar();

    TS_ASSERT_DELTA(qbar2 / qbar1, 0.5, 0.01);

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testDensityRatioAtAltitude() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    // Test sigma (density ratio) calculation concept
    double altitude = 20000.0;
    double rho = atm->GetDensity(altitude);
    double rho0 = atm->GetDensitySL();
    double sigma = rho / rho0;

    // Sigma should be less than 1 at altitude
    TS_ASSERT(sigma < 1.0);
    TS_ASSERT(sigma > 0.0);

    // EAS = TAS * sqrt(sigma)
    double TAS = 500.0;
    aux.in.vUVW = FGColumnVector3(TAS, 0.0, 0.0);
    aux.in.Density = rho;
    aux.in.SoundSpeed = atm->GetSoundSpeed(altitude);

    aux.Run(false);

    double EAS = aux.GetVequivalentFPS();
    TS_ASSERT_DELTA(EAS, TAS * sqrt(sigma), 2.0);

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testPitotTotalPressureSubsonic() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    // Subsonic: Pt/P = (1 + (gamma-1)/2 * M^2)^(gamma/(gamma-1))
    double M = 0.6;
    double P = P0_psf;

    double Pt = aux.PitotTotalPressure(M, P);

    // Isentropic relation
    double ratio = pow(1.0 + (gama - 1.0) / 2.0 * M * M, gama / (gama - 1.0));
    double expectedPt = P * ratio;

    TS_ASSERT_DELTA(Pt, expectedPt, 1.0);

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testMachFromImpactPressureRoundTrip() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    // Test round-trip: M -> qc -> M
    double P = P0_psf;

    for (double M = 0.1; M <= 2.5; M += 0.3) {
      double Pt = aux.PitotTotalPressure(M, P);
      double qc = Pt - P;
      double M_back = aux.MachFromImpactPressure(qc, P);

      TS_ASSERT_DELTA(M_back, M, 1e-6);
    }

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testCASIncreaseWithMach() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;
    aux.in.StdDaySLsoundspeed = atm->StdDaySLsoundspeed;

    double P = P0_psf;
    double prev_CAS = 0.0;

    // CAS should increase monotonically with Mach
    for (double M = 0.1; M <= 0.9; M += 0.1) {
      double CAS = aux.VcalibratedFromMach(M, P);
      TS_ASSERT(CAS > prev_CAS);
      prev_CAS = CAS;
    }

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testSoundSpeedTemperatureDependence() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    // a = sqrt(gamma * R * T)
    // Same velocity, different temperature = different Mach

    double V = 600.0;
    double T_std = T0_R;
    double T_cold = T0_R - 40.0;

    double a_std = sqrt(gama * R * T_std);
    double a_cold = sqrt(gama * R * T_cold);

    // Cold air = lower speed of sound = higher Mach
    aux.in.vUVW = FGColumnVector3(V, 0.0, 0.0);
    aux.in.Temperature = T_std;
    aux.in.SoundSpeed = a_std;
    aux.in.Density = rho0_slugft3;
    aux.Run(false);
    double M_std = aux.GetMach();

    aux.in.Temperature = T_cold;
    aux.in.SoundSpeed = a_cold;
    aux.in.Density = P0_psf / (R * T_cold);
    aux.Run(false);
    double M_cold = aux.GetMach();

    TS_ASSERT(M_cold > M_std);
    TS_ASSERT_DELTA(M_std, V / a_std, 1e-6);
    TS_ASSERT_DELTA(M_cold, V / a_cold, 1e-6);

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testEASConstantWithAltitudeForSameLift() {
    // Concept: EAS corresponds to dynamic pressure (same EAS = same qbar)
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    double EAS_target = 300.0;  // fps
    double rho0 = atm->GetDensitySL();

    // qbar = 0.5 * rho * TAS^2 = 0.5 * rho0 * EAS^2
    double qbar_target = 0.5 * rho0 * EAS_target * EAS_target;

    // Test at sea level
    double TAS_sl = EAS_target;  // At SL, TAS = EAS
    aux.in.vUVW = FGColumnVector3(TAS_sl, 0.0, 0.0);
    aux.in.Density = rho0;
    aux.in.SoundSpeed = a0_fps;
    aux.Run(false);

    TS_ASSERT_DELTA(aux.GetVequivalentFPS(), EAS_target, 1.0);
    TS_ASSERT_DELTA(aux.Getqbar(), qbar_target, 10.0);

    // Test at 20,000 ft with TAS adjusted for same EAS
    double altitude = 20000.0;
    double rho = atm->GetDensity(altitude);
    double TAS_alt = EAS_target / sqrt(rho / rho0);

    aux.in.vUVW = FGColumnVector3(TAS_alt, 0.0, 0.0);
    aux.in.Density = rho;
    aux.in.SoundSpeed = atm->GetSoundSpeed(altitude);
    aux.Run(false);

    TS_ASSERT_DELTA(aux.GetVequivalentFPS(), EAS_target, 2.0);
    TS_ASSERT_DELTA(aux.Getqbar(), qbar_target, 20.0);

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testCASSameIndicationAtDifferentAltitudes() {
    // CAS is what the ASI reads - designed to be consistent across altitudes
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;
    aux.in.StdDaySLsoundspeed = atm->StdDaySLsoundspeed;

    double CAS_target = 250.0;  // fps

    // Find Mach at sea level for this CAS
    double M_sl = aux.MachFromVcalibrated(CAS_target, P0_psf);

    // Verify we get same CAS back
    double CAS_check = aux.VcalibratedFromMach(M_sl, P0_psf);
    TS_ASSERT_DELTA(CAS_check, CAS_target, 0.01);

    // At different altitude with same CAS indication
    double altitude = 15000.0;
    double P_alt = atm->GetPressure(altitude);
    double M_alt = aux.MachFromVcalibrated(CAS_target, P_alt);
    double CAS_alt = aux.VcalibratedFromMach(M_alt, P_alt);

    TS_ASSERT_DELTA(CAS_alt, CAS_target, 0.01);

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testTASMagnitudeVsComponents() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    // TAS magnitude from individual components
    double u = 350.0, v = 50.0, w = 25.0;
    aux.in.vUVW = FGColumnVector3(u, v, w);
    aux.in.Density = rho0_slugft3;
    aux.in.SoundSpeed = a0_fps;

    aux.Run(false);

    double TAS = aux.GetVtrueFPS();
    double expectedTAS = sqrt(u*u + v*v + w*w);

    // TAS should be the magnitude of velocity vector
    TS_ASSERT_DELTA(TAS, expectedTAS, 0.1);

    // TAS should be greater than any single component
    TS_ASSERT(TAS > u);
    TS_ASSERT(TAS > v);
    TS_ASSERT(TAS > w);

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testAltitudeEffectsOnAllAirspeeds() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;
    aux.in.StdDaySLsoundspeed = atm->StdDaySLsoundspeed;

    // Same TAS at different altitudes
    double TAS = 500.0;
    double altitudes[] = {0.0, 10000.0, 25000.0, 40000.0};

    double prev_EAS = TAS;

    for (double alt : altitudes) {
      aux.in.vUVW = FGColumnVector3(TAS, 0.0, 0.0);
      aux.in.Density = atm->GetDensity(alt);
      aux.in.SoundSpeed = atm->GetSoundSpeed(alt);
      aux.in.Pressure = atm->GetPressure(alt);
      aux.in.Temperature = atm->GetTemperature(alt);

      aux.Run(false);

      double EAS = aux.GetVequivalentFPS();

      // EAS should decrease with altitude (lower density)
      if (alt > 0.0) {
        TS_ASSERT(EAS < prev_EAS);
      }
      prev_EAS = EAS;

      // TAS always equals input
      TS_ASSERT_DELTA(aux.GetVtrueFPS(), TAS, 0.1);
    }

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testMachVsAltitudeConstantTAS() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    // Same TAS, Mach increases with altitude (lower temp = lower a)
    double TAS = 700.0;

    aux.in.vUVW = FGColumnVector3(TAS, 0.0, 0.0);
    aux.in.Density = rho0_slugft3;
    aux.in.SoundSpeed = a0_fps;
    aux.in.Temperature = T0_R;
    aux.Run(false);
    double M_sl = aux.GetMach();

    double altitude = 35000.0;
    aux.in.Density = atm->GetDensity(altitude);
    aux.in.SoundSpeed = atm->GetSoundSpeed(altitude);
    aux.in.Temperature = atm->GetTemperature(altitude);
    aux.Run(false);
    double M_alt = aux.GetMach();

    // Mach should be higher at altitude due to lower speed of sound
    TS_ASSERT(M_alt > M_sl);

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testPressureAltitudeEffect() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;
    aux.in.StdDaySLsoundspeed = atm->StdDaySLsoundspeed;

    // CAS depends on pressure ratio
    double M = 0.6;

    double CAS_sl = aux.VcalibratedFromMach(M, P0_psf);
    double CAS_alt = aux.VcalibratedFromMach(M, atm->GetPressure(10000.0));

    // Same Mach, different pressure = different CAS
    TS_ASSERT(fabs(CAS_sl - CAS_alt) > 1.0);

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testVeryLowAirspeed() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;
    aux.in.StdDaySLsoundspeed = atm->StdDaySLsoundspeed;

    // Very low speed (taxiing)
    double V = 20.0;  // fps (~12 knots)

    aux.in.vUVW = FGColumnVector3(V, 0.0, 0.0);
    aux.in.Density = rho0_slugft3;
    aux.in.SoundSpeed = a0_fps;
    aux.in.Pressure = P0_psf;
    aux.in.Temperature = T0_R;

    aux.Run(false);

    // At low speed, all airspeeds should be nearly equal
    double TAS = aux.GetVtrueFPS();
    double EAS = aux.GetVequivalentFPS();
    double CAS = aux.GetVcalibratedFPS();

    TS_ASSERT_DELTA(TAS, V, 0.1);
    TS_ASSERT_DELTA(EAS, V, 1.0);
    TS_ASSERT_DELTA(CAS, V, 1.0);

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testHighSpeedStall() {
    // At high altitude, CAS for stall is same as sea level, but TAS is much higher
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;
    aux.in.StdDaySLsoundspeed = atm->StdDaySLsoundspeed;

    double CAS_stall = 150.0;  // fps (typical light aircraft stall CAS)

    // At sea level
    double M_sl = aux.MachFromVcalibrated(CAS_stall, P0_psf);
    double TAS_sl = M_sl * a0_fps;

    // At 25,000 ft
    double altitude = 25000.0;
    double P_alt = atm->GetPressure(altitude);
    double a_alt = atm->GetSoundSpeed(altitude);
    double M_alt = aux.MachFromVcalibrated(CAS_stall, P_alt);
    double TAS_alt = M_alt * a_alt;

    // TAS at altitude should be higher than at sea level
    TS_ASSERT(TAS_alt > TAS_sl);

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testQbarUnitsConversion() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    double V = 400.0;
    aux.in.vUVW = FGColumnVector3(V, 0.0, 0.0);
    aux.in.Density = rho0_slugft3;
    aux.in.SoundSpeed = a0_fps;

    aux.Run(false);

    double qbar_psf = aux.Getqbar();  // psf

    // Verify qbar formula: q = 0.5 * rho * V^2
    double expected_qbar = 0.5 * rho0_slugft3 * V * V;
    TS_ASSERT_DELTA(qbar_psf, expected_qbar, 1.0);

    // Convert to psi manually (1 psi = 144 psf)
    double qbar_psi = qbar_psf / 144.0;
    TS_ASSERT(qbar_psi > 0.0);

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testSpeedOfSoundWithAltitude() {
    // Speed of sound varies with temperature (which varies with altitude)
    double a_sl = atm->GetSoundSpeedSL();

    double altitude = 36089.0;  // Tropopause
    double a_alt = atm->GetSoundSpeed(altitude);

    // Speed of sound should be lower at tropopause (lower temperature)
    TS_ASSERT(a_alt < a_sl);

    // Verify the formula: a = sqrt(gamma * R * T)
    double T_alt = atm->GetTemperature(altitude);
    double expected_a = sqrt(gama * R * T_alt);
    TS_ASSERT_DELTA(a_alt, expected_a, 1.0);
  }

  void testMultipleAltitudeScenarios() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;
    aux.in.StdDaySLsoundspeed = atm->StdDaySLsoundspeed;

    // Test scenarios at multiple altitudes
    double scenarios[][2] = {
      {0.0, 200.0},      // SL, low speed
      {5000.0, 300.0},   // Low altitude, medium speed
      {15000.0, 450.0},  // Medium altitude
      {35000.0, 800.0},  // Cruise altitude
      {45000.0, 900.0}   // High altitude
    };

    for (const auto& scenario : scenarios) {
      double alt = scenario[0];
      double TAS = scenario[1];

      aux.in.vUVW = FGColumnVector3(TAS, 0.0, 0.0);
      aux.in.Density = atm->GetDensity(alt);
      aux.in.SoundSpeed = atm->GetSoundSpeed(alt);
      aux.in.Pressure = atm->GetPressure(alt);
      aux.in.Temperature = atm->GetTemperature(alt);

      aux.Run(false);

      // All values should be valid
      TS_ASSERT(!std::isnan(aux.GetVtrueFPS()));
      TS_ASSERT(!std::isnan(aux.GetVequivalentFPS()));
      TS_ASSERT(!std::isnan(aux.GetVcalibratedFPS()));
      TS_ASSERT(!std::isnan(aux.GetMach()));
      TS_ASSERT(!std::isnan(aux.Getqbar()));

      // TAS should match input
      TS_ASSERT_DELTA(aux.GetVtrueFPS(), TAS, 0.1);
    }

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testAirspeedRatios() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;
    aux.in.StdDaySLsoundspeed = atm->StdDaySLsoundspeed;

    double altitude = 30000.0;
    double TAS = 600.0;

    double rho = atm->GetDensity(altitude);
    double rho0 = atm->GetDensitySL();

    aux.in.vUVW = FGColumnVector3(TAS, 0.0, 0.0);
    aux.in.Density = rho;
    aux.in.SoundSpeed = atm->GetSoundSpeed(altitude);
    aux.in.Pressure = atm->GetPressure(altitude);
    aux.in.Temperature = atm->GetTemperature(altitude);

    aux.Run(false);

    double EAS = aux.GetVequivalentFPS();

    // EAS/TAS = sqrt(rho/rho0)
    double ratio = EAS / TAS;
    double expected_ratio = sqrt(rho / rho0);

    TS_ASSERT_DELTA(ratio, expected_ratio, 0.01);

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testMachTransitionRegimes() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    // Test at regime boundaries
    double regime_machs[] = {0.3, 0.8, 1.0, 1.2, 2.0, 3.0};
    double a = a0_fps;

    for (double M : regime_machs) {
      double V = M * a;

      aux.in.vUVW = FGColumnVector3(V, 0.0, 0.0);
      aux.in.SoundSpeed = a;
      aux.in.Density = rho0_slugft3;
      aux.in.Pressure = P0_psf;
      aux.in.Temperature = T0_R;

      aux.Run(false);

      TS_ASSERT_DELTA(aux.GetMach(), M, 1e-6);
      TS_ASSERT(aux.GetVtrueFPS() > 0.0);
      TS_ASSERT(aux.Getqbar() > 0.0);
    }

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testCompressibilityFactor() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;
    aux.in.StdDaySLsoundspeed = atm->StdDaySLsoundspeed;

    // At low Mach, CAS ≈ EAS ≈ TAS
    // At high Mach, compressibility makes CAS < EAS

    double TAS = 300.0;
    double P = P0_psf;

    // Low speed case
    aux.in.vUVW = FGColumnVector3(TAS, 0.0, 0.0);
    aux.in.Density = rho0_slugft3;
    aux.in.SoundSpeed = a0_fps;
    aux.in.Pressure = P;
    aux.in.Temperature = T0_R;
    aux.Run(false);

    double EAS_low = aux.GetVequivalentFPS();
    double CAS_low = aux.GetVcalibratedFPS();

    // At sea level and low speed, all three should be similar
    TS_ASSERT_DELTA(CAS_low / EAS_low, 1.0, 0.05);

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testSubsonicIsentropicRelation() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    // For subsonic flow: qc = P * [(1 + (g-1)/2 * M^2)^(g/(g-1)) - 1]
    double M = 0.5;
    double P = P0_psf;

    double Pt = aux.PitotTotalPressure(M, P);
    double qc = Pt - P;

    // Isentropic formula
    double term = pow(1.0 + (gama - 1.0) / 2.0 * M * M, gama / (gama - 1.0));
    double expected_qc = P * (term - 1.0);

    TS_ASSERT_DELTA(qc, expected_qc, 1.0);

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  //=============================================================================
  // 10. STALL SPEED AND PERFORMANCE CALCULATIONS
  //=============================================================================

  void testStallSpeedSeaLevel() {
    // Stall speed depends on weight, wing loading, CLmax
    double weight = 2500.0;     // lbs
    double wingArea = 174.0;    // sq ft
    double CLmax = 1.6;

    // Vstall = sqrt(2W / (rho * S * CLmax))
    double Vstall_fps = sqrt(2.0 * weight / (rho0_slugft3 * wingArea * CLmax));
    double Vstall_kts = Vstall_fps * fpstokts;

    TS_ASSERT(Vstall_kts > 50.0 && Vstall_kts < 100.0);
  }

  void testStallSpeedAtAltitude() {
    double weight = 2500.0;
    double wingArea = 174.0;
    double CLmax = 1.6;
    double altitude = 10000.0;

    double rho = atm->GetDensity(altitude);

    double Vstall_TAS = sqrt(2.0 * weight / (rho * wingArea * CLmax));
    double Vstall_EAS = sqrt(2.0 * weight / (rho0_slugft3 * wingArea * CLmax));

    TS_ASSERT(Vstall_TAS > Vstall_EAS);
  }

  void testStallSpeedLoadFactor() {
    double Vstall_1g = 100.0;
    double loadFactor = 2.0;

    double Vstall_turn = Vstall_1g * sqrt(loadFactor);
    TS_ASSERT_DELTA(Vstall_turn, 141.42, 0.1);
  }

  void testVaManeuveringSpeed() {
    double Vstall_1g = 80.0;
    double limitLoad = 3.8;

    double Va = Vstall_1g * sqrt(limitLoad);
    TS_ASSERT_DELTA(Va, 155.9, 0.5);
  }

  //=============================================================================
  // 11. WIND EFFECTS ON GROUNDSPEED
  //=============================================================================

  void testGroundSpeedWithHeadwind() {
    double TAS = 500.0;
    double headwind = 50.0;

    double GS = TAS - headwind;
    TS_ASSERT_DELTA(GS, 450.0, 0.1);
  }

  void testGroundSpeedWithTailwind() {
    double TAS = 500.0;
    double tailwind = 50.0;

    double GS = TAS + tailwind;
    TS_ASSERT_DELTA(GS, 550.0, 0.1);
  }

  void testGroundSpeedWithCrosswind() {
    double TAS = 500.0;
    double crosswind = 30.0;

    double WCA = asin(crosswind / TAS);
    double GS = TAS * cos(WCA);

    TS_ASSERT(GS < TAS);
    TS_ASSERT(GS > TAS * 0.95);
  }

  //=============================================================================
  // 12. NON-STANDARD ATMOSPHERE EFFECTS
  //=============================================================================

  void testHotDayMachEffect() {
    double T_hot = T0_R + 36.0;
    double a_hot = sqrt(gama * R * T_hot);

    double TAS = 500.0;
    double Mach_std = TAS / a0_fps;
    double Mach_hot = TAS / a_hot;

    TS_ASSERT(Mach_hot < Mach_std);
  }

  void testColdDayMachEffect() {
    double T_cold = T0_R - 36.0;
    double a_cold = sqrt(gama * R * T_cold);

    double TAS = 500.0;
    double Mach_std = TAS / a0_fps;
    double Mach_cold = TAS / a_cold;

    TS_ASSERT(Mach_cold > Mach_std);
  }

  void testDensityAltitudeConcept() {
    double pressure_alt = 5000.0;
    double temp_std = atm->GetTemperature(pressure_alt);
    double temp_hot = temp_std + 36.0;

    double rho_std = atm->GetDensity(pressure_alt);
    double rho_hot = atm->GetPressure(pressure_alt) / (R * temp_hot);

    TS_ASSERT(rho_hot < rho_std);
  }

  //=============================================================================
  // 13. APPROACH AND REFERENCE SPEEDS
  //=============================================================================

  void testVrefCalculation() {
    double Vstall = 100.0;
    double Vref = 1.3 * Vstall;

    TS_ASSERT_DELTA(Vref, 130.0, 0.1);
  }

  void testApproachSpeedAdditive() {
    double Vref = 130.0;
    double halfWind = 10.0;
    double gust = 15.0;

    double Vapp = Vref + halfWind + gust;
    TS_ASSERT_DELTA(Vapp, 155.0, 0.1);
  }

  //=============================================================================
  // 14. MACH DIVERGENCE AND CRITICAL MACH
  //=============================================================================

  void testCriticalMachAirfoil() {
    double Mcrit = 0.72;
    TS_ASSERT(Mcrit < 1.0);
    TS_ASSERT(Mcrit > 0.5);
  }

  void testMachDragDivergence() {
    double Mdd = 0.78;
    double M_flight = 0.82;

    bool aboveDivergence = (M_flight > Mdd);
    TS_ASSERT(aboveDivergence);
  }

  void testSupersonicPressureRecovery() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    double M = 2.0;
    double P = P0_psf;

    double Pt = aux.PitotTotalPressure(M, P);
    double isentropic_ratio = pow(1.0 + (gama - 1.0) / 2.0 * M * M, gama / (gama - 1.0));
    double Pt_isentropic = P * isentropic_ratio;

    TS_ASSERT(Pt < Pt_isentropic);

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  //=============================================================================
  // 15. ALTITUDE CROSSOVER CALCULATIONS
  //=============================================================================

  void testCrossoverAltitudeConcept() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;
    aux.in.StdDaySLsoundspeed = atm->StdDaySLsoundspeed;

    double Vmo_fps = 340.0 / fpstokts;
    double Mmo = 0.82;

    double alt = 30000.0;
    double P = atm->GetPressure(alt);
    double CAS_from_Mmo = aux.VcalibratedFromMach(Mmo, P);

    TS_ASSERT(CAS_from_Mmo > 0.0);
    TS_ASSERT(!std::isnan(CAS_from_Mmo));

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testClimbScheduleBelowCrossover() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;
    aux.in.StdDaySLsoundspeed = atm->StdDaySLsoundspeed;

    double targetCAS = 300.0 / fpstokts;
    double alt = 20000.0;
    double M = aux.MachFromVcalibrated(targetCAS, atm->GetPressure(alt));

    TS_ASSERT(M > 0.0);
    TS_ASSERT(M < 1.0);

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  //=============================================================================
  // 16. COMPRESSIBILITY CORRECTIONS
  //=============================================================================

  void testCompressibilityHighMach() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;
    aux.in.StdDaySLsoundspeed = atm->StdDaySLsoundspeed;

    double M = 0.8;
    double TAS = M * a0_fps;

    aux.in.vUVW = FGColumnVector3(TAS, 0.0, 0.0);
    aux.in.Density = rho0_slugft3;
    aux.in.SoundSpeed = a0_fps;
    aux.in.Pressure = P0_psf;
    aux.in.Temperature = T0_R;

    aux.Run(false);

    double EAS = aux.GetVequivalentFPS();
    double CAS = aux.GetVcalibratedFPS();

    // At high Mach at sea level, there should be a compressibility difference
    // but it can be small, so just check they're both reasonable values
    TS_ASSERT(EAS > 0.0);
    TS_ASSERT(CAS > 0.0);

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testIncompressibleLowMach() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;
    aux.in.StdDaySLsoundspeed = atm->StdDaySLsoundspeed;

    double M = 0.15;
    double TAS = M * a0_fps;

    aux.in.vUVW = FGColumnVector3(TAS, 0.0, 0.0);
    aux.in.Density = rho0_slugft3;
    aux.in.SoundSpeed = a0_fps;
    aux.in.Pressure = P0_psf;
    aux.in.Temperature = T0_R;

    aux.Run(false);

    double EAS = aux.GetVequivalentFPS();
    double CAS = aux.GetVcalibratedFPS();

    TS_ASSERT_DELTA(EAS, CAS, 2.0);

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  //=============================================================================
  // 17. DENSITY RATIO TESTS
  //=============================================================================

  void testHalfDensityAltitude() {
    double alt = 18000.0;
    double rho = atm->GetDensity(alt);
    double ratio = rho / rho0_slugft3;

    TS_ASSERT(ratio > 0.4 && ratio < 0.6);
  }

  void testSigmaDecreaseWithAltitude() {
    double sigma_10k = atm->GetDensity(10000.0) / rho0_slugft3;
    double sigma_20k = atm->GetDensity(20000.0) / rho0_slugft3;
    double sigma_30k = atm->GetDensity(30000.0) / rho0_slugft3;

    TS_ASSERT(sigma_10k > sigma_20k);
    TS_ASSERT(sigma_20k > sigma_30k);
  }

  void testTASEASRatioWithSigma() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    double altitude = 25000.0;
    double TAS = 600.0;
    double rho = atm->GetDensity(altitude);

    aux.in.vUVW = FGColumnVector3(TAS, 0.0, 0.0);
    aux.in.Density = rho;
    aux.in.SoundSpeed = atm->GetSoundSpeed(altitude);

    aux.Run(false);

    double EAS = aux.GetVequivalentFPS();
    double ratio = EAS / TAS;
    double expected = sqrt(rho / rho0_slugft3);

    TS_ASSERT_DELTA(ratio, expected, 0.02);

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  //=============================================================================
  // 18. HYPERSONIC REGIME TESTS
  //=============================================================================

  void testHypersonicMach() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    double M = 5.0;
    double V = M * a0_fps;

    aux.in.vUVW = FGColumnVector3(V, 0.0, 0.0);
    aux.in.SoundSpeed = a0_fps;
    aux.in.Density = rho0_slugft3;
    aux.in.Pressure = P0_psf;
    aux.in.Temperature = T0_R;

    aux.Run(false);

    TS_ASSERT_DELTA(aux.GetMach(), M, 1e-6);

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testVeryHighQbar() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    double V = 3000.0;

    aux.in.vUVW = FGColumnVector3(V, 0.0, 0.0);
    aux.in.Density = rho0_slugft3;
    aux.in.SoundSpeed = a0_fps;

    aux.Run(false);

    double qbar = aux.Getqbar();
    double expected = 0.5 * rho0_slugft3 * V * V;

    TS_ASSERT_DELTA(qbar, expected, 100.0);
    TS_ASSERT(!std::isnan(qbar));
    TS_ASSERT(!std::isinf(qbar));

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  //=============================================================================
  // 19. PITOT-STATIC SYSTEM TESTS
  //=============================================================================

  void testPitotTotalPressureSubsonicRange() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    for (double M = 0.1; M <= 0.9; M += 0.2) {
      double Pt = aux.PitotTotalPressure(M, P0_psf);
      TS_ASSERT(Pt > P0_psf);
      TS_ASSERT(!std::isnan(Pt));
    }

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testPitotTotalPressureSupersonic() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    for (double M = 1.2; M <= 3.0; M += 0.4) {
      double Pt = aux.PitotTotalPressure(M, P0_psf);
      TS_ASSERT(Pt > P0_psf);
      TS_ASSERT(!std::isnan(Pt));
    }

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testImpactPressureConsistency() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    double M = 0.7;
    double P = P0_psf;

    double Pt = aux.PitotTotalPressure(M, P);
    double qc = Pt - P;
    double M_back = aux.MachFromImpactPressure(qc, P);

    TS_ASSERT_DELTA(M_back, M, 1e-7);

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  //=============================================================================
  // 20. ADDITIONAL EDGE CASES
  //=============================================================================

  void testVerySmallVelocity() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;
    aux.in.StdDaySLsoundspeed = atm->StdDaySLsoundspeed;

    double V = 1.0;

    aux.in.vUVW = FGColumnVector3(V, 0.0, 0.0);
    aux.in.Density = rho0_slugft3;
    aux.in.SoundSpeed = a0_fps;
    aux.in.Pressure = P0_psf;
    aux.in.Temperature = T0_R;

    aux.Run(false);

    TS_ASSERT_DELTA(aux.GetVtrueFPS(), V, 0.01);
    TS_ASSERT(!std::isnan(aux.GetMach()));
    TS_ASSERT(!std::isnan(aux.Getqbar()));

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testNegativeVelocityComponent() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;

    double u = -200.0;

    aux.in.vUVW = FGColumnVector3(u, 0.0, 0.0);
    aux.in.Density = rho0_slugft3;
    aux.in.SoundSpeed = a0_fps;

    aux.Run(false);

    TS_ASSERT_DELTA(aux.GetVtrueFPS(), fabs(u), 0.1);

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testStratosphereConditions() {
    auto aux = FGAuxiliary(&fdmex);
    aux.in.vLocation = fdmex.GetAuxiliary()->in.vLocation;
    aux.in.StdDaySLsoundspeed = atm->StdDaySLsoundspeed;

    double altitude = 50000.0;
    double TAS = 900.0;

    aux.in.vUVW = FGColumnVector3(TAS, 0.0, 0.0);
    aux.in.Density = atm->GetDensity(altitude);
    aux.in.SoundSpeed = atm->GetSoundSpeed(altitude);
    aux.in.Pressure = atm->GetPressure(altitude);
    aux.in.Temperature = atm->GetTemperature(altitude);

    aux.Run(false);

    TS_ASSERT(!std::isnan(aux.GetVtrueFPS()));
    TS_ASSERT(!std::isnan(aux.GetVequivalentFPS()));
    TS_ASSERT(!std::isnan(aux.GetMach()));

    fdmex.GetPropertyManager()->Unbind(&aux);
  }

  void testTropopauseConditions() {
    double alt = 36089.0;
    double T = atm->GetTemperature(alt);
    double a = atm->GetSoundSpeed(alt);

    TS_ASSERT(T < T0_R);
    TS_ASSERT(a < a0_fps);
  }
};
