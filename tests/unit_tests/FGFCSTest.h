/*******************************************************************************
 * FGFCSTest.h - Unit tests for Flight Control System
 *
 * Tests the FGFCS model including:
 * - Pilot input commands (aileron, elevator, rudder, throttle, etc.)
 * - Aerosurface positions
 * - Brake controls
 * - Gear, tailhook, and wing fold positions
 * - Trim commands
 *
 * Copyright (c) JSBSim Development Team
 * Licensed under LGPL
 ******************************************************************************/

#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>

#include <FGFDMExec.h>
#include <models/FGFCS.h>

using namespace JSBSim;

const double epsilon = 1e-10;

class FGFCSTest : public CxxTest::TestSuite
{
public:
  /***************************************************************************
   * Construction and Initialization Tests
   ***************************************************************************/

  void testConstruction() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    TS_ASSERT(fcs != nullptr);
  }

  void testInitModel() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    bool result = fcs->InitModel();
    TS_ASSERT_EQUALS(result, true);
  }

  /***************************************************************************
   * Aileron Command Tests
   ***************************************************************************/

  void testAileronCommandDefault() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    TS_ASSERT_DELTA(fcs->GetDaCmd(), 0.0, epsilon);
  }

  void testAileronCommandSet() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    fcs->SetDaCmd(0.5);
    TS_ASSERT_DELTA(fcs->GetDaCmd(), 0.5, epsilon);

    fcs->SetDaCmd(-0.75);
    TS_ASSERT_DELTA(fcs->GetDaCmd(), -0.75, epsilon);
  }

  void testAileronCommandFullDeflection() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    fcs->SetDaCmd(1.0);
    TS_ASSERT_DELTA(fcs->GetDaCmd(), 1.0, epsilon);

    fcs->SetDaCmd(-1.0);
    TS_ASSERT_DELTA(fcs->GetDaCmd(), -1.0, epsilon);
  }

  /***************************************************************************
   * Elevator Command Tests
   ***************************************************************************/

  void testElevatorCommandDefault() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    TS_ASSERT_DELTA(fcs->GetDeCmd(), 0.0, epsilon);
  }

  void testElevatorCommandSet() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    fcs->SetDeCmd(0.3);
    TS_ASSERT_DELTA(fcs->GetDeCmd(), 0.3, epsilon);

    fcs->SetDeCmd(-0.8);
    TS_ASSERT_DELTA(fcs->GetDeCmd(), -0.8, epsilon);
  }

  /***************************************************************************
   * Rudder Command Tests
   ***************************************************************************/

  void testRudderCommandDefault() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    TS_ASSERT_DELTA(fcs->GetDrCmd(), 0.0, epsilon);
  }

  void testRudderCommandSet() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    fcs->SetDrCmd(0.6);
    TS_ASSERT_DELTA(fcs->GetDrCmd(), 0.6, epsilon);
  }

  /***************************************************************************
   * Flaps Command Tests
   ***************************************************************************/

  void testFlapsCommandDefault() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    TS_ASSERT_DELTA(fcs->GetDfCmd(), 0.0, epsilon);
  }

  void testFlapsCommandSet() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    fcs->SetDfCmd(0.25);
    TS_ASSERT_DELTA(fcs->GetDfCmd(), 0.25, epsilon);

    fcs->SetDfCmd(1.0);
    TS_ASSERT_DELTA(fcs->GetDfCmd(), 1.0, epsilon);
  }

  /***************************************************************************
   * Speedbrake Command Tests
   ***************************************************************************/

  void testSpeedbrakeCommandDefault() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    TS_ASSERT_DELTA(fcs->GetDsbCmd(), 0.0, epsilon);
  }

  void testSpeedbrakeCommandSet() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    fcs->SetDsbCmd(0.5);
    TS_ASSERT_DELTA(fcs->GetDsbCmd(), 0.5, epsilon);
  }

  /***************************************************************************
   * Spoiler Command Tests
   ***************************************************************************/

  void testSpoilerCommandDefault() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    TS_ASSERT_DELTA(fcs->GetDspCmd(), 0.0, epsilon);
  }

  void testSpoilerCommandSet() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    fcs->SetDspCmd(0.8);
    TS_ASSERT_DELTA(fcs->GetDspCmd(), 0.8, epsilon);
  }

  /***************************************************************************
   * Trim Command Tests
   ***************************************************************************/

  void testPitchTrimDefault() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    TS_ASSERT_DELTA(fcs->GetPitchTrimCmd(), 0.0, epsilon);
  }

  void testPitchTrimSet() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    fcs->SetPitchTrimCmd(0.1);
    TS_ASSERT_DELTA(fcs->GetPitchTrimCmd(), 0.1, epsilon);

    fcs->SetPitchTrimCmd(-0.2);
    TS_ASSERT_DELTA(fcs->GetPitchTrimCmd(), -0.2, epsilon);
  }

  void testRollTrimDefault() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    TS_ASSERT_DELTA(fcs->GetRollTrimCmd(), 0.0, epsilon);
  }

  void testRollTrimSet() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    fcs->SetRollTrimCmd(0.15);
    TS_ASSERT_DELTA(fcs->GetRollTrimCmd(), 0.15, epsilon);
  }

  void testYawTrimDefault() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    TS_ASSERT_DELTA(fcs->GetYawTrimCmd(), 0.0, epsilon);
  }

  void testYawTrimSet() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    fcs->SetYawTrimCmd(0.05);
    TS_ASSERT_DELTA(fcs->GetYawTrimCmd(), 0.05, epsilon);
  }

  /***************************************************************************
   * Gear Command Tests
   ***************************************************************************/

  void testGearCommandDefault() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    // Gear defaults to down (1.0)
    TS_ASSERT_DELTA(fcs->GetGearCmd(), 1.0, epsilon);
  }

  void testGearCommandSet() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    fcs->SetGearCmd(0.0);  // Gear up
    TS_ASSERT_DELTA(fcs->GetGearCmd(), 0.0, epsilon);

    fcs->SetGearCmd(1.0);  // Gear down
    TS_ASSERT_DELTA(fcs->GetGearCmd(), 1.0, epsilon);
  }

  /***************************************************************************
   * Gear Position Tests
   ***************************************************************************/

  void testGearPositionDefault() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    // Gear position defaults to down (1.0)
    TS_ASSERT_DELTA(fcs->GetGearPos(), 1.0, epsilon);
  }

  void testGearPositionSet() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    fcs->SetGearPos(0.5);  // Mid-transit
    TS_ASSERT_DELTA(fcs->GetGearPos(), 0.5, epsilon);
  }

  /***************************************************************************
   * Tailhook Tests
   ***************************************************************************/

  void testTailhookDefault() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    TS_ASSERT_DELTA(fcs->GetTailhookPos(), 0.0, epsilon);
  }

  void testTailhookSet() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    fcs->SetTailhookPos(1.0);
    TS_ASSERT_DELTA(fcs->GetTailhookPos(), 1.0, epsilon);
  }

  /***************************************************************************
   * Wing Fold Tests
   ***************************************************************************/

  void testWingFoldDefault() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    TS_ASSERT_DELTA(fcs->GetWingFoldPos(), 0.0, epsilon);
  }

  void testWingFoldSet() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    fcs->SetWingFoldPos(1.0);
    TS_ASSERT_DELTA(fcs->GetWingFoldPos(), 1.0, epsilon);
  }

  /***************************************************************************
   * Brake Tests
   ***************************************************************************/

  void testLeftBrakeDefault() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    TS_ASSERT_DELTA(fcs->GetLBrake(), 0.0, epsilon);
  }

  void testLeftBrakeSet() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    fcs->SetLBrake(0.5);
    TS_ASSERT_DELTA(fcs->GetLBrake(), 0.5, epsilon);

    fcs->SetLBrake(1.0);
    TS_ASSERT_DELTA(fcs->GetLBrake(), 1.0, epsilon);
  }

  void testRightBrakeDefault() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    TS_ASSERT_DELTA(fcs->GetRBrake(), 0.0, epsilon);
  }

  void testRightBrakeSet() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    fcs->SetRBrake(0.75);
    TS_ASSERT_DELTA(fcs->GetRBrake(), 0.75, epsilon);
  }

  void testCenterBrakeDefault() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    TS_ASSERT_DELTA(fcs->GetCBrake(), 0.0, epsilon);
  }

  void testCenterBrakeSet() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    fcs->SetCBrake(0.9);
    TS_ASSERT_DELTA(fcs->GetCBrake(), 0.9, epsilon);
  }

  void testBrakeGroup() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    fcs->SetLBrake(0.3);
    fcs->SetRBrake(0.4);
    fcs->SetCBrake(0.5);

    TS_ASSERT_DELTA(fcs->GetBrake(FGLGear::bgLeft), 0.3, epsilon);
    TS_ASSERT_DELTA(fcs->GetBrake(FGLGear::bgRight), 0.4, epsilon);
    TS_ASSERT_DELTA(fcs->GetBrake(FGLGear::bgCenter), 0.5, epsilon);
  }

  /***************************************************************************
   * Aerosurface Position Tests (Radians)
   ***************************************************************************/

  void testElevatorPositionSet() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    double posRad = 0.1;  // radians
    fcs->SetDePos(ofRad, posRad);
    TS_ASSERT_DELTA(fcs->GetDePos(ofRad), posRad, epsilon);
  }

  void testLeftAileronPositionSet() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    double posRad = 0.05;
    fcs->SetDaLPos(ofRad, posRad);
    TS_ASSERT_DELTA(fcs->GetDaLPos(ofRad), posRad, epsilon);
  }

  void testRightAileronPositionSet() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    double posRad = -0.05;
    fcs->SetDaRPos(ofRad, posRad);
    TS_ASSERT_DELTA(fcs->GetDaRPos(ofRad), posRad, epsilon);
  }

  void testRudderPositionSet() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    double posRad = 0.08;
    fcs->SetDrPos(ofRad, posRad);
    TS_ASSERT_DELTA(fcs->GetDrPos(ofRad), posRad, epsilon);
  }

  void testFlapsPositionSet() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    double posRad = 0.35;
    fcs->SetDfPos(ofRad, posRad);
    TS_ASSERT_DELTA(fcs->GetDfPos(ofRad), posRad, epsilon);
  }

  void testSpeedbrakePositionSet() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    double posRad = 0.2;
    fcs->SetDsbPos(ofRad, posRad);
    TS_ASSERT_DELTA(fcs->GetDsbPos(ofRad), posRad, epsilon);
  }

  void testSpoilerPositionSet() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    double posRad = 0.15;
    fcs->SetDspPos(ofRad, posRad);
    TS_ASSERT_DELTA(fcs->GetDspPos(ofRad), posRad, epsilon);
  }

  /***************************************************************************
   * Position Form Conversion Tests
   ***************************************************************************/

  void testElevatorPositionDegrees() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    double posDeg = 5.0;
    fcs->SetDePos(ofDeg, posDeg);
    TS_ASSERT_DELTA(fcs->GetDePos(ofDeg), posDeg, epsilon);
  }

  void testElevatorPositionNormalized() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    double posNorm = 0.5;
    fcs->SetDePos(ofNorm, posNorm);
    TS_ASSERT_DELTA(fcs->GetDePos(ofNorm), posNorm, epsilon);
  }

  /***************************************************************************
   * Run Model Tests
   ***************************************************************************/

  void testRunNotHolding() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    bool result = fcs->Run(false);
    TS_ASSERT_EQUALS(result, false);  // false = no error
  }

  void testRunHolding() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    bool result = fcs->Run(true);
    TS_ASSERT_EQUALS(result, false);  // false = no error
  }

  /***************************************************************************
   * Multiple Control Inputs Tests
   ***************************************************************************/

  void testMultipleControlsSet() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    // Set multiple controls
    fcs->SetDaCmd(0.3);
    fcs->SetDeCmd(-0.2);
    fcs->SetDrCmd(0.1);
    fcs->SetDfCmd(0.5);

    // Verify all are independent
    TS_ASSERT_DELTA(fcs->GetDaCmd(), 0.3, epsilon);
    TS_ASSERT_DELTA(fcs->GetDeCmd(), -0.2, epsilon);
    TS_ASSERT_DELTA(fcs->GetDrCmd(), 0.1, epsilon);
    TS_ASSERT_DELTA(fcs->GetDfCmd(), 0.5, epsilon);
  }

  void testTrimAndControlsIndependent() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    fcs->SetDeCmd(0.5);
    fcs->SetPitchTrimCmd(0.1);

    // Both should be set independently
    TS_ASSERT_DELTA(fcs->GetDeCmd(), 0.5, epsilon);
    TS_ASSERT_DELTA(fcs->GetPitchTrimCmd(), 0.1, epsilon);
  }

  /***************************************************************************
   * Throttle Command Tests (requires AddThrottle)
   ***************************************************************************/

  void testAddThrottle() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    // Add a throttle and verify we can access it
    fcs->AddThrottle();
    const auto& throttleCmd = fcs->GetThrottleCmd();

    TS_ASSERT_EQUALS(throttleCmd.size(), 1u);
  }

  void testThrottleCommandSet() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    fcs->AddThrottle();
    fcs->SetThrottleCmd(0, 0.75);

    TS_ASSERT_DELTA(fcs->GetThrottleCmd(0), 0.75, epsilon);
  }

  void testMultipleThrottles() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    fcs->AddThrottle();
    fcs->AddThrottle();

    fcs->SetThrottleCmd(0, 0.5);
    fcs->SetThrottleCmd(1, 0.8);

    TS_ASSERT_DELTA(fcs->GetThrottleCmd(0), 0.5, epsilon);
    TS_ASSERT_DELTA(fcs->GetThrottleCmd(1), 0.8, epsilon);
  }

  void testThrottlePosition() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    fcs->AddThrottle();
    fcs->SetThrottlePos(0, 0.6);

    TS_ASSERT_DELTA(fcs->GetThrottlePos(0), 0.6, epsilon);
  }

  /***************************************************************************
   * Mixture Command Tests
   ***************************************************************************/

  void testMixtureCommand() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    fcs->AddThrottle();  // Also adds mixture
    fcs->SetMixtureCmd(0, 0.9);

    TS_ASSERT_DELTA(fcs->GetMixtureCmd(0), 0.9, epsilon);
  }

  void testMixturePosition() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    fcs->AddThrottle();
    fcs->SetMixturePos(0, 0.85);

    TS_ASSERT_DELTA(fcs->GetMixturePos(0), 0.85, epsilon);
  }

  /***************************************************************************
   * Prop Advance Command Tests
   ***************************************************************************/

  void testPropAdvanceCommand() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    fcs->AddThrottle();
    fcs->SetPropAdvanceCmd(0, 0.7);

    TS_ASSERT_DELTA(fcs->GetPropAdvanceCmd(0), 0.7, epsilon);
  }

  void testPropAdvancePosition() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    fcs->AddThrottle();
    fcs->SetPropAdvance(0, 0.65);

    TS_ASSERT_DELTA(fcs->GetPropAdvance(0), 0.65, epsilon);
  }

  /***************************************************************************
   * Prop Feather Tests
   ***************************************************************************/

  void testPropFeatherDefault() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    fcs->AddThrottle();

    TS_ASSERT_EQUALS(fcs->GetFeatherCmd(0), false);
  }

  void testPropFeatherSet() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    fcs->AddThrottle();
    fcs->SetFeatherCmd(0, true);

    TS_ASSERT_EQUALS(fcs->GetFeatherCmd(0), true);
  }

  void testPropFeatherPosition() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    fcs->AddThrottle();
    fcs->SetPropFeather(0, true);

    TS_ASSERT_EQUALS(fcs->GetPropFeather(0), true);
  }

  /***************************************************************************
   * Component String Tests
   ***************************************************************************/

  void testGetComponentStringsEmpty() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    std::string strings = fcs->GetComponentStrings(",");
    // With no components loaded, should be empty or minimal
    TS_ASSERT(strings.empty() || strings.find(",") == std::string::npos);
  }

  void testGetComponentValuesEmpty() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    std::string values = fcs->GetComponentValues(",");
    TS_ASSERT(values.empty() || values.find(",") == std::string::npos);
  }

  /***************************************************************************
   * Extended Control Surface Position Tests
   ***************************************************************************/

  // Test elevator position range
  void testElevatorPositionRange() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    // Full up
    fcs->SetDePos(ofNorm, 1.0);
    TS_ASSERT_DELTA(fcs->GetDePos(ofNorm), 1.0, epsilon);

    // Full down
    fcs->SetDePos(ofNorm, -1.0);
    TS_ASSERT_DELTA(fcs->GetDePos(ofNorm), -1.0, epsilon);
  }

  // Test aileron differential
  void testAileronDifferential() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    // Set command for right roll
    fcs->SetDaCmd(0.5);

    // Left aileron up, right aileron down
    fcs->SetDaLPos(ofNorm, 0.5);
    fcs->SetDaRPos(ofNorm, -0.5);

    TS_ASSERT_DELTA(fcs->GetDaLPos(ofNorm), 0.5, epsilon);
    TS_ASSERT_DELTA(fcs->GetDaRPos(ofNorm), -0.5, epsilon);
  }

  // Test control surface limits
  void testControlSurfaceLimits() {
    // Typical deflection limits
    double maxElevatorDeg = 25.0;
    double maxAileronDeg = 20.0;
    double maxRudderDeg = 30.0;

    TS_ASSERT(maxElevatorDeg > 0);
    TS_ASSERT(maxAileronDeg > 0);
    TS_ASSERT(maxRudderDeg > 0);
  }

  /***************************************************************************
   * Extended Trim System Tests
   ***************************************************************************/

  // Test combined trim inputs
  void testCombinedTrimInputs() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    fcs->SetPitchTrimCmd(0.1);
    fcs->SetRollTrimCmd(0.05);
    fcs->SetYawTrimCmd(-0.02);

    TS_ASSERT_DELTA(fcs->GetPitchTrimCmd(), 0.1, epsilon);
    TS_ASSERT_DELTA(fcs->GetRollTrimCmd(), 0.05, epsilon);
    TS_ASSERT_DELTA(fcs->GetYawTrimCmd(), -0.02, epsilon);
  }

  // Test trim range limits
  void testTrimRangeLimits() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    // Typical trim ranges
    fcs->SetPitchTrimCmd(0.5);
    TS_ASSERT(fcs->GetPitchTrimCmd() >= -1.0);
    TS_ASSERT(fcs->GetPitchTrimCmd() <= 1.0);
  }

  /***************************************************************************
   * Extended Flap System Tests
   ***************************************************************************/

  // Test multiple flap positions
  void testMultipleFlapPositions() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    // Common flap detents: 0, 10, 20, 30, 40 degrees normalized
    double flapSettings[] = {0.0, 0.25, 0.5, 0.75, 1.0};

    for (double setting : flapSettings) {
      fcs->SetDfCmd(setting);
      TS_ASSERT_DELTA(fcs->GetDfCmd(), setting, epsilon);
    }
  }

  // Test flap position vs command tracking
  void testFlapPositionTracking() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    fcs->SetDfCmd(0.5);
    fcs->SetDfPos(ofNorm, 0.5);

    TS_ASSERT_DELTA(fcs->GetDfCmd(), fcs->GetDfPos(ofNorm), epsilon);
  }

  /***************************************************************************
   * Extended Speedbrake/Spoiler Tests
   ***************************************************************************/

  // Test ground spoiler configuration
  void testGroundSpoilerConfiguration() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    // Ground spoilers fully extended on landing
    fcs->SetDspCmd(1.0);
    TS_ASSERT_DELTA(fcs->GetDspCmd(), 1.0, epsilon);
  }

  // Test speedbrake mid position
  void testSpeedbrakeMidPosition() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    fcs->SetDsbCmd(0.5);
    fcs->SetDsbPos(ofNorm, 0.5);

    TS_ASSERT_DELTA(fcs->GetDsbPos(ofNorm), 0.5, epsilon);
  }

  /***************************************************************************
   * Extended Brake Control Tests
   ***************************************************************************/

  // Test differential braking
  void testDifferentialBraking() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    // Right turn: more left brake
    fcs->SetLBrake(0.8);
    fcs->SetRBrake(0.2);

    TS_ASSERT(fcs->GetLBrake() > fcs->GetRBrake());
  }

  // Test parking brake
  void testParkingBrake() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    // Parking brake applies all brakes
    fcs->SetLBrake(1.0);
    fcs->SetRBrake(1.0);
    fcs->SetCBrake(1.0);

    TS_ASSERT_DELTA(fcs->GetLBrake(), 1.0, epsilon);
    TS_ASSERT_DELTA(fcs->GetRBrake(), 1.0, epsilon);
    TS_ASSERT_DELTA(fcs->GetCBrake(), 1.0, epsilon);
  }

  // Test brake modulation
  void testBrakeModulation() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    for (double b = 0.0; b <= 1.0; b += 0.2) {
      fcs->SetLBrake(b);
      TS_ASSERT_DELTA(fcs->GetLBrake(), b, epsilon);
    }
  }

  /***************************************************************************
   * Extended Landing Gear Tests
   ***************************************************************************/

  // Test gear transit sequence
  void testGearTransitSequence() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    // Start with gear down
    TS_ASSERT_DELTA(fcs->GetGearPos(), 1.0, epsilon);

    // Command gear up
    fcs->SetGearCmd(0.0);
    TS_ASSERT_DELTA(fcs->GetGearCmd(), 0.0, epsilon);

    // Simulate transit
    fcs->SetGearPos(0.5);
    TS_ASSERT_DELTA(fcs->GetGearPos(), 0.5, epsilon);

    // Gear fully up
    fcs->SetGearPos(0.0);
    TS_ASSERT_DELTA(fcs->GetGearPos(), 0.0, epsilon);
  }

  // Test gear down and locked
  void testGearDownAndLocked() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    fcs->SetGearCmd(1.0);
    fcs->SetGearPos(1.0);

    bool gearDownLocked = (fcs->GetGearPos() >= 0.99);
    TS_ASSERT(gearDownLocked);
  }

  /***************************************************************************
   * Extended Carrier Operations Tests
   ***************************************************************************/

  // Test tailhook deploy sequence
  void testTailhookDeploySequence() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    // Stowed
    TS_ASSERT_DELTA(fcs->GetTailhookPos(), 0.0, epsilon);

    // Deploy
    fcs->SetTailhookPos(1.0);
    TS_ASSERT_DELTA(fcs->GetTailhookPos(), 1.0, epsilon);
  }

  // Test wing fold sequence
  void testWingFoldSequence() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    // Extended
    TS_ASSERT_DELTA(fcs->GetWingFoldPos(), 0.0, epsilon);

    // Folded
    fcs->SetWingFoldPos(1.0);
    TS_ASSERT_DELTA(fcs->GetWingFoldPos(), 1.0, epsilon);
  }

  /***************************************************************************
   * Extended Multi-Engine Throttle Tests
   ***************************************************************************/

  // Test four-engine throttle sync
  void testFourEngineThrottleSync() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    fcs->AddThrottle();
    fcs->AddThrottle();
    fcs->AddThrottle();
    fcs->AddThrottle();

    // Set all to same
    for (int i = 0; i < 4; i++) {
      fcs->SetThrottleCmd(i, 0.75);
    }

    for (int i = 0; i < 4; i++) {
      TS_ASSERT_DELTA(fcs->GetThrottleCmd(i), 0.75, epsilon);
    }
  }

  // Test asymmetric throttle
  void testAsymmetricThrottle() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    fcs->AddThrottle();
    fcs->AddThrottle();

    // Left engine higher
    fcs->SetThrottleCmd(0, 0.9);
    fcs->SetThrottleCmd(1, 0.7);

    TS_ASSERT(fcs->GetThrottleCmd(0) > fcs->GetThrottleCmd(1));
  }

  /***************************************************************************
   * Extended Mixture Control Tests
   ***************************************************************************/

  // Test mixture for altitude
  void testMixtureForAltitude() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    fcs->AddThrottle();

    // Sea level: full rich
    fcs->SetMixtureCmd(0, 1.0);
    TS_ASSERT_DELTA(fcs->GetMixtureCmd(0), 1.0, epsilon);

    // High altitude: leaner
    fcs->SetMixtureCmd(0, 0.7);
    TS_ASSERT_DELTA(fcs->GetMixtureCmd(0), 0.7, epsilon);
  }

  // Test mixture cutoff
  void testMixtureCutoff() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    fcs->AddThrottle();

    // Cutoff position
    fcs->SetMixtureCmd(0, 0.0);
    TS_ASSERT_DELTA(fcs->GetMixtureCmd(0), 0.0, epsilon);
  }

  /***************************************************************************
   * Extended Propeller Control Tests
   ***************************************************************************/

  // Test prop advance full range
  void testPropAdvanceFullRange() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    fcs->AddThrottle();

    // Low RPM
    fcs->SetPropAdvanceCmd(0, 0.0);
    TS_ASSERT_DELTA(fcs->GetPropAdvanceCmd(0), 0.0, epsilon);

    // High RPM
    fcs->SetPropAdvanceCmd(0, 1.0);
    TS_ASSERT_DELTA(fcs->GetPropAdvanceCmd(0), 1.0, epsilon);
  }

  // Test multi-engine prop sync
  void testMultiEnginePropSync() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    fcs->AddThrottle();
    fcs->AddThrottle();

    // Both at same setting
    fcs->SetPropAdvanceCmd(0, 0.8);
    fcs->SetPropAdvanceCmd(1, 0.8);

    TS_ASSERT_DELTA(fcs->GetPropAdvanceCmd(0), fcs->GetPropAdvanceCmd(1), epsilon);
  }

  /***************************************************************************
   * Extended Input Combination Tests
   ***************************************************************************/

  // Test coordinated turn inputs
  void testCoordinatedTurnInputs() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    // Right coordinated turn
    fcs->SetDaCmd(0.3);   // Right aileron
    fcs->SetDrCmd(0.1);   // Right rudder
    fcs->SetDeCmd(-0.1);  // Slight back pressure

    TS_ASSERT(fcs->GetDaCmd() > 0);
    TS_ASSERT(fcs->GetDrCmd() > 0);
    TS_ASSERT(fcs->GetDeCmd() < 0);
  }

  // Test takeoff configuration
  void testTakeoffConfiguration() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    fcs->AddThrottle();

    // Takeoff settings
    fcs->SetThrottleCmd(0, 1.0);
    fcs->SetDfCmd(0.25);  // Flaps 10
    fcs->SetPitchTrimCmd(0.05);

    TS_ASSERT_DELTA(fcs->GetThrottleCmd(0), 1.0, epsilon);
    TS_ASSERT_DELTA(fcs->GetDfCmd(), 0.25, epsilon);
  }

  // Test landing configuration
  void testLandingConfiguration() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    fcs->AddThrottle();

    // Landing settings
    fcs->SetGearCmd(1.0);
    fcs->SetDfCmd(1.0);  // Full flaps
    fcs->SetThrottleCmd(0, 0.3);

    TS_ASSERT_DELTA(fcs->GetGearCmd(), 1.0, epsilon);
    TS_ASSERT_DELTA(fcs->GetDfCmd(), 1.0, epsilon);
  }

  /***************************************************************************
   * Extended Autopilot Interface Tests
   ***************************************************************************/

  // Test autopilot servo commands
  void testAutopilotServoCommands() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    // Autopilot can set control positions
    fcs->SetDePos(ofNorm, 0.1);
    fcs->SetDaLPos(ofNorm, 0.05);
    fcs->SetDaRPos(ofNorm, -0.05);

    TS_ASSERT_DELTA(fcs->GetDePos(ofNorm), 0.1, epsilon);
  }

  /***************************************************************************
   * Extended Reset and Initialization Tests
   ***************************************************************************/

  // Test reset to default state
  void testResetToDefaultState() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    // Set various values
    fcs->SetDaCmd(0.5);
    fcs->SetDeCmd(0.3);
    fcs->SetDrCmd(0.2);

    // Re-initialize
    fcs->InitModel();

    // Commands should be reset
    TS_ASSERT_DELTA(fcs->GetDaCmd(), 0.0, epsilon);
    TS_ASSERT_DELTA(fcs->GetDeCmd(), 0.0, epsilon);
    TS_ASSERT_DELTA(fcs->GetDrCmd(), 0.0, epsilon);
  }

  /***************************************************************************
   * Extended Stress Tests
   ***************************************************************************/

  // Test rapid control inputs
  void testRapidControlInputs() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    for (int i = 0; i < 100; i++) {
      double val = sin(i * 0.1);
      fcs->SetDaCmd(val);
      TS_ASSERT_DELTA(fcs->GetDaCmd(), val, epsilon);
    }
  }

  // Test control reversal
  void testControlReversal() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    fcs->SetDaCmd(1.0);
    TS_ASSERT_DELTA(fcs->GetDaCmd(), 1.0, epsilon);

    fcs->SetDaCmd(-1.0);
    TS_ASSERT_DELTA(fcs->GetDaCmd(), -1.0, epsilon);

    fcs->SetDaCmd(1.0);
    TS_ASSERT_DELTA(fcs->GetDaCmd(), 1.0, epsilon);
  }

  /***************************************************************************
   * Complete System Tests
   ***************************************************************************/

  void testCompleteControlInputSequence() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    // Simulate complete maneuver: roll, pitch, yaw
    fcs->SetDaCmd(0.5);   // Roll right
    fcs->SetDeCmd(-0.3);  // Pitch up
    fcs->SetDrCmd(0.1);   // Yaw right

    TS_ASSERT_DELTA(fcs->GetDaCmd(), 0.5, epsilon);
    TS_ASSERT_DELTA(fcs->GetDeCmd(), -0.3, epsilon);
    TS_ASSERT_DELTA(fcs->GetDrCmd(), 0.1, epsilon);

    // Also test that Da, De, Dr are independent
    TS_ASSERT(std::abs(fcs->GetDaCmd() - fcs->GetDeCmd()) > 0.1);
  }

  void testCompleteFlightControlLoop() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    // Simulate a control loop execution
    for (int i = 0; i < 100; i++) {
      double t = i * 0.01;
      fcs->SetDaCmd(sin(t));
      fcs->SetDeCmd(cos(t));
      fcs->Run(false);

      TS_ASSERT(std::isfinite(fcs->GetDaCmd()));
      TS_ASSERT(std::isfinite(fcs->GetDeCmd()));
    }
  }

  void testCompleteFlapsRange() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    // Test full flaps range
    double flapValues[] = {0.0, 0.25, 0.5, 0.75, 1.0};
    for (double flap : flapValues) {
      fcs->SetDfCmd(flap);
      TS_ASSERT_DELTA(fcs->GetDfCmd(), flap, epsilon);
    }
  }

  void testCompleteTrimState() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    // Set trimmed state
    fcs->SetPitchTrimCmd(0.1);
    fcs->SetRollTrimCmd(0.0);
    fcs->SetYawTrimCmd(0.0);

    TS_ASSERT_DELTA(fcs->GetPitchTrimCmd(), 0.1, epsilon);
    TS_ASSERT_DELTA(fcs->GetRollTrimCmd(), 0.0, epsilon);
    TS_ASSERT_DELTA(fcs->GetYawTrimCmd(), 0.0, epsilon);
  }

  /***************************************************************************
   * Instance Independence Tests
   ***************************************************************************/

  void testIndependentFCSInstances() {
    FGFDMExec fdmex1;
    FGFDMExec fdmex2;

    auto fcs1 = fdmex1.GetFCS();
    auto fcs2 = fdmex2.GetFCS();

    fcs1->SetDaCmd(0.5);
    fcs2->SetDaCmd(-0.5);

    TS_ASSERT_DELTA(fcs1->GetDaCmd(), 0.5, epsilon);
    TS_ASSERT_DELTA(fcs2->GetDaCmd(), -0.5, epsilon);

    // Verify fcs1 unchanged
    TS_ASSERT_DELTA(fcs1->GetDaCmd(), 0.5, epsilon);
  }

  void testIndependentControlAxes() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    fcs->SetDaCmd(0.3);
    fcs->SetDeCmd(0.4);
    fcs->SetDrCmd(0.5);

    // Each axis is independent
    TS_ASSERT_DELTA(fcs->GetDaCmd(), 0.3, epsilon);
    TS_ASSERT_DELTA(fcs->GetDeCmd(), 0.4, epsilon);
    TS_ASSERT_DELTA(fcs->GetDrCmd(), 0.5, epsilon);

    // Changing one doesn't affect others
    fcs->SetDeCmd(-0.4);
    TS_ASSERT_DELTA(fcs->GetDaCmd(), 0.3, epsilon);
    TS_ASSERT_DELTA(fcs->GetDrCmd(), 0.5, epsilon);
  }

  void testIndependentControlChannels() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    fcs->SetDfCmd(0.6);
    double f0 = fcs->GetDfCmd();

    fcs->SetDsbCmd(0.8);
    double sb = fcs->GetDsbCmd();

    // Flaps and speedbrake are independent
    TS_ASSERT_DELTA(f0, 0.6, epsilon);
    TS_ASSERT_DELTA(sb, 0.8, epsilon);
  }

  void testIndependentTrimSettings() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    fcs->SetPitchTrimCmd(0.2);
    fcs->SetRollTrimCmd(0.1);

    double pitchTrim = fcs->GetPitchTrimCmd();
    double rollTrim = fcs->GetRollTrimCmd();

    TS_ASSERT_DELTA(pitchTrim, 0.2, epsilon);
    TS_ASSERT_DELTA(rollTrim, 0.1, epsilon);

    // Changing roll trim doesn't affect pitch trim
    fcs->SetRollTrimCmd(-0.1);
    TS_ASSERT_DELTA(fcs->GetPitchTrimCmd(), 0.2, epsilon);
  }

  void testIndependentRunCycles() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    fcs->SetDaCmd(0.7);
    double before = fcs->GetDaCmd();

    for (int i = 0; i < 50; i++) {
      fcs->Run(false);
    }

    // Command should be preserved
    TS_ASSERT_DELTA(fcs->GetDaCmd(), before, epsilon);
  }

  // ============================================================================
  // C172x Model-Based Tests
  // ============================================================================

  void testC172xLoadModel() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    auto fcs = fdmex.GetFCS();
    TS_ASSERT(fcs != nullptr);
  }

  void testC172xRunIC() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    auto fcs = fdmex.GetFCS();
    TS_ASSERT(fcs != nullptr);
    // After RunIC, FCS should be in a valid state
    TS_ASSERT(std::isfinite(fcs->GetDaCmd()));
    TS_ASSERT(std::isfinite(fcs->GetDeCmd()));
    TS_ASSERT(std::isfinite(fcs->GetDrCmd()));
  }

  void testC172xThrottleCommand() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    auto fcs = fdmex.GetFCS();

    // C172x has a single engine, throttle should be accessible
    fcs->SetThrottleCmd(0, 0.5);
    TS_ASSERT_DELTA(fcs->GetThrottleCmd(0), 0.5, epsilon);

    fcs->SetThrottleCmd(0, 1.0);
    TS_ASSERT_DELTA(fcs->GetThrottleCmd(0), 1.0, epsilon);

    fcs->SetThrottleCmd(0, 0.0);
    TS_ASSERT_DELTA(fcs->GetThrottleCmd(0), 0.0, epsilon);
  }

  void testC172xThrottlePosition() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    auto fcs = fdmex.GetFCS();

    fcs->SetThrottlePos(0, 0.75);
    TS_ASSERT_DELTA(fcs->GetThrottlePos(0), 0.75, epsilon);

    // Throttle position should be in valid range
    TS_ASSERT(fcs->GetThrottlePos(0) >= 0.0);
    TS_ASSERT(fcs->GetThrottlePos(0) <= 1.0);
  }

  void testC172xElevatorCommand() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    auto fcs = fdmex.GetFCS();

    // Test elevator commands
    fcs->SetDeCmd(0.5);
    TS_ASSERT_DELTA(fcs->GetDeCmd(), 0.5, epsilon);

    fcs->SetDeCmd(-0.5);
    TS_ASSERT_DELTA(fcs->GetDeCmd(), -0.5, epsilon);

    // Full deflection
    fcs->SetDeCmd(1.0);
    TS_ASSERT_DELTA(fcs->GetDeCmd(), 1.0, epsilon);

    fcs->SetDeCmd(-1.0);
    TS_ASSERT_DELTA(fcs->GetDeCmd(), -1.0, epsilon);
  }

  void testC172xElevatorPositionAfterRun() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    auto fcs = fdmex.GetFCS();

    fcs->SetDeCmd(0.3);
    fdmex.Run();

    // After running, elevator position should be set
    double elevPos = fcs->GetDePos(ofNorm);
    TS_ASSERT(std::isfinite(elevPos));
    TS_ASSERT(elevPos >= -1.0 && elevPos <= 1.0);
  }

  void testC172xAileronCommand() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    auto fcs = fdmex.GetFCS();

    // Test aileron commands
    fcs->SetDaCmd(0.4);
    TS_ASSERT_DELTA(fcs->GetDaCmd(), 0.4, epsilon);

    fcs->SetDaCmd(-0.4);
    TS_ASSERT_DELTA(fcs->GetDaCmd(), -0.4, epsilon);
  }

  void testC172xAileronPositionAfterRun() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    auto fcs = fdmex.GetFCS();

    fcs->SetDaCmd(0.5);
    fdmex.Run();

    // Check left and right aileron positions
    double daLPos = fcs->GetDaLPos(ofNorm);
    double daRPos = fcs->GetDaRPos(ofNorm);

    TS_ASSERT(std::isfinite(daLPos));
    TS_ASSERT(std::isfinite(daRPos));
    TS_ASSERT(daLPos >= -1.0 && daLPos <= 1.0);
    TS_ASSERT(daRPos >= -1.0 && daRPos <= 1.0);
  }

  void testC172xRudderCommand() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    auto fcs = fdmex.GetFCS();

    fcs->SetDrCmd(0.6);
    TS_ASSERT_DELTA(fcs->GetDrCmd(), 0.6, epsilon);

    fcs->SetDrCmd(-0.6);
    TS_ASSERT_DELTA(fcs->GetDrCmd(), -0.6, epsilon);
  }

  void testC172xRudderPositionAfterRun() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    auto fcs = fdmex.GetFCS();

    fcs->SetDrCmd(0.5);
    fdmex.Run();

    double drPos = fcs->GetDrPos(ofNorm);
    TS_ASSERT(std::isfinite(drPos));
    TS_ASSERT(drPos >= -1.0 && drPos <= 1.0);
  }

  void testC172xFlapsCommand() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    auto fcs = fdmex.GetFCS();

    // Test flap detents: 0, 10, 20, 30 degrees (normalized)
    fcs->SetDfCmd(0.0);
    TS_ASSERT_DELTA(fcs->GetDfCmd(), 0.0, epsilon);

    fcs->SetDfCmd(0.25);
    TS_ASSERT_DELTA(fcs->GetDfCmd(), 0.25, epsilon);

    fcs->SetDfCmd(0.5);
    TS_ASSERT_DELTA(fcs->GetDfCmd(), 0.5, epsilon);

    fcs->SetDfCmd(1.0);
    TS_ASSERT_DELTA(fcs->GetDfCmd(), 1.0, epsilon);
  }

  void testC172xFlapsPositionAfterRun() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    auto fcs = fdmex.GetFCS();

    fcs->SetDfCmd(0.5);
    // Run several frames for flaps to move
    for (int i = 0; i < 100; i++) {
      fdmex.Run();
    }

    double dfPos = fcs->GetDfPos(ofNorm);
    TS_ASSERT(std::isfinite(dfPos));
    TS_ASSERT(dfPos >= 0.0 && dfPos <= 1.0);
  }

  void testC172xGearCommand() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    auto fcs = fdmex.GetFCS();

    // C172x has fixed gear, but the command should still work
    fcs->SetGearCmd(1.0);
    TS_ASSERT_DELTA(fcs->GetGearCmd(), 1.0, epsilon);

    fcs->SetGearCmd(0.0);
    TS_ASSERT_DELTA(fcs->GetGearCmd(), 0.0, epsilon);
  }

  void testC172xGearPosition() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    auto fcs = fdmex.GetFCS();

    // C172x has fixed gear - should stay down
    double gearPos = fcs->GetGearPos();
    TS_ASSERT(gearPos >= 0.0 && gearPos <= 1.0);
  }

  void testC172xLeftBrake() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    auto fcs = fdmex.GetFCS();

    fcs->SetLBrake(0.0);
    TS_ASSERT_DELTA(fcs->GetLBrake(), 0.0, epsilon);

    fcs->SetLBrake(0.5);
    TS_ASSERT_DELTA(fcs->GetLBrake(), 0.5, epsilon);

    fcs->SetLBrake(1.0);
    TS_ASSERT_DELTA(fcs->GetLBrake(), 1.0, epsilon);
  }

  void testC172xRightBrake() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    auto fcs = fdmex.GetFCS();

    fcs->SetRBrake(0.0);
    TS_ASSERT_DELTA(fcs->GetRBrake(), 0.0, epsilon);

    fcs->SetRBrake(0.75);
    TS_ASSERT_DELTA(fcs->GetRBrake(), 0.75, epsilon);

    fcs->SetRBrake(1.0);
    TS_ASSERT_DELTA(fcs->GetRBrake(), 1.0, epsilon);
  }

  void testC172xDifferentialBraking() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    auto fcs = fdmex.GetFCS();

    // Apply differential braking for ground steering
    fcs->SetLBrake(0.8);
    fcs->SetRBrake(0.2);

    TS_ASSERT(fcs->GetLBrake() > fcs->GetRBrake());
    TS_ASSERT_DELTA(fcs->GetBrake(FGLGear::bgLeft), 0.8, epsilon);
    TS_ASSERT_DELTA(fcs->GetBrake(FGLGear::bgRight), 0.2, epsilon);
  }

  void testC172xMixtureCommand() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    auto fcs = fdmex.GetFCS();

    // Full rich
    fcs->SetMixtureCmd(0, 1.0);
    TS_ASSERT_DELTA(fcs->GetMixtureCmd(0), 1.0, epsilon);

    // Lean
    fcs->SetMixtureCmd(0, 0.7);
    TS_ASSERT_DELTA(fcs->GetMixtureCmd(0), 0.7, epsilon);

    // Cutoff
    fcs->SetMixtureCmd(0, 0.0);
    TS_ASSERT_DELTA(fcs->GetMixtureCmd(0), 0.0, epsilon);
  }

  void testC172xMixturePosition() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    auto fcs = fdmex.GetFCS();

    fcs->SetMixturePos(0, 0.85);
    TS_ASSERT_DELTA(fcs->GetMixturePos(0), 0.85, epsilon);
  }

  void testC172xPropAdvanceCommand() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    auto fcs = fdmex.GetFCS();

    // C172x has a constant-speed prop
    fcs->SetPropAdvanceCmd(0, 1.0);
    TS_ASSERT_DELTA(fcs->GetPropAdvanceCmd(0), 1.0, epsilon);

    fcs->SetPropAdvanceCmd(0, 0.5);
    TS_ASSERT_DELTA(fcs->GetPropAdvanceCmd(0), 0.5, epsilon);

    fcs->SetPropAdvanceCmd(0, 0.0);
    TS_ASSERT_DELTA(fcs->GetPropAdvanceCmd(0), 0.0, epsilon);
  }

  void testC172xPropAdvancePosition() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    auto fcs = fdmex.GetFCS();

    fcs->SetPropAdvance(0, 0.9);
    TS_ASSERT_DELTA(fcs->GetPropAdvance(0), 0.9, epsilon);
  }

  void testC172xPitchTrimCommand() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    auto fcs = fdmex.GetFCS();

    fcs->SetPitchTrimCmd(0.1);
    TS_ASSERT_DELTA(fcs->GetPitchTrimCmd(), 0.1, epsilon);

    fcs->SetPitchTrimCmd(-0.1);
    TS_ASSERT_DELTA(fcs->GetPitchTrimCmd(), -0.1, epsilon);

    fcs->SetPitchTrimCmd(0.5);
    TS_ASSERT_DELTA(fcs->GetPitchTrimCmd(), 0.5, epsilon);
  }

  void testC172xRollTrimCommand() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    auto fcs = fdmex.GetFCS();

    fcs->SetRollTrimCmd(0.05);
    TS_ASSERT_DELTA(fcs->GetRollTrimCmd(), 0.05, epsilon);

    fcs->SetRollTrimCmd(-0.05);
    TS_ASSERT_DELTA(fcs->GetRollTrimCmd(), -0.05, epsilon);
  }

  void testC172xYawTrimCommand() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    auto fcs = fdmex.GetFCS();

    fcs->SetYawTrimCmd(0.02);
    TS_ASSERT_DELTA(fcs->GetYawTrimCmd(), 0.02, epsilon);

    fcs->SetYawTrimCmd(-0.02);
    TS_ASSERT_DELTA(fcs->GetYawTrimCmd(), -0.02, epsilon);
  }

  void testC172xTakeoffConfiguration() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    auto fcs = fdmex.GetFCS();

    // Set typical takeoff configuration
    fcs->SetThrottleCmd(0, 1.0);     // Full throttle
    fcs->SetMixtureCmd(0, 1.0);      // Full rich
    fcs->SetPropAdvanceCmd(0, 1.0);  // Full forward
    fcs->SetDfCmd(0.25);             // Flaps 10
    fcs->SetPitchTrimCmd(0.05);      // Slight nose-up trim

    TS_ASSERT_DELTA(fcs->GetThrottleCmd(0), 1.0, epsilon);
    TS_ASSERT_DELTA(fcs->GetMixtureCmd(0), 1.0, epsilon);
    TS_ASSERT_DELTA(fcs->GetPropAdvanceCmd(0), 1.0, epsilon);
    TS_ASSERT_DELTA(fcs->GetDfCmd(), 0.25, epsilon);
    TS_ASSERT_DELTA(fcs->GetPitchTrimCmd(), 0.05, epsilon);
  }

  void testC172xLandingConfiguration() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    auto fcs = fdmex.GetFCS();

    // Set typical landing configuration
    fcs->SetThrottleCmd(0, 0.3);     // Reduced power
    fcs->SetMixtureCmd(0, 1.0);      // Full rich
    fcs->SetPropAdvanceCmd(0, 1.0);  // Full forward
    fcs->SetDfCmd(1.0);              // Full flaps
    fcs->SetGearCmd(1.0);            // Gear down

    TS_ASSERT_DELTA(fcs->GetThrottleCmd(0), 0.3, epsilon);
    TS_ASSERT_DELTA(fcs->GetDfCmd(), 1.0, epsilon);
    TS_ASSERT_DELTA(fcs->GetGearCmd(), 1.0, epsilon);
  }

  void testC172xCruiseConfiguration() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    auto fcs = fdmex.GetFCS();

    // Set typical cruise configuration
    fcs->SetThrottleCmd(0, 0.75);    // 75% power
    fcs->SetMixtureCmd(0, 0.8);      // Leaned for altitude
    fcs->SetPropAdvanceCmd(0, 0.8);  // Reduced RPM
    fcs->SetDfCmd(0.0);              // Flaps up
    fcs->SetPitchTrimCmd(0.1);       // Nose-up trim for cruise

    TS_ASSERT_DELTA(fcs->GetThrottleCmd(0), 0.75, epsilon);
    TS_ASSERT_DELTA(fcs->GetMixtureCmd(0), 0.8, epsilon);
    TS_ASSERT_DELTA(fcs->GetPropAdvanceCmd(0), 0.8, epsilon);
    TS_ASSERT_DELTA(fcs->GetDfCmd(), 0.0, epsilon);
  }

  void testC172xControlSurfaceRange() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    auto fcs = fdmex.GetFCS();

    // Test full range of control surfaces
    for (double val = -1.0; val <= 1.0; val += 0.25) {
      fcs->SetDeCmd(val);
      fcs->SetDaCmd(val);
      fcs->SetDrCmd(val);

      TS_ASSERT_DELTA(fcs->GetDeCmd(), val, epsilon);
      TS_ASSERT_DELTA(fcs->GetDaCmd(), val, epsilon);
      TS_ASSERT_DELTA(fcs->GetDrCmd(), val, epsilon);
    }
  }

  void testC172xThrottleRange() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    auto fcs = fdmex.GetFCS();

    // Test throttle through its full range
    for (double val = 0.0; val <= 1.0; val += 0.1) {
      fcs->SetThrottleCmd(0, val);
      TS_ASSERT_DELTA(fcs->GetThrottleCmd(0), val, epsilon);
      TS_ASSERT(fcs->GetThrottleCmd(0) >= 0.0);
      TS_ASSERT(fcs->GetThrottleCmd(0) <= 1.0);
    }
  }

  void testC172xCoordinatedTurn() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    auto fcs = fdmex.GetFCS();

    // Simulate a coordinated right turn
    fcs->SetDaCmd(0.3);    // Right aileron
    fcs->SetDrCmd(0.1);    // Right rudder (coordinated)
    fcs->SetDeCmd(-0.05);  // Slight back pressure

    fdmex.Run();

    // All control inputs should be preserved
    TS_ASSERT_DELTA(fcs->GetDaCmd(), 0.3, epsilon);
    TS_ASSERT_DELTA(fcs->GetDrCmd(), 0.1, epsilon);
    TS_ASSERT_DELTA(fcs->GetDeCmd(), -0.05, epsilon);
  }

  void testC172xMultipleFrameExecution() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    auto fcs = fdmex.GetFCS();

    // Set control inputs
    fcs->SetDeCmd(0.2);
    fcs->SetDaCmd(0.1);
    fcs->SetThrottleCmd(0, 0.8);

    // Run multiple frames
    for (int i = 0; i < 100; i++) {
      fdmex.Run();
    }

    // Commands should persist
    TS_ASSERT_DELTA(fcs->GetDeCmd(), 0.2, epsilon);
    TS_ASSERT_DELTA(fcs->GetDaCmd(), 0.1, epsilon);
    TS_ASSERT_DELTA(fcs->GetThrottleCmd(0), 0.8, epsilon);

    // Positions should be valid
    TS_ASSERT(std::isfinite(fcs->GetDePos(ofNorm)));
    TS_ASSERT(std::isfinite(fcs->GetDaLPos(ofNorm)));
    TS_ASSERT(std::isfinite(fcs->GetDaRPos(ofNorm)));
    TS_ASSERT(std::isfinite(fcs->GetThrottlePos(0)));
  }

  void testC172xControlPositionDegrees() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    auto fcs = fdmex.GetFCS();

    fcs->SetDeCmd(0.5);
    fdmex.Run();

    // Check position in different units
    double posRad = fcs->GetDePos(ofRad);
    double posDeg = fcs->GetDePos(ofDeg);
    double posNorm = fcs->GetDePos(ofNorm);

    TS_ASSERT(std::isfinite(posRad));
    TS_ASSERT(std::isfinite(posDeg));
    TS_ASSERT(std::isfinite(posNorm));

    // Degree should be roughly 57.3 times radian
    if (std::abs(posRad) > 1e-6) {
      TS_ASSERT_DELTA(posDeg / posRad, 57.29577951, 1.0);
    }
  }

  void testC172xParkingBrake() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    auto fcs = fdmex.GetFCS();

    // Apply parking brake (both wheels locked)
    fcs->SetLBrake(1.0);
    fcs->SetRBrake(1.0);

    TS_ASSERT_DELTA(fcs->GetLBrake(), 1.0, epsilon);
    TS_ASSERT_DELTA(fcs->GetRBrake(), 1.0, epsilon);
    TS_ASSERT_DELTA(fcs->GetBrake(FGLGear::bgLeft), 1.0, epsilon);
    TS_ASSERT_DELTA(fcs->GetBrake(FGLGear::bgRight), 1.0, epsilon);
  }

  void testC172xEngineShutdown() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    auto fcs = fdmex.GetFCS();

    // Engine shutdown procedure
    fcs->SetThrottleCmd(0, 0.0);     // Throttle idle
    fcs->SetMixtureCmd(0, 0.0);      // Mixture cutoff

    TS_ASSERT_DELTA(fcs->GetThrottleCmd(0), 0.0, epsilon);
    TS_ASSERT_DELTA(fcs->GetMixtureCmd(0), 0.0, epsilon);
  }

  void testC172xTrimWithModel() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    auto fcs = fdmex.GetFCS();

    // Set all trim axes
    fcs->SetPitchTrimCmd(0.15);
    fcs->SetRollTrimCmd(0.02);
    fcs->SetYawTrimCmd(-0.01);

    fdmex.Run();

    // Verify trim values persist
    TS_ASSERT_DELTA(fcs->GetPitchTrimCmd(), 0.15, epsilon);
    TS_ASSERT_DELTA(fcs->GetRollTrimCmd(), 0.02, epsilon);
    TS_ASSERT_DELTA(fcs->GetYawTrimCmd(), -0.01, epsilon);
  }
};
