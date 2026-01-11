# Tasks: Increase Test Coverage to 90%+

## Phase 0: Infrastructure Setup

### 0.1 Coverage Tooling
- [x] 0.1.1 Enable branch coverage in LCOV (`--rc lcov_branch_coverage=1`)
- [x] 0.1.2 Update `.github/workflows/coverage.yml` with branch coverage
- [x] 0.1.3 Create `.codecov.yml` with module flags and targets
- [ ] 0.1.4 Configure SonarCloud quality gates for coverage thresholds
- [ ] 0.1.5 Add coverage badges to README.md

### 0.2 Test Infrastructure
- [x] 0.2.1 Create `tests/fixtures/` directory for shared test data
- [x] 0.2.2 Create shared test utilities header `tests/unit_tests/TestUtilities.h`
- [x] 0.2.3 Document test patterns in `tests/README.md`

## Phase 1: Math Foundation (Target: 95% coverage)

### 1.1 FGFunction Tests
- [ ] 1.1.1 Add tests for all function types (sum, product, difference, etc.)
- [ ] 1.1.2 Add tests for nested function evaluation
- [ ] 1.1.3 Add tests for property binding
- [ ] 1.1.4 Add edge case tests (division by zero, overflow)

### 1.2 FGRealValue Tests
- [ ] 1.2.1 Create `FGRealValueTest.h` unit test suite
- [ ] 1.2.2 Test value binding and evaluation
- [ ] 1.2.3 Test property manager integration

### 1.3 Math Edge Cases
- [ ] 1.3.1 Expand FGColumnVector3Test with normalization edge cases
- [ ] 1.3.2 Expand FGMatrix33Test with singular matrix handling
- [ ] 1.3.3 Expand FGQuaternionTest with gimbal lock scenarios
- [ ] 1.3.4 Add FGLocation tests for polar coordinate edge cases

## Phase 2: Core Models (Target: 85% coverage)

### 2.1 FGAccelerations Tests
- [ ] 2.1.1 Create `FGAccelerationsTest.h` unit test suite
- [ ] 2.1.2 Test force/moment accumulation
- [ ] 2.1.3 Test body-to-inertial transformations
- [ ] 2.1.4 Test gravity gradient effects

### 2.2 FGAircraft Tests
- [ ] 2.2.1 Create `FGAircraftTest.h` unit test suite
- [ ] 2.2.2 Test aircraft configuration loading
- [ ] 2.2.3 Test wing area and reference calculations
- [ ] 2.2.4 Test CG position updates

### 2.3 FGModel Base Tests
- [ ] 2.3.1 Create `FGModelTest.h` for base model functionality
- [ ] 2.3.2 Test model rate scheduling
- [ ] 2.3.3 Test model initialization sequence

### 2.4 FGPropagate Extended Tests
- [ ] 2.4.1 Add quaternion integration tests
- [ ] 2.4.2 Add Earth-fixed vs body-fixed frame tests
- [ ] 2.4.3 Add high-altitude/orbital propagation tests
- [ ] 2.4.4 Add ground contact state transitions

### 2.5 FGAuxiliary Extended Tests
- [ ] 2.5.1 Add Mach number calculation edge cases
- [ ] 2.5.2 Add dynamic pressure at extreme altitudes
- [ ] 2.5.3 Add pilot acceleration (Nz) calculations

## Phase 3: Propulsion (Target: 80% coverage)

### 3.1 FGEngine Base Tests
- [ ] 3.1.1 Create `FGEngineTest.h` unit test suite
- [ ] 3.1.2 Test engine state machine (start, run, shutdown)
- [ ] 3.1.3 Test fuel consumption calculations
- [ ] 3.1.4 Test thrust vector orientation

### 3.2 FGPiston Tests
- [ ] 3.2.1 Create `FGPistonTest.h` unit test suite
- [ ] 3.2.2 Test manifold pressure calculations
- [ ] 3.2.3 Test mixture effects on power
- [ ] 3.2.4 Test altitude performance degradation
- [ ] 3.2.5 Test engine temperature dynamics

### 3.3 FGTurbine Extended Tests
- [ ] 3.3.1 Add compressor stall tests
- [ ] 3.3.2 Add afterburner engagement tests
- [ ] 3.3.3 Add fuel flow at various N1/N2 settings
- [ ] 3.3.4 Add bleed air extraction effects

### 3.4 FGRocket Tests
- [ ] 3.4.1 Create `FGRocketTest.h` unit test suite
- [ ] 3.4.2 Test solid propellant burn profiles
- [ ] 3.4.3 Test liquid engine throttling
- [ ] 3.4.4 Test nozzle expansion effects

### 3.5 FGElectric Tests
- [ ] 3.5.1 Create `FGElectricTest.h` unit test suite
- [ ] 3.5.2 Test motor torque curves
- [ ] 3.5.3 Test battery discharge modeling
- [ ] 3.5.4 Test regenerative braking

### 3.6 FGPropeller Tests
- [ ] 3.6.1 Create `FGPropellerTest.h` unit test suite
- [ ] 3.6.2 Test variable pitch calculations
- [ ] 3.6.3 Test advance ratio effects
- [ ] 3.6.4 Test propeller efficiency maps

### 3.7 FGRotor Tests
- [ ] 3.7.1 Create `FGRotorTest.h` unit test suite
- [ ] 3.7.2 Test collective pitch response
- [ ] 3.7.3 Test cyclic pitch effects
- [ ] 3.7.4 Test rotor RPM governing

### 3.8 Other Propulsion Components
- [ ] 3.8.1 Create `FGNozzleTest.h` for nozzle thrust calculations
- [ ] 3.8.2 Create `FGThrusterTest.h` for generic thruster tests
- [ ] 3.8.3 Create `FGTransmissionTest.h` for gearbox tests
- [ ] 3.8.4 Expand `FGTankTest.h` with fuel transfer tests

## Phase 4: Flight Control (Target: 85% coverage)

### 4.1 FGPID Tests
- [ ] 4.1.1 Create `FGPIDTest.h` unit test suite
- [ ] 4.1.2 Test proportional response
- [ ] 4.1.3 Test integral windup and anti-windup
- [ ] 4.1.4 Test derivative kick prevention
- [ ] 4.1.5 Test output limiting

### 4.2 FGSensor Base Tests
- [ ] 4.2.1 Create `FGSensorTest.h` unit test suite
- [ ] 4.2.2 Test noise injection
- [ ] 4.2.3 Test quantization effects
- [ ] 4.2.4 Test sensor lag modeling
- [ ] 4.2.5 Test failure modes

### 4.3 FGGyro Tests
- [ ] 4.3.1 Create `FGGyroTest.h` unit test suite
- [ ] 4.3.2 Test rate sensing accuracy
- [ ] 4.3.3 Test drift characteristics
- [ ] 4.3.4 Test spin axis orientation

### 4.4 Actuator Extended Tests
- [ ] 4.4.1 Add rate limiting tests
- [ ] 4.4.2 Add position limiting tests
- [ ] 4.4.3 Add hysteresis tests
- [ ] 4.4.4 Add actuator failure modes

### 4.5 Filter Extended Tests
- [ ] 4.5.1 Add all filter types (lag, lead-lag, washout, etc.)
- [ ] 4.5.2 Add frequency response validation
- [ ] 4.5.3 Add step response tests

## Phase 5: I/O and Infrastructure (Target: 75% coverage)

### 5.1 XML Parsing Tests
- [ ] 5.1.1 Create `FGXMLElementTest.h` unit test suite
- [ ] 5.1.2 Test attribute parsing
- [ ] 5.1.3 Test nested element handling
- [ ] 5.1.4 Test malformed XML error handling

### 5.2 FGXMLParse Tests
- [ ] 5.2.1 Create `FGXMLParseTest.h` unit test suite
- [ ] 5.2.2 Test file loading
- [ ] 5.2.3 Test include directive processing
- [ ] 5.2.4 Test encoding handling

### 5.3 FGScript Tests
- [ ] 5.3.1 Expand script execution tests
- [ ] 5.3.2 Test event triggering
- [ ] 5.3.3 Test property setting actions
- [ ] 5.3.4 Test notify conditions

### 5.4 FGOutput Tests
- [ ] 5.4.1 Create `FGOutputTest.h` unit test suite
- [ ] 5.4.2 Test CSV output formatting
- [ ] 5.4.3 Test socket output
- [ ] 5.4.4 Test output rate scheduling

### 5.5 FGInput Tests
- [ ] 5.5.1 Create `FGInputTest.h` unit test suite
- [ ] 5.5.2 Test socket input handling
- [ ] 5.5.3 Test property input mapping

## Phase 6: Specialized Models (Target: 80% coverage)

### 6.1 FGBuoyantForces Tests
- [ ] 6.1.1 Create `FGBuoyantForcesTest.h` unit test suite
- [ ] 6.1.2 Test gas cell buoyancy calculations
- [ ] 6.1.3 Test ballonet pressure effects
- [ ] 6.1.4 Test temperature-driven lift changes

### 6.2 FGGasCell Tests
- [ ] 6.2.1 Create `FGGasCellTest.h` unit test suite
- [ ] 6.2.2 Test gas properties (hydrogen, helium, air)
- [ ] 6.2.3 Test cell pressure limits
- [ ] 6.2.4 Test heat transfer modeling

### 6.3 FGGroundReactions Extended Tests
- [ ] 6.3.1 Add tire friction tests (dry, wet, ice)
- [ ] 6.3.2 Add brake effectiveness tests
- [ ] 6.3.3 Add gear compression tests
- [ ] 6.3.4 Add gear failure modes

### 6.4 FGExternalForce Tests
- [ ] 6.4.1 Create `FGExternalForceTest.h` unit test suite
- [ ] 6.4.2 Test force application points
- [ ] 6.4.3 Test force frame transformations

### 6.5 FGSurface Tests
- [ ] 6.5.1 Create `FGSurfaceTest.h` unit test suite
- [ ] 6.5.2 Test surface contact detection
- [ ] 6.5.3 Test friction coefficient lookup

## Phase 7: Quality Gates & Documentation

### 7.1 Coverage Enforcement
- [ ] 7.1.1 Enable Codecov PR checks with 80% patch coverage requirement
- [ ] 7.1.2 Configure SonarCloud quality gate for coverage
- [ ] 7.1.3 Add coverage check to PR template

### 7.2 Documentation
- [ ] 7.2.1 Update CONTRIBUTING.md with testing requirements
- [ ] 7.2.2 Create test writing guide
- [ ] 7.2.3 Document coverage targets per module
- [ ] 7.2.4 Add testing FAQ

### 7.3 CI Optimization
- [ ] 7.3.1 Parallelize test execution
- [ ] 7.3.2 Create fast test suite for quick PR checks
- [ ] 7.3.3 Create comprehensive test suite for nightly runs
- [ ] 7.3.4 Optimize test fixture loading

## Verification Checkpoints

- [ ] **Checkpoint 1**: Phase 0+1 complete - Math coverage at 95%
- [ ] **Checkpoint 2**: Phase 2 complete - Core models at 85%
- [ ] **Checkpoint 3**: Phase 3 complete - Propulsion at 80%
- [ ] **Checkpoint 4**: Phase 4 complete - Flight control at 85%
- [ ] **Checkpoint 5**: Phase 5 complete - I/O at 75%
- [ ] **Checkpoint 6**: Phase 6 complete - Specialized models at 80%
- [ ] **Checkpoint 7**: Overall coverage at 90%+, quality gates active
