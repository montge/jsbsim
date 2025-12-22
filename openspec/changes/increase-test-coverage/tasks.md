# Tasks: Increase Test Coverage to 90%+

## Phase 0: Infrastructure Setup

### 0.1 Coverage Tooling
- [x] 0.1.1 Enable branch coverage in LCOV (`--rc lcov_branch_coverage=1`)
- [x] 0.1.2 Update `.github/workflows/coverage.yml` with branch coverage
- [x] 0.1.3 Create `.codecov.yml` with module flags and targets
- [x] 0.1.4 Configure SonarCloud quality gates for coverage thresholds
- [x] 0.1.5 Add coverage badges to README.md

### 0.2 Test Infrastructure
- [x] 0.2.1 Create `tests/fixtures/` directory for shared test data
- [x] 0.2.2 Create shared test utilities header `tests/unit_tests/TestUtilities.h`
- [x] 0.2.3 Document test patterns in `tests/README.md`

## Phase 1: Math Foundation (Target: 95% coverage)

### 1.1 FGNelderMead Tests
- [x] 1.1.1 Create `FGNelderMeadTest.h` unit test suite
- [x] 1.1.2 Test 1D parabola optimization
- [x] 1.1.3 Test 2D quadratic optimization
- [x] 1.1.4 Test Rosenbrock function (challenging case)
- [x] 1.1.5 Test bound constraints
- [x] 1.1.6 Test status and getSolution methods
- [x] 1.1.7 Test max iterations handling

### 1.2 FGRungeKutta Tests
- [x] 1.2.1 Create `FGRungeKuttaTest.h` unit test suite for FGRK4 and FGRKFehlberg
- [x] 1.2.2 Test exponential growth/decay ODEs
- [x] 1.2.3 Test linear and polynomial ODEs
- [x] 1.2.4 Test sinusoidal ODEs
- [x] 1.2.5 Test logistic growth (nonlinear ODE)
- [x] 1.2.6 Test init/status/iteration methods
- [x] 1.2.7 Test Fehlberg adaptive parameters (epsilon, shrinkAvail)
- [x] 1.2.8 Compare RK4 vs Fehlberg accuracy

### 1.3 FGFunction Tests
- [x] 1.3.1 Add tests for all function types (sum, product, difference, etc.)
- [x] 1.3.2 Add tests for nested function evaluation
- [x] 1.3.3 Add tests for property binding
- [x] 1.3.4 Add edge case tests (division by zero, overflow)

### 1.4 FGRealValue Tests
- [x] 1.4.1 Expand `FGRealValueTest.h` unit test suite
- [x] 1.4.2 Test positive, negative, zero values
- [x] 1.4.3 Test very small/large values
- [x] 1.4.4 Test special IEEE values (max, min, denorm)
- [x] 1.4.5 Test IsConstant and GetValue idempotency
- [x] 1.4.6 Test GetName format

### 1.5 Math Edge Cases
- [x] 1.5.1 Expand FGColumnVector3Test with normalization edge cases
- [x] 1.5.2 Expand FGMatrix33Test with singular matrix handling
- [x] 1.5.3 FGQuaternionTest gimbal lock scenarios (already covered in FGMatrix33Test)
- [x] 1.5.4 Add FGLocation tests for polar coordinate edge cases

## Phase 2: Core Models (Target: 85% coverage)

### 2.1 FGAccelerations Tests
- [x] 2.1.1 Create `FGAccelerationsTest.h` unit test suite
- [x] 2.1.2 Test force/moment accumulation
- [x] 2.1.3 Test body-to-inertial transformations
- [x] 2.1.4 Test Newton's second law (F=ma)
- [x] 2.1.5 Test hold-down functionality
- [x] 2.1.6 Test gravity acceleration

### 2.2 FGAircraft Tests
- [x] 2.2.1 Create `FGAircraftTest.h` unit test suite
- [x] 2.2.2 Test wing area and reference calculations
- [x] 2.2.3 Test reference point getters/setters
- [x] 2.2.4 Test force aggregation from subsystems
- [x] 2.2.5 Test moment aggregation from subsystems
- [x] 2.2.6 Test buoyant forces (lighter-than-air)

### 2.3 FGModel Base Tests
- [x] 2.3.1 Create `FGModelTest.h` for base model functionality
- [x] 2.3.2 Test model rate scheduling
- [x] 2.3.3 Test model initialization sequence
- [x] 2.3.4 Test GetExec and GetName accessors
- [x] 2.3.5 Test Run behavior with holding flag

### 2.4 FGPropagate Extended Tests
- [x] 2.4.1 Create `FGPropagateTest.h` unit test suite
- [x] 2.4.2 Add velocity getter tests (body, local, ECEF, inertial)
- [x] 2.4.3 Add angular rate tests (PQR, PQRi)
- [x] 2.4.4 Add Euler angle tests with radian/degree conversion
- [x] 2.4.5 Add altitude tests (ASL, AGL, geodetic)
- [x] 2.4.6 Add transformation matrix tests (Tl2b, Tb2l, Ti2b, Tb2i, Tec2b, Tb2ec)
- [x] 2.4.7 Add quaternion tests (local, ECI, unit magnitude verification)
- [x] 2.4.8 Add location tests (latitude/longitude bounds)
- [x] 2.4.9 Add Euler trig function tests (cos^2 + sin^2 = 1)

### 2.5 FGAuxiliary Extended Tests
- [x] 2.5.1 Add Mach number calculation edge cases (transonic, hypersonic)
- [x] 2.5.2 Add dynamic pressure tests
- [x] 2.5.3 Add pilot acceleration (Nz, Nx, Ny) tests
- [x] 2.5.4 Add alpha/beta tests
- [x] 2.5.5 Add true/calibrated/equivalent airspeed tests
- [x] 2.5.6 Add total temperature tests
- [x] 2.5.7 Add Reynolds number tests
- [x] 2.5.8 Add wind-to-body transformation tests
- [x] 2.5.9 Add altitude-based Mach tests

## Phase 3: Propulsion (Target: 80% coverage)

### 3.1 FGEngine Base Tests
- [x] 3.1.1 Create `FGEngineTest.h` unit test suite
- [x] 3.1.2 Test engine state machine (start, run, shutdown)
- [x] 3.1.3 Test fuel consumption calculations
- [x] 3.1.4 Test thrust vector orientation

### 3.2 FGPiston Tests
- [x] 3.2.1 Create `FGPistonTest.h` unit test suite
- [x] 3.2.2 Test manifold pressure calculations
- [x] 3.2.3 Test mixture effects on power
- [x] 3.2.4 Test altitude performance degradation
- [x] 3.2.5 Test engine temperature dynamics

### 3.3 FGTurbine Extended Tests
- [x] 3.3.1 Add compressor stall tests
- [x] 3.3.2 Add afterburner engagement tests
- [x] 3.3.3 Add fuel flow at various N1/N2 settings
- [x] 3.3.4 Add bleed air extraction effects

### 3.4 FGRocket Tests
- [x] 3.4.1 Create `FGRocketTest.h` unit test suite
- [x] 3.4.2 Test solid propellant burn profiles
- [x] 3.4.3 Test liquid engine throttling
- [x] 3.4.4 Test nozzle expansion effects

### 3.5 FGElectric Tests
- [x] 3.5.1 Create `FGElectricTest.h` unit test suite
- [x] 3.5.2 Test motor torque curves
- [x] 3.5.3 Test battery discharge modeling
- [x] 3.5.4 Test regenerative braking

### 3.6 FGPropeller Tests
- [x] 3.6.1 Create `FGPropellerTest.h` unit test suite
- [x] 3.6.2 Test variable pitch calculations
- [x] 3.6.3 Test advance ratio effects
- [x] 3.6.4 Test propeller efficiency maps

### 3.7 FGRotor Tests
- [x] 3.7.1 Create `FGRotorTest.h` unit test suite
- [x] 3.7.2 Test collective pitch response
- [x] 3.7.3 Test cyclic pitch effects
- [x] 3.7.4 Test rotor RPM governing

### 3.8 Other Propulsion Components
- [x] 3.8.1 Create `FGNozzleTest.h` for nozzle thrust calculations
- [x] 3.8.2 Create `FGThrusterTest.h` for generic thruster tests
- [x] 3.8.3 Create `FGTransmissionTest.h` for gearbox tests
- [x] 3.8.4 Create `FGTankTest.h` with fuel density and tank calculation tests
- [x] 3.8.5 Create `FGForceTest.h` for force/moment transformations

## Phase 4: Flight Control (Target: 85% coverage)

### 4.1 FGPID Tests
- [x] 4.1.1 Create `FGPIDTest.h` unit test suite
- [x] 4.1.2 Test proportional response
- [x] 4.1.3 Test integral windup and anti-windup
- [x] 4.1.4 Test derivative kick prevention
- [x] 4.1.5 Test output limiting

### 4.2 FGSensor Base Tests
- [x] 4.2.1 Create `FGSensorTest.h` unit test suite
- [x] 4.2.2 Test noise injection
- [x] 4.2.3 Test quantization effects
- [x] 4.2.4 Test sensor lag modeling
- [x] 4.2.5 Test failure modes

### 4.3 FGGyro Tests
- [x] 4.3.1 Create `FGGyroTest.h` unit test suite
- [x] 4.3.2 Test rate sensing accuracy
- [x] 4.3.3 Test drift characteristics
- [x] 4.3.4 Test spin axis orientation

### 4.4 Actuator Extended Tests
- [x] 4.4.1 Add rate limiting tests
- [x] 4.4.2 Add position limiting tests
- [x] 4.4.3 Add hysteresis tests
- [x] 4.4.4 Add actuator failure modes

### 4.5 Filter Extended Tests
- [x] 4.5.1 Add all filter types (lag, lead-lag, washout, etc.)
- [x] 4.5.2 Add frequency response validation
- [x] 4.5.3 Add step response tests

## Phase 5: I/O and Infrastructure (Target: 75% coverage)

### 5.1 XML Parsing Tests
- [x] 5.1.1 Create `FGXMLElementTest.h` unit test suite
- [x] 5.1.2 Test attribute parsing
- [x] 5.1.3 Test nested element handling
- [x] 5.1.4 Test unit conversions (M/FT, LBS/KG, DEG/RAD, etc.)

### 5.2 FGXMLParse Tests
- [x] 5.2.1 Create `FGXMLParseTest.h` unit test suite
- [x] 5.2.2 Test file loading
- [x] 5.2.3 Test include directive processing
- [x] 5.2.4 Test encoding handling

### 5.3 FGScript Tests
- [x] 5.3.1 Expand script execution tests
- [x] 5.3.2 Test event triggering
- [x] 5.3.3 Test property setting actions
- [x] 5.3.4 Test notify conditions

### 5.4 FGOutput Tests
- [x] 5.4.1 Create `FGOutputTest.h` unit test suite
- [x] 5.4.2 Test CSV output formatting
- [x] 5.4.3 Test socket output
- [x] 5.4.4 Test output rate scheduling

### 5.5 FGInput Tests
- [x] 5.5.1 Create `FGInputTest.h` unit test suite
- [x] 5.5.2 Test socket input handling
- [x] 5.5.3 Test property input mapping

## Phase 6: Specialized Models (Target: 80% coverage)

### 6.1 FGBuoyantForces Tests
- [x] 6.1.1 Create `FGBuoyantForcesTest.h` unit test suite
- [x] 6.1.2 Test gas cell buoyancy calculations (Archimedes, ideal gas law)
- [x] 6.1.3 Test ballonet pressure effects
- [x] 6.1.4 Test temperature-driven lift changes
- [x] 6.1.5 Test hydrogen vs helium lift comparison
- [x] 6.1.6 Test valve release and overpressure

### 6.2 FGGasCell Tests
- [x] 6.2.1 Tests included in FGBuoyantForcesTest.h
- [x] 6.2.2 Test gas properties (hydrogen, helium, hot air)
- [x] 6.2.3 Test cell pressure limits
- [x] 6.2.4 Test altitude effects on gas behavior

### 6.3 FGGroundReactions Extended Tests
- [x] 6.3.1 Create `FGGroundReactionsTest.h` with friction tests (dry, wet, ice)
- [x] 6.3.2 Add brake effectiveness tests
- [x] 6.3.3 Add gear compression tests (spring/damper model)
- [x] 6.3.4 Add tire slip and lateral force tests
- [x] 6.3.5 Add hydroplaning and crosswind tests

### 6.4 FGExternalForce Tests
- [x] 6.4.1 Create `FGExternalForceTest.h` unit test suite
- [x] 6.4.2 Test force application points
- [x] 6.4.3 Test force frame transformations

### 6.5 FGSurface Tests
- [x] 6.5.1 Create `FGSurfaceTest.h` unit test suite
- [x] 6.5.2 Test surface contact detection
- [x] 6.5.3 Test friction coefficient lookup

## Phase 7: Quality Gates & Documentation

### 7.1 Coverage Enforcement
- [x] 7.1.1 Enable Codecov PR checks with 80% patch coverage requirement
- [x] 7.1.2 Configure SonarCloud quality gate for coverage
- [x] 7.1.3 Add coverage check to PR template

### 7.2 Documentation
- [x] 7.2.1 Update CONTRIBUTING.md with testing requirements
- [x] 7.2.2 Create test writing guide
- [x] 7.2.3 Document coverage targets per module
- [x] 7.2.4 Add testing FAQ

### 7.3 CI Optimization
- [x] 7.3.1 Parallelize test execution
- [x] 7.3.2 Create fast test suite for quick PR checks
- [x] 7.3.3 Create comprehensive test suite for nightly runs
- [x] 7.3.4 Optimize test fixture loading

## Verification Checkpoints

- [ ] **Checkpoint 1**: Phase 0+1 complete - Math coverage at 95%
- [ ] **Checkpoint 2**: Phase 2 complete - Core models at 85%
- [ ] **Checkpoint 3**: Phase 3 complete - Propulsion at 80%
- [ ] **Checkpoint 4**: Phase 4 complete - Flight control at 85%
- [ ] **Checkpoint 5**: Phase 5 complete - I/O at 75%
- [ ] **Checkpoint 6**: Phase 6 complete - Specialized models at 80%
- [ ] **Checkpoint 7**: Overall coverage at 90%+, quality gates active
