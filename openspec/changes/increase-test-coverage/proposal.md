# Change: Increase Test Coverage to 90%+

## Why

Current test coverage is 21.8% (4,616 of 21,178 coverable lines). This low coverage means:
- Regressions can be introduced undetected
- Refactoring is risky without adequate test safety nets
- Code quality and reliability cannot be confidently assured

Industry best practices for safety-critical simulation software (which JSBSim is, given its use in flight dynamics) recommend 80-90%+ coverage.

## What Changes

### Coverage Targets
- **Overall line coverage**: 90%+ (currently 21.8%)
- **Branch coverage per module**: 80%+
- **Function coverage**: 95%+

### Testing Infrastructure
- Add comprehensive CxxTest unit tests for untested components
- Expand Python integration test suite
- Add branch coverage measurement to CI
- Configure quality gates in Codecov and SonarCloud

### Priority Components (by coverage gap)
1. **Propulsion** (`src/models/propulsion/`) - FGEngine, FGPiston, FGRocket, FGPropeller, FGNozzle, FGRotor, FGElectric
2. **Flight Control** (`src/models/flight_control/`) - FGPID, FGSensor, FGGyro
3. **Core Models** (`src/models/`) - FGAccelerations, FGAircraft, FGBuoyantForces, FGInput, FGOutput, FGSurface
4. **I/O** (`src/input_output/`) - FGXMLElement, FGXMLParse, FGScript
5. **Math** (`src/math/`) - FGFunction edge cases, FGRealValue

## Impact

- **Affected specs**: New `testing` capability spec
- **Affected code**:
  - `tests/unit_tests/` - New CxxTest files
  - `tests/` - New Python integration tests
  - `.github/workflows/coverage.yml` - Enhanced coverage reporting
  - `CMakeLists.txt` - Test registration
- **CI/CD**: Longer test runs, quality gates enforced
- **Dependencies**: None (uses existing CxxTest, pytest frameworks)

## Success Criteria

1. Overall line coverage reaches 90%+
2. Each module in `src/models/` has 80%+ branch coverage
3. Each module in `src/math/` has 90%+ coverage
4. Codecov and SonarCloud dashboards reflect targets
5. Quality gates block PRs that reduce coverage below thresholds
