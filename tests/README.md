# JSBSim Test Suite

This directory contains the test suite for JSBSim, including both C++ unit tests and Python integration tests.

## Coverage Targets

### Target Summary by Phase

| Phase | Component | Target Coverage | Path |
|-------|-----------|----------------|------|
| Overall | All Components | 90%+ | `src/` |
| Phase 1 | Math Foundation | 95% | `src/math/` |
| Phase 2 | Core Models | 85% | `src/models/` (excluding propulsion, flight_control) |
| Phase 3 | Propulsion | 80% | `src/models/propulsion/` |
| Phase 4 | Flight Control | 85% | `src/models/flight_control/` |
| Phase 5 | I/O Infrastructure | 75% | `src/input_output/`, `src/initialization/` |
| Phase 6 | Specialized Models | 80% | Atmosphere, GroundReactions, BuoyantForces |

### Component Categories

**Phase 1: Math Foundation (95% target)**
- `FGColumnVector3`, `FGMatrix33`, `FGQuaternion` - Vector/matrix math
- `FGLocation` - Geographic position handling
- `FGFunction`, `FGTable`, `FGPropertyValue` - Mathematical functions
- `FGCondition`, `FGParameter` - Conditional logic
- **Rationale**: Math is foundational and deterministic; should have highest coverage

**Phase 2: Core Models (85% target)**
- `FGPropagate` - State propagation (position, velocity, orientation)
- `FGAccelerations` - Acceleration computations
- `FGAuxiliary` - Auxiliary flight parameters
- `FGMassBalance` - Mass, CG, and inertia
- `FGInertial` - Gravity and rotating Earth
- `FGAerodynamics` - Aerodynamic forces
- **Rationale**: Critical simulation models; high coverage but some complex physics edge cases

**Phase 3: Propulsion (80% target)**
- `FGEngine`, `FGThruster`, `FGTank` - Propulsion base classes
- `FGForce` - Force application
- Engine types: Piston, Turbine, Turboprop, Electric, Rocket
- Propeller and Nozzle models
- **Rationale**: Complex domain with many engine-specific paths; slightly lower target

**Phase 4: Flight Control (85% target)**
- `FGFCS` - Flight Control System
- Control components: Actuator, Switch, Gain, Filter, etc.
- Sensors and PID controllers
- **Rationale**: Critical for simulation behavior; needs high coverage

**Phase 5: I/O Infrastructure (75% target)**
- `FGXMLElement`, `FGXMLParse` - XML parsing
- `FGPropertyManager` - Property tree
- `FGScript` - Script execution
- `FGInitialCondition`, `FGTrim` - Initialization
- Output classes: `FGOutputSocket`, `FGOutputTextFile`, etc.
- **Rationale**: I/O has many external dependencies; lower target acceptable

**Phase 6: Specialized Models (80% target)**
- `FGAtmosphere` and variants (MSIS, Mars)
- `FGGroundReactions`, `FGLGear` - Landing gear
- `FGBuoyantForces`, `FGGasCell` - Lighter-than-air
- **Rationale**: Domain-specific models with reasonable complexity

### Why These Targets Were Chosen

1. **Overall 90%+**: Industry best practice for safety-critical simulation software
2. **Math 95%**: Foundational layer must be rock-solid; deterministic and fully testable
3. **Core/FCS 85%**: Critical simulation behavior; balance between thoroughness and practicality
4. **Propulsion/Specialized 80%**: Complex domain models with many variants; some paths hard to exercise
5. **I/O 75%**: External dependencies (files, XML, sockets) make 100% coverage impractical

### How to Check Current Coverage

#### Generate Coverage Report

```bash
# Configure with coverage enabled
cmake -DCMAKE_BUILD_TYPE=Debug -DENABLE_COVERAGE=ON ..
make

# Run tests
ctest -j4

# Generate coverage report (requires lcov)
make lcov

# View HTML report
open coverage/index.html  # macOS
xdg-open coverage/index.html  # Linux
```

#### Check Coverage for Specific Module

```bash
# After generating coverage
lcov --list coverage.info | grep "src/math/"
lcov --list coverage.info | grep "src/models/propulsion/"
```

#### Per-Phase Coverage Analysis

```bash
# Math Foundation (Phase 1)
lcov --extract coverage.info "*/src/math/*" -o math_coverage.info
lcov --summary math_coverage.info

# Core Models (Phase 2) - exclude propulsion and flight_control
lcov --extract coverage.info "*/src/models/*" -o models_all.info
lcov --remove models_all.info "*/src/models/propulsion/*" "*/src/models/flight_control/*" -o core_models.info
lcov --summary core_models.info

# Propulsion (Phase 3)
lcov --extract coverage.info "*/src/models/propulsion/*" -o propulsion_coverage.info
lcov --summary propulsion_coverage.info

# Flight Control (Phase 4)
lcov --extract coverage.info "*/src/models/flight_control/*" -o fcs_coverage.info
lcov --summary fcs_coverage.info

# I/O Infrastructure (Phase 5)
lcov --extract coverage.info "*/src/input_output/*" "*/src/initialization/*" -o io_coverage.info
lcov --summary io_coverage.info

# Specialized Models (Phase 6)
lcov --extract coverage.info "*/src/models/atmosphere/*" "*/src/models/FGGroundReactions.*" "*/src/models/FGLGear.*" "*/src/models/FGBuoyantForces.*" "*/src/models/FGGasCell.*" -o specialized_coverage.info
lcov --summary specialized_coverage.info
```

## Directory Structure

```
tests/
├── README.md           # This file
├── fixtures/           # Shared test data and configurations
├── unit_tests/         # C++ CxxTest unit tests
│   ├── CMakeLists.txt
│   ├── TestUtilities.h # Shared test utilities
│   └── FG*Test.h       # Individual test suites
└── *.py                # Python integration tests
```

## Running Tests

### All Tests
```bash
cd build
ctest -j4
```

### Unit Tests Only
```bash
ctest -R Test1
```

### Specific Test
```bash
ctest -R TestAerodynamics
```

### With Coverage
```bash
cmake -DENABLE_COVERAGE=ON ..
make
ctest -R Test1
make lcov
```

## Writing Tests

### C++ Unit Tests (CxxTest)

1. Create a new header file in `unit_tests/` named `FG<Component>Test.h`
2. Include the required headers and `TestUtilities.h`
3. Inherit from `CxxTest::TestSuite`
4. Add test methods prefixed with `test`

Example:
```cpp
#include <cxxtest/TestSuite.h>
#include "TestUtilities.h"
#include <MyComponent.h>

class FGMyComponentTest : public CxxTest::TestSuite {
public:
    void testNormalOperation() {
        // GIVEN: initial state
        MyComponent comp;

        // WHEN: operation performed
        double result = comp.calculate(1.0);

        // THEN: expected result
        TS_ASSERT_DELTA(result, 2.0, 1e-10);
    }

    void testEdgeCase() {
        // Test boundary conditions
    }

    void testErrorHandling() {
        // Test invalid inputs
    }
};
```

### Python Integration Tests

1. Create a new file in `tests/` named `Test<Feature>.py`
2. Inherit from `JSBSimTestCase`
3. Add test methods prefixed with `test`

Example:
```python
from JSBSim_utils import JSBSimTestCase

class TestMyFeature(JSBSimTestCase):
    def test_normal_operation(self):
        """Test description."""
        fdm = self.create_fdm()
        fdm.load_model('aircraft')
        fdm.run_ic()
        self.assertAlmostEqual(
            fdm['metrics/expected'],
            expected_value,
            delta=tolerance
        )
```

## Test Categories

- **Unit Tests**: Test individual classes/functions in isolation
- **Integration Tests**: Test component interactions
- **System Tests**: Test full simulation scenarios
- **Regression Tests**: Validate against known-good results

## Aircraft Validation

The `ValidateAircrafts.py` test performs validation checks on all aircraft configurations:

### Checks Performed

1. **Metadata Validation**
   - Presence of `<fileheader>` with `<author>`, `<description>`, and `<license>`
   - License must include `licenseName` or `licenseURL` attribute

2. **Physical Plausibility**
   - Positive wing area, empty weight, and inertia values (Ixx, Iyy, Izz)
   - Wing loading within reasonable range (5-150 lb/ft²)
   - CG location defined in mass_balance

### Running Validation

```bash
# Run as part of test suite
ctest -R ValidateAircrafts

# View detailed output
ctest -R ValidateAircrafts -V
```

### Behavior

- Validation emits **warnings only** - tests always pass
- Warnings are visible in CI logs for visibility
- Does not block merges or fail builds

### Adding New Checks

To add validation checks, edit `ValidateAircrafts.py`:
- Metadata checks: `_validate_metadata()` method
- Plausibility checks: `_validate_physical_plausibility()` method

## Best Practices

1. **One concept per test**: Each test should verify one specific behavior
2. **Descriptive names**: Test names should describe what is being tested
3. **GIVEN-WHEN-THEN**: Structure tests with clear setup, action, and assertion
4. **Test edge cases**: Include boundary conditions, zero values, negative values
5. **Test error paths**: Verify error handling and invalid inputs
6. **Use tolerances**: Use appropriate floating-point tolerances for comparisons
7. **Document purpose**: Add comments explaining what each test validates
