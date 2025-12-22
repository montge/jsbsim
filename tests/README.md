# JSBSim Test Suite

This directory contains the test suite for JSBSim, including both C++ unit tests and Python integration tests.

## Coverage Targets

| Component | Target Coverage |
|-----------|----------------|
| Overall | 90%+ |
| Math (`src/math/`) | 95% |
| Core Models (`src/models/`) | 85% |
| Propulsion (`src/models/propulsion/`) | 80% |
| Flight Control (`src/models/flight_control/`) | 85% |
| I/O (`src/input_output/`) | 75% |

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

## Best Practices

1. **One concept per test**: Each test should verify one specific behavior
2. **Descriptive names**: Test names should describe what is being tested
3. **GIVEN-WHEN-THEN**: Structure tests with clear setup, action, and assertion
4. **Test edge cases**: Include boundary conditions, zero values, negative values
5. **Test error paths**: Verify error handling and invalid inputs
6. **Use tolerances**: Use appropriate floating-point tolerances for comparisons
7. **Document purpose**: Add comments explaining what each test validates
