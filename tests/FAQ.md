# JSBSim Testing FAQ

Frequently asked questions about testing in JSBSim.

---

## 1. How do I run a single test?

### For C++ Unit Tests

**Option A: Using ctest (recommended)**
```bash
cd build
ctest -R FGColumnVector3Test1
```

**Option B: Run the test executable directly**
```bash
cd build
./tests/unit_tests/FGColumnVector3Test1
```

**Run with verbose output**
```bash
ctest -V -R FGColumnVector3Test1
```

### For Python Integration Tests

```bash
cd build
ctest -R TestActuator  # Runs TestActuator.py
```

**Run Python test directly**
```bash
cd tests
python3 TestActuator.py
```

See also: [CLAUDE.md](/home/e/Development/jsbsim/CLAUDE.md#running-a-single-c-unit-test)

---

## 2. Why is my test failing with "CxxTest not found"?

This error occurs when the CxxTest framework headers cannot be found during the build process.

### Solution

CxxTest is typically downloaded automatically by CMake. Ensure you have:

1. **Internet connection** - CMake needs to download CxxTest
2. **Proper CMake version** - Minimum version 3.15
3. **Clean build directory**

```bash
rm -rf build
mkdir build && cd build
cmake ..
make
```

If the issue persists, check that `tests/unit_tests/CMakeModules/FindCxxTest.cmake` exists in your source tree.

### Troubleshooting

Check CMake output for errors:
```bash
cmake .. 2>&1 | grep -i cxxtest
```

---

## 3. How do I add a new test file?

### For C++ Unit Tests

**Step 1: Create the test header file**

Create `/home/e/Development/jsbsim/tests/unit_tests/FGMyClassTest.h`:

```cpp
#include <cxxtest/TestSuite.h>
#include <my_component/FGMyClass.h>

class FGMyClassTest : public CxxTest::TestSuite
{
public:
  void testBasicOperation() {
    JSBSim::FGMyClass obj;
    TS_ASSERT_EQUALS(obj.GetValue(), 0.0);
  }
};
```

**Step 2: Register the test in CMake**

Edit `/home/e/Development/jsbsim/tests/unit_tests/CMakeLists.txt` and add your test name (without `.h` or `Test` suffix) to the `UNIT_TESTS` list:

```cmake
set(UNIT_TESTS FGColumnVector3Test
               FGMyClassTest        # Add here
               FGPropagateTest
               # ... other tests
```

**Step 3: Rebuild**

```bash
cd build
cmake ..
make
ctest -R FGMyClassTest1
```

### For Python Integration Tests

**Step 1: Create the test file**

Create `/home/e/Development/jsbsim/tests/TestMyFeature.py`:

```python
from JSBSim_utils import JSBSimTestCase

class TestMyFeature(JSBSimTestCase):
    def test_basic_operation(self):
        """Test description."""
        fdm = self.create_fdm()
        fdm.load_model('c172x')
        fdm.run_ic()

        self.assertAlmostEqual(fdm['position/h-sl-ft'], 0.0, delta=1e-6)
```

**Step 2: Run the test**

```bash
cd build
ctest -R TestMyFeature
```

Python tests are auto-discovered by the test framework - no CMake registration needed.

See also: [TESTING_GUIDE.md](/home/e/Development/jsbsim/tests/TESTING_GUIDE.md#test-file-structure)

---

## 4. What's the difference between unit tests and integration tests?

### Unit Tests (C++ with CxxTest)

- **Location**: `/home/e/Development/jsbsim/tests/unit_tests/`
- **Language**: C++ (header files ending in `Test.h`)
- **Purpose**: Test individual classes/functions in isolation
- **Speed**: Very fast (no XML parsing, minimal setup)
- **Scope**: Single class or small component
- **Example**: Testing `FGColumnVector3` math operations

```cpp
void testVectorAddition() {
  JSBSim::FGColumnVector3 v1(1.0, 2.0, 3.0);
  JSBSim::FGColumnVector3 v2(4.0, 5.0, 6.0);
  JSBSim::FGColumnVector3 result = v1 + v2;

  TS_ASSERT_EQUALS(result(1), 5.0);
}
```

### Integration Tests (Python)

- **Location**: `/home/e/Development/jsbsim/tests/` (root of tests directory)
- **Language**: Python (`.py` files)
- **Purpose**: Test interactions between components and full simulations
- **Speed**: Slower (loads aircraft, runs full simulation)
- **Scope**: Multiple components working together
- **Example**: Testing aircraft trim with specific configuration

```python
def test_trim_aircraft(self):
    fdm = self.create_fdm()
    fdm.load_model('c172x')
    fdm['ic/h-sl-ft'] = 5000.0
    fdm.run_ic()
    fdm.do_trim(1)  # Full trim operation

    self.assertAlmostEqual(fdm['accelerations/pdot-rad_sec2'], 0.0, delta=1e-3)
```

### When to Use Which?

- **Unit tests**: Testing pure logic, math operations, individual class methods
- **Integration tests**: Testing XML configuration, aircraft behavior, full simulation scenarios

---

## 5. How do I test XML configuration parsing?

Use the `readFromXML()` helper function from `TestUtilities.h`:

```cpp
#include <cxxtest/TestSuite.h>
#include "TestUtilities.h"
#include <input_output/FGXMLElement.h>

class FGMyXMLTest : public CxxTest::TestSuite
{
public:
  void testXMLParsing() {
    // Create XML element from string
    Element_ptr elm = readFromXML(
      "<condition>\n"
      "  property &gt; 1.0\n"
      "</condition>"
    );

    TS_ASSERT(elm != nullptr);
    TS_ASSERT_EQUALS(elm->GetName(), "condition");
  }

  void testXMLWithProperties() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto x_node = pm->GetNode("test/value", true);

    Element_ptr elm = readFromXML("<value> test/value </value>");

    x_node->setDoubleValue(42.0);
    // Use element with property manager...
  }
};
```

### Common XML Testing Patterns

**Test element attributes:**
```cpp
Element_ptr elm = readFromXML("<component name=\"mycomp\" type=\"summer\"/>");
TS_ASSERT_EQUALS(elm->GetAttributeValue("name"), "mycomp");
TS_ASSERT_EQUALS(elm->GetAttributeValue("type"), "summer");
```

**Test nested elements:**
```cpp
Element_ptr parent = readFromXML(
  "<parent>\n"
  "  <child>value</child>\n"
  "</parent>"
);

Element_ptr child = parent->FindElement("child");
TS_ASSERT(child != nullptr);
TS_ASSERT_EQUALS(child->GetDataLine(), "value");
```

**Handle XML special characters:**
```xml
&lt;   is <
&gt;   is >
&amp;  is &
```

See also: [FGXMLElementTest.h](/home/e/Development/jsbsim/tests/unit_tests/FGXMLElementTest.h)

---

## 6. How do I mock FGFDMExec for unit tests?

In most cases, you don't need to mock `FGFDMExec` - you can use the real object in a minimal configuration.

### Option A: Use Real FGFDMExec (Recommended)

```cpp
#include <FGFDMExec.h>
#include <models/FGPropagate.h>

void testWithFDMExec() {
  JSBSim::FGFDMExec fdmex;

  // Get the component you want to test
  auto propagate = fdmex.GetPropagate();

  // Initialize
  bool result = propagate->InitModel();
  TS_ASSERT(result);

  // Test operations
  JSBSim::FGColumnVector3 velocity = propagate->GetUVW();
  TS_ASSERT(!std::isnan(velocity(1)));
}
```

### Option B: Test Without FGFDMExec (Pure Unit Test)

For math classes and simple components, test them directly:

```cpp
void testStandaloneComponent() {
  // No FGFDMExec needed for pure math
  JSBSim::FGColumnVector3 v1(1.0, 2.0, 3.0);
  JSBSim::FGColumnVector3 v2(4.0, 5.0, 6.0);

  JSBSim::FGColumnVector3 result = v1 + v2;
  TS_ASSERT_EQUALS(result(1), 5.0);
}
```

### Option C: Minimal Property Manager Setup

For components that need property management but not full FGFDMExec:

```cpp
#include <input_output/FGPropertyManager.h>

void testWithProperties() {
  auto pm = std::make_shared<FGPropertyManager>();

  // Create properties
  auto altitude = pm->GetNode("position/h-sl-ft", true);
  altitude->setDoubleValue(5000.0);

  // Test component that uses properties
  TS_ASSERT_DELTA(altitude->getDoubleValue(), 5000.0, 1e-9);
}
```

### When to Use Full Integration

If you need aircraft XML configuration, use Python integration tests instead:

```python
def test_full_aircraft(self):
    fdm = self.create_fdm()
    fdm.load_model('c172x')  # Load full aircraft
    fdm.run_ic()
```

See also: [TESTING_GUIDE.md](/home/e/Development/jsbsim/tests/TESTING_GUIDE.md#common-test-patterns)

---

## 7. What tolerance should I use for floating-point comparisons?

JSBSim provides standard tolerances in `TestUtilities.h`:

```cpp
#include "TestUtilities.h"

using namespace JSBSimTest;

// Standard tolerances
DEFAULT_TOLERANCE   // 1e-10 (for high-precision math)
LOOSE_TOLERANCE     // 1e-6  (for physical calculations with error accumulation)
ANGLE_TOLERANCE     // 1e-8  (for angular values in radians)
```

### General Guidelines

| Calculation Type | Recommended Tolerance | Example |
|------------------|----------------------|---------|
| Pure math operations | `1e-10` (DEFAULT_TOLERANCE) | Vector addition, matrix multiplication |
| Trigonometric functions | `1e-8` (ANGLE_TOLERANCE) | sin, cos, atan2 |
| Integration/propagation | `1e-6` to `1e-3` (LOOSE_TOLERANCE) | Multi-step simulation, accumulated error |
| Physical measurements | `1e-6` | Altitude, velocity, forces |
| Iterative solvers | `1e-4` to `1e-2` | Trim, Newton-Raphson methods |

### Usage Examples

```cpp
#include "TestUtilities.h"
using namespace JSBSimTest;

// High-precision math
void testPreciseMath() {
  double result = 1.0 + 2.0;
  TS_ASSERT_DELTA(result, 3.0, DEFAULT_TOLERANCE);
}

// Physical calculation with error accumulation
void testPropagation() {
  double position = propagatePosition(1000);
  TS_ASSERT_DELTA(position, expected, LOOSE_TOLERANCE);
}

// Custom tolerance when needed
void testIterativeSolver() {
  double converged_value = trim();
  TS_ASSERT_DELTA(converged_value, 0.0, 1e-4);
}
```

### CxxTest Assertions

```cpp
TS_ASSERT_DELTA(actual, expected, tolerance)     // Standard floating-point comparison
TS_ASSERT_EQUALS(actual, expected)               // Exact equality (use for integers)
TS_ASSERT(expression)                            // Boolean assertion
```

### Why Use Tolerances?

Floating-point arithmetic is not exact:
```cpp
double a = 0.1 + 0.2;
// a might be 0.30000000000000004, not exactly 0.3

TS_ASSERT_EQUALS(a, 0.3);        // May fail!
TS_ASSERT_DELTA(a, 0.3, 1e-10);  // Correct approach
```

See also: [TestUtilities.h](/home/e/Development/jsbsim/tests/unit_tests/TestUtilities.h)

---

## 8. How do I run tests with coverage?

### Prerequisites

Install coverage tools:
```bash
# Ubuntu/Debian
sudo apt-get install lcov

# macOS
brew install lcov
```

### Generate Coverage Report

```bash
# Clean previous build
rm -rf build
mkdir build && cd build

# Configure with coverage enabled
cmake -DCMAKE_BUILD_TYPE=Debug ..

# Build and run tests
make
ctest

# Generate coverage report
make lcov

# View HTML report
xdg-open coverage/index.html  # Linux
open coverage/index.html      # macOS
```

### Coverage for Specific Test

```bash
ctest -R FGColumnVector3Test1
make lcov
```

### View Coverage in Terminal

```bash
lcov --list coverage.info
```

### Coverage Targets

JSBSim aims for:
- **Overall**: 90%+ line coverage
- **Math components**: 95%+
- **Core models**: 85%+
- **Propulsion**: 80%+
- **Flight control**: 85%+
- **I/O**: 75%+

### CI/CD Integration

Coverage is automatically measured in GitHub Actions. View reports:
- Check pull request comments for coverage changes
- See detailed reports in SonarCloud (if configured)

### Troubleshooting

**Issue**: No coverage generated

**Solution**: Ensure Debug build type:
```bash
cmake -DCMAKE_BUILD_TYPE=Debug ..
```

**Issue**: "make lcov" not found

**Solution**: Check that codecov module is found:
```bash
cmake .. 2>&1 | grep codecov
```

See also: [README.md](/home/e/Development/jsbsim/tests/README.md#with-coverage)

---

## 9. How do I debug a failing test?

### Method 1: Run Test Directly with Debugger

```bash
cd build

# GDB (Linux)
gdb ./tests/unit_tests/FGColumnVector3Test1
(gdb) run
(gdb) backtrace

# LLDB (macOS)
lldb ./tests/unit_tests/FGColumnVector3Test1
(lldb) run
(lldb) bt
```

### Method 2: Add Print Statements

```cpp
#include <iostream>

void testMyFunction() {
  JSBSim::FGColumnVector3 v(1.0, 2.0, 3.0);

  std::cout << "Vector v: " << v.Dump(", ") << std::endl;
  std::cout << "Magnitude: " << v.Magnitude() << std::endl;

  TS_ASSERT_DELTA(v.Magnitude(), 3.74166, 1e-5);
}
```

### Method 3: Verbose Test Output

```bash
# CTest verbose mode
ctest -V -R FGColumnVector3Test1

# Run test executable directly (shows more detail)
./tests/unit_tests/FGColumnVector3Test1

# Run with output
./tests/unit_tests/FGColumnVector3Test1 2>&1 | less
```

### Method 4: Isolate the Failing Test

Temporarily comment out other tests:

```cpp
class FGMyTest : public CxxTest::TestSuite
{
public:
  void testWorking() {
    // This passes - comment it out temporarily
    // TS_ASSERT_EQUALS(1, 1);
  }

  void testFailing() {
    // Focus on debugging this one
    TS_ASSERT_DELTA(calculate(), 42.0, 1e-6);
  }
};
```

### Method 5: Use Assert Messages

CxxTest assertions can include messages:

```cpp
TS_ASSERT_DELTA_MESSAGE(actual, expected, tolerance, "Value outside expected range");
```

### Method 6: Visual Studio Code Debug Configuration

Create `.vscode/launch.json`:

```json
{
  "version": "0.2.0",
  "configurations": [
    {
      "name": "Debug Test",
      "type": "cppdbg",
      "request": "launch",
      "program": "${workspaceFolder}/build/tests/unit_tests/FGColumnVector3Test1",
      "args": [],
      "cwd": "${workspaceFolder}/build",
      "environment": [],
      "MIMode": "gdb"
    }
  ]
}
```

### Common Debugging Scenarios

**NaN values:**
```cpp
TS_ASSERT(!std::isnan(result));  // Catches NaN early
```

**Unexpected tolerance failures:**
```cpp
double diff = std::abs(actual - expected);
std::cout << "Difference: " << diff << std::endl;
TS_ASSERT_DELTA(actual, expected, tolerance);
```

**Memory issues:**
```bash
# Run with Valgrind
valgrind ./tests/unit_tests/FGColumnVector3Test1
```

### Python Test Debugging

```python
# Add print statements
def test_feature(self):
    fdm = self.create_fdm()
    fdm.load_model('c172x')

    print(f"Altitude: {fdm['position/h-sl-ft']}")
    print(f"Velocity: {fdm['velocities/u-fps']}")

    self.assertAlmostEqual(fdm['position/h-sl-ft'], 0.0)
```

```bash
# Run with verbose Python output
python3 -v TestActuator.py
```

---

## 10. Where do I find example tests to copy?

### Best Examples by Category

#### Math Classes (Pure Unit Tests)

**[FGColumnVector3Test.h](/home/e/Development/jsbsim/tests/unit_tests/FGColumnVector3Test.h)** - Excellent example:
- Comprehensive operator testing
- Edge case coverage (zero vectors, normalization)
- Clean test structure with descriptive names

**[FGMatrix33Test.h](/home/e/Development/jsbsim/tests/unit_tests/FGMatrix33Test.h)** - Matrix operations:
- Matrix multiplication, inversion
- Transformation matrices
- Identity matrix checks

**[FGQuaternionTest.h](/home/e/Development/jsbsim/tests/unit_tests/FGQuaternionTest.h)** - Rotation math:
- Quaternion algebra
- Rotation representations

#### Property Management

**[FGPropertyManagerTest.h](/home/e/Development/jsbsim/tests/unit_tests/FGPropertyManagerTest.h)** - Property tree:
- Creating and accessing properties
- Property binding
- Type conversions

#### XML Parsing

**[FGXMLElementTest.h](/home/e/Development/jsbsim/tests/unit_tests/FGXMLElementTest.h)** - XML handling:
- Element creation and navigation
- Attribute parsing
- Data line handling
- Good use of `readFromXML()` helper

#### Flight Control Components

**[FGActuatorTest.h](/home/e/Development/jsbsim/tests/unit_tests/FGActuatorTest.h)** - Actuator dynamics:
- Rate limiting, hysteresis
- XML configuration
- Property integration

**[FGFilterTest.h](/home/e/Development/jsbsim/tests/unit_tests/FGFilterTest.h)** - Filter implementations:
- Various filter types
- Transfer functions
- State management

**[FGPIDTest.h](/home/e/Development/jsbsim/tests/unit_tests/FGPIDTest.h)** - PID controller:
- Control loop testing
- Integral windup
- Gain scheduling

#### Models

**[FGPropagateTest.h](/home/e/Development/jsbsim/tests/unit_tests/FGPropagateTest.h)** - State propagation:
- Integration with FGFDMExec
- Position/velocity updates
- Coordinate transformations

**[FGAtmosphereTest.h](/home/e/Development/jsbsim/tests/unit_tests/FGAtmosphereTest.h)** - Atmosphere models:
- Standard atmosphere calculations
- Altitude/temperature/pressure relationships

#### Utility Classes

**[TestUtilities.h](/home/e/Development/jsbsim/tests/unit_tests/TestUtilities.h)** - Shared utilities:
- Standard tolerances
- Helper functions
- Physical constants

**[TestAssertions.h](/home/e/Development/jsbsim/tests/unit_tests/TestAssertions.h)** - Custom assertions:
- Vector/matrix comparison macros
- Identity matrix checks

#### Python Integration Tests

**[TestActuator.py](/home/e/Development/jsbsim/tests/TestActuator.py)** - Actuator system test:
- Full simulation setup
- XML configuration testing
- Property-based validation

**[TestInitialConditions.py](/home/e/Development/jsbsim/tests/TestInitialConditions.py)** - IC setup:
- Aircraft loading
- Initial state configuration
- Comprehensive scenarios

### Quick Reference: What Test to Copy?

| You're Testing | Copy This Example |
|----------------|-------------------|
| Math class with operators | `FGColumnVector3Test.h` |
| XML parsing | `FGXMLElementTest.h` |
| Flight control component | `FGActuatorTest.h` or `FGFilterTest.h` |
| Model with FGFDMExec | `FGPropagateTest.h` |
| Properties | `FGPropertyManagerTest.h` |
| Full aircraft simulation | `TestInitialConditions.py` |
| I/O and scripting | `FGScriptTest.h` |

### Template to Start With

For a new C++ unit test, start with this minimal template:

```cpp
#include <cxxtest/TestSuite.h>
#include "TestUtilities.h"
#include <path/to/MyClass.h>

using namespace JSBSim;
using namespace JSBSimTest;

class FGMyClassTest : public CxxTest::TestSuite
{
public:
  void testConstruction() {
    // GIVEN: default constructor
    FGMyClass obj;

    // THEN: initialized to expected state
    TS_ASSERT_DELTA(obj.GetValue(), 0.0, DEFAULT_TOLERANCE);
  }

  void testBasicOperation() {
    // GIVEN: object with initial state
    FGMyClass obj(5.0);

    // WHEN: performing operation
    obj.SetValue(10.0);

    // THEN: result matches expectation
    TS_ASSERT_DELTA(obj.GetValue(), 10.0, DEFAULT_TOLERANCE);
  }

  void testEdgeCase() {
    // Test boundary conditions
  }
};
```

Then add your test to `CMakeLists.txt` and build!

---

## Additional Resources

- [CLAUDE.md](/home/e/Development/jsbsim/CLAUDE.md) - Build and test commands
- [TESTING_GUIDE.md](/home/e/Development/jsbsim/tests/TESTING_GUIDE.md) - Detailed testing guide
- [README.md](/home/e/Development/jsbsim/tests/README.md) - Test suite overview
- [CxxTest Documentation](http://cxxtest.com/) - Official CxxTest docs
- [All unit tests](/home/e/Development/jsbsim/tests/unit_tests/) - Browse existing tests

---

**Last Updated**: December 2025
