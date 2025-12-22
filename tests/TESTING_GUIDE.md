# JSBSim Testing Guide

This guide helps developers write effective unit tests for JSBSim using the CxxTest framework.

## Overview

JSBSim uses [CxxTest](http://cxxtest.com/), a unit testing framework for C++. Tests are header-based, and each test class extends `CxxTest::TestSuite`. The build system automatically discovers tests and generates executable test runners.

## Test File Structure

### Naming Convention

All test files follow the pattern `FG*Test.h` (e.g., `FGColumnVector3Test.h`, `FGPropagateTest.h`). The file should be placed in `/home/e/Development/jsbsim/tests/unit_tests/`.

### Basic Template

```cpp
#include <cxxtest/TestSuite.h>
#include <math/FGColumnVector3.h>  // Include the class under test

class FGMyClassTest : public CxxTest::TestSuite
{
public:
  void testDefaultConstructor() {
    JSBSim::FGMyClass obj;

    TS_ASSERT_EQUALS(obj.GetValue(), 0.0);
  }

  void testBasicOperation() {
    JSBSim::FGMyClass obj(5.0);

    obj.SetValue(10.0);
    TS_ASSERT_DELTA(obj.GetValue(), 10.0, 1e-9);
  }
};
```

## Adding Tests to the Build System

Edit `/home/e/Development/jsbsim/tests/unit_tests/CMakeLists.txt` and add your test name (without the `.h` extension or `Test` suffix) to the `UNIT_TESTS` list:

```cmake
set(UNIT_TESTS FGColumnVector3Test
               FGMyNewTest        # Add your test here
               FGPropagateTest
               # ... other tests
```

The CMake system will automatically create a test executable named `FGMyNewTest1`.

## Common Test Patterns

### 1. Formula-Based Tests (Pure Math)

Test mathematical operations without XML configuration or FGFDMExec:

```cpp
void testVectorAddition() {
  JSBSim::FGColumnVector3 v1(1.0, 2.0, 3.0);
  JSBSim::FGColumnVector3 v2(4.0, 5.0, 6.0);

  JSBSim::FGColumnVector3 result = v1 + v2;

  TS_ASSERT_EQUALS(result(1), 5.0);
  TS_ASSERT_EQUALS(result(2), 7.0);
  TS_ASSERT_EQUALS(result(3), 9.0);
}
```

### 2. Integration Tests with FGFDMExec

Test components within the full simulation environment:

```cpp
void testPropagateIntegration() {
  FGFDMExec fdmex;
  auto propagate = fdmex.GetPropagate();

  bool result = propagate->InitModel();
  TS_ASSERT_EQUALS(result, true);

  FGColumnVector3 uvw = propagate->GetUVW();
  TS_ASSERT(!std::isnan(uvw(1)));
  TS_ASSERT(!std::isnan(uvw(2)));
  TS_ASSERT(!std::isnan(uvw(3)));
}
```

### 3. XML-Based Tests

Test components that require XML configuration:

```cpp
#include "TestUtilities.h"

void testXMLCondition() {
  auto pm = std::make_shared<FGPropertyManager>();
  auto x = pm->GetNode("x", true);

  Element_ptr elm = readFromXML("<test> x &gt; 1.0 </test>");
  FGCondition cond(elm, pm);

  x->setDoubleValue(0.5);
  TS_ASSERT(!cond.Evaluate());

  x->setDoubleValue(2.0);
  TS_ASSERT(cond.Evaluate());
}
```

## Test Assertions

CxxTest provides several assertion macros:

| Assertion | Purpose | Example |
|-----------|---------|---------|
| `TS_ASSERT(expr)` | Assert expression is true | `TS_ASSERT(value > 0)` |
| `TS_ASSERT_EQUALS(a, b)` | Assert exact equality | `TS_ASSERT_EQUALS(count, 5)` |
| `TS_ASSERT_DELTA(a, b, tol)` | Assert floating-point equality within tolerance | `TS_ASSERT_DELTA(velocity, 100.0, 1e-6)` |
| `TS_ASSERT_THROWS(expr, exception)` | Assert expression throws exception | `TS_ASSERT_THROWS(func(), BaseException&)` |
| `TS_ASSERT(!expr)` | Assert expression is false | `TS_ASSERT(!isError)` |

### Using TestUtilities.h

JSBSim provides common test utilities:

```cpp
#include "TestUtilities.h"

using namespace JSBSimTest;

void testWithTolerance() {
  double result = calculateSomething();

  // Use DEFAULT_TOLERANCE (1e-10)
  TS_ASSERT_DELTA(result, 42.0, DEFAULT_TOLERANCE);

  // Or use LOOSE_TOLERANCE (1e-6) for less precise comparisons
  TS_ASSERT_DELTA(result, 42.0, LOOSE_TOLERANCE);
}
```

### Custom Vector/Matrix Assertions

For JSBSim math types, use specialized assertions from `TestAssertions.h`:

```cpp
#include "TestAssertions.h"

const double epsilon = 1e-8;

void testTransformationMatrix() {
  FGMatrix33 Tl2b = getTransformationMatrix();
  FGMatrix33 Tb2l = getInverseMatrix();

  FGMatrix33 product = Tl2b * Tb2l;
  TS_ASSERT_MATRIX_IS_IDENTITY(product);
}
```

## Best Practices

### 1. Keep Tests Independent

Each test method should be self-contained and not depend on other tests:

```cpp
// GOOD: Each test creates its own objects
void testFeatureA() {
  FGMyClass obj;
  obj.SetValue(5.0);
  TS_ASSERT_EQUALS(obj.GetValue(), 5.0);
}

void testFeatureB() {
  FGMyClass obj;  // Fresh object
  obj.DoSomething();
  TS_ASSERT(obj.IsReady());
}

// BAD: Don't use class member variables that persist between tests
```

### 2. Use Descriptive Test Names

Test method names should clearly describe what is being tested:

```cpp
void testNormalizeVerySmallVector()      // GOOD: Specific scenario
void testNormalize3DVector()             // GOOD: Describes input
void testConstructorWithLitterals()      // GOOD: Clear intent
void test1()                             // BAD: No information
```

### 3. Test Edge Cases

Cover boundary conditions and special cases:

```cpp
void testDivisionByZero() {
  FGColumnVector3 v(1.0, 0.5, -2.0);
  FGColumnVector3 result = v / 0.0;

  // Verify safe handling of division by zero
  TS_ASSERT_EQUALS(result(1), 0.0);
}

void testNormalizeZeroVector() {
  FGColumnVector3 v0;
  v0.Normalize();

  // Zero vector should remain zero
  TS_ASSERT_EQUALS(v0(1), 0.0);
}
```

### 4. Verify Operands Aren't Modified

When testing operations, ensure they don't have side effects:

```cpp
void testAddition() {
  FGColumnVector3 v1(1.0, 2.0, 3.0);
  FGColumnVector3 v2(4.0, 5.0, 6.0);

  FGColumnVector3 result = v1 + v2;

  // Verify operands remain unchanged
  TS_ASSERT_EQUALS(v1(1), 1.0);
  TS_ASSERT_EQUALS(v2(1), 4.0);
}
```

### 5. Test for NaN Values

When working with floating-point calculations:

```cpp
void testValidResults() {
  auto propagate = fdmex.GetPropagate();
  FGColumnVector3 velocity = propagate->GetUVW();

  TS_ASSERT(!std::isnan(velocity(1)));
  TS_ASSERT(!std::isnan(velocity(2)));
  TS_ASSERT(!std::isnan(velocity(3)));
}
```

## Running Tests

### Run All Tests

```bash
cd build
ctest -j4
```

### Run a Specific Test

```bash
ctest -R FGColumnVector3Test1
```

### Run Tests Matching a Pattern

```bash
ctest -R Vector  # Runs all tests with "Vector" in the name
```

### Run Test Executable Directly

```bash
./tests/unit_tests/FGColumnVector3Test1
```

### Verbose Output

```bash
ctest -V -R FGColumnVector3Test1
```

## Common Patterns Summary

- **Math classes**: Test constructors, operators, edge cases (zero, infinity, NaN)
- **Model classes**: Test initialization, Run() method, getter/setter consistency
- **XML parsing**: Use `readFromXML()` helper, test valid and invalid inputs
- **Property management**: Create `FGPropertyManager`, get nodes, test late binding
- **Transformation matrices**: Verify inverse relationships, check for identity matrices
- **State propagation**: Test multiple Run() calls, verify state remains valid

## Additional Resources

- CxxTest documentation: http://cxxtest.com/
- Existing tests in `/home/e/Development/jsbsim/tests/unit_tests/`
- CLAUDE.md for build and test commands
