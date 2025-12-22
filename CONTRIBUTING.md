# Contributing to JSBSim

Thank you for your interest in contributing to JSBSim! This document provides guidelines for contributing code, documentation, and other improvements to the project.

## Table of Contents

- [Getting Started](#getting-started)
- [Development Workflow](#development-workflow)
- [Testing Requirements](#testing-requirements)
- [Code Style and Standards](#code-style-and-standards)
- [Pull Request Process](#pull-request-process)
- [Community Guidelines](#community-guidelines)

## Getting Started

1. Fork the repository on GitHub
2. Clone your fork locally:
   ```bash
   git clone https://github.com/YOUR-USERNAME/jsbsim.git
   cd jsbsim
   ```
3. Add the upstream repository:
   ```bash
   git remote add upstream https://github.com/JSBSim-Team/jsbsim.git
   ```
4. Build the project following the instructions in [CLAUDE.md](CLAUDE.md)

## Development Workflow

1. Create a new branch for your feature or bug fix:
   ```bash
   git checkout -b feature/your-feature-name
   ```
2. Make your changes, following the [Code Style and Standards](#code-style-and-standards)
3. Write tests for your changes (see [Testing Requirements](#testing-requirements))
4. Ensure all tests pass before submitting
5. Commit your changes with clear, descriptive commit messages
6. Push to your fork and submit a pull request

## Testing Requirements

All code contributions must include appropriate tests. JSBSim uses both C++ unit tests (CxxTest framework) and Python integration tests.

### Running Tests

#### Run All Tests
```bash
cd build
ctest -j4
```

#### Run Unit Tests Only
```bash
ctest -R Test1
```

#### Run Specific Test
```bash
ctest -R TestAerodynamics
```

#### Run with Verbose Output
```bash
ctest -V -R TestName
```

#### Run Individual Test Executable
```bash
./tests/unit_tests/FGColumnVector3Test1
```

### Test Coverage Expectations

New code submissions should maintain or improve test coverage:

- **New code**: Minimum 80% coverage required
- **Overall project target**: 90%+
- **Math components**: 95% target
- **Core models**: 85% target
- **Propulsion systems**: 80% target

To check coverage locally:
```bash
cd build
cmake -DENABLE_COVERAGE=ON ..
make
ctest -R Test1
make lcov
```

Coverage reports will be generated in `build/coverage/`.

### Writing Tests

#### C++ Unit Tests (CxxTest)

Create new test files in `tests/unit_tests/` following this pattern:

**File**: `tests/unit_tests/FGMyComponentTest.h`

```cpp
#include <cxxtest/TestSuite.h>
#include "TestUtilities.h"
#include <models/MyComponent.h>

class FGMyComponentTest : public CxxTest::TestSuite {
public:
    void testNormalOperation() {
        // GIVEN: initial state
        JSBSim::MyComponent comp;

        // WHEN: operation performed
        double result = comp.calculate(1.0);

        // THEN: expected result
        TS_ASSERT_DELTA(result, 2.0, 1e-10);
    }

    void testEdgeCase() {
        // Test boundary conditions
        JSBSim::MyComponent comp;
        TS_ASSERT_EQUALS(comp.calculate(0.0), 0.0);
    }

    void testErrorHandling() {
        // Test invalid inputs
        JSBSim::MyComponent comp;
        TS_ASSERT_THROWS(comp.calculate(-1.0), std::runtime_error);
    }
};
```

**Key Points**:
- Inherit from `CxxTest::TestSuite`
- Use descriptive test method names prefixed with `test`
- Follow GIVEN-WHEN-THEN structure for clarity
- Use `TS_ASSERT_DELTA` for floating-point comparisons with appropriate tolerance
- Test both normal operation and edge cases
- Include error handling tests where applicable

#### Python Integration Tests

Create new test files in `tests/` following this pattern:

**File**: `tests/TestMyFeature.py`

```python
from JSBSim_utils import JSBSimTestCase

class TestMyFeature(JSBSimTestCase):
    def test_normal_operation(self):
        """Test that MyFeature works correctly under normal conditions."""
        fdm = self.create_fdm()
        fdm.load_model('c172x')
        fdm.run_ic()

        # Verify expected behavior
        self.assertAlmostEqual(
            fdm['metrics/expected-value'],
            1.0,
            delta=1e-6
        )
```

### Test Best Practices

1. **One concept per test**: Each test should verify one specific behavior
2. **Descriptive names**: Test names should clearly describe what is being tested
3. **GIVEN-WHEN-THEN**: Structure tests with clear setup, action, and assertion sections
4. **Test edge cases**: Include boundary conditions, zero values, negative values
5. **Test error paths**: Verify error handling and invalid inputs
6. **Use appropriate tolerances**: Use suitable floating-point tolerances for comparisons (typically 1e-10 for double precision)
7. **Document test purpose**: Add comments explaining what each test validates
8. **Keep tests independent**: Tests should not depend on execution order

For more details, see [tests/README.md](tests/README.md).

## Code Style and Standards

- JSBSim is written in C++17
- Follow existing code style and conventions
- Use meaningful variable and function names
- Add comments for complex logic
- Update documentation when adding new features
- Ensure code compiles without warnings on supported platforms (Windows, Linux, macOS)

## Pull Request Process

### Before Submitting

1. Ensure your branch is up-to-date with upstream master:
   ```bash
   git fetch upstream
   git rebase upstream/master
   ```

2. Verify all tests pass:
   ```bash
   cd build
   ctest -j4
   ```

3. Ensure your code builds without warnings:
   ```bash
   cmake --build . 2>&1 | grep -i warning
   ```

4. Write tests for all new functionality (minimum 80% coverage)

### PR Requirements

Your pull request must include:

- **Clear description**: Explain what changes you made and why
- **Tests**: All new code must have corresponding tests
- **Test results**: Confirm all tests pass locally
- **Documentation**: Update relevant documentation (code comments, README, etc.)
- **No regressions**: Ensure existing functionality is not broken

### PR Review Process

1. Submit your pull request against the `master` branch
2. Automated CI/CD checks will run (build + tests on multiple platforms)
3. Maintainers will review your code
4. Address any review comments
5. Once approved, your PR will be merged

## Community Guidelines

- Be respectful and constructive in all interactions
- Ask questions on [GitHub Discussions](https://github.com/JSBSim-Team/jsbsim/discussions)
- Report bugs and issues on the [issue tracker](https://github.com/JSBSim-Team/jsbsim/issues)
- Join our [Facebook community](https://www.facebook.com/jsbsim/)

## Additional Resources

- [JSBSim Reference Manual](https://jsbsim-team.github.io/jsbsim-reference-manual)
- [Developer Documentation](doc/DevelopersDocs.md)
- [Project Wiki](https://github.com/JSBSim-Team/jsbsim/wiki)
- [Contributing Source Code Changes Wiki](https://github.com/JSBSim-Team/jsbsim/wiki/Contributing-Source-Code-Changes-to-JSBSim)

## License

By contributing to JSBSim, you agree that your contributions will be licensed under the same license as the project (LGPL 2.1 for the core library). See [COPYING](COPYING) for details.

Thank you for contributing to JSBSim!
