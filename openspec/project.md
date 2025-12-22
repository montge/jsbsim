# Project Context

## Purpose
JSBSim is a multi-platform, open-source Flight Dynamics Model (FDM) library written in C++17. It simulates the physics and math defining aircraft, rocket, and other vehicle movement under applied forces. Used standalone in batch mode or integrated with FlightGear, Unreal Engine, and other simulators.

## Tech Stack
- C++17 (core library)
- CMake (build system)
- Python/Cython (Python bindings)
- CxxTest (C++ unit testing)
- pytest (Python integration tests)
- GitHub Actions (CI/CD)
- Codecov (code coverage)

## Project Conventions

### Code Style
- C++ classes prefixed with `FG` (e.g., `FGFDMExec`, `FGAerodynamics`)
- Header and implementation files in same directory
- XML-based configuration for aircraft, engines, scripts

### Architecture Patterns
- Executive pattern: `FGFDMExec` instantiates and coordinates all subsystem models
- Model hierarchy: All simulation models inherit from `FGModel`
- Property tree: Runtime variable access via `FGPropertyManager`

### Testing Strategy
- C++ unit tests: CxxTest framework in `tests/unit_tests/`
- Python integration tests: pytest-based in `tests/`
- CI runs tests on Linux, macOS, Windows
- Code coverage via Codecov

### Git Workflow
- Main branch: `master`
- Feature branches for development
- PRs require CI passing
- Commit messages: descriptive, reference issues when applicable

## Domain Context
- Flight dynamics simulation (6 DOF)
- Aerodynamics, propulsion, flight control systems
- Atmosphere modeling (ISA, MSIS)
- Ground reactions and landing gear
- XML-based aircraft definitions in `aircraft/`

## Important Constraints
- Cross-platform compatibility (Linux, macOS, Windows)
- No external dependencies for core C++ library (Expat bundled)
- Python bindings must expose full C++ API
- Aircraft XML schemas must remain backward compatible

## External Dependencies
- Expat (XML parsing, bundled or system)
- Python 3.9+ with Cython (for Python module)
- numpy, scipy, pandas (Python test dependencies)
- GeographicLib (bundled)
- SimGear components (bundled subset)
