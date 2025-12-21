# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

JSBSim is a multi-platform, open-source Flight Dynamics Model (FDM) written in C++17. It simulates the physics and math defining aircraft, rocket, and other vehicle movement under applied forces. The library can run standalone (console batch mode) or integrate with FlightGear, Unreal Engine, and other simulators.

## Build Commands

### Building with CMake (recommended)
```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
```

The executable is built at `build/src/JSBSim`.

### Common CMake options
```bash
# Debug build
cmake -DCMAKE_BUILD_TYPE=Debug ..

# Release build with optimizations
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS_RELEASE="-O3" ..

# Build shared library (reduces memory when using Python module)
cmake -DBUILD_SHARED_LIBS=ON ..

# Use system Expat instead of bundled
cmake -DSYSTEM_EXPAT=ON ..
```

### Running Tests
Tests require the Python module (needs Cython):
```bash
# Run all tests in parallel
ctest -j4

# Run a specific test
ctest -R TestDensityAltitude

# Run tests matching pattern
ctest -R Altitude

# Exclude tests matching pattern
ctest -E Altitude
```

If tests fail with shared library errors, set:
```bash
export LD_LIBRARY_PATH=$PWD/src:$LD_LIBRARY_PATH
```

### Running a Single C++ Unit Test
Unit tests use CxxTest framework:
```bash
# After building, run individual unit test executable
./tests/unit_tests/FGColumnVector3Test1
```

### Generate Documentation
```bash
make doc  # Requires Doxygen and Graphviz
```

## Architecture

### Core Components (src/)

- **FGFDMExec** (`FGFDMExec.h/cpp`): The executive class that instantiates, initializes, and runs all simulation components. Entry point for integration.

- **FGJSBBase**: Base class providing common functionality, constants, and utilities for all JSBSim classes.

### Models (src/models/)
Each model handles a specific aspect of flight dynamics:
- `FGPropagate`: Propagates state (position, velocity, orientation) over time
- `FGAccelerations`: Computes accelerations from forces/moments
- `FGAerodynamics`: Aerodynamic forces and moments
- `FGPropulsion`: Engines, fuel, propellers (propulsion/ subdirectory)
- `FGFCS`: Flight Control System (flight_control/ subdirectory)
- `FGAtmosphere`: Atmosphere modeling (atmosphere/ subdirectory includes MSIS, Mars)
- `FGGroundReactions`, `FGLGear`: Landing gear and ground interaction
- `FGMassBalance`: Mass, CG, and inertia calculations
- `FGAuxiliary`: Auxiliary flight parameters (Mach, dynamic pressure, etc.)
- `FGInertial`: Gravity and rotating Earth effects
- `FGBuoyantForces`, `FGGasCell`: Lighter-than-air vehicle support

### Math (src/math/)
- `FGColumnVector3`, `FGMatrix33`, `FGQuaternion`: Vector/matrix/quaternion math
- `FGLocation`: Geographic position handling
- `FGFunction`, `FGTable`: Configurable mathematical functions and lookup tables
- `FGCondition`: Conditional logic for flight control

### Input/Output (src/input_output/)
- `FGXMLElement`, `FGXMLParse`: XML parsing (aircraft configs, scripts)
- `FGPropertyManager`: Property tree for runtime variable access
- `FGScript`: Script execution for automated simulations
- `FGOutputSocket`, `FGOutputTextFile`: Data output to files/sockets

### Initialization (src/initialization/)
- `FGInitialCondition`: Sets initial state (position, velocity, orientation)
- `FGTrim`: Aircraft trim routines

### External Libraries (src/)
- `simgear/`: SimGear components (properties, XML, misc utilities)
- `GeographicLib/`: Geographic calculations (geodetic conversions)

## Aircraft Configuration

Aircraft are defined via XML files in `aircraft/[name]/`:
- Main aircraft file specifies aerodynamics, propulsion, FCS, mass properties
- References engine definitions from `engine/`
- References systems from `systems/`

Scripts in `scripts/` define simulation scenarios that load aircraft and control execution.

## Language Bindings

- **Python** (`python/`): Cython-based module exposing full C++ API
- **MATLAB** (`matlab/`): S-Function for Simulink integration
- **Julia** (`julia/`): Julia package
- **Unreal Engine** (`UnrealEngine/`): Plugin for Unreal Engine integration

<!-- OPENSPEC:START -->
# OpenSpec Instructions

These instructions are for AI assistants working in this project.

Always open `@/openspec/AGENTS.md` when the request:
- Mentions planning or proposals (words like proposal, spec, change, plan)
- Introduces new capabilities, breaking changes, architecture shifts, or big performance/security work
- Sounds ambiguous and you need the authoritative spec before coding

Use `@/openspec/AGENTS.md` to learn:
- How to create and apply change proposals
- Spec format and conventions
- Project structure and guidelines

Keep this managed block so 'openspec update' can refresh the instructions.

<!-- OPENSPEC:END -->
