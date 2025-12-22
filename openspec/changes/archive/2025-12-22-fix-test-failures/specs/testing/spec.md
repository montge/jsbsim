# Testing Specification

## ADDED Requirements

### Requirement: Test Dependencies Complete
The Python test suite SHALL have all required dependencies specified in `python/requirements.txt`.

#### Scenario: Socket test dependencies available
- **WHEN** the test suite is executed
- **THEN** the `telnetlib3` module is available for socket-based tests
- **AND** `TestInputSocket` can import required modules

### Requirement: Scipy API Compatibility
Tests using scipy statistical functions SHALL be compatible with scipy 1.10+ API changes.

#### Scenario: Linear regression uses explicit arguments
- **WHEN** `TestActuator` performs linear regression analysis
- **THEN** `scipy.stats.linregress()` is called with explicit `x` and `y` arguments
- **AND** the test passes with scipy versions 1.7 through 1.14+
