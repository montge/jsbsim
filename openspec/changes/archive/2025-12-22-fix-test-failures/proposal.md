# Change: Fix Test Suite Failures

## Why
Two tests in the Python test suite are currently failing due to dependency and API compatibility issues:
1. `TestInputSocket` - Missing `telnetlib3` Python module and compatibility issues with async telnet parsing
2. `TestActuator` - Incompatible with newer scipy `linregress()` API

These failures prevent clean CI runs and mask potential real issues.

## What Changes
- Add `telnetlib3` to Python test dependencies in `python/requirements.txt` (already done)
- Fix `TestInputSocket.py` to handle `telnetlib3` output format (filter empty strings from parsed command list)
- Update `TestActuator.py` to use scipy `linregress(x, y)` two-argument form instead of single array

## Impact
- Affected specs: testing
- Affected code: `python/requirements.txt`, `tests/TestActuator.py`, `tests/TestInputSocket.py`
- No breaking changes
- Fixes 2 test failures, bringing test suite to 100% pass rate
