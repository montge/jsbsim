# Tasks: Fix Test Suite Failures

## 1. Fix TestInputSocket
- [x] 1.1 Add `telnetlib3` to `python/requirements.txt`
- [x] 1.2 Fix telnetlib3 compatibility in sanity_check (filter empty strings)
- [x] 1.3 Verify TestInputSocket passes locally

## 2. Fix TestActuator
- [x] 2.1 Review `tests/TestActuator.py` line 78 (`stats.linregress` call)
- [x] 2.2 Update to use `linregress(x, y)` two-argument form
- [x] 2.3 Verify TestActuator passes locally

## 3. Verification
- [x] 3.1 Run full test suite (`ctest -j$(nproc)`)
- [x] 3.2 Confirm all 56 tests pass
- [x] 3.3 Test on CI (push to branch and verify)
