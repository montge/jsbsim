# Design: Test Coverage Improvement Strategy

## Context

JSBSim is a flight dynamics simulation library used in safety-critical applications. The current 21.8% test coverage is insufficient for:
- Confident refactoring
- Regression detection
- Compliance with aerospace software standards

### Current State
| Metric | Current | Target |
|--------|---------|--------|
| Line Coverage | 21.8% | 90%+ |
| Lines Covered | 4,616 | ~19,060 |
| Lines to Add | - | ~14,444 |
| Branch Coverage | Unknown | 80%+ per module |

### Stakeholders
- JSBSim maintainers
- FlightGear integration users
- Academic/research users
- Aerospace simulation developers

## Goals / Non-Goals

### Goals
- Achieve 90%+ overall line coverage
- Achieve 80%+ branch coverage per module
- Maintain test execution time under 10 minutes for CI
- Create maintainable, readable tests
- Document testing patterns for future contributors

### Non-Goals
- 100% coverage (diminishing returns, some code paths are platform-specific)
- Mutation testing (future enhancement)
- Performance benchmarking (separate initiative)
- Testing third-party code (simgear, GeographicLib excluded)

## Decisions

### Decision 1: Phased Approach by Module Priority

**Rationale**: Tackling all modules at once is overwhelming. Prioritize by:
1. Risk (core simulation accuracy)
2. Coverage gap (currently 0% vs partial)
3. Complexity (simpler modules first for momentum)

**Priority Order**:
1. **Phase 1 - Math Foundation** (2 weeks)
   - `src/math/` - Already partially tested, fill gaps
   - Target: 95% coverage

2. **Phase 2 - Core Models** (3 weeks)
   - `FGAccelerations`, `FGAircraft`, `FGModel`
   - `FGPropagate`, `FGAuxiliary`
   - Target: 85% coverage

3. **Phase 3 - Propulsion** (3 weeks)
   - All engine types: FGPiston, FGTurbine, FGRocket, FGElectric
   - Propellers, nozzles, thrusters
   - Target: 80% coverage

4. **Phase 4 - Flight Control** (2 weeks)
   - FGPID, FGSensor, FGGyro
   - Complete actuator/filter coverage
   - Target: 85% coverage

5. **Phase 5 - I/O and Infrastructure** (2 weeks)
   - XML parsing, script execution
   - Output formatting
   - Target: 75% coverage

6. **Phase 6 - Specialized Models** (2 weeks)
   - FGBuoyantForces, FGGasCell
   - FGGroundReactions edge cases
   - Target: 80% coverage

### Decision 2: Test Framework Strategy

**CxxTest for unit tests** (existing choice, continue):
- Header-only, minimal dependencies
- Good IDE integration
- Fast compilation

**Python for integration tests** (existing choice, continue):
- Tests full simulation runs
- Validates XML configuration loading
- Tests Python bindings themselves

**New: Parameterized test patterns**:
- Use CxxTest's test suite features for data-driven tests
- Reduce test code duplication
- Cover edge cases systematically

### Decision 3: Coverage Tooling Enhancements

**Current**: LCOV + Codecov
**Additions**:
- Add branch coverage flags (`--rc lcov_branch_coverage=1`)
- Configure SonarCloud for branch coverage display
- Add per-module coverage reports
- Create coverage badges per component

### Decision 4: Quality Gates

**Codecov Configuration** (`.codecov.yml`):
```yaml
coverage:
  status:
    project:
      default:
        target: 90%
        threshold: 1%
    patch:
      default:
        target: 80%
  flags:
    math:
      paths: src/math/
      target: 95%
    models:
      paths: src/models/
      target: 85%
```

**SonarCloud Quality Gate**:
- Coverage on new code: 80%+
- Overall coverage: 85%+ (stepping stone to 90%)

## Risks / Trade-offs

### Risk 1: Test Execution Time
- **Risk**: Adding 14,000+ lines of test coverage could slow CI significantly
- **Mitigation**:
  - Parallelize test execution (`ctest -j`)
  - Group fast unit tests separately from slow integration tests
  - Use test fixtures to reduce setup/teardown

### Risk 2: Flaky Tests
- **Risk**: Complex simulation tests may be timing-sensitive
- **Mitigation**:
  - Use deterministic test inputs
  - Avoid real-time dependencies
  - Set appropriate tolerances for floating-point comparisons

### Risk 3: Maintenance Burden
- **Risk**: More tests = more maintenance
- **Mitigation**:
  - Follow consistent patterns
  - Use shared test utilities
  - Document test purposes clearly

### Risk 4: False Confidence
- **Risk**: High coverage doesn't guarantee correctness
- **Mitigation**:
  - Focus on meaningful assertions, not just execution
  - Test edge cases and error paths
  - Validate against known-good simulation results

## Test Pattern Guidelines

### CxxTest Unit Test Pattern
```cpp
class FGExampleTest : public CxxTest::TestSuite {
public:
  void setUp() {
    // Initialize test fixtures
  }

  void tearDown() {
    // Clean up
  }

  void testNormalOperation() {
    // GIVEN: initial state
    // WHEN: operation performed
    // THEN: expected result
    TS_ASSERT_EQUALS(actual, expected);
  }

  void testEdgeCase() {
    // Test boundary conditions
  }

  void testErrorHandling() {
    // Test invalid inputs
  }
};
```

### Python Integration Test Pattern
```python
class TestExample(JSBSimTestCase):
    def setUp(self):
        super().setUp()
        self.fdm = self.create_fdm()

    def test_normal_operation(self):
        """Test description."""
        # GIVEN
        self.fdm.load_model('aircraft')

        # WHEN
        self.fdm.run_ic()

        # THEN
        self.assertAlmostEqual(
            self.fdm['metrics/expected'],
            expected_value,
            delta=tolerance
        )
```

## Migration Plan

### Phase 0: Infrastructure (Week 1)
1. Configure branch coverage in LCOV
2. Set up Codecov flags for modules
3. Create coverage reporting dashboard
4. Establish baseline metrics

### Phases 1-6: Implementation (Weeks 2-13)
- Implement tests per phase schedule
- Weekly coverage check-ins
- Adjust priorities based on findings

### Phase 7: Quality Gates (Week 14)
1. Enable Codecov quality gates
2. Configure SonarCloud thresholds
3. Update PR templates with coverage requirements
4. Document testing standards

## Open Questions

1. **Test data management**: Should we create a shared test fixtures directory?
   - Recommendation: Yes, `tests/fixtures/` for shared XML configs

2. **Coverage exclusions**: Are current exclusions appropriate?
   - Current: simgear, GeographicLib, MSIS, unit_tests
   - Recommendation: Keep, these are third-party or test code

3. **Performance budget**: What's acceptable CI time increase?
   - Recommendation: Target <10 min total, <5 min for unit tests alone
