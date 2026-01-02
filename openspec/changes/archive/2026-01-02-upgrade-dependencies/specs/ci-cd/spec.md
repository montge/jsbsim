## ADDED Requirements

### Requirement: Automated Version Management
The project SHALL use bump-my-version for consistent version updates across all version-containing files.

#### Scenario: Version bump updates all locations
- **WHEN** `bump-my-version bump <part>` is executed
- **THEN** version is updated in CMakeLists.txt
- **AND** version is updated in python/setup.py.in
- **AND** all version strings remain synchronized

#### Scenario: Version format follows semver
- **WHEN** a version is set
- **THEN** it SHALL follow semantic versioning (MAJOR.MINOR.PATCH)
- **AND** development versions use `.devN` suffix

## MODIFIED Requirements

### Requirement: SonarCloud Static Analysis
The CI pipeline SHALL run SonarCloud static analysis on C++ source code to detect bugs, code smells, and security vulnerabilities.

#### Scenario: Analysis runs on push to master
- **WHEN** code is pushed to the master branch
- **THEN** SonarCloud analysis workflow is triggered
- **AND** analysis results are uploaded to SonarCloud dashboard

#### Scenario: Analysis runs on pull requests
- **WHEN** a pull request is opened or updated
- **THEN** SonarCloud analysis workflow is triggered
- **AND** PR decoration shows quality gate status
- **AND** new issues are reported as PR comments

#### Scenario: Third-party code is excluded
- **WHEN** SonarCloud analyzes the codebase
- **THEN** bundled third-party code (simgear, GeographicLib, MSIS) is excluded from analysis
- **AND** only JSBSim-owned code is analyzed for quality metrics

#### Scenario: CI uses Python 3.10 or later
- **WHEN** CI workflows execute Python-based tasks
- **THEN** Python version SHALL be 3.10 or later
- **AND** Python 3.9 is no longer supported
