# ci-cd Specification

## Purpose
Define continuous integration and deployment standards for JSBSim. This specification covers static analysis via SonarCloud, code coverage reporting via Codecov, pre-commit hooks for local validation, and automated quality gates to maintain code health across all contributions.
## Requirements
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

### Requirement: Codecov Integration Preserved
The existing Codecov integration SHALL continue to function alongside SonarCloud.

#### Scenario: Both tools run independently
- **WHEN** CI pipeline executes
- **THEN** Codecov coverage workflow runs unchanged
- **AND** SonarCloud analysis workflow runs separately
- **AND** both tools report their respective metrics

### Requirement: Pre-commit Hook Framework
The project SHALL use pre-commit framework to run automated checks before commits are created.

#### Scenario: Developer installs pre-commit hooks
- **WHEN** a developer runs `pre-commit install` in the repository
- **THEN** git hooks are installed to run pre-commit checks
- **AND** subsequent commits trigger automated validation

#### Scenario: Hooks run on commit
- **WHEN** a developer creates a commit
- **THEN** pre-commit hooks execute configured checks
- **AND** the commit is blocked if any check fails
- **AND** the developer sees clear error messages for failures

### Requirement: OpenSpec Validation Hook
Pre-commit SHALL validate OpenSpec changes to ensure spec integrity before commits.

#### Scenario: Valid OpenSpec changes pass
- **WHEN** a commit includes valid OpenSpec changes
- **THEN** `openspec validate --changes --strict` passes
- **AND** the commit proceeds normally

#### Scenario: Invalid OpenSpec changes blocked
- **WHEN** a commit includes OpenSpec changes with validation errors
- **THEN** `openspec validate --changes --strict` fails
- **AND** the commit is blocked
- **AND** validation errors are displayed to the developer

#### Scenario: Commits without OpenSpec changes
- **WHEN** a commit does not include changes to `openspec/` directory
- **THEN** OpenSpec validation is skipped or passes quickly
- **AND** the commit proceeds without delay

### Requirement: Standard Quality Hooks
Pre-commit SHALL include standard code quality hooks for common issues.

#### Scenario: Trailing whitespace detected
- **WHEN** staged files contain trailing whitespace
- **THEN** the trailing-whitespace hook fails or auto-fixes the issue
- **AND** the developer is notified of the change

#### Scenario: Missing end-of-file newline
- **WHEN** staged files are missing a final newline
- **THEN** the end-of-file-fixer hook adds the newline
- **AND** the developer is notified of the change

#### Scenario: Invalid YAML syntax
- **WHEN** staged YAML files contain syntax errors
- **THEN** the YAML check hook fails
- **AND** the developer sees the syntax error location

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
