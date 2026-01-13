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

### Requirement: Automated Changelog Generation
The project SHALL generate changelogs automatically from conventional commit messages.

#### Scenario: Changelog generated on release
- **WHEN** a version tag (v*) is pushed
- **THEN** git-cliff generates changelog from commits since last tag
- **AND** changelog follows conventional commit categories (feat, fix, docs, etc.)

#### Scenario: Full changelog maintained
- **WHEN** releases are created over time
- **THEN** CHANGELOG.md contains cumulative release history
- **AND** each release section shows version, date, and categorized changes

### Requirement: Tag-Based Release Workflow
The project SHALL create GitHub releases automatically when version tags are pushed.

#### Scenario: Release created from tag
- **WHEN** a tag matching pattern v* is pushed (e.g., v1.2.5)
- **THEN** a GitHub release is created with that tag
- **AND** release notes are generated from changelog

#### Scenario: Release includes version info
- **WHEN** a GitHub release is created
- **THEN** the release title includes the version number
- **AND** the release body includes categorized changes since last release

### Requirement: Gitflow Branching Model
The project SHALL follow gitflow branching conventions to maintain upstream compatibility.

#### Scenario: Master mirrors upstream
- **WHEN** viewing the master branch
- **THEN** it SHALL match upstream/master exactly
- **AND** only release merges update master

#### Scenario: Develop contains fork enhancements
- **WHEN** viewing the develop branch
- **THEN** it SHALL contain all upstream commits plus fork-specific enhancements
- **AND** feature branches merge into develop

### Requirement: Upstream Sync Process
The project SHALL periodically synchronize with upstream JSBSim-Team/jsbsim.

#### Scenario: Upstream changes merged
- **WHEN** upstream/master has new commits
- **THEN** they SHALL be merged into develop branch
- **AND** conflicts prioritize upstream for core FDM code

#### Scenario: Sync frequency
- **WHEN** upstream has activity
- **THEN** sync SHALL occur at least monthly
- **AND** critical bug fixes sync immediately
