## ADDED Requirements

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
