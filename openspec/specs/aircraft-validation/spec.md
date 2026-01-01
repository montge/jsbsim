# aircraft-validation Specification

## Purpose
TBD - created by archiving change add-aircraft-validation. Update Purpose after archive.
## Requirements
### Requirement: XML Schema Validation
Aircraft XML files SHALL be validated against the JSBSim.xsd schema.

#### Scenario: Valid aircraft passes schema check
- **WHEN** an aircraft XML file conforms to JSBSim.xsd
- **THEN** schema validation passes without warnings

#### Scenario: Invalid aircraft reports schema errors
- **WHEN** an aircraft XML file contains schema violations
- **THEN** validation emits warnings describing the violations
- **AND** the CI pipeline continues (non-blocking)

### Requirement: Physical Plausibility Checks
Aircraft configurations SHALL be checked for physically plausible values.

#### Scenario: Wing loading within reasonable range
- **WHEN** an aircraft is validated
- **THEN** wing loading (weight/wing area) SHALL be checked against typical range (5-150 lb/ft²)
- **AND** values outside range emit a warning

#### Scenario: CG within aircraft bounds
- **WHEN** an aircraft is validated
- **THEN** CG location SHALL be checked to be within the aircraft's defined geometry
- **AND** CG outside reasonable bounds emits a warning

#### Scenario: Positive mass and inertia values
- **WHEN** an aircraft is validated
- **THEN** empty weight, Ixx, Iyy, Izz SHALL be positive
- **AND** non-positive values emit a warning

### Requirement: Metadata Validation
Aircraft files SHALL include required metadata fields.

#### Scenario: Required metadata present
- **WHEN** an aircraft XML file is validated
- **THEN** the fileheader SHALL contain author, license, and description elements
- **AND** missing elements emit a warning

#### Scenario: License information complete
- **WHEN** an aircraft has a license element
- **THEN** it SHALL include either licenseName or licenseURL
- **AND** missing license details emit a warning

### Requirement: Validation Reporting
Validation results SHALL be reported clearly without blocking CI.

#### Scenario: Validation summary displayed
- **WHEN** aircraft validation completes
- **THEN** a summary of warnings per aircraft is displayed
- **AND** total warning count is reported

#### Scenario: CI continues on warnings
- **WHEN** validation emits warnings
- **THEN** the CI pipeline continues to completion
- **AND** warnings are visible in CI logs
