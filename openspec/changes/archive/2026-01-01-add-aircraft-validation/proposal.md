# Change: Add Basic Aircraft Validation

## Why
The 61 aircraft models in the repository have minimal validation - only "does it load?" checks exist. This makes it easy to introduce invalid XML, physically implausible values, or missing metadata without detection.

## What Changes
- Add XML schema validation against JSBSim.xsd
- Add physical plausibility checks (reasonable ranges for key parameters)
- Add metadata validation (required fields: author, license, description)
- Validation runs in CI as warnings (non-blocking)
- Create new `aircraft-validation` spec to define standards

## Impact
- Affected specs: New `aircraft-validation` capability
- Affected code: `tests/CheckAircrafts.py` or new validation script
- Non-breaking: Warnings only, does not block CI
