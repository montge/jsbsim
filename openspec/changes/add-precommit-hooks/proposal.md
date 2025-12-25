# Change: Add Pre-commit Hooks with OpenSpec Validation

## Why
Currently there are no automated checks before commits, allowing invalid OpenSpec changes, formatting issues, and other problems to enter the repository. Adding pre-commit hooks catches issues early in the development workflow, before code review.

## What Changes
- Add `.pre-commit-config.yaml` with hook configuration
- Include OpenSpec validation hook (`openspec validate --strict`)
- Add basic code quality hooks (trailing whitespace, end-of-file, YAML validation)
- Document setup instructions in CONTRIBUTING.md or README

## Impact
- Affected specs: `ci-cd`
- Affected code: New `.pre-commit-config.yaml`, documentation updates
- Developer workflow: Requires one-time `pre-commit install` setup
