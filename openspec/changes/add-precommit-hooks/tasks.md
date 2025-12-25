## 1. Pre-commit Configuration
- [x] 1.1 Create `.pre-commit-config.yaml` with standard hooks
- [x] 1.2 Add local hook for `openspec validate --changes --strict`
- [x] 1.3 Test hooks work correctly on sample commits

## 2. Documentation
- [x] 2.1 Add pre-commit setup instructions to project docs
- [x] 2.2 Document how to skip hooks when necessary (`--no-verify`)

## 3. Validation
- [x] 3.1 Verify hooks catch OpenSpec validation errors
- [x] 3.2 Verify hooks don't block normal commits without OpenSpec changes
