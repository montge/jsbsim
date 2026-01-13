# Change: Establish Upstream Sync Workflow

## Why
The fork needs a systematic approach to stay synchronized with upstream JSBSim-Team/jsbsim while maintaining local enhancements. Currently 336 commits ahead of upstream with significant improvements to testing, CI/CD, and code quality.

## What Changes
- Establish gitflow branching: `master` mirrors upstream, `develop` contains fork enhancements
- Define periodic sync process to merge upstream changes into develop
- Document workflow for contributing changes back to upstream

## Impact
- Affected specs: ci-cd (workflow changes)
- Affected code: No code changes, workflow/process only

## Current State
- **master**: Now mirrors `upstream/master` (commit 9ea4925a)
- **develop**: Contains 336 commits of fork enhancements including:
  - OpenSpec framework for spec-driven development
  - 118 CxxTest unit tests (expanded from baseline)
  - CI/CD improvements (coverage, SonarCloud, pre-commit hooks)
  - Code quality fixes (C++17, JSBSIM_API exports, nullptr)
  - Release automation (bump-my-version, git-cliff)

## Sync Workflow
1. **Periodic upstream fetch**: `git fetch upstream`
2. **Check for new commits**: `git log develop..upstream/master`
3. **Merge into develop**: `git checkout develop && git merge upstream/master`
4. **Resolve conflicts**: Prioritize upstream for core FDM, keep fork enhancements
5. **Update master**: Only when cutting a release aligned with upstream tag

## Contribution Strategy
Changes suitable for upstream PR:
- Bug fixes with tests
- API exports (JSBSIM_API)
- C++17 modernization
- Test coverage improvements

Changes to keep fork-only:
- OpenSpec framework
- Fork-specific CI/CD
- Pre-commit hooks
