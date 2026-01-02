# Change: Add Release Automation for Fork

## Why
This fork needs its own release process independent of upstream. Currently there's no changelog, and releases require manual steps. Automating releases with conventional commits improves traceability and reduces manual work.

## What Changes
- Add git-cliff for changelog generation from conventional commits
- Create simplified release workflow triggered by git tags (v*)
- Integrate with bump-my-version for version management
- Generate CHANGELOG.md automatically on release
- Create GitHub releases with auto-generated notes

## Impact
- Affected specs: `ci-cd` (modified)
- Affected code:
  - New `cliff.toml` (git-cliff config)
  - New `.github/workflows/release.yml`
  - New `CHANGELOG.md` (generated)
- Non-breaking: Adds new workflow, doesn't modify existing CI
