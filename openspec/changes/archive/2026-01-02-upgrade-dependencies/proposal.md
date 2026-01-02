# Change: Upgrade Dependencies and Add Version Management

## Why
- Python 3.9 reaches EOL December 2025 - should upgrade minimum to 3.10
- Dependencies are outdated (numpy>=1.20, setuptools<72)
- No automated version management - version is manually edited in CMakeLists.txt
- Fork needs modern tooling for independent releases

## What Changes
- Add bump-my-version for automated version management
- Upgrade Python minimum to 3.10
- Update Python dependencies to latest compatible versions
- Update CI workflows to use Python 3.10+
- Update setuptools upper bound

## Impact
- Affected specs: `ci-cd` (modified)
- Affected code:
  - `python/requirements.txt`
  - `python/pyproject.toml`
  - `python/setup.py.in`
  - `.github/workflows/*.yml`
  - `CMakeLists.txt` (version management)
  - New `.bumpversion.toml`
- **BREAKING**: Drops Python 3.9 support
