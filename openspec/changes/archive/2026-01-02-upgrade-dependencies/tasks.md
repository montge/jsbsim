## 1. Version Management

- [x] 1.1 Install bump-my-version and create `.bumpversion.toml` config
- [x] 1.2 Configure version locations (CMakeLists.txt, python/setup.py.in)
- [x] 1.3 Test version bump workflow (dry-run)

## 2. Python Dependencies

- [x] 2.1 Update `python/requirements.txt` with latest versions
- [x] 2.2 Update `python/pyproject.toml` setuptools constraint
- [x] 2.3 Update `python/setup.py.in` python_requires to >=3.10

## 3. CI Workflow Updates

- [x] 3.1 Update `.github/workflows/cpp-python-build.yml` Python versions
- [x] 3.2 Update `.github/workflows/coverage.yml` if needed (no Python, skipped)
- [x] 3.3 Update `.github/workflows/sonarcloud.yml` if needed (no Python, skipped)
- [x] 3.4 Update cibuildwheel Python targets in pyproject.toml

## 4. Validation

- [x] 4.1 Run local build to verify changes (TOML config validated)
- [x] 4.2 Update CLAUDE.md or documentation if needed (not required)
