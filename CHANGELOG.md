# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Features
- Add aircraft validation for metadata and physical plausibility
- Add bump-my-version for automated version management
- Add release automation with git-cliff changelog generation

### CI/CD
- Upgrade to Python 3.10+ (drop 3.8/3.9 support)
- Update dependencies: numpy>=1.23, cython>=3.0, pandas>=2.0, scipy>=1.10
- Add pre-commit hooks for code quality
- Add SonarCloud static analysis
