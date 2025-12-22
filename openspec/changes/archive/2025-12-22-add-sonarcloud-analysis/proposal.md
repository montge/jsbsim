# Change: Add SonarCloud Static Analysis

## Why
The project currently uses Codecov for code coverage but lacks static analysis tooling. SonarCloud provides comprehensive code quality analysis including bug detection, code smells, security vulnerabilities, and maintainability metrics. Running both Codecov (coverage) and SonarCloud (quality) provides complementary insights.

## What Changes
- Add new GitHub Actions workflow for SonarCloud analysis
- Configure SonarCloud project settings (`sonar-project.properties`)
- Integrate with existing CI pipeline (run on push/PR to master)
- Keep existing Codecov workflow unchanged

## Impact
- Affected specs: ci-cd (new capability)
- Affected code: `.github/workflows/`, `sonar-project.properties` (new file)
- No breaking changes to existing workflows
- Requires SonarCloud organization setup and `SONAR_TOKEN` secret
