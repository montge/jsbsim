# Tasks: Add SonarCloud Static Analysis

## 1. SonarCloud Setup (Manual Steps Required)
- [x] 1.1 Create SonarCloud project at https://sonarcloud.io (organization: montge, project: montge_jsbsim)
- [x] 1.2 Generate SONAR_TOKEN and add to GitHub repository secrets
- [x] 1.3 Verify project key and organization match configuration

## 2. Configuration
- [x] 2.1 Create `sonar-project.properties` in repository root
- [x] 2.2 Configure source directories (`src/`)
- [x] 2.3 Configure exclusions (third-party code: `src/simgear/`, `src/GeographicLib/`)
- [x] 2.4 Set C++ language standard and compiler settings

## 3. GitHub Actions Workflow
- [x] 3.1 Create `.github/workflows/sonarcloud.yml`
- [x] 3.2 Configure build-wrapper for C++ analysis
- [x] 3.3 Run sonar-scanner with appropriate parameters
- [x] 3.4 Configure to run on push to master and PRs

## 4. Verification
- [x] 4.1 Push changes and verify workflow runs
- [x] 4.2 Check SonarCloud dashboard for initial analysis results
- [x] 4.3 Verify PR decoration works (comments on PRs)
- [x] 4.4 Document any quality gate failures for future fixes
