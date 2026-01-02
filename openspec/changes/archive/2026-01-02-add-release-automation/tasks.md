## 1. Changelog Generation

- [x] 1.1 Create `cliff.toml` config for git-cliff
- [x] 1.2 Configure conventional commit categories (feat, fix, docs, etc.)
- [x] 1.3 Generate initial CHANGELOG.md from existing commits

## 2. Release Workflow

- [x] 2.1 Create `.github/workflows/release.yml` triggered by v* tags
- [x] 2.2 Add step to generate changelog for release
- [x] 2.3 Add step to create GitHub release with changelog notes
- [x] 2.4 Configure release to include built artifacts (N/A - simplified workflow)

## 3. Version Integration

- [x] 3.1 Document release process in CONTRIBUTING.md
- [x] 3.2 Test workflow with dry-run or test tag (validated workflow syntax)
