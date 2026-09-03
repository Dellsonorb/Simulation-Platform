# GitHub Publication and README Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Publish the complete Simulation Platform V1.0 repository as GitHub `main` with a bilingual, reproducible, and technically accurate README.

**Architecture:** Treat `SIM/p450_sim_v1` as the only publication root and `feature/bunker-a-implementation` as the complete source branch. Add a small README contract test, replace the terse README with a Chinese orientation plus canonical English operations guide, audit only tracked publication content, verify the existing Catkin workspace, and perform a normal first push to the empty GitHub repository.

**Tech Stack:** Git, GitHub SSH, Markdown, Python `unittest`, ROS 1 Noetic, Catkin Tools, Gazebo 11, PX4 SITL

---

### Task 1: Define the public README contract

**Files:**
- Create: `tests/test_public_readme.py`
- Test: `tests/test_public_readme.py`

- [ ] **Step 1: Write the failing README contract test**

Create a test that reads `README.md` and asserts:

```python
class PublicReadmeTest(unittest.TestCase):
    def test_readme_has_bilingual_entrypoints_and_runtime_sections(self):
        text = README.read_text(encoding="utf-8")
        for marker in (
                "中文概览", "English Documentation", "Architecture",
                "Robot-facing interfaces", "Prerequisites", "Build",
                "Run the platform", "Air-Ground Pick Demo",
                "Sim-to-Real boundary", "Repository layout",
                "Verification", "Known limitations"):
            self.assertIn(marker, text)

    def test_readme_commands_reference_real_files_and_packages(self):
        text = README.read_text(encoding="utf-8")
        self.assertIn("catkin build --profile p450-clean", text)
        self.assertIn("source install/p450-clean/setup.bash", text)
        for script in (
                "scripts/smoke_air_ground_standalone.bash",
                "scripts/smoke_air_ground_pick_demo.bash"):
            self.assertIn(script, text)
            self.assertTrue((ROOT / script).is_file())

    def test_readme_has_no_local_machine_path_or_real_backend_overclaim(self):
        text = README.read_text(encoding="utf-8")
        self.assertNotIn("/media/lu/", text)
        self.assertNotIn("/home/lu/", text)
        self.assertIn("not included in V1.0", text)
```

- [ ] **Step 2: Run the test and verify it fails against the old README**

Run:

```bash
/usr/bin/python3 -m unittest -v tests.test_public_readme
```

Expected: FAIL because the old README lacks the bilingual entrypoints and
operational sections.

### Task 2: Write the bilingual project README

**Files:**
- Modify: `README.md`
- Test: `tests/test_public_readme.py`

- [ ] **Step 1: Replace the README with the approved structure**

Write one document with:

```text
Simulation Platform V1.0
  Chinese overview and navigation
  English Documentation
    Status and scope
    Architecture
    Robot-facing interfaces
    Shared TF/localization contract
    Prerequisites
    External PX4 requirement
    Build
    Run the platform
    Air-Ground Pick Demo
    Verification
    Sim-to-Real boundary
    Repository layout
    Known limitations
    Design documentation
```

Use repository-relative examples. The operational commands must use an
explicit placeholder owned by the reader rather than a developer path:

```bash
export P450_PX4_ROOT=/absolute/path/to/the/compatible/px4-checkout
scripts/with_noetic_env.bash catkin build --workspace "$PWD" \
  --profile p450-clean --no-status
source install/p450-clean/setup.bash
P450_PX4_ROOT="$P450_PX4_ROOT" \
  ./scripts/smoke_air_ground_standalone.bash --gui false
P450_PX4_ROOT="$P450_PX4_ROOT" \
  ./scripts/smoke_air_ground_pick_demo.bash --gui false
```

State explicitly that REAL hardware adapters are an interface target and are
not included in V1.0. Do not add benchmark or paper claims.

- [ ] **Step 2: Run the README contract test**

Run:

```bash
/usr/bin/python3 -m unittest -v tests.test_public_readme
```

Expected: all README tests PASS.

- [ ] **Step 3: Validate every referenced local path**

Run a short Python check that extracts the documented `scripts/...` paths and
confirms each file exists. Manually compare package names in launch commands
with `src/**/package.xml`.

- [ ] **Step 4: Commit the README**

```bash
git add README.md tests/test_public_readme.py
git diff --cached --check
git commit -m "docs: publish Simulation Platform V1.0 guide"
```

### Task 3: Audit the GitHub publication tree

**Files:**
- Modify if required: `.gitignore`
- Inspect: all tracked files

- [ ] **Step 1: Prove generated directories are ignored and untracked**

Run:

```bash
git check-ignore build devel install logs .catkin_tools .worktrees
git ls-files build devel install logs .catkin_tools .worktrees
```

Expected: every generated directory is ignored and the second command prints
nothing.

- [ ] **Step 2: Check GitHub file-size compatibility**

Enumerate all tracked blobs and fail if any exceeds GitHub's 100 MiB hard
limit. Report the largest tracked asset and total Git object size.

- [ ] **Step 3: Scan tracked text for credentials and active absolute paths**

Use `git grep -I` over tracked files for private-key headers, GitHub tokens,
generic API-secret assignments, `/media/lu/`, and `/home/lu/`. No credentials
may remain. Developer paths in historical planning documents may remain only
as historical execution records; active `README.md`, `scripts/`, `src/`, and
`config/` must not depend on them.

- [ ] **Step 4: Commit `.gitignore` only if the audit requires a change**

```bash
git add .gitignore
git diff --cached --check
git commit -m "chore: exclude local publication artifacts"
```

Skip this commit if `.gitignore` already satisfies the contract.

### Task 4: Verify the publication commit

**Files:**
- Test: maintained Python contract suite
- Build: all 26 Catkin packages

- [ ] **Step 1: Run the maintained Python suite including the README contract**

Run from a sourced installed workspace:

```bash
source install/p450-clean/setup.bash
/usr/bin/python3 -m unittest -q \
  tests.test_air_ground_pick_demo \
  tests.test_air_ground_platform \
  tests.test_bunker_build_contract \
  tests.test_bunker_shell_contract \
  tests.test_bunker_simple_smoke \
  tests.test_ground_manipulator_platform \
  tests.test_mid360_runtime_contract \
  tests.test_p450_build_contract \
  tests.test_p450_runtime_contract \
  tests.test_public_readme \
  tests.test_realsense_runtime_safety \
  tests.test_sim_to_real_interface_contract \
  src.demos.air_ground_pick_demo.test.test_approach \
  src.ground.bunker_navigation.test.test_common_navigation \
  src.ground.bunker_navigation.test.test_configuration \
  src.platform.bunker_sim_runtime.test.test_renderer \
  src.platform.p450_flight_facade.test.test_translation
```

Expected: zero failures.

- [ ] **Step 2: Run a clean system-toolchain Catkin build**

Run:

```bash
PATH=/opt/ros/noetic/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  catkin clean -y
PATH=/opt/ros/noetic/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  catkin build --no-status
```

Expected: all 26 packages succeed. The sanitized `PATH` prevents a local
Miniconda Protobuf installation from overriding ROS/Gazebo's system Protobuf.

- [ ] **Step 3: Re-run the README and focused interface tests from the fresh install**

```bash
source install/p450-clean/setup.bash
/usr/bin/python3 -m unittest -q \
  tests.test_public_readme tests.test_sim_to_real_interface_contract
git diff --check
git status --short --branch
```

Expected: tests pass and the worktree is clean.

### Task 5: Publish the verified commit as GitHub main

**Files:**
- Git configuration only; no repository file changes

- [ ] **Step 1: Reconfirm the destination has no branch race**

```bash
git ls-remote --heads git@github.com:Dellsonorb/Simulation-Platform.git
```

Expected: no heads before the first push.

- [ ] **Step 2: Add the approved remote**

```bash
git remote add origin git@github.com:Dellsonorb/Simulation-Platform.git
git remote get-url origin
```

Expected: the printed URL exactly matches the approved repository.

- [ ] **Step 3: Perform a normal first push**

```bash
git push -u origin HEAD:main
```

Expected: a new remote `main` is created without force.

- [ ] **Step 4: Verify the remote commit and published tree**

```bash
test "$(git rev-parse HEAD)" = \
  "$(git ls-remote origin refs/heads/main | cut -f1)"
git status --short --branch
git remote -v
```

Expected: local HEAD equals GitHub `main`, the worktree is clean, and the local
implementation branch/worktree remains present.
