# GitHub Publication and README Design

**Date:** 2026-09-03
**Status:** Approved
**Scope:** Documentation and first publication of `SIM/p450_sim_v1` only

## Goal

Publish the completed Simulation Platform repository to
`git@github.com:Dellsonorb/Simulation-Platform.git` as the remote `main`
branch, with a clear bilingual landing page for robotics users and future
contributors.

The published project is the Git repository rooted at `SIM/p450_sim_v1`.
The empty `SIM/.git` placeholder is not a repository, and the local worktree
directory is never part of the published tree.

## README structure

Use a Chinese overview followed by the complete English documentation in one
`README.md`.

The README will contain:

1. Project purpose and current V1.0 status.
2. Supported robots, sensors, control stacks, and ROS/Gazebo versions.
3. The `AGENT -> common runtime -> official native robot stack` boundary.
4. A compact architecture and shared-TF description.
5. The public P450, BUNKER, manipulation, sensor, and grasp interfaces.
6. Host prerequisites and explicit external PX4 dependency requirements.
7. Reproducible build, environment, platform launch, and natural demo commands.
8. Expected Air-Ground Pick task sequence and verification commands.
9. The SIM/REAL replacement boundary and links to the detailed design docs.
10. Repository layout, project scope, and known limitations.

The Chinese section is an orientation and navigation layer rather than a full
duplicate translation. The English section is the canonical operational guide.
The README must distinguish verified current functionality from planned real
hardware adapters.

## Publication contents

Publish the full history reachable from
`feature/bunker-a-implementation` as remote `main`.

Include source, launch files, robot assets required by the repository,
configuration, tests, scripts, and design documentation. Exclude all ignored
generated or machine-local content, including:

- `build/`, `devel/`, `install/`, and `logs/`;
- `.catkin_tools/`, Python caches, import staging, and `.worktrees/`;
- content outside `SIM/p450_sim_v1`, including `P450-PAPER`;
- credentials, private keys, tokens, and machine-specific environment files.

No benchmark, paper evidence pipeline, new safety framework, or new robot
feature is added as part of publication.

## Git strategy

The target GitHub repository is currently empty. Add it as `origin`, then push
the current complete commit graph using:

```text
feature/bunker-a-implementation -> refs/heads/main
```

Use a normal first push, never a force push. Preserve both local branches and
the implementation worktree. Configure the current implementation branch to
track `origin/main` only if Git accepts that mapping cleanly; otherwise leave
the local branch without upstream and report the result.

## Pre-publication checks

Before changing the remote:

1. Confirm the implementation worktree is clean except for the approved README
   and publication documentation commits.
2. Review the final tracked-file set and largest files against GitHub's file
   limits.
3. Scan tracked text for credentials and accidental machine-absolute paths.
   Expected references in historical design documents must be distinguished
   from active runtime dependencies.
4. Verify `.gitignore` excludes generated and worktree content.
5. Run the maintained unit/interface suite and a clean Catkin build using the
   system ROS/Gazebo toolchain.
6. Validate the README commands against installed package and script names.

The already-maintained natural Gazebo E2E remains the functional acceptance
test. It may be rerun if README or publication checks reveal a runtime change;
documentation-only edits do not require inventing an additional benchmark or
evidence workflow.

## Success criteria

- GitHub `main` resolves to the locally verified publication commit.
- The remote tree contains no generated workspaces, logs, worktrees, or
  out-of-scope `P450-PAPER` material.
- The root README gives a new ROS 1 user enough information to understand,
  build, launch, and verify the platform.
- README claims match current tested functionality and label REAL adapters as
  future replacements.
- Local feature branches and worktrees remain intact.
