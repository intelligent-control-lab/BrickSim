# AGENTS.md

Isaac Sim 5.0 extension for simulating LEGO bricks and their assembly. This repository supports a research project.

## Codebase Structure
```
.
├─ .venv/                 # Project virtualenv; bin/activate customized to add Isaac Sim paths/env
├─ extension/
│  ├─ source/             # Extension source
│  │  ├─ lego_assemble/   # Python package
│  │  ├─ cpp/             # C++ code
│  │  └─ .gitignore       # Extension-specific gitignore
│  └─ scripts/            # Scripts for conducting research experiments
├─ isaacsim/              # Isaac Sim app (git-ignored)
├─ IsaacLab/              # IsaacLab submodule
│  └─ _isaac_sim/         # Symlink to ./isaacsim/
├─ resources/             # USD files and other assets
├─ scripts/
│  ├─ venv-activate-addon.sh.template  # Snippet appended to venv activate
│  ├─ build_fast_debug.sh               # Invokes CMake to build C++ code
│  └─ launch_isaacsim.sh                # Launches Isaac Sim for debugging
└─ .vscode/               # VS Code workspace settings
../IsaacSim/              # IsaacSim source
../PhysX/                 # PhysX source
```

## Build & Test
1. Only if you modified the C++ code, run `scripts/build_fast_debug.sh` to build and sanity‑check.
2. Scripts under `scripts/` auto‑activate the virtualenv (they use `/usr/bin/env bash`), so you don’t need to source `.venv` when running them.
3. If you run commands manually (e.g., Python modules), ensure the virtualenv is active (`source .venv/bin/activate`) unless your shell already has it.

## Coding Style
1. Fail fast: if something might error and we can’t recover, let it error. Don’t add catch‑and‑rethrow or cosmetic error handling—keep code concise.
2. No preemptive engineering: implement only what’s needed now.
   - If a header is required, include it. Don’t add existence checks—let the build fail if it’s missing.
   - Assume Linux unless otherwise requested; don’t add Windows/macOS branches preemptively.
   - In CMake, specify the header/library paths you need; don’t add detection logic—let it error if absent.
3. Math formulation first: always be explicit about
   - Units: SI vs stage units
   - Storage: row‑major vs column‑major
   - Tensor shapes: ordering and conventions
   - Quaternions: ordering (xyzw vs wxyz)
4. Less code > more code: avoid unnecessary abstractions and boilerplate.

## APIs & Docs
- Isaac Sim 5.0 is partially open‑sourced.
  - Python is open‑sourced.
  - Some C++ is open; other parts are closed or header‑only.
- Treat online docs as potentially stale; verify against local Isaac Sim code.
- When blocked, consult local sources.
- Isaac Sim source code lives at `../IsaacSim` (relative to this project directory).
- PhysX source checkout lives at `../PhysX` (relative to this project directory). The version is 107.3-physx-5.6.1, which is used by current Isaac Sim build.

### Where to Look / How to Inspect
1. Consult anything under `../IsaacSim` and `../PhysX`.
2. Dependent libraries are under `../IsaacSim/_build` (many in `../IsaacSim/_build/target-deps`).
3. Use `rg` (ripgrep) or similar to search symbols in headers or even binary files.
4. Decompile/disassemble/reverse‑engineer binaries when source is unavailable.
5. Many dirs are symlinks created by repoman; they may point outside the tree. Follow them, and enable following links in searches.

## Safety
You may operate in:
1) This project directory
2) `../IsaacSim` directory and `../PhysX` directory
3) Common system locations (e.g., system headers)
4) Any other directories/files that are explicitly referenced
5) NEVER change files not belonging to this repository (including `../PhysX`, `../IsaacSim` and `IsaacLab`), UNLESS you ask the user for this an get an explicit approval 

You must NOT:
- Run global searches from filesystem root, or from the home directory, or other directories not listed above—they are too large—unless you first ask the user for approval.

## Collaboration Rules (How to Work With the User)
- Respect explicit decisions: do not change established values or designs (e.g., callback orders, algorithm choices) without strong evidence and prior approval.
- No overdesign by default: avoid adding retries, background tasks, or new behaviors unless the user requests them. Propose ideas first; only implement after approval in a subsequent round.
- Minimize unsolicited scope changes: keep edits surgical and aligned with the exact request; prefer proposing alternatives rather than implementing them.
- Reflect and learn: when asked to revise behavior, summarize the lesson and how to apply it across future tasks.
- Update this file only on request: edit AGENTS.md if and only if the user explicitly says “update agents.md”.

## Designs

- Dimensions: LEGO bricks are defined by L × W × H.
  - L and W are measured in studs.
  - H is measured in plate heights; a plate has height 1, and a brick typically has height 3.
- Rigid bodies: Bricks are modeled as rigid bodies.
- Assembly mechanics: Assembly is detected when two bricks are aligned and sufficient force is applied; a joint is added between them after assembly.

- Geometry
  - Visual: a main body cube plus L × W cylindrical studs.
  - Colliders: a main body cube (`BodyCollider`) plus a cube approximating the bounding box of the top studs (`TopCollider`).
    - `BodyCollider`: the solid portion of a brick, excluding the top studs.
    - `TopCollider`: sits on top of `BodyCollider`; same length and width; height equals the stud height. This is an approximation.
  - Holes are not modeled.

- Assembly behavior
  - When two bricks assemble, their poses are adjusted to produce a “snap‑fit” effect. This removes small alignment errors within the threshold that permits assembly.
  - After assembly, collisions between the upper brick and the lower brick’s `TopCollider` are excluded from physics simulation.
  - The relative distance is adjusted so the bottom of the upper brick contacts the top of the lower brick’s `BodyCollider`. In this state, the lower brick’s `TopCollider` (studs) lies completely within the upper brick (with collisions disabled), mimicking studs entering holes.
