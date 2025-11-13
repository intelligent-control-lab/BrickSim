# AGENTS.md

Isaac Sim 5.1 extension for simulating LEGO bricks and their assembly. This repository supports a research project.

This project is written in C++26 (with modules) and Python 3.11. The project is mostly C++.

## Codebase Structure
```
.
├─ native/                # C++ part of the extension
│  ├─ modules/            # C++ Modules (Core)
│  ├─ py/                 # C++ Modules (pybind11 bindings)
│  ├─ tests/              # C++ unit tests
│  └─ ...
├─ exts/lego_assemble/    # Omniverse extension (Python)
├─ demos/                 # Demos for conducting research experiments
├─ IsaacLab/              # IsaacLab submodule
├─ resources/             # USD files and other assets
├─ scripts/
│  ├─ build.sh            # Build & test C++ code
│  ├─ launch_isaacsim.sh  # Launches Isaac Sim for debugging
│  └─ ...
├─ .vscode/               # VS Code workspace settings
├─ .venv/                 # Project virtualenv
└─ ...
../IsaacSim/              # IsaacSim source checkout (optional; for dev purpose)
../PhysX/                 # PhysX source checkout (optional; for dev purpose)
../OpenUSD/               # OpenUSD source checkout (optional; for dev purpose)
```

## Current Status
The project is undergoing huge rewriting for the C++ part.
In `native/modules/lego_assemble`:
 * `core`, `physx`, `usd`, `utils`, `vendor` are the new rewritten code work in progress.
 * Everything else in this directory is old code.

## Build & Test
1. Only if you modified the C++ code, run `scripts/build.sh` to build and sanity‑check.

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
5. Use the `lego_assemble.utils.*` module wherever needed.

## APIs & Docs
- Isaac Sim 5.1 is partially open‑sourced.
  - Python is open‑sourced.
  - Some C++ is open; other parts are closed or header‑only.
- Treat online docs as potentially stale; verify against local Isaac Sim code.
- When blocked, consult local sources.
- Isaac Sim source code lives at `../IsaacSim` (relative to this project directory).
- PhysX source checkout lives at `../PhysX` (relative to this project directory).
- OpenUSD source checkout lives at `../OpenUSD` (relative to this project directory).

### Where to Look / How to Inspect
1. If you need to know something about Isaac Sim or PhysX or OpenUSD, consult `../IsaacSim` and `../PhysX` and `../OpenUSD`.
2. Use `rg` (ripgrep) or similar to search symbols in headers or even binary files.
3. Decompile/disassemble/reverse‑engineer binaries when source is unavailable.
4. Many dirs are symlinks created by repoman; they may point outside the tree. Follow them, and enable following links in searches.
5. If you are looking for Isaac Sim's source, search in `../IsaacSim/`, which contains its source checkout.
6. If you are looking for PhysX's source, search in `../PhysX/`, which contains its full source. `../IsaacSim/` doesn't contain PhysX's source.
7. If you are looking for OpenUSD's source, search in `../OpenUSD/`, which contains its full source. `../IsaacSim/` doesn't contain OpenUSD's source.
8. Omni PhysX is not open-source, so do reverse engineering (disassemble / decompile) on its binaries. You can find the binaries by performing `find -L ../IsaacSim '*.so'` with `grep` to filter what you are looking for.

## Safety
You may operate in:
1) This project directory
2) `../IsaacSim` directory and `../PhysX` directory and `../OpenUSD` directory
3) Common system locations (e.g., system headers)
4) Any other directories/files that are explicitly referenced
5) NEVER change files not belonging to this repository (including `../OpenUSD`, `../PhysX`, `../IsaacSim` and `IsaacLab`), UNLESS you ask the user for this an get an explicit approval 

You must NOT:
- Run global searches from filesystem root, or from the home directory, or other directories not listed above—they are too large—unless you first ask the user for approval.

## Collaboration Rules (How to Work With the User)
- NEVER modify code unless the user explicitly asks (e.g., says "modify", "implement", "refactor", "add", or similar). Observations and diagnostics are fine; changes require explicit instruction.
- Respect explicit decisions: do not change established values or designs (e.g., callback orders, algorithm choices) without strong evidence and prior approval.
- No overdesign by default: avoid adding retries, background tasks, or new behaviors unless the user requests them. Propose ideas first; only implement after approval in a subsequent round.
- Minimize unsolicited scope changes: keep edits surgical and aligned with the exact request; prefer proposing alternatives rather than implementing them.
- Reflect and learn: when asked to revise behavior, summarize the lesson and how to apply it across future tasks.
- Update this file only on request: edit AGENTS.md if and only if the user explicitly says “update agents.md”.

## Designs
IMPORTANT: Due to the refactor, the design below no longer holds! They are only for reference!
> - Dimensions: LEGO bricks are defined by L × W × H.
>   - L and W are measured in studs.
>   - H is measured in plate heights; a plate has height 1, and a brick typically has height 3.
> - Rigid bodies: Bricks are modeled as rigid bodies.
> - Assembly mechanics: Assembly is detected when two bricks are aligned and sufficient force is applied; a joint is added between them after assembly.
> 
> - Geometry
>   - Visual: a main body cube plus L × W cylindrical studs.
>   - Colliders: a main body cube (`BodyCollider`) plus a cube approximating the bounding box of the top studs (`TopCollider`).
>     - `BodyCollider`: the solid portion of a brick, excluding the top studs.
>     - `TopCollider`: sits on top of `BodyCollider`; same length and width; height equals the stud height. This is an approximation.
>   - Holes are not modeled.
> 
> - Assembly behavior
>   - When two bricks assemble, their poses are adjusted to produce a “snap‑fit” effect. This removes small alignment errors within the threshold that permits assembly.
>   - After assembly, collisions between the upper brick's `BodyCollider` and the lower brick’s `TopCollider` are excluded from physics simulation.
>   - The relative distance is adjusted so the bottom of the upper brick contacts the top of the lower brick’s `BodyCollider`. In this state, the lower brick’s `TopCollider` (studs) lies completely within the upper brick (with collisions disabled), mimicking studs entering holes.
> 
> - Constraints & connections
>   - USD authoring: Each connection is an Xform prim `Conn_i_j` with:
>     - `lego_conn:enabled` (Bool); `lego_conn:body0`, `lego_conn:body1` (relationships to brick prims).
>     - `lego_conn:pos` (Float3) and `lego_conn:rot` (Quatf): relative transform T_parent_child from parent actor‑local origin to child actor‑local origin; positions are in stage units.
>   - Base constraint: Custom PhysX weld locking all 6 DOF. Anchors are in COM frames computed from the authored origin‑to‑origin transform:
>     - parentLocal (A_com → B_com) = (A_com → A_orig) × T_parent_child × (B_orig → B_com); childLocal = identity.
>   - Auxiliary constraints: Skip‑graph adds extra welds at graph distances 2^i (i=1..8) for stack stiffness; each aux transform is the composed transform along the path; recomputed on connect/disconnect.
>   - Contact filtering: While connected, collisions between parent `TopCollider` and child `BodyCollider` are excluded via a patched simulation filter; filtering is refreshed on affected actors.
>   - Lifecycle: Native joint manager listens to USD edits and PhysX object creation; creates/tears down constraints when both bodies exist in the same PhysX scene.

## Debugging
- If `gdb` tool is available, you can use gdb to do interactive debugging.
- Isaac Sim's PID can be obtained by `pgrep -nx isaacsim`. If the process not running, ask the user to start it. Don't start on yourself.
- The executable to use is `.venv/bin/python`.
- When you need the user to trigger error, ask the user to do so before proceeding.
- Be cautious not producing large amount of output, otherwise your context window would be full.
- Use `zenity` command! ****IMPORTANT****
  - You can ask the user to assist with debugging, such as perform an operation, notify you when something happens (tool call will block until user proceed), ask user's opinion or preference, let user describe what happened.
  - You should ALWAYS use `zenity` in an interactive scenario, and AVOID STOPPING response.
  - Don't worry about being blocked, it's intended to be blocked. If it times out, you can always retry.
  - Always use text input, don't use buttons so the user can provide feedback.
