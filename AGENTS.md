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
├─ exts/bricksim/         # Omniverse extension (Python)
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

## Build & Test
1. Only if you modified the C++ code, run `RUN_TESTS=1 scripts/build.sh` to build and sanity‑check.

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
5. Use the `bricksim.utils.*` module wherever needed.
6. Never delete tests because they can't pass. If you do this, you are cheating. A programmer who cheats will be fired.

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
8. The source code of Omni PhysX, the extension that bridges Omniverse and PhysX, is located at `../PhysX/omni/`.

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
