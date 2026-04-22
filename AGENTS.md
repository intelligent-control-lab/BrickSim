# AGENTS.md

Isaac Sim 6.0 extension for simulating LEGO bricks and their assembly. This repository supports a research project.

This project is written in C++26 (with modules) and Python 3.12. The project is mostly C++.

## Build & Test
- Only if you modified the C++ code, run `pixi run build-native` to build and sanity‑check. Testing is needed only if you're ready to commit, do not run them in regular development because it's time-consuming.

## Linting & Type Checking
- Use Ruff and ty for Python linting and type checking.
- Do not use `typing.Any` or `typing.cast` to bypass type checking. Add precise types, protocols, or local stubs instead.

## Coding Style
- Fail fast: if something might error and we can’t recover, let it error. Don’t add catch‑and‑rethrow or cosmetic error handling -- keep code concise.
- No preemptive engineering: implement only what’s needed now.
   - If a header is required, include it. Don’t add existence checks -- let the build fail if it’s missing.
   - Assume Linux unless otherwise requested; don’t add Windows/macOS branches preemptively.
   - In CMake, specify the header/library paths you need; don’t add detection logic -- let it error if absent.
- Math formulation first: always be explicit about
   - Units: SI vs stage units
   - Storage: row‑major vs column‑major
   - Tensor shapes: ordering and conventions
   - Quaternions: ordering (xyzw vs wxyz)
- Less code > more code: avoid unnecessary abstractions and boilerplate.
- Use the `bricksim.utils.*` module wherever needed.
- Never delete tests because they can't pass. If you do this, you are cheating. A programmer who cheats will be fired.
- In Python, do not use `from __future__ import annotations`.
- In Python, do not use `TYPE_CHECKING` blocks.
- In Python, unless necessary or by common convention, do not import whole packages; import the specific functions, classes, or symbols you use.
- In Python, do not use `... as ...` imports unless truly necessary.
- Naming conventions:
   - Reuse canonical domain terms. Do not invent synonyms.
   - Optimize names for readability at the call site.
   - Avoid vague names and unnecessary abbreviations.
- In C++ modules, use `bricksim.vendor` for external header dependencies. Add all third-party dependencies there instead of including third-party headers ad hoc across other module units or translation units. For details, refer to `docs/CPP26_MODULES_CHEATSHEET.md`.
- In `.cppm` files other than `vendor.cppm`, `#include` is prohibited. Use `import std;` for standard library dependencies. For third-party dependencies, add them to `bricksim.vendor`, re-export the needed symbols there, and then `import bricksim.vendor;`. Refer to `docs/CPP26_MODULES_CHEATSHEET.md` for the expected pattern.
- `export import` is prohibited. Do not re-export modules.
- Top-level `static` is prohibited in modules. It is unnecessary there.
- Anonymous namespaces (`namespace { ... }`) are prohibited in modules. They are unnecessary there.
- Export public definitions. If a type, function, constant, or other definition is part of the public module API, mark it `export`.

## APIs & Docs
- Isaac Sim 6.0 is partially open‑sourced.
  - Some C++ is open; other parts are closed or header‑only.
- Treat online docs as potentially stale; verify against local Isaac Sim code.
- When blocked, consult local sources.
   - Isaac Sim source code lives at `../IsaacSim` (relative to this project directory).
   - PhysX source checkout lives at `../PhysX` (relative to this project directory).
   - OpenUSD source checkout lives at `../OpenUSD` (relative to this project directory).

### Where to Look / How to Inspect
- If you need to know something about Isaac Sim or PhysX or OpenUSD, consult their source code.
- The source code of Omni PhysX, the extension that bridges Omniverse and PhysX, is located in `omni/` inside PhysX source.
- Decompile/disassemble/reverse‑engineer binaries when source is unavailable.
- Many dirs are symlinks created by repoman; they may point outside the tree. Follow them, and enable following links in searches.

## Safety
You may operate in:
- This project directory
- Source directories of Isaac Sim, PhysX, and OpenUSD (relative paths as above)
- Common system locations (e.g., system headers)
- Any other directories/files that are explicitly referenced
- NEVER change files not belonging to this repository, UNLESS you ask the user for this an get an explicit approval 

You must NOT:
- Run global searches from filesystem root, or from the home directory, or other directories not listed above -- they are too large -- unless you first ask the user for approval.

If you start Isaac Sim or other processes, you must ensure it's terminated after you are done. These processes can consume significant resources, and might not respond to SIGINT. Use `pgrep` to find their PIDs and `kill` to terminate them if needed.

## Collaboration Rules (How to Work With the User)
- NEVER modify code unless the user explicitly asks (e.g., says "modify", "implement", "refactor", "add", or similar). Observations and diagnostics are fine; changes require explicit instruction.
- Respect explicit decisions: do not change established values or designs (e.g., callback orders, algorithm choices) without strong evidence and prior approval.
- No overdesign by default: avoid adding retries, background tasks, or new behaviors unless the user requests them. Propose ideas first; only implement after approval in a subsequent round.
- Minimize unsolicited scope changes: keep edits surgical and aligned with the exact request; prefer proposing alternatives rather than implementing them.
- Reflect and learn: when asked to revise behavior, summarize the lesson and how to apply it across future tasks.
- Update this file only on request: edit AGENTS.md if and only if the user explicitly says “update agents.md”.

## Debugging
- You can use gdb to do interactive debugging.
- You can use `scripts/find_bricksim_pid.py` to find the PID of running BrickSim process. If the process is not running, ask the user to start it. Don't start it yourself.
- The executable to use is `.venv/bin/python`.
- When you need the user to trigger error, ask the user to do so before proceeding.
- Be cautious not producing large amount of output, otherwise your context window would be full.
- ****IMPORTANT**** Use `functions.request_user_input` command to do interactive debugging with the user. You can ask the user to perform an operation, notify you when something happens (tool call will block until user proceed), ask user's opinion or preference, let user describe what happened.
