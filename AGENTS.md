# AGENTS.md

Isaac Sim 5.1 extension for simulating LEGO bricks and their assembly. This repository supports a research project.

This project is written in C++26 (with modules) and Python 3.11. The project is mostly C++.

## ExecPlans
- When writing complex features or significant refactors, use an ExecPlan (as described in .agents/PLANS.md) from design to implementation.
- When a task is updated/finished, add/update the relevant docs and update the relevant ExecPlan.
- Store task ExecPlans under `.agents/private-plans/`.

## Subagents
- If you are Codex, use gpt-5.5 xhigh when spawning subagents.

## Build & Test
`uv` is used to manage the Python part. `pixi` is used to manage the C++ native extension part.

### Python workflow
- `uv sync --locked` to set up the virtual environment and install dependencies, only if the environment is not set up yet. Run without `--locked` if dependencies have changed and you want to update them.
- If Isaac Sim / Isaac Lab dependencies have changed, or `ty.toml` is outdated, run `uv run bricksim-type-configs -y` to regenerate `ty.toml` from `ty.base.toml` and the current environment.
- `uv run ruff check` and `uv run ty check` for linting and type checking.

### C++ workflow
You need this only when you modify the C++ code.
- `pixi run build-native` to build the native extension.
- `pixi run test-native-release` and `pixi run test-native-debug` to run native tests in both release and debug modes. This is time-consuming, so run them only when you're ready to commit.

## Coding Style
These rules are non-negotiable. Enforce them strictly and always check for violations after writing code.

### General Principles
- Fail fast: if something might error and we can’t recover, let it error. Don’t add catch‑and‑rethrow or cosmetic error handling -- keep code concise.
- No preemptive engineering: implement only what’s needed now. For example:
   - If a header is required, include it. Don’t add existence checks -- let the build fail if it’s missing.
   - Assume Linux unless otherwise requested; don’t add Windows/macOS branches preemptively.
   - In CMake, specify the header/library paths you need; don’t add detection logic -- let it error if absent.
- Math formulation first: always be explicit about
   - Units: SI vs stage units
   - Storage: row‑major vs column‑major
   - Tensor shapes: ordering and conventions
   - Quaternions: ordering (xyzw vs wxyz)
- Less code > more code: avoid unnecessary abstractions and boilerplate.
- Never delete tests because they can't pass. If you do this, you are cheating. A programmer who cheats will be fired.
- Naming conventions:
   - Reuse canonical domain terms. Do not invent synonyms.
   - Optimize names for readability at the call site.
   - Avoid vague names and unnecessary abbreviations.

### Python Specific Rules
- Do not use `typing.Any` or `typing.cast` to bypass type checking. Add precise types, protocols, or local stubs instead. However, do not over-engineer types in sacrifice of performance.
- Do not use:
   - `from __future__ import annotations`
   - `TYPE_CHECKING` blocks
   - `... as ...` imports unless truly necessary
- Unless necessary or by common convention, do not import whole packages; import the specific functions, classes, or symbols you use.

### C++ Specific Rules
This is a modular C++26 project using modules. Modular C++ is different from ordinary C++.
Non modular 3rd-party dependencies are wrapped in a single `bricksim.vendor` (`vendor.cppm`) module, which includes required 3rd-party headers in GMF and re-exports the needed symbols in a module. See `docs/CPP26_MODULES_CHEATSHEET.md` for details.

#### The vendor module (`vendor.cppm`)
- This is the only single module where `#include` is allowed. Only 3rd-party header includes are allowed here. STD header includes are prohibited.
- Except third-party includes and re-exports. Nothing is allowed here, except with direct approval from the user.
- `vendor.cppm` must re-export the third-party definitions that the rest of the code uses.
- Re-export third-party names in the same top-level namespace they normally use, for example `export namespace Eigen { using Eigen::ColMajor; }`.
- Importers should use the same names they would use in regular C++. No namespace alias should be needed just to use third-party APIs.
- Do not prefix names with `::` unless a documented build failure proves it is necessary because of a real name conflict.
- Do not write template aliases for third-party names when a normal `using` re-export is sufficient.
- Wrap all vendor re-export sections with a single one
  `// NOLINTBEGIN(misc-unused-using-decls)` and a single one
  `// NOLINTEND(misc-unused-using-decls)`.
- `import std;` is strictly prohibited in `vendor.cppm`

#### Non-Vendor Modules
- Do not split declarations and definitions in a header-style layout.
   - Put declarations and implementation together in `.cppm` files.
   - Header-style companion in traditional C++ is strictly prohibited.
   - This rule is zero-tolerance. A `.cppm` file must not read like a header plus implementation file.
   - If a forward declaration is truly required because of C++ ordering, keep it minimal, local, and exceptional.
- `#include` is prohibited strictly in all non-vendor modules.
- `#include` of standard library headers is also prohibited. Use `import std;` instead.
- Use third-party dependencies by importing the vendor module.
- `export import` (re-exporting) is prohibited.
- Top-level `static` is prohibited.
- Anonymous namespaces (`namespace { ... }`) are prohibited.
- Nested namespaces are prohibited unless explicit approval from user. Prefer a single-level namespace such as `bricksim`.
   - Do not create implementation-only namespaces such as `impl` or similar. Modules already provide symbol isolation.
- Do not reopen the same nearby namespace repeatedly. Use one fused namespace block instead.
- Export public definitions. If a type, function, constant, or other definition is part of the public module API, mark it `export`.
- Function return types must be explicit.
   - `auto` return types are prohibited.
   - Prefer writing the return type in front of the function name.
   - Use `auto` or `decltype(auto)` only when the function is a template and the return type genuinely cannot be written explicitly.
- Use the `bricksim.utils.*` module wherever needed.

#### Non-modules
- `.cpp` is allowed only for tests and the final entrypoint translation unit.

## Inspecting Isaac Sim dependencies
- Isaac Sim 5.1 is partially open‑sourced.
   - Some C++ is open; other parts are closed or header‑only.
   - Source code lives at `../IsaacSim-5.1` (relative to this project directory).
- PhysX source checkout lives at `../PhysX-107.3-omni-and-physx-5.6.1` (relative to this project directory).
   - The source code of Omni PhysX, the extension that bridges Omniverse and PhysX, is located in `omni/` inside PhysX source.
- OpenUSD source checkout lives at `../OpenUSD-v24.05` (relative to this project directory).
- Treat online docs as potentially stale; verify against local Isaac Sim code.
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
- Long running code must show progress. If something isn't right during running, do not hope it will be okay. Do not waste time. Kill immediately and debug.

## Collaboration Rules
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
