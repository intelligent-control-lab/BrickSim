# breakage_cuda

A standalone CUDA prototype of the per-frame hot path of
`bricksim::BreakageChecker::solve` (see
`native/modules/bricksim/physx/breakage.cppm`).

This is a sub-project of BrickSim, but it does **not** link against any
`bricksim_modules` target. It builds and tests on its own so it can be
iterated on independently.

## What is implemented

End-to-end on the GPU, mirroring `BreakageChecker::solve`:

1. `fit_se3` (weighted SE(3) Procrustes + 3x3 SVD)
2. `fit_twist` (weighted twist fit + 3x3 LDLT)
3. `compute_Pi`, `compute_L`
4. RHS `b` assembly
5. Three-stage QP solve (projection -> relaxation -> optimization)
6. Whitening reversal + per-clutch utilization

The CPU baseline reuses Eigen + the OSQP C library (built via FetchContent at
`v1.0.0`) to keep the comparison apples-to-apples with the production code
path.

## What is NOT implemented

- `BreakageChecker::build_system` (geometry-driven, runs once per
  reassembly; not on the per-step hot path). The CUDA layer consumes an
  already-built system serialized as the JSON layout produced by
  `bricksim::to_json(BreakageDebugDump)`.
- OSQP polish, infeasibility / unboundedness detection, scaling/Ruiz
  equilibration, indirect (PCG) mode, multiple algebra backends.
- The full OSQP rho-update heuristic; we only implement
  `rho_new = rho * sqrt(||r_prim|| / ||r_dual||)` and refactor when the
  ratio crosses 5x.

## Build

Linux + CUDA Toolkit 12.x recommended (the parent project assumes Linux).
Windows users should build inside WSL2.

```bash
cd native/cuda/breakage_cuda
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
```

The first configure pulls Eigen, nlohmann_json and OSQP via `FetchContent`.

### WSL2 specifics

Two non-obvious gotchas surfaced during initial bring-up:

1. **`configure_file: Operation not permitted` when the build dir is on
   `/mnt/c/`**. The Windows mount lacks the POSIX bits CMake needs for
   `configure_file`. Put the build directory on the WSL native filesystem,
   e.g.:
   ```bash
   cmake -S "/mnt/c/.../native/cuda/breakage_cuda" -B ~/breakage_cuda_build
   cmake --build ~/breakage_cuda_build -j
   ```
2. **`no CUDA-capable device is detected` at runtime**, even though
   `nvidia-smi` works. WSL2 ships the real `libcuda.so.1` at
   `/usr/lib/wsl/lib/libcuda.so.1`, which is not on the default loader
   search path. `libcudart` resolves the driver through `dlopen`, so
   binary RPATH/RUNPATH does **not** help; you have to set
   `LD_LIBRARY_PATH`. Two options:
   - Run via `ctest` -- the CMake test definitions inject
     `LD_LIBRARY_PATH=/usr/lib/wsl/lib:$LD_LIBRARY_PATH` automatically.
   - Run a binary directly with the env var set:
     ```bash
     LD_LIBRARY_PATH=/usr/lib/wsl/lib:$LD_LIBRARY_PATH ./test_dense_kernels
     ```

### Default CUDA architecture

We pin `CMAKE_CUDA_ARCHITECTURES = "70;75;80;86;89;90"` (Volta to Hopper).
CMake's auto-default of `52` is replaced unless the user passes an explicit
`-DCMAKE_CUDA_ARCHITECTURES=...`. SM 6.0+ is required because we use
`atomicAdd(double*, double)` directly.

## Generating a test fixture

`tests/test_full_pipeline.cpp` consumes a `BreakageDebugDump` JSON. To
produce one from a real BrickSim run, set the debug-dump directory and flip
the `DebugDump` threshold on:

```cpp
BreakageChecker checker;
checker.set_debug_dump_dir("/tmp/breakage_dumps");
checker.thresholds.DebugDump = true;
// ... run a sim step ...
```

Then point the test at the produced JSON via the
`BREAKAGE_CUDA_FIXTURE` environment variable.

If you do not have a real dump, the test falls back to a small synthetic
problem generated in-process (asserting the CUDA pipeline matches the CPU
baseline up to relative tolerance).

## Layout

```
include/breakage_cuda/
  system.hpp     - Host POD types + JSON loaders
  device.hpp     - CUDA RAII helpers + DeviceSystem mirror
  ref_cpu.hpp    - CPU reference functions
  kernels.hpp    - Host-side launchers for dense kernels
  qp_solver.hpp  - CUDA ADMM solver + end-to-end pipeline
src/
  system.cpp     - JSON loaders, CSC->CSR upload
  ref_cpu.cpp    - CPU reference implementation
  kernels.cu     - Dense kernels
  linalg.cu      - Shared device helpers (3x3 SVD/LDLT, reductions, SpMV glue)
  qp_solver.cu   - CUDA ADMM
tests/
  test_dense_kernels.cpp - per-kernel CPU vs CUDA tests
  test_qp_random.cpp     - random QP CPU OSQP vs CUDA ADMM
  test_full_pipeline.cpp - JSON dump end-to-end
  bench.cpp              - N-sweep wall clock
```

## Verified test results (RTX 4090, WSL2, CUDA 12.0)

`ctest --output-on-failure` passes 3/3:

```
1/3 Test #1: test_dense_kernels ...............   Passed    0.27 sec
2/3 Test #2: test_qp_random ...................   Passed    0.35 sec
3/3 Test #3: test_full_pipeline ...............   Passed    0.36 sec
```

Per-test numerical agreement (CPU reference vs CUDA, max abs / rel error):

| test                          | metric                                 | value       | tol  |
|-------------------------------|----------------------------------------|-------------|------|
| test_dense_kernels::fit_se3   | max abs                                | 1.11e-16    | 1e-9 |
| test_dense_kernels::fit_twist | max abs                                | 5.55e-17    | 1e-9 |
| test_dense_kernels::compute_Pi| max abs                                | 1.67e-16    | 1e-12|
| test_dense_kernels::compute_L | max abs                                | 7.43e-16    | 1e-9 |
| test_dense_kernels::assemble_b| max abs                                | 2.84e-14    | 1e-9 |
| test_qp_random                | rel inf-norm err on `x`                | 6.13e-08    | 1e-2 |
| test_full_pipeline            | rel inf-norm err on `x_unwhitened`     | 5.48e-06    | 1e-2 |
| test_full_pipeline            | rel inf-norm err on utilization        | 7.32e-08    | 1e-2 |

Dense kernels match the CPU reference to floating-point round-off. The
hand-written ADMM solver agrees with OSQP within ~1e-5 (ADMM iterates
independently in each backend, so bit-equality is not expected).

## Bench results (RTX 4090, WSL2, CUDA 12.0)

Full numbers, charts, and analysis are in
[`BENCHMARK.md`](BENCHMARK.md) (markdown) and
[`benchmark.html`](benchmark.html) (self-contained interactive page,
double-click to open in a browser; no network access required).

Headline numbers from a synthetic linear-chain `HostSystem` with
`Pipeline::solve` vs `ref_cpu::solve_full` (Eigen + OSQP):

| N        | GPU mean | CPU mean      | speedup (CPU / GPU) |
|---------:|---------:|--------------:|--------------------:|
|        4 |  77.7 ms |       24.1 us | 0.0003x (CPU wins)  |
|    1,024 |   3.53 s |     16.63 ms  | 0.005x  (CPU wins)  |
|    4,096 |   9.39 s |    480.64 ms  | 0.05x   (CPU wins)  |
|    8,192 |   3.79 s |      5.00 s   | 1.32x   (cross-over)|
|   16,384 |   3.45 s |     68.36 s   | 19.8x               |
|   32,768 |   3.93 s |    >= 254.7 s | >= 64.8x            |
|   65,536 |   4.46 s |  >= 2,596.8 s | >= 583x             |

Cross-over thresholds:

- **GPU >= 1x faster**: between N = 4,096 and 8,192
- **GPU >= 10x faster**: between N = 8,192 and 16,384
- **GPU >= 100x faster**: between N = 32,768 and 65,536 (~ N = 40,000)
- **GPU >= 500x faster**: by N = 65,536

The two largest rows have CPU "did not converge" within
`OSQP_MAX_ITER = 2,000,000`, so CPU wall times (and the speedup) are
lower bounds. The GPU side converged on every row.

The GPU pipeline pays three large fixed costs that hurt at small N:

- Three full `cusolverSpDcsrlsvchol` factorizations per `solve` (one per
  stage).
- Per-iteration kernel launch overhead in the ADMM loop (~ 100 small
  launches per stage).
- No batching of the dense per-part kernels (one `<<<>>>` per kernel
  per part).

These dominate runtime for `N <= 1,024`. Once `N >= 4,096`, the
per-iteration arithmetic crosses that fixed-cost floor and the GPU
runtime grows much more slowly than the CPU OSQP runtime, opening a
> 500x gap by N = 65,536.

## Caveats and known differences

- Numerical convergence of the hand-written ADMM matches OSQP only up to
  the configured `eps_abs / eps_rel` tolerances. Tests use relative
  comparisons, not bit-exact matching.
- Stage 3 inequality `epsilon` is taken from `osqp.cppm` defaults (`1e-4`).
  Override via `Settings`.
- The current direct KKT factorization uses `cusolverSp` Cholesky; this is
  acceptable for small/medium problems but does not warm-update the
  factorization across rho changes (we refactor instead).
- Double precision throughout.
- WSL2 driver loading: see "WSL2 specifics" above.
- `Eigen::Quaterniond` member init must use `=` instead of `{...}`; the
  brace-init form trips a g++ parser bug visible from `nvcc`'s host
  compile pass.

## TODO

- Replace the direct KKT factorization with PCG + diagonal preconditioner
  to remove the dominant overhead (`cusolverSp` factorization is ~80% of
  per-step time).
- Replace the hand-written ADMM with cuOSQP (when its CMake integration
  matures) for closer numerical agreement with the CPU baseline.
- Batch dense per-part kernels (currently one `<<<>>>` per kernel).
- Optionally lift `build_system` to GPU for end-to-end reassembly speedups.
- Mixed-precision experiments (fp32 ADMM with double-precision residual
  checks).
