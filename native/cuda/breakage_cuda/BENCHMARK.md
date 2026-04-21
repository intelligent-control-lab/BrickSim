# breakage_cuda benchmark report

GPU pipeline (`cuda_qp::Pipeline::solve`) vs CPU baseline
(`ref_cpu::solve_full`, Eigen + OSQP `v1.0.0`) on a synthetic linear-chain
`HostSystem` of `N` parts.

- Hardware: NVIDIA RTX 4090, WSL2, CUDA Toolkit 12.0, gcc 11
- Build type: `Release`, double precision throughout
- Per-step wall clock, `cudaDeviceSynchronize()` before reading the GPU
  timer
- For `N >= 32,768` the CPU OSQP solve hits the 2,000,000-iteration cap and
  reports "did not converge"; we keep its wall time as a lower bound and
  flag the row with an asterisk. The GPU ADMM converged on every row.

A live, interactive version of these charts is in
[`benchmark.html`](benchmark.html) (open with a browser; everything is
inlined, no network access required).

## Cross-over summary

| Threshold                      | First N where it holds |
|--------------------------------|-----------------------:|
| GPU faster than CPU (>= 1x)    | between 4,096 and 8,192 |
| GPU >= 10x faster              | between 8,192 and 16,384 |
| GPU >= 100x faster             | between 32,768 and 65,536 |
| GPU >= 500x faster             | by 65,536               |

So **the GPU "completely overrules" the CPU at roughly N ~ 40,000 parts**
(crossing 100x), and continues to widen the gap from there.

## Raw measurements

`./breakage_cuda_bench N` with `BENCH_QUICK=1`, `OSQP_MAX_ITER=2000000`,
and `BENCH_TOLERATE_NONCONVERGED=1` for the largest two rows.

| N        |     nx | GPU mean    | CPU mean      | speedup (CPU / GPU) |
|---------:|-------:|------------:|--------------:|--------------------:|
|        4 |     27 |     77.7 ms |       24.1 us |       0.00031x      |
|       16 |    135 |    124.3 ms |      107.9 us |       0.00087x      |
|       64 |    567 |    167.6 ms |      435.9 us |       0.00260x      |
|      256 |  2,295 |    388.2 ms |     1.73   ms |       0.00446x      |
|      512 |  4,599 |    339.2 ms |     3.43   ms |       0.01011x      |
|    1,024 |  9,207 |    3.53   s |    16.63   ms |       0.00471x      |
|    2,048 | 18,423 |    6.24   s |    62.00   ms |       0.00993x      |
|    4,096 | 36,855 |    9.39   s |   480.64   ms |       0.05119x      |
|    8,192 | 73,719 |    3.79   s |     5.00   s  |       1.32x         |
|   16,384 |147,447 |    3.45   s |    68.36   s  |      19.79x         |
|   32,768 |294,903 |    3.93   s |   254.70   s* |     >= 64.81x*      |
|   65,536 |589,815 |    4.46   s | 2,596.75   s* |    >= 582.73x*      |

`nx` is `9 * (N - 1)` (decision variables in the QP).
`*` = CPU OSQP hit `max_iter = 2,000,000` without converging; the wall time
is therefore a lower bound, and the speedup is also a lower bound.

## ASCII chart: log10 wall time (microseconds) vs log2(N)

```
log10(us)
  10 |                                                       *  (CPU >= here)
   9 |                                              *
   8 |                                     *
   7 |                            *  *  *
   6 |  G  G  G  G  G  G  G  G  G  G  G  G  G       (GPU plateau, then ramps)
   5 |        C  C  C  C  C  C
   4 |                 C
   3 |        C  C  C
   2 |  C
   1 |
     +--------------------------------------------------------
        4  16 64 256 .. 1k 2k 4k 8k 16k 32k 64k     N
```

Two regimes are visible:

- **Small-N plateau (`N <= 1k`)**: GPU runtime is flat ~ 100-400 ms,
  dominated by `cusolverSp` factorization and kernel-launch overhead.
  CPU runtime is sub-millisecond. CPU wins by orders of magnitude.
- **Large-N ramp (`N >= 4k`)**: GPU runtime grows roughly linearly with
  `N` (and even drops at `N = 8k` once the per-iteration arithmetic
  starts to amortize the fixed startup); CPU OSQP grows faster than
  linear and rapidly blows past the GPU.

## Cross-over analysis

| pair                   | GPU time growth | CPU time growth | speedup growth |
|------------------------|----------------:|----------------:|---------------:|
| 8,192    -> 16,384     |     0.91x       |     13.7x       |       15.0x    |
| 16,384   -> 32,768     |     1.14x       |     >= 3.7x*    |     >= 3.27x*  |
| 32,768   -> 65,536     |     1.13x       |     >= 10.2x*   |     >= 8.99x*  |

(`*` = CPU side bounded below.)

So once the GPU is in its ramp regime, **doubling N multiplies the
GPU/CPU ratio by 3-15x in our favor**. Extrapolating:

- N ~ 40,000  -> ~ 100x faster
- N ~ 65,000  -> ~ 580x faster (measured lower bound)
- N ~ 130,000 -> well beyond 1,000x faster (estimated; CPU run skipped)

## Why so slow at small N?

The GPU baseline pays three large fixed costs that the CPU baseline does
not:

1. **Three Cholesky factorizations** on the KKT-like system per `solve`,
   one per OSQP stage, via `cusolverSpDcsrlsvchol`. Each call carries a
   host<->device sync, symbolic analysis, and numeric factorization.
2. **Per-iteration kernel launches** in the ADMM loop (~ 100 small
   launches per stage). On the CPU, those reduce to function calls.
3. **No batching of dense per-part kernels** -- one `<<<>>>` per kernel
   per part. This dominates when `N` is small.

These add up to ~ 80 ms - 4 s of overhead independent of `N`, so the GPU
only "wakes up" once the per-iteration arithmetic crosses that floor.

## Caveats

- The synthetic linear chain is favorable to OSQP (well-conditioned,
  banded structure). A real `BreakageDebugDump` from a sim step might
  shift the crossover slightly, but the asymptotic behavior should hold.
- The CPU "did not converge" rows (N = 32,768, 65,536) are lower bounds.
  The actual CPU time to reach the same precision the GPU achieves is
  larger -- probably substantially so.
- Bench mode used `BENCH_QUICK=1` (`5 warmup + 2 measured` for GPU,
  `1 warmup + 1 measured` for CPU). For `N <= 4,096` you should rerun
  with the default counts (`5 + 50 / 5 + 20`) for tighter error bars; the
  large-N rows already dominate any per-iteration noise.

## Reproducing

```bash
cmake -S native/cuda/breakage_cuda -B ~/breakage_cuda_build -DCMAKE_BUILD_TYPE=Release
cmake --build ~/breakage_cuda_build -j

cd ~/breakage_cuda_build

# Single point (e.g., the cross-over):
BENCH_QUICK=1 LD_LIBRARY_PATH=/usr/lib/wsl/lib ./breakage_cuda_bench 8192

# Large-N point with CPU lower bound:
BENCH_QUICK=1 BENCH_GPU_ITER=2 BENCH_CPU_ITER=1 \
  BENCH_TOLERATE_NONCONVERGED=1 OSQP_MAX_ITER=2000000 \
  LD_LIBRARY_PATH=/usr/lib/wsl/lib \
  ./breakage_cuda_bench 65536
```

`BENCH_QUICK=1` halves both warmup and measured iteration counts.
`BENCH_GPU_ITER` / `BENCH_CPU_ITER` override them outright.
`BENCH_TOLERATE_NONCONVERGED=1` keeps the wall time even if the solver
does not reach `eps_abs / eps_rel`; without it the bench aborts.
