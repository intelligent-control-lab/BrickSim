// SPDX-License-Identifier: MIT
//
// Lightweight RAII helpers and a device-side mirror of HostSystem. Not a
// full Eigen-on-GPU layer; we just keep raw pointers + sizes and let kernels
// operate on them directly.
//
// All matrices are stored in CSR (row-major) on the device because cuSPARSE
// expects CSR.

#pragma once

#include "breakage_cuda/system.hpp"

#include <cuda_runtime.h>
#include <cusparse.h>

#include <stdexcept>
#include <string>

namespace breakage_cuda {

// ---------------------------------------------------------------------------
// Error helpers.
// ---------------------------------------------------------------------------

#define BREAKAGE_CUDA_CHECK(call)                                              \
  do {                                                                         \
    cudaError_t _err = (call);                                                 \
    if (_err != cudaSuccess) {                                                 \
      throw std::runtime_error(std::string("CUDA error at " __FILE__ ":") +    \
                               std::to_string(__LINE__) + ": " +               \
                               cudaGetErrorString(_err));                      \
    }                                                                          \
  } while (0)

#define BREAKAGE_CUSPARSE_CHECK(call)                                          \
  do {                                                                         \
    cusparseStatus_t _s = (call);                                              \
    if (_s != CUSPARSE_STATUS_SUCCESS) {                                       \
      throw std::runtime_error(std::string("cuSPARSE error at " __FILE__ ":") +\
                               std::to_string(__LINE__) + ": " +               \
                               cusparseGetErrorString(_s));                    \
    }                                                                          \
  } while (0)

// ---------------------------------------------------------------------------
// RAII device buffer of arbitrary scalar.
// ---------------------------------------------------------------------------

template <class T> class DeviceBuffer {
public:
  DeviceBuffer() = default;

  explicit DeviceBuffer(std::size_t n) { resize(n); }

  ~DeviceBuffer() { free(); }

  DeviceBuffer(const DeviceBuffer &) = delete;
  DeviceBuffer &operator=(const DeviceBuffer &) = delete;

  DeviceBuffer(DeviceBuffer &&o) noexcept : ptr_(o.ptr_), n_(o.n_) {
    o.ptr_ = nullptr;
    o.n_ = 0;
  }

  DeviceBuffer &operator=(DeviceBuffer &&o) noexcept {
    if (this != &o) {
      free();
      ptr_ = o.ptr_;
      n_ = o.n_;
      o.ptr_ = nullptr;
      o.n_ = 0;
    }
    return *this;
  }

  void resize(std::size_t n) {
    if (n == n_)
      return;
    free();
    if (n > 0) {
      BREAKAGE_CUDA_CHECK(cudaMalloc(&ptr_, n * sizeof(T)));
      n_ = n;
    }
  }

  void free() {
    if (ptr_ != nullptr) {
      cudaFree(ptr_);
      ptr_ = nullptr;
      n_ = 0;
    }
  }

  void copy_from_host(const T *host, std::size_t n) {
    if (n != n_)
      resize(n);
    if (n_ == 0)
      return;
    BREAKAGE_CUDA_CHECK(
        cudaMemcpy(ptr_, host, n_ * sizeof(T), cudaMemcpyHostToDevice));
  }

  void copy_to_host(T *host, std::size_t n) const {
    if (n != n_)
      throw std::invalid_argument("DeviceBuffer::copy_to_host size mismatch");
    if (n_ == 0)
      return;
    BREAKAGE_CUDA_CHECK(
        cudaMemcpy(host, ptr_, n_ * sizeof(T), cudaMemcpyDeviceToHost));
  }

  void zero() {
    if (n_ == 0)
      return;
    BREAKAGE_CUDA_CHECK(cudaMemset(ptr_, 0, n_ * sizeof(T)));
  }

  T *data() { return ptr_; }
  const T *data() const { return ptr_; }
  std::size_t size() const { return n_; }
  bool empty() const { return n_ == 0; }

private:
  T *ptr_{nullptr};
  std::size_t n_{0};
};

// ---------------------------------------------------------------------------
// Device CSR matrix (owns indptr/indices/data buffers).
// ---------------------------------------------------------------------------

struct DeviceCsr {
  int rows{0};
  int cols{0};
  int nnz{0};
  DeviceBuffer<int> rowptr;   // (rows + 1)
  DeviceBuffer<int> colind;   // (nnz)
  DeviceBuffer<double> values; // (nnz)

  // Lazy cuSPARSE descriptor (created on first use).
  cusparseSpMatDescr_t descr{nullptr};

  ~DeviceCsr() {
    if (descr != nullptr) {
      cusparseDestroySpMat(descr);
    }
  }

  DeviceCsr() = default;
  DeviceCsr(const DeviceCsr &) = delete;
  DeviceCsr &operator=(const DeviceCsr &) = delete;
  DeviceCsr(DeviceCsr &&) = default;
  DeviceCsr &operator=(DeviceCsr &&) = default;
};

// Uploads an Eigen CSC matrix as CSR on the device. Performs the
// CSC->CSR transpose on the host (it is small enough for the matrices we
// have here; KKT factorizations dominate the runtime budget).
void upload_csc_as_csr(const SparseMatrixCSC &src, DeviceCsr &dst);

// ---------------------------------------------------------------------------
// Device system: GPU-resident view of HostSystem.
//
// All matrices are CSR. Dense per-part data is stored row-major.
// ---------------------------------------------------------------------------

struct DeviceSystem {
  int N{0};   // num_parts
  int K{0};   // num_clutches
  int ncv{0}; // num_contact_vertices
  int nx{0};  // num_vars
  int me{0};  // num_eq = 6 * N
  int mi{0};  // num_ineq
  int mh{0};  // num_relaxed_ineq
  int ncap{0}; // capacity_clutch_indices.size()

  double total_mass{0.0};
  double L0{1.0};

  DeviceBuffer<double> mass;          // (N,)
  DeviceBuffer<double> q_CC;          // (N, 4) row-major xyzw
  DeviceBuffer<double> c_CC;          // (N, 3) row-major
  DeviceBuffer<double> I_CC;          // (N, 9) row-major flatten

  DeviceCsr Q;
  DeviceCsr A;
  DeviceCsr G;
  DeviceCsr H;
  DeviceCsr V;

  DeviceBuffer<int> capacity_clutch_indices; // (ncap,)
  DeviceBuffer<double> capacities;            // (ncap, 9) row-major
  DeviceBuffer<double> clutch_whiten;         // (K, 4) row-major
};

// Uploads HostSystem to DeviceSystem.
void upload_system(const HostSystem &host, DeviceSystem &dev);

// ---------------------------------------------------------------------------
// Per-step input mirror.
// ---------------------------------------------------------------------------

struct DeviceInput {
  double dt{};
  DeviceBuffer<double> w;  // (N, 3)
  DeviceBuffer<double> v;  // (N, 3)
  DeviceBuffer<double> q;  // (N, 4)
  DeviceBuffer<double> c;  // (N, 3)
  DeviceBuffer<double> J;  // (N, 3)
  DeviceBuffer<double> H;  // (N, 3)
};

void upload_input(const HostInput &host, DeviceInput &dev);

// ---------------------------------------------------------------------------
// Per-step state mirror (the part the dense kernels need).
// ---------------------------------------------------------------------------

struct DeviceState {
  DeviceBuffer<double> q_W_CC_prev; // (4,) xyzw
  DeviceBuffer<double> v_W_prev;    // (N, 3)
  DeviceBuffer<double> L_prev;      // (N, 3)
};

void upload_state(const HostState &host, DeviceState &dev, int N);
void download_state(const DeviceState &dev, HostState &host, int N);

} // namespace breakage_cuda
