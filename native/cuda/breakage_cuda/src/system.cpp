// SPDX-License-Identifier: MIT
//
// JSON loaders + host-to-device transfers for the breakage_cuda subproject.
//
// JSON layout matches bricksim::matrix_to_json (see
// modules/bricksim/utils/matrix_serialization.cppm) and the to_json overloads
// for BreakageDebugDump in modules/bricksim/physx/breakage.cppm.
//
// On the device side, all dense per-part data is stored row-major and all
// sparse matrices are CSR (cuSPARSE-friendly).

#include "breakage_cuda/device.hpp"
#include "breakage_cuda/system.hpp"

#include <Eigen/Geometry>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

namespace breakage_cuda {

// ===========================================================================
// JSON helpers (matrix_serialization-compatible).
//
//   dense  -> {"kind":"dense", "dtype","order"("C"|"F"),"shape":[r,c],
//              "data_b64"}
//   sparse -> {"kind":"sparse","format"("csr"|"csc"),"shape":[r,c],"nnz",
//              "data_dtype","index_dtype",
//              "data_b64","indices_b64","indptr_b64"}
//
// Bytes inside *_b64 are raw little-/big-endian according to the dtype prefix
// ('<','>','|','='). On x86_64 Linux native is little-endian, so the common
// case '<f8' / '<i4' reduces to a memcpy.
// ===========================================================================

namespace {

// ----- minimal base64 decoder (standard alphabet, '=' padding) -------------

constexpr std::array<std::int8_t, 256> make_base64_lut() {
  std::array<std::int8_t, 256> lut{};
  for (std::size_t i = 0; i < 256; ++i) lut[i] = -1;
  std::string_view alpha =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  for (std::size_t i = 0; i < alpha.size(); ++i) {
    lut[static_cast<unsigned char>(alpha[i])] = static_cast<std::int8_t>(i);
  }
  lut[static_cast<unsigned char>('=')] = -2;
  return lut;
}

constexpr auto kBase64Lut = make_base64_lut();

std::vector<std::byte> base64_decode(std::string_view s) {
  if (s.empty()) return {};
  if ((s.size() & 3u) != 0) {
    throw std::runtime_error{"base64: input length not divisible by 4"};
  }
  std::size_t pad = 0;
  if (s.back() == '=') {
    pad = (s.size() >= 2 && s[s.size() - 2] == '=') ? 2 : 1;
  }
  std::vector<std::byte> out((s.size() / 4) * 3 - pad);
  const auto *p = reinterpret_cast<const unsigned char *>(s.data());
  std::byte *o = out.data();
  const std::size_t full = s.size() / 4 - (pad != 0 ? 1 : 0);
  auto get6 = [&]() {
    int v = kBase64Lut[*p++];
    if (v < 0) throw std::runtime_error{"base64: invalid character"};
    return v;
  };
  for (std::size_t i = 0; i < full; ++i) {
    std::uint32_t x = (std::uint32_t(get6()) << 18) |
                      (std::uint32_t(get6()) << 12) |
                      (std::uint32_t(get6()) << 6) | std::uint32_t(get6());
    *o++ = std::byte((x >> 16) & 0xFFu);
    *o++ = std::byte((x >> 8) & 0xFFu);
    *o++ = std::byte(x & 0xFFu);
  }
  if (pad == 1) {
    std::uint32_t x = (std::uint32_t(get6()) << 18) |
                      (std::uint32_t(get6()) << 12) |
                      (std::uint32_t(get6()) << 6);
    *o++ = std::byte((x >> 16) & 0xFFu);
    *o++ = std::byte((x >> 8) & 0xFFu);
  } else if (pad == 2) {
    std::uint32_t x = (std::uint32_t(get6()) << 18) |
                      (std::uint32_t(get6()) << 12);
    *o++ = std::byte((x >> 16) & 0xFFu);
  }
  return out;
}

// ----- numpy dtype validation + per-element byteswap -----------------------

template <class T>
constexpr char numpy_kind() {
  if constexpr (std::is_floating_point_v<T>)
    return 'f';
  else if constexpr (std::is_signed_v<T>)
    return 'i';
  else
    return 'u';
}

template <class T>
bool dtype_needs_swap(std::string_view dtype, std::string_view field) {
  if (dtype.size() < 3) {
    throw std::runtime_error{
        std::string{"dtype too short for "}.append(field)};
  }
  std::size_t isz = 0;
  for (std::size_t i = 2; i < dtype.size(); ++i) {
    char c = dtype[i];
    if (c < '0' || c > '9') {
      throw std::runtime_error{
          std::string{"dtype itemsize parse error in "}.append(field)};
    }
    isz = isz * 10 + static_cast<std::size_t>(c - '0');
  }
  if (dtype[1] != numpy_kind<T>() || isz != sizeof(T)) {
    throw std::runtime_error{std::string{"dtype mismatch for "}
                                 .append(field)
                                 .append(": ")
                                 .append(dtype)};
  }
  if (sizeof(T) <= 1) return false;
  const char endian = dtype[0];
  if (endian == '|' || endian == '=') return false;
  const bool native_le = (std::endian::native == std::endian::little);
  if (endian == '<') return !native_le;
  if (endian == '>') return native_le;
  throw std::runtime_error{
      std::string{"invalid endian prefix in "}.append(field)};
}

template <class T>
void byteswap_inplace(std::span<T> xs) {
  for (auto &x : xs) {
    auto bytes = std::as_writable_bytes(std::span<T, 1>{&x, 1});
    std::ranges::reverse(bytes);
  }
}

template <class T>
void decode_typed(std::string_view b64, std::span<T> out,
                  std::string_view dtype, std::string_view field) {
  const bool swap = dtype_needs_swap<T>(dtype, field);
  std::vector<std::byte> bytes = base64_decode(b64);
  if (bytes.size() != out.size_bytes()) {
    throw std::runtime_error{
        std::string{"decoded byte count mismatch for "}.append(field)};
  }
  if (!bytes.empty()) {
    std::memcpy(out.data(), bytes.data(), bytes.size());
  }
  if (swap) byteswap_inplace(out);
}

// ----- dense matrix loader -------------------------------------------------

template <class EigenType>
EigenType load_dense(const nlohmann::json &j, std::string_view field) {
  using Scalar = typename EigenType::Scalar;
  if (j.at("kind").get_ref<const std::string &>() != "dense") {
    throw std::runtime_error{
        std::string{field}.append(": expected kind 'dense'")};
  }
  const auto &shape = j.at("shape");
  if (!shape.is_array() || shape.size() != 2) {
    throw std::runtime_error{
        std::string{field}.append(": shape must be [rows, cols]")};
  }
  const Eigen::Index rows = shape.at(0).get<Eigen::Index>();
  const Eigen::Index cols = shape.at(1).get<Eigen::Index>();

  const auto &order = j.at("order").get_ref<const std::string &>();
  const bool input_rm = (order == "C");
  if (!input_rm && order != "F") {
    throw std::runtime_error{std::string{field}
                                 .append(": invalid order '")
                                 .append(order)
                                 .append("'")};
  }

  const auto &dtype = j.at("dtype").get_ref<const std::string &>();
  const auto &b64 = j.at("data_b64").get_ref<const std::string &>();

  EigenType out;
  out.resize(rows, cols);
  const bool out_rm = static_cast<bool>(EigenType::IsRowMajor);
  // For vectors (one extent <= 1), storage order is identical either way.
  const bool degenerate = (rows <= 1) || (cols <= 1);

  if (input_rm == out_rm || degenerate) {
    decode_typed<Scalar>(
        b64,
        std::span<Scalar>{out.data(), static_cast<std::size_t>(out.size())},
        dtype, field);
  } else if (input_rm) {
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> tmp(
        rows, cols);
    decode_typed<Scalar>(
        b64,
        std::span<Scalar>{tmp.data(), static_cast<std::size_t>(tmp.size())},
        dtype, field);
    out = tmp;
  } else {
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor> tmp(
        rows, cols);
    decode_typed<Scalar>(
        b64,
        std::span<Scalar>{tmp.data(), static_cast<std::size_t>(tmp.size())},
        dtype, field);
    out = tmp;
  }
  return out;
}

// ----- sparse matrix loader (always returns CSC) ---------------------------

SparseMatrixCSC load_sparse_csc(const nlohmann::json &j,
                                std::string_view field) {
  if (j.at("kind").get_ref<const std::string &>() != "sparse") {
    throw std::runtime_error{
        std::string{field}.append(": expected kind 'sparse'")};
  }
  const auto &shape = j.at("shape");
  if (!shape.is_array() || shape.size() != 2) {
    throw std::runtime_error{
        std::string{field}.append(": shape must be [rows, cols]")};
  }
  const Eigen::Index rows = shape.at(0).get<Eigen::Index>();
  const Eigen::Index cols = shape.at(1).get<Eigen::Index>();
  const std::size_t nnz = j.at("nnz").get<std::size_t>();

  const auto &format = j.at("format").get_ref<const std::string &>();
  const bool is_csc = (format == "csc");
  const bool is_csr = (format == "csr");
  if (!is_csc && !is_csr) {
    throw std::runtime_error{std::string{field}
                                 .append(": invalid sparse format '")
                                 .append(format)
                                 .append("'")};
  }
  const std::size_t outer = is_csc ? static_cast<std::size_t>(cols)
                                   : static_cast<std::size_t>(rows);

  const auto &data_dtype =
      j.at("data_dtype").get_ref<const std::string &>();
  const auto &index_dtype =
      j.at("index_dtype").get_ref<const std::string &>();

  std::vector<double> values(nnz);
  std::vector<int> indices(nnz);
  std::vector<int> indptr(outer + 1);
  decode_typed<double>(j.at("data_b64").get_ref<const std::string &>(),
                       std::span<double>{values}, data_dtype, field);
  decode_typed<int>(j.at("indices_b64").get_ref<const std::string &>(),
                    std::span<int>{indices}, index_dtype, field);
  decode_typed<int>(j.at("indptr_b64").get_ref<const std::string &>(),
                    std::span<int>{indptr}, index_dtype, field);

  if (nnz == 0) {
    SparseMatrixCSC empty(rows, cols);
    empty.makeCompressed();
    return empty;
  }

  if (is_csc) {
    using MapType = Eigen::SparseMatrix<double, Eigen::ColMajor, int>;
    Eigen::Map<MapType> mapped(rows, cols, static_cast<Eigen::Index>(nnz),
                               indptr.data(), indices.data(), values.data());
    return SparseMatrixCSC{mapped};
  }
  // CSR -> re-encode to CSC; Eigen handles the layout change on assignment.
  using MapType = Eigen::SparseMatrix<double, Eigen::RowMajor, int>;
  Eigen::Map<MapType> mapped(rows, cols, static_cast<Eigen::Index>(nnz),
                             indptr.data(), indices.data(), values.data());
  return SparseMatrixCSC{mapped};
}

}  // namespace

// ===========================================================================
// from_json overloads.
// ===========================================================================

void from_json(const nlohmann::json &j, Thresholds &out) {
  j.at("enabled").get_to(out.enabled);
  j.at("contact_regularization").get_to(out.contact_regularization);
  j.at("clutch_axial_compliance").get_to(out.clutch_axial_compliance);
  j.at("clutch_radial_compliance").get_to(out.clutch_radial_compliance);
  j.at("clutch_tangential_compliance")
      .get_to(out.clutch_tangential_compliance);
  j.at("friction_coefficient").get_to(out.friction_coefficient);
  j.at("preloaded_force").get_to(out.preloaded_force);
  j.at("slack_fraction_warn").get_to(out.slack_fraction_warn);
  j.at("slack_fraction_b_floor").get_to(out.slack_fraction_b_floor);
  j.at("debug_dump").get_to(out.debug_dump);
  j.at("breakage_cooldown_time").get_to(out.breakage_cooldown_time);
}

void from_json(const nlohmann::json &j, HostSystem &out) {
  j.at("num_parts").get_to(out.num_parts);
  j.at("num_clutches").get_to(out.num_clutches);
  j.at("num_contact_vertices").get_to(out.num_contact_vertices);
  j.at("num_vars").get_to(out.num_vars);
  j.at("num_eq").get_to(out.num_eq);
  j.at("num_ineq").get_to(out.num_ineq);
  j.at("num_relaxed_ineq").get_to(out.num_relaxed_ineq);
  j.at("total_mass").get_to(out.total_mass);
  j.at("L0").get_to(out.L0);

  out.mass = load_dense<VectorXd>(j.at("mass"), "mass");
  out.q_CC = load_dense<MatrixX4d>(j.at("q_CC"), "q_CC");
  out.c_CC = load_dense<MatrixX3d>(j.at("c_CC"), "c_CC");
  out.I_CC = load_dense<Eigen::Matrix<double, Dynamic, 9, RowMajor>>(
      j.at("I_CC"), "I_CC");

  out.Q = load_sparse_csc(j.at("Q"), "Q");
  out.A = load_sparse_csc(j.at("A"), "A");
  out.G = load_sparse_csc(j.at("G"), "G");
  out.H = load_sparse_csc(j.at("H"), "H");
  out.V = load_sparse_csc(j.at("V"), "V");

  j.at("capacity_clutch_indices").get_to(out.capacity_clutch_indices);
  out.capacities = load_dense<Eigen::Matrix<double, Dynamic, 9, RowMajor>>(
      j.at("capacities"), "capacities");
  out.clutch_whiten = load_dense<Eigen::Matrix<double, Dynamic, 4, RowMajor>>(
      j.at("clutch_whiten"), "clutch_whiten");
  // part_ids / clutch_ids are intentionally ignored; GPU only needs indices.
}

void from_json(const nlohmann::json &j, HostInput &out) {
  out.w = load_dense<MatrixX3d>(j.at("w"), "w");
  out.v = load_dense<MatrixX3d>(j.at("v"), "v");
  out.q = load_dense<MatrixX4d>(j.at("q"), "q");
  out.c = load_dense<MatrixX3d>(j.at("c"), "c");
  j.at("dt").get_to(out.dt);
  out.J = load_dense<MatrixX3d>(j.at("J"), "J");
  out.H = load_dense<MatrixX3d>(j.at("H"), "H");
}

void from_json(const nlohmann::json &j, HostState &out) {
  out.q_W_CC_prev.coeffs() =
      load_dense<Vector4d>(j.at("q_W_CC_prev"), "q_W_CC_prev");
  out.v_W_prev = load_dense<MatrixX3d>(j.at("v_W_prev"), "v_W_prev");
  out.L_prev = load_dense<MatrixX3d>(j.at("L_prev"), "L_prev");
  // The JSON solver_state is backend-specific (OSQP); each backend manages
  // its own warm start, so we drop it.
  out.qp.reset();
}

void from_json(const nlohmann::json &j, DebugDump &out) {
  j.at("thresholds").get_to(out.thresholds);
  j.at("system").get_to(out.system);
  j.at("input").get_to(out.input);
  j.at("state").get_to(out.state);

  // Solution: only x / utilization / slack_fraction; OsqpInfo is backend-
  // specific and dropped.
  const auto &sol = j.at("solution");
  out.solution.x = load_dense<VectorXd>(sol.at("x"), "solution.x");
  out.solution.utilization =
      load_dense<VectorXd>(sol.at("utilization"), "solution.utilization");
  sol.at("slack_fraction").get_to(out.solution.slack_fraction);
  out.solution.info = SolveInfo{};

  out.b = load_dense<VectorXd>(j.at("b"), "b");

  if (j.contains("prev_state")) {
    HostState ps;
    from_json(j.at("prev_state"), ps);
    out.prev_state = std::move(ps);
  } else {
    out.prev_state.reset();
  }
}

DebugDump load_debug_dump(const std::string &path) {
  std::ifstream stream(path);
  if (!stream.is_open()) {
    throw std::runtime_error{"failed to open debug dump file: " + path};
  }
  nlohmann::json j;
  try {
    stream >> j;
  } catch (const std::exception &e) {
    throw std::runtime_error{"failed to parse JSON in debug dump '" + path +
                             "': " + e.what()};
  }
  DebugDump dump;
  from_json(j, dump);
  return dump;
}

double relative_inf_error(const VectorXd &a, const VectorXd &b, double floor) {
  if (a.size() != b.size()) {
    throw std::invalid_argument{
        "relative_inf_error: vector size mismatch"};
  }
  if (a.size() == 0) return 0.0;
  const double denom = std::max(b.cwiseAbs().maxCoeff(), floor);
  return (a - b).cwiseAbs().maxCoeff() / denom;
}

// ===========================================================================
// Device upload helpers.
// ===========================================================================

void upload_csc_as_csr(const SparseMatrixCSC &src, DeviceCsr &dst) {
  // The descriptor caches raw device pointers; tear it down before any
  // potential buffer reallocation in copy_from_host.
  if (dst.descr != nullptr) {
    cusparseDestroySpMat(dst.descr);
    dst.descr = nullptr;
  }

  dst.rows = static_cast<int>(src.rows());
  dst.cols = static_cast<int>(src.cols());
  dst.nnz = static_cast<int>(src.nonZeros());

  // Eigen does the CSC -> CSR re-encoding on assignment.
  SparseMatrixCSR tmp = src;
  tmp.makeCompressed();

  dst.rowptr.copy_from_host(tmp.outerIndexPtr(),
                            static_cast<std::size_t>(dst.rows) + 1);
  dst.colind.copy_from_host(tmp.innerIndexPtr(),
                            static_cast<std::size_t>(dst.nnz));
  dst.values.copy_from_host(tmp.valuePtr(),
                            static_cast<std::size_t>(dst.nnz));

  // cuSPARSE allows null colind/values when nnz == 0.
  BREAKAGE_CUSPARSE_CHECK(cusparseCreateCsr(
      &dst.descr, static_cast<std::int64_t>(dst.rows),
      static_cast<std::int64_t>(dst.cols),
      static_cast<std::int64_t>(dst.nnz), dst.rowptr.data(), dst.colind.data(),
      dst.values.data(), CUSPARSE_INDEX_32I, CUSPARSE_INDEX_32I,
      CUSPARSE_INDEX_BASE_ZERO, CUDA_R_64F));
}

void upload_system(const HostSystem &host, DeviceSystem &dev) {
  dev.N = host.num_parts;
  dev.K = host.num_clutches;
  dev.ncv = host.num_contact_vertices;
  dev.nx = host.num_vars;
  dev.me = host.num_eq;
  dev.mi = host.num_ineq;
  dev.mh = host.num_relaxed_ineq;
  dev.ncap = static_cast<int>(host.capacity_clutch_indices.size());
  dev.total_mass = host.total_mass;
  dev.L0 = host.L0;

  dev.mass.copy_from_host(host.mass.data(),
                          static_cast<std::size_t>(host.mass.size()));

  // Device convention: row-major dense per-part data. q_CC / c_CC are
  // ColMajor on the host so we transcode here.
  Eigen::Matrix<double, Dynamic, 4, RowMajor> q_rm = host.q_CC;
  Eigen::Matrix<double, Dynamic, 3, RowMajor> c_rm = host.c_CC;
  dev.q_CC.copy_from_host(q_rm.data(), static_cast<std::size_t>(q_rm.size()));
  dev.c_CC.copy_from_host(c_rm.data(), static_cast<std::size_t>(c_rm.size()));
  // I_CC / capacities / clutch_whiten are already RowMajor on the host.
  dev.I_CC.copy_from_host(host.I_CC.data(),
                          static_cast<std::size_t>(host.I_CC.size()));

  upload_csc_as_csr(host.Q, dev.Q);
  upload_csc_as_csr(host.A, dev.A);
  upload_csc_as_csr(host.G, dev.G);
  upload_csc_as_csr(host.H, dev.H);
  upload_csc_as_csr(host.V, dev.V);

  dev.capacity_clutch_indices.copy_from_host(
      host.capacity_clutch_indices.data(),
      host.capacity_clutch_indices.size());
  dev.capacities.copy_from_host(
      host.capacities.data(),
      static_cast<std::size_t>(host.capacities.size()));
  dev.clutch_whiten.copy_from_host(
      host.clutch_whiten.data(),
      static_cast<std::size_t>(host.clutch_whiten.size()));
}

void upload_input(const HostInput &host, DeviceInput &dev) {
  dev.dt = host.dt;

  auto upload_rm3 = [](const MatrixX3d &src, DeviceBuffer<double> &dst_buf) {
    Eigen::Matrix<double, Dynamic, 3, RowMajor> rm = src;
    dst_buf.copy_from_host(rm.data(), static_cast<std::size_t>(rm.size()));
  };
  auto upload_rm4 = [](const MatrixX4d &src, DeviceBuffer<double> &dst_buf) {
    Eigen::Matrix<double, Dynamic, 4, RowMajor> rm = src;
    dst_buf.copy_from_host(rm.data(), static_cast<std::size_t>(rm.size()));
  };

  upload_rm3(host.w, dev.w);
  upload_rm3(host.v, dev.v);
  upload_rm4(host.q, dev.q);
  upload_rm3(host.c, dev.c);
  upload_rm3(host.J, dev.J);
  upload_rm3(host.H, dev.H);
}

void upload_state(const HostState &host, DeviceState &dev, int N) {
  Vector4d quat = host.q_W_CC_prev.coeffs();
  dev.q_W_CC_prev.copy_from_host(quat.data(), 4);

  if (host.v_W_prev.rows() != N || host.L_prev.rows() != N) {
    throw std::invalid_argument{
        "upload_state: v_W_prev / L_prev row count does not match N"};
  }
  Eigen::Matrix<double, Dynamic, 3, RowMajor> v_rm = host.v_W_prev;
  Eigen::Matrix<double, Dynamic, 3, RowMajor> L_rm = host.L_prev;
  const std::size_t n3 =
      static_cast<std::size_t>(3) * static_cast<std::size_t>(N);
  dev.v_W_prev.copy_from_host(v_rm.data(), n3);
  dev.L_prev.copy_from_host(L_rm.data(), n3);
}

void download_state(const DeviceState &dev, HostState &host, int N) {
  Vector4d quat;
  dev.q_W_CC_prev.copy_to_host(quat.data(), 4);
  host.q_W_CC_prev.coeffs() = quat;

  const std::size_t n3 =
      static_cast<std::size_t>(3) * static_cast<std::size_t>(N);
  Eigen::Matrix<double, Dynamic, 3, RowMajor> v_rm(N, 3);
  Eigen::Matrix<double, Dynamic, 3, RowMajor> L_rm(N, 3);
  dev.v_W_prev.copy_to_host(v_rm.data(), n3);
  dev.L_prev.copy_to_host(L_rm.data(), n3);
  host.v_W_prev = v_rm;
  host.L_prev = L_rm;
}

}  // namespace breakage_cuda
