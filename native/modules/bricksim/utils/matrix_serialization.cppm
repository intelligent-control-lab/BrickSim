// Use bricksim.utils.matrix_loader Python module to load serialized matrices.

export module bricksim.utils.matrix_serialization;

import std;
import bricksim.utils.base64;
import bricksim.vendor;

namespace bricksim {

// ==== Serialization ====

template <class T> struct is_std_complex : std::false_type {};
template <class T> struct is_std_complex<std::complex<T>> : std::true_type {};
template <class T>
constexpr bool is_std_complex_v = is_std_complex<std::remove_cv_t<T>>::value;

template <class T> consteval char numpy_kind() {
	using U = std::remove_cv_t<T>;
	if constexpr (is_std_complex_v<U>) {
		return 'c';
	} else if constexpr (std::is_same_v<U, bool>) {
		return 'b';
	} else if constexpr (std::is_floating_point_v<U>) {
		return 'f';
	} else if constexpr (std::is_integral_v<U> && std::is_signed_v<U>) {
		return 'i';
	} else if constexpr (std::is_integral_v<U> && std::is_unsigned_v<U>) {
		return 'u';
	} else {
		static_assert(false, "Unsupported scalar type for NumPy dtype");
	}
}

template <class T> consteval std::size_t numpy_itemsize() {
	using U = std::remove_cv_t<T>;
	if constexpr (is_std_complex_v<U>) {
		using R = typename U::value_type;
		return 2 * sizeof(R);
	}
	return sizeof(U);
}

constexpr char numpy_endian_prefix(std::size_t itemsize) {
	// NumPy uses '|' for byte-order independent (1-byte) types.
	if (itemsize <= 1)
		return '|';
	return (std::endian::native == std::endian::little) ? '<' : '>';
}

template <class T> std::string numpy_dtype() {
	constexpr char k = numpy_kind<T>();
	constexpr std::size_t sz = numpy_itemsize<T>();

	std::string s;
	s.reserve(4);
	s.push_back(numpy_endian_prefix(sz));
	s.push_back(k);
	s += std::to_string(sz);
	return s;
}

template <class T> consteval void assert_numpy_raw_serializable() {
	static_assert(
	    std::is_trivially_copyable_v<T>,
	    "Scalar/Index must be trivially copyable for raw-byte serialization");

	using U = std::remove_cv_t<T>;
	if constexpr (is_std_complex_v<U>) {
		using R = typename U::value_type;
		static_assert(std::is_floating_point_v<R>,
		              "NumPy complex dtypes assume floating-point components");
		static_assert(
		    sizeof(U) == 2 * sizeof(R),
		    "std::complex layout not 2x real; cannot NumPy-serialize safely");
	}
}

template <class D>
concept DenseStrideQueryable = requires(const D &A) {
	typename D::Scalar;
	{ A.data() } -> std::convertible_to<const typename D::Scalar *>;
	{ A.innerStride() } -> std::convertible_to<Eigen::Index>;
	{ A.outerStride() } -> std::convertible_to<Eigen::Index>;
};

// One function name, overloaded for dense vs sparse.
//
// Dense JSON:
//  {
//    "kind":"dense", "dtype":"<f8", "order":"C"|"F", "shape":[r,c], "data_b64":"..."
//  }
//
// Sparse JSON (CSR/CSC depending on major):
//  {
//    "kind":"sparse", "format":"csr"|"csc", "shape":[r,c], "nnz":...,
//    "data_dtype":"...", "index_dtype":"...",
//    "data_b64":"...", "indices_b64":"...", "indptr_b64":"..."
//  }

// -------- dense (RowMajor => order "C", ColMajor => order "F") --------
export template <class Derived>
nlohmann::ordered_json matrix_to_json(const Eigen::MatrixBase<Derived> &M) {
	using Scalar = typename Derived::Scalar;
	assert_numpy_raw_serializable<Scalar>();

	nlohmann::ordered_json j;
	j["kind"] = "dense";
	j["dtype"] = numpy_dtype<Scalar>();
	j["order"] = (Derived::IsRowMajor ? "C" : "F");
	const auto &A = M.derived();
	j["shape"] = {A.rows(), A.cols()};

	// If we can view it as one contiguous buffer in the claimed order, no copy.
	if constexpr (DenseStrideQueryable<Derived>) {
		bool contiguous =
		    (A.innerStride() == 1) &&
		    (A.outerStride() == (Derived::IsRowMajor ? A.cols() : A.rows()));
		if (contiguous) {
			j["data_b64"] = base64::encode(std::as_bytes(
			    std::span{A.data(), static_cast<std::size_t>(A.size())}));
			return j;
		}
	}

	// Otherwise, make a copy into a contiguous buffer in the desired order.
	using MatrixType = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic,
	                                 (Derived::IsRowMajor ? Eigen::RowMajor
	                                                      : Eigen::ColMajor)>;
	MatrixType X = M.derived();
	j["data_b64"] = base64::encode(
	    std::as_bytes(std::span{X.data(), static_cast<std::size_t>(X.size())}));
	return j;
}

template <class S>
concept SparseCompressedPtrAccess = requires(const S &A) {
	typename S::Scalar;
	typename S::StorageIndex;
	{ A.isCompressed() } -> std::convertible_to<bool>;
	{ A.valuePtr() } -> std::convertible_to<const typename S::Scalar *>;
	{
		A.innerIndexPtr()
	} -> std::convertible_to<const typename S::StorageIndex *>;
	{
		A.outerIndexPtr()
	} -> std::convertible_to<const typename S::StorageIndex *>;
	{ A.outerSize() } -> std::convertible_to<Eigen::Index>;
};

// -------- sparse (RowMajor => CSR, ColMajor => CSC), must already be compressed --------
export template <SparseCompressedPtrAccess Derived>
nlohmann::ordered_json
matrix_to_json(const Eigen::SparseMatrixBase<Derived> &A0) {
	using Scalar = typename Derived::Scalar;
	using StorageIndex = typename Derived::StorageIndex;

	assert_numpy_raw_serializable<Scalar>();
	assert_numpy_raw_serializable<StorageIndex>();
	static_assert(std::is_integral_v<StorageIndex>,
	              "Sparse StorageIndex must be integral");

	const Derived &A = A0.derived();
	if (!A.isCompressed()) {
		throw std::invalid_argument{
		    "matrix_to_json(sparse): input must be compressed"};
	}

	std::size_t rows = static_cast<std::size_t>(A.rows());
	std::size_t cols = static_cast<std::size_t>(A.cols());
	std::size_t nnz = static_cast<std::size_t>(A.nonZeros());
	std::size_t outer = static_cast<std::size_t>(A.outerSize());

	// Basic sanity: outerSize matches major dimension.
	if constexpr (Derived::IsRowMajor) {
		if (outer != rows) {
			throw std::invalid_argument{
			    "matrix_to_json(sparse): RowMajor but outerSize != rows"};
		}
	} else {
		if (outer != cols) {
			throw std::invalid_argument{
			    "matrix_to_json(sparse): ColMajor but outerSize != cols"};
		}
	}

	nlohmann::ordered_json j;
	j["kind"] = "sparse";
	j["format"] = (Derived::IsRowMajor ? "csr" : "csc");
	j["shape"] = {rows, cols};
	j["nnz"] = nnz;
	j["data_dtype"] = numpy_dtype<Scalar>();
	j["index_dtype"] = numpy_dtype<StorageIndex>();
	j["data_b64"] = base64::encode(std::as_bytes(std::span{A.valuePtr(), nnz}));
	j["indices_b64"] =
	    base64::encode(std::as_bytes(std::span{A.innerIndexPtr(), nnz}));
	j["indptr_b64"] =
	    base64::encode(std::as_bytes(std::span{A.outerIndexPtr(), outer + 1}));

	return j;
}

// ==== Deserialization ====

struct NumpyDtypeParsed {
	char endian{}; // '<' '>' '|' '='
	char kind{};   // 'f' 'i' 'u' 'b' 'c'
	std::size_t itemsize{};
};

NumpyDtypeParsed parse_numpy_dtype(std::string_view s) {
	if (s.size() < 3) {
		throw std::invalid_argument{"json_to_matrix: dtype too short"};
	}
	NumpyDtypeParsed dt;
	dt.endian = s[0];
	dt.kind = s[1];
	std::size_t isz = 0;
	auto *p = s.data() + 2;
	auto *e = s.data() + s.size();
	auto [ptr, ec] = std::from_chars(p, e, isz);
	if (ec != std::errc{} || ptr != e || isz == 0) {
		throw std::invalid_argument{"json_to_matrix: invalid dtype itemsize"};
	}
	dt.itemsize = isz;
	return dt;
}

constexpr bool is_little_endian_prefix(char c) {
	return (c == '<');
}
constexpr bool is_big_endian_prefix(char c) {
	return (c == '>');
}
constexpr bool is_native_endian_prefix(char c) {
	return (c == '=');
}
constexpr bool is_na_endian_prefix(char c) {
	return (c == '|'); // byte-order independent / not applicable
}

template <class T>
bool dtype_needs_byteswap(std::string_view dtype, std::string_view field_name) {
	constexpr char exp_kind = numpy_kind<T>();
	constexpr std::size_t exp_itemsize = numpy_itemsize<T>();
	auto dt = parse_numpy_dtype(dtype);
	if (dt.kind != exp_kind || dt.itemsize != exp_itemsize) {
		throw std::invalid_argument{std::format(
		    "json_to_matrix: dtype mismatch in {} (got '{}', expected '{}')",
		    field_name, dtype, numpy_dtype<T>())};
	}
	if (exp_itemsize <= 1)
		return false;
	// '=' means native endian in NumPy dtypes.
	if (is_native_endian_prefix(dt.endian))
		return false;
	// For robustness, allow '|' but treat it as no-swap (shouldn't occur for >1B).
	if (is_na_endian_prefix(dt.endian))
		return false;
	const bool native_little = (std::endian::native == std::endian::little);
	if (is_little_endian_prefix(dt.endian))
		return !native_little;
	if (is_big_endian_prefix(dt.endian))
		return native_little;
	throw std::invalid_argument{
	    std::format("json_to_matrix: invalid endian prefix '{}' in {}",
	                dt.endian, field_name)};
}

template <class T> void byteswap_inplace(std::span<T> xs) {
	using U = std::remove_cv_t<T>;
	if constexpr (sizeof(U) <= 1) {
		return;
	} else if constexpr (is_std_complex_v<U>) {
		// Reverse bytes *within* each real/imag component; do NOT swap components.
		using R = typename U::value_type;
		static_assert(
		    sizeof(U) == 2 * sizeof(R),
		    "std::complex layout not 2x real; cannot byteswap safely");
		constexpr std::size_t half = sizeof(R);

		for (auto &z : xs) {
			auto b = std::as_writable_bytes(std::span<U, 1>{&z, 1});
			std::ranges::reverse(b.first(half));
			std::ranges::reverse(b.subspan(half, half));
		}
	} else {
		for (auto &x : xs) {
			auto b = std::as_writable_bytes(std::span<U, 1>{&x, 1});
			std::ranges::reverse(b);
		}
	}
}

std::tuple<Eigen::Index, Eigen::Index>
parse_shape_2d(const nlohmann::ordered_json &j) {
	const auto &shape = j.at("shape");
	if (!shape.is_array() || shape.size() != 2) {
		throw std::invalid_argument{
		    "json_to_matrix: shape must be [rows, cols]"};
	}
	std::size_t r = shape.at(0).template get<std::size_t>();
	std::size_t c = shape.at(1).template get<std::size_t>();
	return {static_cast<Eigen::Index>(r), static_cast<Eigen::Index>(c)};
}

template <class T>
concept DenseMatrixObject = std::is_base_of_v<Eigen::MatrixBase<T>, T>;

template <class T>
concept SparseMatrixObject = std::is_base_of_v<Eigen::SparseMatrixBase<T>, T>;

template <class T>
void decode_b64_into(std::string_view b64, std::span<T> out,
                     std::string_view dtype, std::string_view field_name) {
	std::size_t need = out.size_bytes();
	std::size_t got = base64::decoded_size(b64);
	if (got != need) {
		throw std::invalid_argument{std::format(
		    "json_to_matrix: decoded size mismatch in {} (got {}, expected {})",
		    field_name, got, need)};
	}

	(void)base64::decode_into(b64, std::as_writable_bytes(out));

	bool swap = dtype_needs_byteswap<T>(dtype, field_name);
	if (swap) {
		byteswap_inplace(out);
	}
}

template <class DenseType>
void validate_dense_shape(Eigen::Index rows, Eigen::Index cols) {
	if constexpr (DenseType::RowsAtCompileTime != Eigen::Dynamic) {
		if (rows != DenseType::RowsAtCompileTime) {
			throw std::invalid_argument{std::format(
			    "json_to_matrix(dense): rows mismatch (got {}, expected {})",
			    rows, static_cast<long>(DenseType::RowsAtCompileTime))};
		}
	} else if constexpr (DenseType::MaxRowsAtCompileTime != Eigen::Dynamic) {
		if (rows > DenseType::MaxRowsAtCompileTime) {
			throw std::invalid_argument{std::format(
			    "json_to_matrix(dense): rows exceed MaxRowsAtCompileTime (got "
			    "{}, max {})",
			    rows, static_cast<long>(DenseType::MaxRowsAtCompileTime))};
		}
	}

	if constexpr (DenseType::ColsAtCompileTime != Eigen::Dynamic) {
		if (cols != DenseType::ColsAtCompileTime) {
			throw std::invalid_argument{std::format(
			    "json_to_matrix(dense): cols mismatch (got {}, expected {})",
			    cols, static_cast<long>(DenseType::ColsAtCompileTime))};
		}
	} else if constexpr (DenseType::MaxColsAtCompileTime != Eigen::Dynamic) {
		if (cols > DenseType::MaxColsAtCompileTime) {
			throw std::invalid_argument{std::format(
			    "json_to_matrix(dense): cols exceed MaxColsAtCompileTime (got "
			    "{}, max {})",
			    cols, static_cast<long>(DenseType::MaxColsAtCompileTime))};
		}
	}
}

export template <DenseMatrixObject EigenType>
EigenType json_to_matrix(const nlohmann::ordered_json &j) {
	using Scalar = typename EigenType::Scalar;
	assert_numpy_raw_serializable<Scalar>();

	const auto &kind = j.at("kind").get_ref<const std::string &>();
	if (kind != "dense") {
		throw std::invalid_argument{std::format(
		    "json_to_matrix(dense): expected kind 'dense', got '{}'", kind)};
	}

	auto [rows, cols] = parse_shape_2d(j);
	validate_dense_shape<EigenType>(rows, cols);

	const auto &order = j.at("order").get_ref<const std::string &>();
	bool input_row_major = false;
	if (order == "C") {
		input_row_major = true;
	} else if (order == "F") {
		input_row_major = false;
	} else {
		throw std::invalid_argument{std::format(
		    "json_to_matrix(dense): invalid order '{}' (expected 'C' or 'F')",
		    order)};
	}

	const auto &dtype = j.at("dtype").get_ref<const std::string &>();
	const auto &b64 = j.at("data_b64").get_ref<const std::string &>();

	EigenType out;
	if constexpr (EigenType::RowsAtCompileTime == Eigen::Dynamic ||
	              EigenType::ColsAtCompileTime == Eigen::Dynamic) {
		out.resize(rows, cols);
	}

	bool out_row_major = static_cast<bool>(EigenType::IsRowMajor);
	if (input_row_major == out_row_major) {
		decode_b64_into<Scalar>(
		    b64,
		    std::span<Scalar>{out.data(), static_cast<std::size_t>(out.size())},
		    dtype, "dtype");
		return out;
	}

	if (input_row_major) {
		using RawEigenType = Eigen::Matrix<Scalar, Eigen::Dynamic,
		                                   Eigen::Dynamic, Eigen::RowMajor>;
		RawEigenType tmp(rows, cols);
		decode_b64_into<Scalar>(
		    b64,
		    std::span<Scalar>{tmp.data(), static_cast<std::size_t>(tmp.size())},
		    dtype, "dtype");
		out = tmp;
		return out;
	} else {
		using RawEigenType = Eigen::Matrix<Scalar, Eigen::Dynamic,
		                                   Eigen::Dynamic, Eigen::ColMajor>;
		RawEigenType tmp(rows, cols);
		decode_b64_into<Scalar>(
		    b64,
		    std::span<Scalar>{tmp.data(), static_cast<std::size_t>(tmp.size())},
		    dtype, "dtype");
		out = tmp;
		return out;
	}
}

export template <SparseMatrixObject EigenType>
EigenType json_to_matrix(const nlohmann::ordered_json &j) {
	using Scalar = typename EigenType::Scalar;
	using StorageIndex = typename EigenType::StorageIndex;

	assert_numpy_raw_serializable<Scalar>();
	assert_numpy_raw_serializable<StorageIndex>();
	static_assert(std::is_integral_v<StorageIndex>,
	              "Sparse StorageIndex must be integral");

	const auto &kind = j.at("kind").get_ref<const std::string &>();
	if (kind != "sparse") {
		throw std::invalid_argument{std::format(
		    "json_to_matrix(sparse): expected kind 'sparse', got '{}'", kind)};
	}

	auto [rows, cols] = parse_shape_2d(j);

	std::size_t nnz = j.at("nnz").template get<std::size_t>();

	const auto &format = j.at("format").get_ref<const std::string &>();
	bool is_csr = (format == "csr");
	bool is_csc = (format == "csc");
	if (!is_csr && !is_csc) {
		throw std::invalid_argument{
		    std::format("json_to_matrix(sparse): invalid format '{}' (expected "
		                "'csr' or 'csc')",
		                format)};
	}

	std::size_t outer = is_csr ? static_cast<std::size_t>(rows)
	                           : static_cast<std::size_t>(cols);
	std::size_t inner = is_csr ? static_cast<std::size_t>(cols)
	                           : static_cast<std::size_t>(rows);

	const auto &data_dtype = j.at("data_dtype").get_ref<const std::string &>();
	const auto &index_dtype =
	    j.at("index_dtype").get_ref<const std::string &>();
	const auto &data_b64 = j.at("data_b64").get_ref<const std::string &>();
	const auto &indices_b64 =
	    j.at("indices_b64").get_ref<const std::string &>();
	const auto &indptr_b64 = j.at("indptr_b64").get_ref<const std::string &>();

	std::vector<Scalar> values(nnz);
	std::vector<StorageIndex> indices(nnz);
	std::vector<StorageIndex> indptr(outer + 1);

	decode_b64_into<Scalar>(data_b64, std::span<Scalar>{values}, data_dtype,
	                        "data_dtype");
	decode_b64_into<StorageIndex>(indices_b64, std::span<StorageIndex>{indices},
	                              index_dtype, "index_dtype");
	decode_b64_into<StorageIndex>(indptr_b64, std::span<StorageIndex>{indptr},
	                              index_dtype, "index_dtype");

	if (indptr.empty()) {
		throw std::invalid_argument{
		    "json_to_matrix(sparse): indptr must have at least 1 element"};
	}
	if (indptr[0] != StorageIndex{0}) {
		throw std::invalid_argument{
		    "json_to_matrix(sparse): indptr[0] must be 0"};
	}

	StorageIndex prev = indptr[0];
	for (std::size_t i = 1; i < indptr.size(); ++i) {
		StorageIndex cur = indptr[i];

		if constexpr (std::is_signed_v<StorageIndex>) {
			if (cur < 0) {
				throw std::invalid_argument{std::format(
				    "json_to_matrix(sparse): indptr[{}] is negative", i)};
			}
		}
		if (cur < prev) {
			throw std::invalid_argument{
			    std::format("json_to_matrix(sparse): indptr must be "
			                "non-decreasing (indptr[{}]={} < indptr[{}]={})",
			                i, cur, i - 1, prev)};
		}
		if (static_cast<std::size_t>(cur) > nnz) {
			throw std::invalid_argument{std::format(
			    "json_to_matrix(sparse): indptr[{}]={} exceeds nnz={}", i, cur,
			    nnz)};
		}
		prev = cur;
	}

	StorageIndex last = indptr.back();
	if constexpr (std::is_signed_v<StorageIndex>) {
		if (last < 0) {
			throw std::invalid_argument{
			    "json_to_matrix(sparse): indptr.back() is negative"};
		}
	}
	if (static_cast<std::size_t>(last) != nnz) {
		throw std::invalid_argument{
		    std::format("json_to_matrix(sparse): indptr.back() mismatch (got "
		                "{}, expected nnz={})",
		                last, nnz)};
	}

	for (std::size_t k = 0; k < indices.size(); ++k) {
		StorageIndex idx = indices[k];
		if constexpr (std::is_signed_v<StorageIndex>) {
			if (idx < 0) {
				throw std::invalid_argument{std::format(
				    "json_to_matrix(sparse): indices[{}] is negative", k)};
			}
		}
		if (static_cast<std::size_t>(idx) >= inner) {
			throw std::invalid_argument{
			    std::format("json_to_matrix(sparse): indices[{}]={} out of "
			                "range (inner={})",
			                k, idx, inner)};
		}
	}

	if (is_csr) {
		using MapType =
		    Eigen::SparseMatrix<Scalar, Eigen::RowMajor, StorageIndex>;
		Eigen::Map<MapType> mapped(rows, cols, static_cast<Eigen::Index>(nnz),
		                           indptr.data(), indices.data(),
		                           values.data());
		EigenType out = mapped;
		return out;
	} else {
		using MapType =
		    Eigen::SparseMatrix<Scalar, Eigen::ColMajor, StorageIndex>;
		Eigen::Map<MapType> mapped(rows, cols, static_cast<Eigen::Index>(nnz),
		                           indptr.data(), indices.data(),
		                           values.data());
		EigenType out = mapped;
		return out;
	}
}

} // namespace bricksim
