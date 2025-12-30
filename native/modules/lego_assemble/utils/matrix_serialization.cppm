// Use lego_assemble.utils.matrix_loader Python module to load serialized matrices.

export module lego_assemble.utils.matrix_serialization;

import std;
import lego_assemble.utils.base64;
import lego_assemble.vendor;

namespace lego_assemble {

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
	{ A.valuePtr() } -> std::same_as<const typename S::Scalar *>;
	{ A.innerIndexPtr() } -> std::same_as<const typename S::StorageIndex *>;
	{ A.outerIndexPtr() } -> std::same_as<const typename S::StorageIndex *>;
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

} // namespace lego_assemble
