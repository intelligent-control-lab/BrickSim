export module lego_assemble.utils.eigen_format;

import std;
import lego_assemble.vendor;

namespace std {

export template <class Scalar, int Rows, int Cols, int Options, int MaxRows,
                 int MaxCols>
struct formatter<Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>,
                 char> {
	std::formatter<Scalar, char> scalar_fmt;

	constexpr auto parse(std::format_parse_context &ctx) {
		return scalar_fmt.parse(ctx);
	}

	template <class FormatContext>
	auto format(
	    const Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols> &m,
	    FormatContext &ctx) const {
		auto out = ctx.out();

		*out++ = '[';
		for (Eigen::Index r = 0; r < m.rows(); ++r) {
			if (r != 0) {
				*out++ = ';';
				*out++ = ' ';
			}
			for (Eigen::Index c = 0; c < m.cols(); ++c) {
				if (c != 0) {
					*out++ = ',';
					*out++ = ' ';
				}
				ctx.advance_to(out);
				out = scalar_fmt.format(m(r, c), ctx);
			}
		}
		*out++ = ']';

		return out;
	}
};

} // namespace std
