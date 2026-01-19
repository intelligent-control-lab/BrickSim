export module lego_assemble.physx.polygon_clipping;

import std;
import lego_assemble.utils.math;
import lego_assemble.vendor;

namespace lego_assemble {

bool near2(const Eigen::Vector2d &a, const Eigen::Vector2d &b, double eps) {
	return (a - b).squaredNorm() <= eps * eps;
}

// Removes consecutive duplicates and (optionally) collinear points.
void cleanup_polygon(std::vector<Eigen::Vector2d> &poly, double eps = 1e-9) {
	if (poly.empty())
		return;

	// Remove consecutive duplicates.
	{
		std::vector<Eigen::Vector2d> tmp;
		tmp.reserve(poly.size());
		for (const auto &p : poly) {
			if (tmp.empty() || !near2(tmp.back(), p, eps)) {
				tmp.push_back(p);
			}
		}
		if (tmp.size() >= 2 && near2(tmp.front(), tmp.back(), eps)) {
			tmp.pop_back();
		}
		poly = std::move(tmp);
	}

	if (poly.size() < 3)
		return;

	// Remove nearly-collinear vertices.
	{
		std::vector<Eigen::Vector2d> tmp;
		tmp.reserve(poly.size());

		auto is_collinear = [&](const Eigen::Vector2d &prev,
		                        const Eigen::Vector2d &curr,
		                        const Eigen::Vector2d &next) {
			Eigen::Vector2d a = curr - prev;
			Eigen::Vector2d b = next - curr;
			double cr = std::abs(cross2(a, b));
			// Scale eps by edge lengths to be less unit-sensitive.
			double scale = std::max(1.0, a.norm() + b.norm());
			return cr <= eps * scale;
		};

		for (std::size_t i = 0; i < poly.size(); ++i) {
			const auto &prev = poly[(i + poly.size() - 1) % poly.size()];
			const auto &curr = poly[i];
			const auto &next = poly[(i + 1) % poly.size()];
			if (!is_collinear(prev, curr, next)) {
				tmp.push_back(curr);
			}
		}
		poly = std::move(tmp);
	}

	if (poly.size() >= 2 && near2(poly.front(), poly.back(), eps)) {
		poly.pop_back();
	}
}

// Intersect a segment P0->P1 with the infinite line through A->B,
// assuming the segment crosses the line (or is numerically very close).
Eigen::Vector2d segment_line_intersection(const Eigen::Vector2d &p0,
                                          const Eigen::Vector2d &p1,
                                          const Eigen::Vector2d &a,
                                          const Eigen::Vector2d &b,
                                          double eps = 1e-18) {
	Eigen::Vector2d e = b - a;

	// Signed "distance" from point to line in units of cross product.
	double d0 = cross2(e, p0 - a);
	double d1 = cross2(e, p1 - a);
	double denom = (d0 - d1);

	if (std::abs(denom) < eps) {
		// Nearly parallel / numerically unstable; return an endpoint.
		return p0;
	}
	double t = d0 / denom; // where cross(e, p(t)-a)=0
	t = std::clamp(t, 0.0, 1.0);
	return p0 + t * (p1 - p0);
}

// Sutherland-Hodgman polygon clipping algorithm.
// Returns intersection polygon (convex) of subject ∩ clip.
// Works for CCW polygons.
export std::vector<Eigen::Vector2d>
convex_polygon_intersection(std::span<const Eigen::Vector2d> subject,
                            std::span<const Eigen::Vector2d> clip,
                            double eps = 1e-12) {
	if (subject.size() < 3 || clip.size() < 3)
		return {};

	// Start with subject polygon.
	std::vector<Eigen::Vector2d> out(subject.begin(), subject.end());

	auto inside = [&](const Eigen::Vector2d &a, const Eigen::Vector2d &b,
	                  const Eigen::Vector2d &p) {
		return cross2(b - a, p - a) >= -eps;
	};

	for (std::size_t i = 0; i < clip.size(); ++i) {
		Eigen::Vector2d A = clip[i];
		Eigen::Vector2d B = clip[(i + 1) % clip.size()];

		if (out.size() < 3)
			return {}; // already empty/degenerate

		std::vector<Eigen::Vector2d> input = std::move(out);
		out.clear();
		out.reserve(input.size() + 2);

		Eigen::Vector2d S = input.back();
		bool S_in = inside(A, B, S);

		for (const Eigen::Vector2d &E : input) {
			bool E_in = inside(A, B, E);

			if (E_in) {
				if (!S_in) {
					out.push_back(segment_line_intersection(S, E, A, B));
				}
				out.push_back(E);
			} else {
				if (S_in) {
					out.push_back(segment_line_intersection(S, E, A, B));
				}
			}

			S = E;
			S_in = E_in;
		}
	}

	cleanup_polygon(out, eps);
	if (out.size() < 3)
		return {}; // intersection is a segment/point/empty
	return out;
}

} // namespace lego_assemble
