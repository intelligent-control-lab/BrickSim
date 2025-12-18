export module lego_assemble.physx.touching_face_detection;

import std;
import lego_assemble.core.specs;
import lego_assemble.core.graph;
import lego_assemble.utils.transforms;
import lego_assemble.utils.hash;
import lego_assemble.utils.bbox;
import lego_assemble.vendor;

namespace lego_assemble {

template <int S2Level, int DExp2> struct FaceBinKey {
	static_assert(S2Level >= 0 && S2Level <= 30);

	// Packed normal-cell id: [face:3 bits][i:30 bits][j:30 bits][1 spare bit]
	std::uint64_t s2_cell = 0;

	// Quantized displacement bin.
	std::int64_t d_bin = 0;

	static FaceBinKey from_face(const Eigen::Vector3d &n, double d) {
		return {
		    .s2_cell = quantize_normal(n),
		    .d_bin = quantize_displacement(d),
		};
	}

	// Yields **this key** plus its neighbor bins:
	// - 3x3 neighborhood on S^2 at fixed S2Level (deduped)
	// - d_bin +/- 1
	void for_each_neighbor(auto &&fn) const {
		int face = 0;
		std::uint32_t i = 0;
		std::uint32_t j = 0;
		unpack_cell(s2_cell, &face, &i, &j);

		// Collect unique neighbor S2 cells (<= 9).
		std::uint64_t cells[9];
		int cell_count = 0;
		auto add_cell = [&](std::uint64_t cid) {
			for (int k = 0; k < cell_count; ++k) {
				if (cells[k] == cid)
					return;
			}
			cells[cell_count++] = cid;
		};

		constexpr std::uint32_t size = (std::uint32_t{1} << S2Level);
		constexpr double inv_size = 1.0 / static_cast<double>(size);
		double s0 = (static_cast<double>(i) + 0.5) * inv_size;
		double t0 = (static_cast<double>(j) + 0.5) * inv_size;
		for (int di = -1; di <= 1; ++di) {
			for (int dj = -1; dj <= 1; ++dj) {
				int ni = static_cast<int>(i) + di;
				int nj = static_cast<int>(j) + dj;
				if (0 <= ni && ni < static_cast<int>(size) && 0 <= nj &&
				    nj < static_cast<int>(size)) {
					// ---- Fast path: stays on same face, no reprojection ----
					add_cell(pack_cell(face, static_cast<std::uint32_t>(ni),
					                   static_cast<std::uint32_t>(nj)));
				} else {
					// ---- Wrap path (S2-style): step in (s,t), unproject, re-quantize ----
					double s = s0 + static_cast<double>(di) * inv_size;
					double t = t0 + static_cast<double>(dj) * inv_size;
					double u = st_to_uv(s);
					double v = st_to_uv(t);
					Eigen::Vector3d p = face_uv_to_xyz(face, u, v);
					add_cell(quantize_normal(p));
				}
			}
		}

		// Emit (S2-neighbor cells) x (d neighbors).
		for (int c = 0; c < cell_count; ++c) {
			for (int dd = -1; dd <= 1; ++dd) {
				FaceBinKey neighbor_key{
				    .s2_cell = cells[c],
				    .d_bin = add_d_saturating(d_bin, dd),
				};
				std::invoke(fn, neighbor_key);
			}
		}
	}

	bool operator==(const FaceBinKey &other) const = default;

	struct Hash {
		std::size_t operator()(const FaceBinKey &key) const {
			std::size_t h1 = std::hash<std::uint64_t>{}(key.s2_cell);
			std::size_t h2 = std::hash<std::int64_t>{}(key.d_bin);
			hash_combine(h2, h1);
			return h2;
		}
	};

  private:
	// ---- S2 quadratic projection (from s2coords.h; standalone) ----
	static double st_to_uv(double s) {
		// Quadratic projection
		if (s >= 0.5)
			return (1.0 / 3.0) * (4.0 * s * s - 1.0);
		double t = 1.0 - s;
		return (1.0 / 3.0) * (1.0 - 4.0 * t * t);
	}

	static double uv_to_st(double u) {
		if (u >= 0.0)
			return 0.5 * std::sqrt(1.0 + 3.0 * u);
		return 1.0 - 0.5 * std::sqrt(1.0 - 3.0 * u);
	}

	static Eigen::Vector3d face_uv_to_xyz(int face, double u, double v) {
		switch (face) {
		case 0:
			return {1, u, v};
		case 1:
			return {-u, 1, v};
		case 2:
			return {-u, -v, 1};
		case 3:
			return {-1, -v, -u};
		case 4:
			return {v, -1, -u};
		default:
			return {v, u, -1};
		}
	}

	static int get_face(const Eigen::Vector3d &p) {
		// Same rule as S2::GetFace: choose largest abs component, break ties deterministically.
		int axis = 0;
		double ax = std::abs(p.x());
		double ay = std::abs(p.y());
		double az = std::abs(p.z());
		if (ay > ax) {
			ax = ay;
			axis = 1;
		}
		if (az > ax) {
			axis = 2;
		}
		if (p[axis] < 0)
			axis += 3;
		return axis;
	}

	static void valid_face_xyz_to_uv(int face, const Eigen::Vector3d &p,
	                                 double *u, double *v) {
		// Assumes 'face' is valid for p (i.e., face component has correct sign and is dominant).
		switch (face) {
		case 0:
			*u = p.y() / p.x();
			*v = p.z() / p.x();
			break;
		case 1:
			*u = -p.x() / p.y();
			*v = p.z() / p.y();
			break;
		case 2:
			*u = -p.x() / p.z();
			*v = -p.y() / p.z();
			break;
		case 3:
			*u = p.z() / p.x();
			*v = p.y() / p.x();
			break;
		case 4:
			*u = p.z() / p.y();
			*v = -p.x() / p.y();
			break;
		default:
			*u = -p.y() / p.z();
			*v = -p.x() / p.z();
			break;
		}
	}

	static int xyz_to_face_uv(const Eigen::Vector3d &p, double *u, double *v) {
		int face = get_face(p);
		valid_face_xyz_to_uv(face, p, u, v);
		return face;
	}

	static std::uint32_t st_to_ij(double s) {
		// Map s in [0,1] to integer in [0, 2^S2Level - 1] using floor, with clamping.
		constexpr std::uint32_t size = (std::uint32_t{1} << S2Level);
		if (!(s > 0.0))
			return 0; // also catches NaN
		if (!(s < 1.0))
			return size - 1; // also catches NaN
		double scaled = s * static_cast<double>(size);
		std::uint32_t ij =
		    static_cast<std::uint32_t>(scaled); // floor for positive
		if (ij >= size)
			ij = size - 1;
		return ij;
	}

	static constexpr std::uint64_t kCoordMask = (std::uint64_t{1} << 30) - 1;
	static constexpr std::uint64_t kFaceShift = 61;

	static std::uint64_t pack_cell(int face, std::uint32_t i, std::uint32_t j) {
		return (std::uint64_t(face) << kFaceShift) |
		       (std::uint64_t(i & kCoordMask) << 31) |
		       (std::uint64_t(j & kCoordMask) << 1);
	}

	static void unpack_cell(std::uint64_t id, int *face, std::uint32_t *i,
	                        std::uint32_t *j) {
		*face = static_cast<int>(id >> kFaceShift);
		*i = static_cast<std::uint32_t>((id >> 31) & kCoordMask);
		*j = static_cast<std::uint32_t>((id >> 1) & kCoordMask);
	}

	static std::uint64_t quantize_normal(const Eigen::Vector3d &n_dir) {
		double u = 0.0, v = 0.0;
		int face = xyz_to_face_uv(n_dir, &u, &v);
		double s = uv_to_st(u);
		double t = uv_to_st(v);
		std::uint32_t i = st_to_ij(s);
		std::uint32_t j = st_to_ij(t);
		return pack_cell(face, i, j);
	}

	static std::int64_t quantize_displacement(double d) {
		constexpr auto int64_min = std::numeric_limits<std::int64_t>::min();
		constexpr auto int64_max = std::numeric_limits<std::int64_t>::max();
		// Power-of-two scaling keeps this relatively stable and fast.
		double scaled = std::ldexp(d, DExp2); // d * 2^DExp2
		double f = std::floor(scaled);
		if (f <= static_cast<double>(int64_min))
			return int64_min;
		if (f >= static_cast<double>(int64_max))
			return int64_max;
		return static_cast<std::int64_t>(f);
	}

	static std::int64_t add_d_saturating(std::int64_t base, int delta) {
		constexpr auto int64_min = std::numeric_limits<std::int64_t>::min();
		constexpr auto int64_max = std::numeric_limits<std::int64_t>::max();
		if (delta > 0) {
			std::int64_t lim = int64_max - static_cast<std::int64_t>(delta);
			if (base > lim)
				return int64_max;
		} else if (delta < 0) {
			std::int64_t lim = int64_min - static_cast<std::int64_t>(delta);
			if (base < lim)
				return int64_min;
		}
		return base + static_cast<std::int64_t>(delta);
	}
};

export struct FaceRef {
	PartId pid;
	FaceId fid;
	auto operator<=>(const FaceRef &) const = default;
};

export struct Face {
	FaceRef ref;
	Transformd T;
	BBox2d bbox;
};

constexpr double kSATMinPenetration = 1e-6;             // meters
constexpr double kCoplanarDisplacementThreshold = 1e-6; // meters
constexpr double kCoplanarAngleThreshold = 1e-6;        // radians

// Use this to compute max DExp2 and S2Level:
//  double kEpsilon = 1e-6;
//  int kMaxDExp2   = std::ceil(-std::log2(kCoplanarDisplacementThreshold   + 2 * kEpsilon)) - 1;
//  int kMaxS2Level = std::ceil(-std::log2(kCoplanarAngleThreshold          + 2 * kEpsilon)) - 1;

constexpr int kDExp2 = 18;
constexpr int kS2Level = 18;

using LegoFaceBinKey = FaceBinKey<kS2Level, kDExp2>;
using FaceKey = std::pair<PartId, FaceId>;
using LegoFaceBinMap =
    std::unordered_map<LegoFaceBinKey, std::vector<Face>, LegoFaceBinKey::Hash>;

// Projects vertices onto an axis and returns {min, max}
std::tuple<double, double>
project_polygon(std::span<const Eigen::Vector2d> poly,
                const Eigen::Vector2d &axis) {
	double min_val = std::numeric_limits<double>::infinity();
	double max_val = -std::numeric_limits<double>::infinity();
	for (const Eigen::Vector2d &p : poly) {
		double val = p.dot(axis);
		if (val < min_val)
			min_val = val;
		if (val > max_val)
			max_val = val;
	}
	return {min_val, max_val};
}

double sat(std::span<const Eigen::Vector2d> poly_a,
           std::span<const Eigen::Vector2d> poly_b) {
	constexpr double inf = std::numeric_limits<double>::infinity();
	double min_penetration = inf;
	for (auto &poly : {poly_a, poly_b}) {
		std::size_t n = poly.size();
		for (std::size_t i = 0; i < n; ++i) {
			Eigen::Vector2d edge = poly[(i + 1) % n] - poly[i];
			Eigen::Vector2d axis{-edge.y(), edge.x()};
			if (axis.squaredNorm() < 1e-12) {
				// Degenerate edge
				continue;
			}
			axis.normalize();
			auto [min_a, max_a] = project_polygon(poly_a, axis);
			auto [min_b, max_b] = project_polygon(poly_b, axis);
			double overlap = std::min(max_a, max_b) - std::max(min_a, min_b);
			if (overlap <= 0.0) {
				// Separating axis found
				return overlap;
			}
			if (overlap < min_penetration) {
				min_penetration = overlap;
			}
		}
	}
	if (min_penetration == inf) {
		// Degenerate polygons
		return -inf;
	}
	return min_penetration;
}

void for_each_coplanar_face_pair(const LegoFaceBinMap &face_bins, auto &&fn) {
	double dot_threshold = -std::cos(kCoplanarAngleThreshold);
	for (const auto &[bk_u, f_us] : face_bins) {
		for (const Face &f_u : f_us) {
			const auto &[q_u, t_u] = f_u.T;
			Eigen::Vector3d n_u = q_u * Eigen::Vector3d::UnitZ();
			double d_u = n_u.dot(t_u);
			auto bk_u_opposite = LegoFaceBinKey::from_face(-n_u, -d_u);
			bk_u_opposite.for_each_neighbor([&](const LegoFaceBinKey &bk_v) {
				auto it_f_vs = face_bins.find(bk_v);
				if (it_f_vs == face_bins.end()) {
					return;
				}
				for (const Face &f_v : it_f_vs->second) {
					if (f_u.ref >= f_v.ref) {
						continue;
					}
					const auto &[q_v, t_v] = f_v.T;
					Eigen::Vector3d n_v = q_v * Eigen::Vector3d::UnitZ();
					if (n_u.dot(n_v) > dot_threshold) {
						continue;
					}
					double d = (t_v - t_u).dot(n_u);
					if (std::abs(d) > kCoplanarDisplacementThreshold) {
						continue;
					}
					std::invoke(fn, f_u, f_v);
				}
			});
		}
	}
}

BBox2d transform_bbox2d(const BBox2d &bbox, const Eigen::Matrix2d &R,
                        const Eigen::Vector2d &t) {
	constexpr double inf = std::numeric_limits<double>::infinity();
	Eigen::Vector2d new_min{inf, inf};
	Eigen::Vector2d new_max{-inf, -inf};
	for (int ix = 0; ix <= 1; ++ix) {
		double cx = (ix == 0) ? bbox.min.x() : bbox.max.x();
		for (int iy = 0; iy <= 1; ++iy) {
			double cy = (iy == 0) ? bbox.min.y() : bbox.max.y();
			Eigen::Vector2d p_local{cx, cy};
			Eigen::Vector2d p = R * p_local + t;
			new_min = new_min.cwiseMin(p);
			new_max = new_max.cwiseMax(p);
		}
	}
	return {.min = new_min, .max = new_max};
}

export template <class G, class Fn>
    requires std::invocable<Fn, const Face &, const Face &>
void detect_touching_faces(const G &g, PartId root, Fn &&fn) {
	LegoFaceBinMap face_bins;
	// Broadphase: Collect faces
	for (auto [u, T_root_u] : g.component_view(root).transforms()) {
		g.parts().visit(u, [&](const auto &pw) {
			for (auto &&face : pw.wrapped().faces()) {
				Transformd T_root_face = T_root_u * face.transform();
				auto &[q, t] = T_root_face;
				// Outward normal
				Eigen::Vector3d n = q * Eigen::Vector3d::UnitZ();
				// Displacement from origin
				double d = n.dot(t);
				auto bin_key = LegoFaceBinKey::from_face(n, d);
				face_bins[bin_key].emplace_back(Face{
				    .ref = FaceRef{.pid = u, .fid = face.id()},
				    .T = T_root_face,
				    .bbox = face.bbox(),
				});
			}
		});
	}
	// Narrowphase: Test coplanar face pairs
	std::vector<Eigen::Vector2d> vertices_u; // CCW order
	std::vector<Eigen::Vector2d> vertices_v; // CW order
	for_each_coplanar_face_pair(
	    face_bins, [&](const Face &f_u, const Face &f_v) {
		    const Transformd &T_root_u = f_u.T;
		    const Transformd &T_root_v = f_v.T;
		    Transformd T_u_v = inverse(T_root_u) * T_root_v;
		    const auto &[q_u_v, t_u_v] = T_u_v;
		    Eigen::Vector3d e1 = q_u_v * Eigen::Vector3d::UnitX();
		    Eigen::Vector3d e2 = q_u_v * Eigen::Vector3d::UnitY();
		    // CAUTION: det(R2d) = -1 (improper rotation)
		    // CAUTION: because R2d is improper, the vertex order for f_u is CCW, but for f_v it's CW
		    Eigen::Matrix2d R2d;
		    R2d.col(0) = e1.head<2>();
		    R2d.col(1) = e2.head<2>();
		    Eigen::Vector2d t2d = t_u_v.head<2>();

		    // 1. Fast reject check by bbox overlap
		    const BBox2d &bbox_u = f_u.bbox;
		    const BBox2d &bbox_v = f_v.bbox;
		    BBox2d bbox_v_in_u = transform_bbox2d(bbox_v, R2d, t2d);
		    if (!bbox_u.overlaps(bbox_v_in_u)) {
			    return;
		    }

		    // 2. Collect vertices
		    vertices_u.clear();
		    vertices_v.clear();
		    g.parts().visit(f_u.ref.pid, [&](const auto &pw) {
			    for (const Eigen::Vector2d &vertex :
			         pw.wrapped().get_face(f_u.ref.fid)->polygon_vertices()) {
				    vertices_u.emplace_back(vertex);
			    }
		    });
		    g.parts().visit(f_v.ref.pid, [&](const auto &pw) {
			    for (const Eigen::Vector2d &vertex :
			         pw.wrapped().get_face(f_v.ref.fid)->polygon_vertices()) {
				    vertices_v.emplace_back(R2d * vertex + t2d);
			    }
		    });

		    // 3. SAT
		    // It works with CW also.
		    double penetration = sat(vertices_u, vertices_v);
		    if (penetration >= kSATMinPenetration) {
			    std::invoke(fn, f_u, f_v);
		    }
	    });
}

} // namespace lego_assemble
