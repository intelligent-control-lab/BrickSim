export module bricksim.physx.overlap_detection;

import std;
import bricksim.utils.transforms;
import bricksim.utils.hash;
import bricksim.utils.bbox;
import bricksim.utils.memory;
import bricksim.utils.concepts;
import bricksim.vendor;

namespace bricksim {

template <int S2Level, int DExp2> class PlaneBinKey {
	static_assert(S2Level >= 0 && S2Level <= 30);

  public:
	// Packed normal-cell id: [face:3 bits][i:30 bits][j:30 bits][1 spare bit]
	std::uint64_t s2_cell = 0;

	// Quantized displacement bin.
	std::int64_t d_bin = 0;

	static PlaneBinKey from_plane(const Eigen::Vector3d &n, double d) {
		return {
		    .s2_cell = quantize_normal(n),
		    .d_bin = quantize_displacement(d),
		};
	}

	// Yields **this key** plus its neighbor bins:
	// - 3x3 neighborhood on S^2 at fixed S2Level (deduped)
	// - d_bin +/- 1
	void for_each_neighbor(std::invocable<PlaneBinKey> auto consumer) const {
		int face = 0;
		std::uint32_t i = 0;
		std::uint32_t j = 0;
		unpack_cell(s2_cell, &face, &i, &j);

		// Collect unique neighbor S2 cells (<= 9).
		std::uint64_t cells[9];
		int cell_count = 0;
		auto add_cell = [&](std::uint64_t cid) {
			for (int k = 0; k < cell_count; ++k) {
				if (cells[k] == cid) {
					return;
				}
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
				std::invoke(consumer, PlaneBinKey{
				                          .s2_cell = cells[c],
				                          .d_bin = add_d_saturating(d_bin, dd),
				                      });
			}
		}
	}

	bool operator==(const PlaneBinKey &other) const = default;

	struct Hash {
		std::size_t operator()(const PlaneBinKey &key) const {
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
	static constexpr std::uint64_t kPatchShift = 61;

	static std::uint64_t pack_cell(int face, std::uint32_t i, std::uint32_t j) {
		return (std::uint64_t(face) << kPatchShift) |
		       (std::uint64_t(i & kCoordMask) << 31) |
		       (std::uint64_t(j & kCoordMask) << 1);
	}

	static void unpack_cell(std::uint64_t id, int *face, std::uint32_t *i,
	                        std::uint32_t *j) {
		*face = static_cast<int>(id >> kPatchShift);
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
} // namespace bricksim

namespace std {
template <int S2Level, int DExp2>
struct hash<bricksim::PlaneBinKey<S2Level, DExp2>> {
	using PlaneBinKey = bricksim::PlaneBinKey<S2Level, DExp2>;
	std::size_t operator()(const PlaneBinKey &key) const {
		return typename PlaneBinKey::Hash{}(key);
	}
};
} // namespace std

namespace bricksim {

export template <class T>
concept PlanarPatchLike = requires(const T &f) {
	{ f.transform() } -> std::convertible_to<Transformd>;
	{ f.bbox() } -> std::convertible_to<BBox2d>;
	{ f.polygon_vertices() } -> range_of<Eigen::Vector2d>;
};

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

// Use this to compute max DExp2 and S2Level:
//  double kEpsilon = 1e-6;
//  int kMaxDExp2   = std::ceil(-std::log2(kCoplanarDisplacementThreshold   + 2 * kEpsilon)) - 1;
//  int kMaxS2Level = std::ceil(-std::log2(kCoplanarAngleThreshold          + 2 * kEpsilon)) - 1;

export template <PlanarPatchLike Patch, int S2Level = 18, int DExp2 = 18,
                 double SATMinPenetration = 1e-6,
                 double CoplanarDisplacementThreshold = 1e-6,
                 double CoplanarAngleThreshold = 1e-6>
class PlanarPatchBinMap {
  public:
	using PatchIdx = std::uint32_t;

	PatchIdx insert(Patch patch) {
		Key key = compute_key(patch);
		PatchIdx idx = static_cast<PatchIdx>(patches_.size());
		patches_.push_back({.key = key, .patch = std::move(patch)});
		bins_[key].push_back(idx);
		return idx;
	}
	const Patch &patch_at(PatchIdx idx) const {
		return patches_[idx].patch;
	}
	std::size_t size() const {
		return patches_.size();
	}
	void clear() {
		patches_.clear();
		bins_.clear();
	}

	struct OverlapPatchPair {
		PatchIdx idx_u;
		PatchIdx idx_v;
		const Patch &patch_u;
		const Patch &patch_v;

		// Valid only during the callback invocation.
		std::span<const Eigen::Vector2d> polygon_u;
		std::span<const Eigen::Vector2d> polygon_v;

		OverlapPatchPair(PatchIdx idx_u, PatchIdx idx_v, const Patch &patch_u,
		                 const Patch &patch_v,
		                 std::span<const Eigen::Vector2d> polygon_u,
		                 std::span<const Eigen::Vector2d> polygon_v)
		    : idx_u(idx_u), idx_v(idx_v), patch_u(patch_u), patch_v(patch_v),
		      polygon_u(polygon_u), polygon_v(polygon_v) {}
	};

	void
	for_each_overlap(std::invocable<OverlapPatchPair> auto consumer) const {
		std::vector<Eigen::Vector2d> vbuf_u;
		std::vector<Eigen::Vector2d> vbuf_v;
		for_each_coplanar([&](PatchIdx f_u, PatchIdx f_v) {
			const Patch &patch_u = patches_[f_u].patch;
			const Patch &patch_v = patches_[f_v].patch;
			const Transformd &T_u = patch_u.transform();
			const Transformd &T_v = patch_v.transform();
			Transformd T_u_v = inverse(T_u) * T_v;
			const auto &[q_u_v, t_u_v] = T_u_v;
			Eigen::Vector3d e1 = q_u_v * Eigen::Vector3d::UnitX();
			Eigen::Vector3d e2 = q_u_v * Eigen::Vector3d::UnitY();
			// CAUTION: det(R2d) = -1 (improper rotation)
			Eigen::Matrix2d R2d;
			R2d.col(0) = e1.head<2>();
			R2d.col(1) = e2.head<2>();
			Eigen::Vector2d t2d = t_u_v.head<2>();

			// 1. Fast reject check by bbox overlap
			const BBox2d &bbox_u = patch_u.bbox();
			const BBox2d &bbox_v = patch_v.bbox();
			BBox2d bbox_v_in_u = transform_bbox2d(bbox_v, R2d, t2d);
			if (!bbox_u.overlaps(bbox_v_in_u)) {
				return;
			}

			// 2. Collect vertices
			// CAUTION: because R2d is improper, the vertex order for vbuf_u is CCW, but for vbuf_v it's CW
			vbuf_u.clear();
			vbuf_v.clear();
			for (const Eigen::Vector2d &vertex : patch_u.polygon_vertices()) {
				vbuf_u.emplace_back(vertex);
			}
			for (const Eigen::Vector2d &vertex : patch_v.polygon_vertices()) {
				vbuf_v.emplace_back(R2d * vertex + t2d);
			}

			// 3. SAT
			// SAT also accepts CW ordering
			double penetration = sat(vbuf_u, vbuf_v);
			if (penetration >= SATMinPenetration) {
				// Convert vbuf_v to CCW before yielding
				std::ranges::reverse(vbuf_v);
				std::invoke(consumer,
				            OverlapPatchPair{
				                f_u,
				                f_v,
				                patches_[f_u].patch,
				                patches_[f_v].patch,
				                std::span<const Eigen::Vector2d>(vbuf_u),
				                std::span<const Eigen::Vector2d>(vbuf_v),
				            });
			}
		});
	}

  private:
	using Key = PlaneBinKey<S2Level, DExp2>;
	struct StagedPatch {
		Key key;
		Patch patch;
	};

	static Key compute_key(const Patch &patch) {
		const Transformd &T = patch.transform();
		auto &[q, t] = T;
		Eigen::Vector3d n = q * Eigen::Vector3d::UnitZ();
		double d = n.dot(t);
		return Key::from_plane(n, d);
	}

	std::vector<StagedPatch> patches_;
	std::unordered_map<Key, std::vector<PatchIdx>> bins_;

	void
	for_each_coplanar(std::invocable<PatchIdx, PatchIdx> auto consumer) const {
		double dot_threshold = -std::cos(CoplanarAngleThreshold);
		for (const auto &[bk_u, f_us] : bins_) {
			for (PatchIdx f_u : f_us) {
				const Transformd &T_u = patches_[f_u].patch.transform();
				const auto &[q_u, t_u] = T_u;
				Eigen::Vector3d n_u = q_u * Eigen::Vector3d::UnitZ();
				double d_u = n_u.dot(t_u);
				Key bk_u_opposite = Key::from_plane(-n_u, -d_u);
				bk_u_opposite.for_each_neighbor([&](Key bk_v) {
					auto it_f_vs = bins_.find(bk_v);
					if (it_f_vs != bins_.end()) {
						for (PatchIdx f_v : it_f_vs->second) {
							if (f_u >= f_v) {
								continue;
							}
							const Transformd &T_v =
							    patches_[f_v].patch.transform();
							const auto &[q_v, t_v] = T_v;
							Eigen::Vector3d n_v =
							    q_v * Eigen::Vector3d::UnitZ();
							if (n_u.dot(n_v) > dot_threshold) {
								continue;
							}
							double d = (t_v - t_u).dot(n_u);
							if (std::abs(d) > CoplanarDisplacementThreshold) {
								continue;
							}
							std::invoke(consumer, f_u, f_v);
						}
					}
				});
			}
		}
	}
};

} // namespace bricksim
