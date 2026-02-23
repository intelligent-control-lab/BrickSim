export module bricksim.utils.pack2d_maxrect;

import std;

namespace bricksim::pack2d {

export struct RectInput {
	std::int32_t id{};
	std::int32_t width{};
	std::int32_t height{};

	[[nodiscard]] constexpr std::int64_t area() const noexcept {
		return static_cast<std::int64_t>(width) *
		       static_cast<std::int64_t>(height);
	}
};

export struct PackedRect {
	std::int32_t id{};
	std::int32_t x{};
	std::int32_t y{};
	std::int32_t width{};
	std::int32_t height{};
	bool rotated{}; // true if input (w,h) was rotated to (h,w)
};

export struct Bin {
	std::int32_t width{};
	std::int32_t height{};
};

export enum class Heuristic {
	BestShortSideFit,
	BestLongSideFit,
	BestAreaFit,
	BottomLeft
};

export struct PackResult {
	std::int32_t bin_width{};
	std::int32_t bin_height{};
	std::vector<PackedRect> packed;
	std::vector<std::int32_t> not_packed_ids;

	[[nodiscard]] double occupancy() const noexcept {
		const auto bin_area = static_cast<std::int64_t>(bin_width) *
		                      static_cast<std::int64_t>(bin_height);
		if (bin_area <= 0)
			return 0.0;

		std::int64_t used_area = 0;
		for (const auto &r : packed) {
			used_area += static_cast<std::int64_t>(r.width) *
			             static_cast<std::int64_t>(r.height);
		}
		return static_cast<double>(used_area) / static_cast<double>(bin_area);
	}
};

// Internal axis-aligned rectangle type: half-open [x, x+w) × [y, y+h)
export struct Rect {
	std::int32_t x{};
	std::int32_t y{};
	std::int32_t w{};
	std::int32_t h{};
};

export class MaxRectsBinPack {
  public:
	explicit MaxRectsBinPack(std::int32_t bin_width, std::int32_t bin_height,
	                         bool allow_rotations = true)
	    : bin_width_{bin_width}, bin_height_{bin_height},
	      allow_rotations_{allow_rotations} {
		if (bin_width_ > 0 && bin_height_ > 0) {
			free_rects_.push_back(Rect{0, 0, bin_width_, bin_height_});
		}
	}

	[[nodiscard]] std::int32_t bin_width() const noexcept {
		return bin_width_;
	}

	[[nodiscard]] std::int32_t bin_height() const noexcept {
		return bin_height_;
	}

	[[nodiscard]] bool allow_rotations() const noexcept {
		return allow_rotations_;
	}

	// Insert a single rectangle; returns std::nullopt if it doesn't fit.
	[[nodiscard]] std::optional<PackedRect> insert(std::int32_t width,
	                                               std::int32_t height,
	                                               std::int32_t id,
	                                               Heuristic method) {
		if (width <= 0 || height <= 0) {
			return std::nullopt;
		}

		Rect best_node{};
		bool best_rotated = false;
		int best_score1 = std::numeric_limits<int>::max();
		int best_score2 = std::numeric_limits<int>::max();

		const auto try_candidate = [&](const Rect &free_rect, std::int32_t rw,
		                               std::int32_t rh, bool rotated) {
			if (free_rect.w < rw || free_rect.h < rh) {
				return;
			}

			int score1 = 0;
			int score2 = 0;
			score_candidate(method, free_rect, rw, rh, score1, score2);

			if (score1 < best_score1 ||
			    (score1 == best_score1 && score2 < best_score2)) {
				best_score1 = score1;
				best_score2 = score2;
				best_rotated = rotated;
				best_node = Rect{free_rect.x, free_rect.y, rw, rh};
			}
		};

		for (const auto &fr : free_rects_) {
			// Try non-rotated
			try_candidate(fr, width, height, false);

			// Try rotated if allowed
			if (allow_rotations_) {
				try_candidate(fr, height, width, true);
			}
		}

		if (best_score1 == std::numeric_limits<int>::max()) {
			// No fit
			return std::nullopt;
		}

		// Commit placement and update free space.
		split_free_rects(best_node);
		used_rects_.push_back(best_node);

		return PackedRect{.id = id,
		                  .x = best_node.x,
		                  .y = best_node.y,
		                  .width = best_node.w,
		                  .height = best_node.h,
		                  .rotated = best_rotated};
	}

	// Add a single obstacle: it will be carved out of free space.
	// Obstacle is clipped to the bin; zero/negative-area or fully
	// out-of-bounds obstacles are ignored.
	void add_obstacle(int x, int y, int w, int h) {
		if (w == 0 || h == 0) {
			return;
		}

		// Canonicalize (support negative widths/heights).
		auto canon = [](std::int32_t x, std::int32_t w) {
			if (w >= 0) {
				return std::pair{x, w};
			}
			// [x+w, x) -> [x+w, x+0) with positive width
			return std::pair{x + w, -w};
		};

		auto [ox, ow] = canon(x, w);
		auto [oy, oh] = canon(y, h);

		const std::int32_t x0 = std::max<std::int32_t>(0, ox);
		const std::int32_t y0 = std::max<std::int32_t>(0, oy);
		const std::int32_t x1 = std::min<std::int32_t>(bin_width_, ox + ow);
		const std::int32_t y1 = std::min<std::int32_t>(bin_height_, oy + oh);

		if (x1 <= x0 || y1 <= y0) {
			return; // no intersection with bin
		}

		Rect clipped{x0, y0, x1 - x0, y1 - y0};
		split_free_rects(clipped);      // carve space
		used_rects_.push_back(clipped); // keep for debugging
	}

  private:
	std::int32_t bin_width_{};
	std::int32_t bin_height_{};
	bool allow_rotations_{true};

	std::vector<Rect> free_rects_;
	std::vector<Rect> used_rects_; // optional, but handy for debugging

	static constexpr bool intersects(const Rect &a, const Rect &b) noexcept {
		// Half-open boxes
		const auto ax1 = a.x;
		const auto ay1 = a.y;
		const auto ax2 = a.x + a.w;
		const auto ay2 = a.y + a.h;

		const auto bx1 = b.x;
		const auto by1 = b.y;
		const auto bx2 = b.x + b.w;
		const auto by2 = b.y + b.h;

		return !(ax2 <= bx1 || ax1 >= bx2 || ay2 <= by1 || ay1 >= by2);
	}

	static constexpr bool is_contained_in(const Rect &a,
	                                      const Rect &b) noexcept {
		const auto ax1 = a.x;
		const auto ay1 = a.y;
		const auto ax2 = a.x + a.w;
		const auto ay2 = a.y + a.h;

		const auto bx1 = b.x;
		const auto by1 = b.y;
		const auto bx2 = b.x + b.w;
		const auto by2 = b.y + b.h;

		return ax1 >= bx1 && ay1 >= by1 && ax2 <= bx2 && ay2 <= by2;
	}

	static void score_candidate(Heuristic method, const Rect &free_rect,
	                            std::int32_t rw, std::int32_t rh,
	                            int &out_score1, int &out_score2) {
		const int leftover_horiz = free_rect.w - rw;
		const int leftover_vert = free_rect.h - rh;
		const int short_side_fit = std::min(leftover_horiz, leftover_vert);
		const int long_side_fit = std::max(leftover_horiz, leftover_vert);
		const int area_fit = free_rect.w * free_rect.h - rw * rh;

		switch (method) {
		case Heuristic::BestShortSideFit:
			out_score1 = short_side_fit;
			out_score2 = long_side_fit;
			break;
		case Heuristic::BestLongSideFit:
			out_score1 = long_side_fit;
			out_score2 = short_side_fit;
			break;
		case Heuristic::BestAreaFit:
			out_score1 = area_fit;
			out_score2 = short_side_fit;
			break;
		case Heuristic::BottomLeft:
			// lower y is better, then smaller x
			out_score1 = free_rect.y;
			out_score2 = free_rect.x;
			break;
		}
	}

	void split_free_rects(const Rect &used) {
		std::vector<Rect> new_free;
		new_free.reserve(free_rects_.size() * 2);

		for (const auto &fr : free_rects_) {
			if (!intersects(fr, used)) {
				new_free.push_back(fr);
				continue;
			}

			const auto fr_right = fr.x + fr.w;
			const auto fr_top = fr.y + fr.h;
			const auto used_right = used.x + used.w;
			const auto used_top = used.y + used.h;

			// Left
			if (used.x > fr.x) {
				Rect r{fr.x, fr.y, used.x - fr.x, fr.h};
				if (r.w > 0 && r.h > 0) {
					new_free.push_back(r);
				}
			}
			// Right
			if (used_right < fr_right) {
				Rect r{used_right, fr.y, fr_right - used_right, fr.h};
				if (r.w > 0 && r.h > 0) {
					new_free.push_back(r);
				}
			}
			// Bottom
			if (used.y > fr.y) {
				Rect r{fr.x, fr.y, fr.w, used.y - fr.y};
				if (r.w > 0 && r.h > 0) {
					new_free.push_back(r);
				}
			}
			// Top
			if (used_top < fr_top) {
				Rect r{fr.x, used_top, fr.w, fr_top - used_top};
				if (r.w > 0 && r.h > 0) {
					new_free.push_back(r);
				}
			}
		}

		free_rects_.swap(new_free);
		prune_free_list();
	}

	void prune_free_list() {
		// Remove contained rectangles to keep
		// the free list small and non-redundant.
		for (std::size_t i = 0; i < free_rects_.size(); ++i) {
			for (std::size_t j = i + 1; j < free_rects_.size();) {
				Rect &a = free_rects_[i];
				Rect &b = free_rects_[j];

				if (is_contained_in(b, a)) {
					// b inside a -> erase b
					free_rects_.erase(free_rects_.begin() + j);
				} else if (is_contained_in(a, b)) {
					// a inside b -> erase a
					free_rects_.erase(free_rects_.begin() + i);
					--i;
					break;
				} else {
					++j;
				}
			}
		}
	}
};

// High-level helper: sort rectangles largest-first
// and pack them into a single bin.
export PackResult pack_all(const Bin &bin, std::span<const RectInput> rects,
                           std::span<const Rect> obstacles,
                           Heuristic method = Heuristic::BestShortSideFit,
                           bool allow_rotations = true) {
	MaxRectsBinPack packer(bin.width, bin.height, allow_rotations);
	for (const auto &obs : obstacles) {
		packer.add_obstacle(obs.x, obs.y, obs.w, obs.h);
	}

	std::vector<RectInput> sorted(rects.begin(), rects.end());
	// Largest-area-first ordering.
	std::ranges::sort(sorted, [](const RectInput &a, const RectInput &b) {
		return a.area() > b.area();
	});

	PackResult result;
	result.bin_width = bin.width;
	result.bin_height = bin.height;

	for (const auto &r : sorted) {
		auto placed = packer.insert(r.width, r.height, r.id, method);
		if (placed) {
			result.packed.push_back(*placed);
		} else {
			result.not_packed_ids.push_back(r.id);
		}
	}

	return result;
}

export PackResult pack_all(const Bin &bin, std::span<const RectInput> rects,
                           Heuristic method = Heuristic::BestShortSideFit,
                           bool allow_rotations = true) {
	return pack_all(bin, rects, {}, method, allow_rotations);
}

} // namespace bricksim::pack2d
