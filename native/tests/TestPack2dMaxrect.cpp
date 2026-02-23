import std;
import bricksim.utils.pack2d_maxrect;

#include <cassert>

using namespace bricksim::pack2d;

namespace {

template <typename R1, typename R2> bool intersects(const R1 &a, const R2 &b) {
	const auto ax1 = a.x;
	const auto ay1 = a.y;
	const auto ax2 = a.x + a.width;
	const auto ay2 = a.y + a.height;

	const auto bx1 = b.x;
	const auto by1 = b.y;
	const auto bx2 = b.x + b.width;
	const auto by2 = b.y + b.height;

	return !(ax2 <= bx1 || ax1 >= bx2 || ay2 <= by1 || ay1 >= by2);
}

void check_no_overlap(const Bin &bin, const PackResult &res) {
	// Each rect within bin
	for (const auto &r : res.packed) {

		assert(r.x >= 0);
		assert(r.y >= 0);
		assert(r.width > 0);
		assert(r.height > 0);
		assert(r.x + r.width <= bin.width);
		assert(r.y + r.height <= bin.height);
	}

	// No pair overlaps
	for (std::size_t i = 0; i < res.packed.size(); ++i) {
		for (std::size_t j = i + 1; j < res.packed.size(); ++j) {
			assert(!intersects(res.packed[i], res.packed[j]));
		}
	}
}

void test_basic_2x2_in_4x4() {
	Bin bin{4, 4};
	RectInput rs[] = {{0, 2, 2}, {1, 2, 2}, {2, 2, 2}, {3, 2, 2}};

	auto res = pack_all(bin, std::span<const RectInput>(rs, 4),
	                    Heuristic::BestShortSideFit,
	                    /*allow_rotations=*/false);

	assert(res.packed.size() == 4);
	assert(res.not_packed_ids.empty());
	check_no_overlap(bin, res);
	// 4 * 4 area, 4 * (2*2) used => occupancy 1.0
	assert(res.occupancy() > 0.99);
}

void test_rotation_fit() {
	Bin bin{5, 3};
	RectInput rs[] = {{42, 3, 5}};

	auto res = pack_all(bin, std::span<const RectInput>(rs, 1),
	                    Heuristic::BestShortSideFit,
	                    /*allow_rotations=*/true);

	assert(res.packed.size() == 1);
	assert(res.not_packed_ids.empty());
	const auto &p = res.packed.front();
	assert(p.id == 42);
	assert(p.rotated); // we expect rotation
	assert(p.width == 5);
	assert(p.height == 3);
	check_no_overlap(bin, res);
}

void test_impossible_case() {
	Bin bin{5, 5};
	// Three 3x3 squares: total area 27 > 25 => impossible.
	RectInput rs[] = {{0, 3, 3}, {1, 3, 3}, {2, 3, 3}};

	auto res =
	    pack_all(bin, std::span<const RectInput>(rs, 3), Heuristic::BestAreaFit,
	             /*allow_rotations=*/false);

	assert(res.packed.size() <= 2);
	assert(!res.not_packed_ids.empty());
	check_no_overlap(bin, res);
}

void test_bottom_left_style() {
	Bin bin{10, 10};
	RectInput rs[] = {{0, 5, 5}, {1, 5, 5}, {2, 2, 2}};

	auto res =
	    pack_all(bin, std::span<const RectInput>(rs, 3), Heuristic::BottomLeft,
	             /*allow_rotations=*/false);

	// All should fit easily
	assert(res.packed.size() == 3);
	check_no_overlap(bin, res);
}

} // namespace

int main() {
	test_basic_2x2_in_4x4();
	test_rotation_fit();
	test_impossible_case();
	test_bottom_left_style();

	std::cout << "All pack2d tests passed.\n";
}
