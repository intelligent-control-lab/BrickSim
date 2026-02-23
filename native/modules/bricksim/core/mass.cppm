export module bricksim.core.mass;

import std;

namespace bricksim {

constexpr bool UseRealBrickMass = true;

constexpr double fit_brick_mass(int L, int W, int H) {
	// Experimental formula to fit mass of arbitrary bricks
	int bricks = H / 3;
	int rem = H % 3;
	double m_brick = 0.1523 * (L * W) + 0.2498 * (L + W) - 0.2485;
	double m_plate = 0.10937 * (L * W) + 0.05671 * (L + W) - 0.02207;
	return (bricks * m_brick + rem * m_plate) / 1000;
}

export constexpr double brick_mass_in_kg(int L, int W, int H) {
	if constexpr (UseRealBrickMass) {
		if (H == 3) {
			auto [d1, d2] = std::minmax(L, W);
			if (d1 == 1) {
				if (d2 == 1)
					return 0.00043; // 1x1
				if (d2 == 2)
					return 0.00081; // 1x2
				if (d2 == 4)
					return 0.00157; // 1x4
				if (d2 == 6)
					return 0.00228; // 1x6
				if (d2 == 8)
					return 0.00303; // 1x8
			} else if (d1 == 2) {
				if (d2 == 2)
					return 0.00115; // 2x2
				if (d2 == 4)
					return 0.00216; // 2x4
				if (d2 == 6)
					return 0.00323; // 2x6
			}
		}
	}
	return fit_brick_mass(L, W, H);
}

} // namespace bricksim
