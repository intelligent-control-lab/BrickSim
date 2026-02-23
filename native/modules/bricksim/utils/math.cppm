export module bricksim.utils.math;

import std;
import bricksim.vendor;

namespace bricksim {
    
export double cross2(const Eigen::Vector2d &a, const Eigen::Vector2d &b) {
	return a.x() * b.y() - a.y() * b.x();
}

export double cross2(const Eigen::Vector2d &O, const Eigen::Vector2d &A,
                     const Eigen::Vector2d &B) {
	return (A.x() - O.x()) * (B.y() - O.y()) -
	       (A.y() - O.y()) * (B.x() - O.x());
};

} // namespace bricksim
