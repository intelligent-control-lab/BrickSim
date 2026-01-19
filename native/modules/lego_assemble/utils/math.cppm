export module lego_assemble.utils.math;

import std;
import lego_assemble.vendor;

namespace lego_assemble {
    
export double cross2(const Eigen::Vector2d &a, const Eigen::Vector2d &b) {
	return a.x() * b.y() - a.y() * b.x();
}

export double cross2(const Eigen::Vector2d &O, const Eigen::Vector2d &A,
                     const Eigen::Vector2d &B) {
	return (A.x() - O.x()) * (B.y() - O.y()) -
	       (A.y() - O.y()) * (B.x() - O.x());
};

} // namespace lego_assemble
