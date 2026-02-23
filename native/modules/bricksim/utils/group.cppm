export module bricksim.utils.group;

import std;

namespace bricksim {

export template <typename G>
concept GroupLike =
    requires(G g, const G::ElementType &e1, const G::ElementType &e2) {
	    { g.identity() } -> std::same_as<typename G::ElementType>;
	    { g.combine(e1, e2) } -> std::same_as<typename G::ElementType>;
	    { g.invert(e1) } -> std::same_as<typename G::ElementType>;
    };

export template <typename G>
concept MetricGroupLike =
    GroupLike<G> &&
    requires(G g, const G::ElementType &e1, const G::ElementType &e2) {
	    { g.distance(e1, e2) } -> std::convertible_to<double>;
	    { g.almost_equal(e1, e2) } -> std::convertible_to<bool>;
    };

export template <class G>
concept ProjectableGroupLike =
    GroupLike<G> && requires(G g, const G::ElementType &e) {
	    { g.project(e) } -> std::same_as<typename G::ElementType>;
    };

} // namespace bricksim
