export module lego_assemble.core.specs;

import std;
import lego_assemble.utils.type_list;
import lego_assemble.utils.transforms;
import lego_assemble.utils.ranges;
import lego_assemble.utils.bbox;
import lego_assemble.vendor;

namespace lego_assemble {

// ==== General Lego Parts ====

export using BrickUnit = int;
export constexpr double BrickUnitLength = 0.008; // 8 mm per stud

export using PlateUnit = int;
export constexpr double PlateUnitHeight = 0.0032;   // 3.2 mm per plate
export constexpr BrickUnit BrickHeightPerPlate = 3; // 1 brick height = 3 plates

export constexpr double StudDiameter = 0.0048; // 4.8 mm stud diameter
export constexpr double StudHeight = 0.0017;   // 1.7 mm stud height

// ==== Definition of Interface ====

export using InterfaceId = std::uint32_t;

export enum class InterfaceType : std::uint8_t {
	Stud = 0,
	Hole = 1,
};

export struct InterfaceSpec {
	// The studs/holes are distributed on a rectangular grid.
	// The origin of the interface is at min-coordinate corner.
	// For studs, the z-axis is pointing outward from the surface.
	// For holes, the z-axis is pointing inward to the surface.
	// The centers of the studs/holes are offset by half a brick unit,
	// i.e. in the middle of the grid cells.
	InterfaceId id{};
	InterfaceType type{};
	BrickUnit L{}, W{};
	Transformd pose{};
};

// ==== Definition of Part ====

export using BrickColor = std::array<std::uint8_t, 3>; // RGB

export template <class P>
concept PartLike = requires(const P &p, InterfaceId ifid) {
	{ p.mass() } -> std::convertible_to<double>;
	{ p.color() } -> std::convertible_to<BrickColor>;
	{ p.bbox() } -> std::convertible_to<BBox3d>;
	{
		p.get_interface(ifid)
	} -> std::convertible_to<std::optional<InterfaceSpec>>;
	{ p.interfaces() } -> range_of<InterfaceSpec>;
} && std::equality_comparable<P>;

export template <class... Ps>
    requires((PartLike<Ps> && ...) && unique_types<Ps...> && sizeof...(Ps) > 0)
using PartList = type_list<Ps...>;

// ==== Definition of Brick ====

export constexpr double brickMassInKg(BrickUnit L, BrickUnit W, PlateUnit H) {
	// Experimental formula to fit mass of arbitrary bricks
	int bricks = H / 3;
	int rem = H % 3;
	double m_brick = 0.1523 * (L * W) + 0.2498 * (L + W) - 0.2485;
	double m_plate = 0.10937 * (L * W) + 0.05671 * (L + W) - 0.02207;
	return (bricks * m_brick + rem * m_plate) / 1000;
}

export class BrickPart {
  public:
	static constexpr InterfaceId HoleId = 0;
	static constexpr InterfaceId StudId = 1;

	BrickPart(BrickUnit L, BrickUnit W, PlateUnit H, BrickColor color)
	    : L_(L), W_(W), H_(H), color_(color) {}

	BrickUnit L() const {
		return L_;
	}
	BrickUnit W() const {
		return W_;
	}
	PlateUnit H() const {
		return H_;
	}
	double mass() const {
		return brickMassInKg(L_, W_, H_);
	}
	BrickColor color() const {
		return color_;
	}
	InterfaceSpec hole_interface() const {
		return {
		    .id = HoleId,
		    .type = InterfaceType::Hole,
		    .L = L_,
		    .W = W_,
		    .pose = {{1.0, 0.0, 0.0, 0.0},
		             {
		                 -((L_ * BrickUnitLength) / 2.0),
		                 -((W_ * BrickUnitLength) / 2.0),
		                 0.0,
		             }},
		};
	}
	InterfaceSpec stud_interface() const {
		return {
		    .id = StudId,
		    .type = InterfaceType::Stud,
		    .L = L_,
		    .W = W_,
		    .pose = {{1.0, 0.0, 0.0, 0.0},
		             {
		                 -((L_ * BrickUnitLength) / 2.0),
		                 -((W_ * BrickUnitLength) / 2.0),
		                 H_ * PlateUnitHeight,
		             }},
		};
	}
	std::optional<InterfaceSpec> get_interface(InterfaceId ifid) const {
		if (ifid == HoleId) {
			return hole_interface();
		} else if (ifid == StudId) {
			return stud_interface();
		} else {
			return std::nullopt;
		}
	}
	std::array<InterfaceSpec, 2> interfaces() const {
		return {hole_interface(), stud_interface()};
	}
	BBox3d bbox() const {
		return BBox3d{.min =
		                  Eigen::Vector3d{
		                      -((L_ * BrickUnitLength) / 2.0),
		                      -((W_ * BrickUnitLength) / 2.0),
		                      0.0,
		                  },
		              .max = Eigen::Vector3d{
		                  (L_ * BrickUnitLength) / 2.0,
		                  (W_ * BrickUnitLength) / 2.0,
		                  H_ * PlateUnitHeight + StudHeight,
		              }};
	}

	bool operator==(const BrickPart &other) const = default;

  private:
	BrickUnit L_;
	BrickUnit W_;
	PlateUnit H_;
	BrickColor color_;
};
static_assert(PartLike<BrickPart>);

// ==== Definition of Custom Part ====

export struct CustomPart {
  public:
	CustomPart(double mass, BrickColor color,
	           std::initializer_list<InterfaceSpec> ifs, BBox3d bbox,
	           std::pmr::memory_resource *r = std::pmr::get_default_resource())
	    : mass_{mass}, color_{color}, interfaces_{ifs, r},
	      bbox_{std::move(bbox)} {}

	CustomPart(double mass, BrickColor color,
	           std::initializer_list<InterfaceSpec> ifs,
	           std::pmr::memory_resource *r = std::pmr::get_default_resource())
	    : mass_{mass}, color_{color}, interfaces_{ifs, r},
	      bbox_{
	          .min = {0.0, 0.0, 0.0},
	          .max = {0.0, 0.0, 0.0},
	      } {}

	double mass() const {
		return mass_;
	}
	BrickColor color() const {
		return color_;
	}
	std::span<const InterfaceSpec> interfaces() const {
		return interfaces_;
	}
	std::optional<InterfaceSpec> get_interface(InterfaceId id) const {
		// Linear search because number of interfaces is usually small
		for (const auto &iface : interfaces_) {
			if (iface.id == id) {
				return iface;
			}
		}
		return std::nullopt;
	}
	BBox3d bbox() const {
		return bbox_;
	}

	bool operator==(const CustomPart &other) const = default;

  private:
	double mass_;
	BrickColor color_;
	std::pmr::vector<InterfaceSpec> interfaces_;
	BBox3d bbox_;
};
static_assert(PartLike<CustomPart>);

} // namespace lego_assemble
