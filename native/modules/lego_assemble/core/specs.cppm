export module lego_assemble.core.specs;

import std;
import lego_assemble.utils.type_list;
import lego_assemble.utils.transforms;
import lego_assemble.vendor.eigen;

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

export using InterfaceId = std::size_t;

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
concept PartLike = requires(const P &p) {
	{ p.mass() } -> std::convertible_to<double>;
	{ p.color() } -> std::convertible_to<BrickColor>;
};

export template <class... Ps>
    requires((PartLike<Ps> && ...) && unique_types<Ps...> && sizeof...(Ps) > 0)
using PartList = type_list<Ps...>;

// ==== Part with Fixed Interfaces ====

export template <InterfaceId... Ids>
using InterfaceIdList = std::integer_sequence<InterfaceId, Ids...>;

export template <class U> constexpr bool IsInterfaceIdList = false;

export template <InterfaceId... Ids>
constexpr bool IsInterfaceIdList<InterfaceIdList<Ids...>> = true;

export template <class P, class Ids> constexpr bool HasInterfaceAt = false;

export template <class P, InterfaceId... Ids>
constexpr bool HasInterfaceAt<P, InterfaceIdList<Ids...>> =
    (requires(const P &p) {
	    {
		    p.template interface_at<Ids>()
	    } -> std::convertible_to<const InterfaceSpec &>;
    } && ...);

export template <class P>
concept PartWithFixedInterfaces =
    PartLike<P> && IsInterfaceIdList<typename P::InterfaceIds> &&
    HasInterfaceAt<P, typename P::InterfaceIds>;

// ==== Part with Dynamic Interfaces ====

export template <class R>
concept InterfaceRange =
    std::ranges::input_range<R> &&
    std::same_as<std::ranges::range_value_t<R>, InterfaceSpec>;

export template <class P>
concept PartWithDynamicInterfaces = PartLike<P> && requires(const P &p) {
	{ p.interfaces() } -> InterfaceRange;
	{
		p.interface_at(InterfaceId{})
	} -> std::convertible_to<const InterfaceSpec *>;
};

// ==== Utility Functions ====

export template <PartWithFixedInterfaces P,
                 std::invocable<const InterfaceSpec &> Func>
void for_each_interface(const P &part, Func &&func) {
	[&]<InterfaceId... Ids>(InterfaceIdList<Ids...>) {
		(std::invoke(func, part.template interface_at<Ids>()), ...);
	}(typename P::InterfaceIds{});
}

export template <PartWithDynamicInterfaces P,
                 std::invocable<const InterfaceSpec &> Func>
    requires(!PartWithFixedInterfaces<P>)
void for_each_interface(const P &part, Func &&func) {
	for (const auto &iface : part.interfaces()) {
		std::invoke(func, iface);
	}
}

export template <class P>
std::optional<InterfaceSpec> get_interface_at(const P &part, InterfaceId id)
    requires(PartWithFixedInterfaces<P> || PartWithDynamicInterfaces<P>)
{
	if constexpr (PartWithFixedInterfaces<P>) {
		std::optional<InterfaceSpec> out;
		// Expand the fixed ID list, pick the matching one at runtime.
		[&]<InterfaceId... Is>(InterfaceIdList<Is...>) {
			((id == Is ? (out = part.template interface_at<Is>(), true)
			           : false) ||
			 ...);
		}(typename P::InterfaceIds{});
		return out;
	} else {
		// Dynamic case
		if (const InterfaceSpec *s = part.interface_at(id)) {
			return *s; // copies into the optional
		}
		return std::nullopt;
	}
}

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
	using InterfaceIds = InterfaceIdList<0, 1>;

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
	template <InterfaceId Id> InterfaceSpec interface_at() const {
		if constexpr (Id == 0) {
			return {.id = 0,
			        .type = InterfaceType::Hole,
			        .L = L_,
			        .W = W_,
			        .pose = {{1.0, 0.0, 0.0, 0.0},
			                 {
			                     -((L_ * BrickUnitLength) / 2.0),
			                     -((W_ * BrickUnitLength) / 2.0),
			                     0.0,
			                 }}};
		} else if constexpr (Id == 1) {
			return {.id = 1,
			        .type = InterfaceType::Stud,
			        .L = L_,
			        .W = W_,
			        .pose = {{1.0, 0.0, 0.0, 0.0},
			                 {
			                     -((L_ * BrickUnitLength) / 2.0),
			                     -((W_ * BrickUnitLength) / 2.0),
			                     H_ * PlateUnitHeight,
			                 }}};
		} else {
			static_assert(Id < 2, "Invalid interface id");
		}
	}

  private:
	BrickUnit L_;
	BrickUnit W_;
	PlateUnit H_;
	BrickColor color_;
};
static_assert(PartWithFixedInterfaces<BrickPart>);

// ==== Definition of Custom Part ====

export struct CustomPart {
  public:
	CustomPart(double mass, BrickColor color,
	           std::initializer_list<InterfaceSpec> ifs,
	           std::pmr::memory_resource *r = std::pmr::get_default_resource())
	    : mass_{mass}, color_{color}, interfaces_{ifs, r} {}

	double mass() const {
		return mass_;
	}
	BrickColor color() const {
		return color_;
	}
	std::span<const InterfaceSpec> interfaces() const {
		return interfaces_;
	}
	const InterfaceSpec *interface_at(InterfaceId id) const {
		// Linear search because number of interfaces is usually small
		for (const auto &iface : interfaces_) {
			if (iface.id == id) {
				return &iface;
			}
		}
		return nullptr;
	}

  private:
	double mass_;
	BrickColor color_;
	std::pmr::vector<InterfaceSpec> interfaces_;
};
static_assert(PartWithDynamicInterfaces<CustomPart>);

} // namespace lego_assemble
