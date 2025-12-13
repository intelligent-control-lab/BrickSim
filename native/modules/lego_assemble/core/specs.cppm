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

// ==== Definition of Face ====

export using FaceId = std::uint32_t;

export template <class T>
concept FaceSpecLike = requires(const T &s) {
	{ s.id() } -> std::convertible_to<FaceId>;
	// Vertices must be CCW ordered
	{ s.polygon_vertices() } -> range_of<Eigen::Vector2d>;
	{ s.bbox() } -> std::convertible_to<BBox2d>;
	// +z is the outward normal
	{ s.transform() } -> std::convertible_to<Transformd>;
} && std::equality_comparable<T>;

export template <class R>
concept FaceSpecRange =
    std::ranges::range<R> && FaceSpecLike<std::ranges::range_value_t<R>>;

export struct RectFaceSpec {
  public:
	RectFaceSpec(FaceId id, double L, double W, const Transformd &tf)
	    : id_{id}, L_{L}, W_{W}, tf_{tf} {}

	FaceId id() const {
		return id_;
	}
	Eigen::Vector2d bottom_left() const {
		return Eigen::Vector2d{-L_ / 2.0, -W_ / 2.0};
	}
	Eigen::Vector2d bottom_right() const {
		return Eigen::Vector2d{L_ / 2.0, -W_ / 2.0};
	}
	Eigen::Vector2d top_right() const {
		return Eigen::Vector2d{L_ / 2.0, W_ / 2.0};
	}
	Eigen::Vector2d top_left() const {
		return Eigen::Vector2d{-L_ / 2.0, W_ / 2.0};
	}
	std::array<Eigen::Vector2d, 4> polygon_vertices() const {
		return {bottom_left(), bottom_right(), top_right(), top_left()};
	}
	BBox2d bbox() const {
		return BBox2d{.min = bottom_left(), .max = top_right()};
	}
	const Transformd &transform() const {
		return tf_;
	}
	bool operator==(const RectFaceSpec &other) const = default;

  private:
	FaceId id_;
	double L_, W_;
	Transformd tf_;
};
static_assert(FaceSpecLike<RectFaceSpec>);

export struct CustomFaceSpec {
  public:
	CustomFaceSpec(
	    FaceId id, std::initializer_list<Eigen::Vector2d> vertices,
	    const Transformd &tf,
	    std::pmr::memory_resource *r = std::pmr::get_default_resource())
	    : id_{id}, vertices_{vertices, r}, tf_{tf},
	      bbox_{BBox2d::from_vertices(vertices_)} {}

	FaceId id() const {
		return id_;
	}
	std::span<const Eigen::Vector2d> polygon_vertices() const {
		return vertices_;
	}
	const BBox2d &bbox() const {
		return bbox_;
	}
	const Transformd &transform() const {
		return tf_;
	}
	bool operator==(const CustomFaceSpec &other) const = default;

  private:
	FaceId id_;
	std::pmr::vector<Eigen::Vector2d> vertices_;
	Transformd tf_;
	BBox2d bbox_;
};
static_assert(FaceSpecLike<CustomFaceSpec>);

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
	{ p.faces() } -> FaceSpecRange;
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

	static constexpr FaceId PosXFaceId = 0;
	static constexpr FaceId NegXFaceId = 1;
	static constexpr FaceId PosYFaceId = 2;
	static constexpr FaceId NegYFaceId = 3;
	static constexpr FaceId PosZFaceId = 4;
	static constexpr FaceId NegZFaceId = 5;

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
		return BBox3d{
		    .min =
		        Eigen::Vector3d{
		            -((L_ * BrickUnitLength) / 2.0),
		            -((W_ * BrickUnitLength) / 2.0),
		            0.0,
		        },
		    .max =
		        Eigen::Vector3d{
		            (L_ * BrickUnitLength) / 2.0,
		            (W_ * BrickUnitLength) / 2.0,
		            H_ * PlateUnitHeight + StudHeight,
		        },
		};
	}
	RectFaceSpec pos_x_face() const {
		return {PosXFaceId,
		        W_ * BrickUnitLength,
		        H_ * PlateUnitHeight,
		        {
		            {0.5, -0.5, 0.5, -0.5},
		            {
		                (L_ * BrickUnitLength) / 2.0,
		                0.0,
		                (H_ * PlateUnitHeight) / 2.0,
		            },
		        }};
	}
	RectFaceSpec neg_x_face() const {
		return {NegXFaceId,
		        W_ * BrickUnitLength,
		        H_ * PlateUnitHeight,
		        {
		            {0.5, 0.5, -0.5, -0.5},
		            {
		                -(L_ * BrickUnitLength) / 2.0,
		                0.0,
		                (H_ * PlateUnitHeight) / 2.0,
		            },
		        }};
	}
	RectFaceSpec pos_y_face() const {
		return {PosYFaceId,
		        L_ * BrickUnitLength,
		        H_ * PlateUnitHeight,
		        {
		            {std::sqrt(0.5), -std::sqrt(0.5), 0.0, 0.0},
		            {
		                0.0,
		                (W_ * BrickUnitLength) / 2.0,
		                (H_ * PlateUnitHeight) / 2.0,
		            },
		        }};
	}
	RectFaceSpec neg_y_face() const {
		return {NegYFaceId,
		        L_ * BrickUnitLength,
		        H_ * PlateUnitHeight,
		        {
		            {std::sqrt(0.5), std::sqrt(0.5), 0.0, 0.0},
		            {
		                0.0,
		                (-W_ * BrickUnitLength) / 2.0,
		                (H_ * PlateUnitHeight) / 2.0,
		            },
		        }};
	}
	RectFaceSpec pos_z_face() const {
		return {PosZFaceId,
		        L_ * BrickUnitLength,
		        W_ * BrickUnitLength,
		        {
		            {1.0, 0.0, 0.0, 0.0},
		            {
		                0.0,
		                0.0,
		                H_ * PlateUnitHeight,
		            },
		        }};
	}
	RectFaceSpec neg_z_face() const {
		return {NegZFaceId,
		        L_ * BrickUnitLength,
		        W_ * BrickUnitLength,
		        {
		            {0.0, 1.0, 0.0, 0.0},
		            {
		                0.0,
		                0.0,
		                0.0,
		            },
		        }};
	}

	std::array<RectFaceSpec, 6> faces() const {
		return {pos_z_face(), neg_z_face(), neg_y_face(),
		        pos_y_face(), neg_x_face(), pos_x_face()};
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
	           std::initializer_list<CustomFaceSpec> faces,
	           std::pmr::memory_resource *r = std::pmr::get_default_resource())
	    : mass_{mass}, color_{color}, interfaces_{ifs, r},
	      bbox_{std::move(bbox)}, faces_{faces, r} {}

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
	std::span<const CustomFaceSpec> faces() const {
		return faces_;
	}

	bool operator==(const CustomPart &other) const = default;

  private:
	double mass_;
	BrickColor color_;
	std::pmr::vector<InterfaceSpec> interfaces_;
	BBox3d bbox_;
	std::pmr::vector<CustomFaceSpec> faces_;
};
static_assert(PartLike<CustomPart>);

} // namespace lego_assemble
