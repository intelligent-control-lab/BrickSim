export module bricksim.core.specs;

import std;
import bricksim.core.mass;
import bricksim.utils.type_list;
import bricksim.utils.transforms;
import bricksim.utils.concepts;
import bricksim.utils.bbox;
import bricksim.vendor;

namespace bricksim {

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

export template <class T>
concept FaceSpecOptional =
    is_optional_v<T> && FaceSpecLike<optional_value_t<T>>;

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
	CustomFaceSpec(FaceId id, std::initializer_list<Eigen::Vector2d> vertices,
	               const Transformd &tf)
	    : id_{id}, vertices_{vertices}, tf_{tf},
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
	std::vector<Eigen::Vector2d> vertices_;
	Transformd tf_;
	BBox2d bbox_;
};
static_assert(FaceSpecLike<CustomFaceSpec>);

// ==== Definition of Part ====

export using BrickColor = std::array<std::uint8_t, 3>; // RGB

export template <class P>
concept PartLike = requires(const P &p, InterfaceId ifid, FaceId fid) {
	{ p.mass() } -> std::convertible_to<double>;
	{ p.color() } -> std::convertible_to<BrickColor>;
	{ p.bbox() } -> std::convertible_to<BBox3d>;
	{
		p.get_interface(ifid)
	} -> std::convertible_to<std::optional<InterfaceSpec>>;
	{ p.interfaces() } -> range_of<InterfaceSpec>;
	{ p.get_face(fid) } -> FaceSpecOptional;
	{ p.faces() } -> FaceSpecRange;
	{ p.com() } -> std::convertible_to<Eigen::Vector3d>;
	{ p.inertia_tensor() } -> std::convertible_to<Eigen::Matrix3d>;
} && std::equality_comparable<P>;

export template <class... Ps>
    requires((PartLike<Ps> && ...) && unique_types<Ps...> && sizeof...(Ps) > 0)
using PartList = type_list<Ps...>;

// ==== Definition of Brick ====

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
		return brick_mass_in_kg(L_, W_, H_);
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

	std::optional<RectFaceSpec> get_face(FaceId fid) const {
		switch (fid) {
		case PosXFaceId:
			return pos_x_face();
		case NegXFaceId:
			return neg_x_face();
		case PosYFaceId:
			return pos_y_face();
		case NegYFaceId:
			return neg_y_face();
		case PosZFaceId:
			return pos_z_face();
		case NegZFaceId:
			return neg_z_face();
		default:
			return std::nullopt;
		}
	}
	std::array<RectFaceSpec, 6> faces() const {
		return {pos_z_face(), neg_z_face(), neg_y_face(),
		        pos_y_face(), neg_x_face(), pos_x_face()};
	}

	Eigen::Vector3d com() const {
		return {0.0, 0.0, H_ * PlateUnitHeight / 2.0};
	}

	Eigen::Matrix3d inertia_tensor() const {
		double m = this->mass();
		double lx = L_ * BrickUnitLength;
		double ly = W_ * BrickUnitLength;
		double lz = H_ * PlateUnitHeight;
		double Ixx = (1.0 / 12.0) * m * (ly * ly + lz * lz);
		double Iyy = (1.0 / 12.0) * m * (lx * lx + lz * lz);
		double Izz = (1.0 / 12.0) * m * (lx * lx + ly * ly);
		return Eigen::Vector3d{Ixx, Iyy, Izz}.asDiagonal();
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
	CustomPart(double mass, const BrickColor &color,
	           std::initializer_list<InterfaceSpec> ifs, const BBox3d &bbox,
	           std::initializer_list<CustomFaceSpec> faces,
	           const Eigen::Vector3d &com,
	           const Eigen::Matrix3d &inertia_tensor)
	    : mass_{mass}, color_{color}, interfaces_{ifs}, bbox_{bbox},
	      faces_{faces}, com_{com}, inertia_tensor_{inertia_tensor} {}

	CustomPart(double mass, BrickColor color,
	           std::initializer_list<InterfaceSpec> ifs)
	    : mass_{mass}, color_{color}, interfaces_{ifs},
	      bbox_{
	          .min = {0.0, 0.0, 0.0},
	          .max = {0.0, 0.0, 0.0},
	      } {}

	double mass() const {
		return mass_;
	}
	const BrickColor &color() const {
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
	const BBox3d &bbox() const {
		return bbox_;
	}
	std::optional<CustomFaceSpec> get_face(FaceId fid) const {
		// Linear search because number of faces is usually small
		for (const auto &face : faces_) {
			if (face.id() == fid) {
				return face;
			}
		}
		return std::nullopt;
	}
	std::span<const CustomFaceSpec> faces() const {
		return faces_;
	}
	const Eigen::Vector3d &com() const {
		return com_;
	}
	const Eigen::Matrix3d &inertia_tensor() const {
		return inertia_tensor_;
	}

	bool operator==(const CustomPart &other) const = default;

  private:
	double mass_;
	BrickColor color_;
	std::vector<InterfaceSpec> interfaces_;
	BBox3d bbox_;
	std::vector<CustomFaceSpec> faces_;
	Eigen::Vector3d com_;
	Eigen::Matrix3d inertia_tensor_;
};
static_assert(PartLike<CustomPart>);

} // namespace bricksim
