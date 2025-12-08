import std;
import lego_assemble.core.specs;
import lego_assemble.core.connections;
import lego_assemble.core.graph;
import lego_assemble.io.topology;
import lego_assemble.usd.usd_graph;
import lego_assemble.usd.allocator;
import lego_assemble.usd.author;
import lego_assemble.usd.parse;
import lego_assemble.utils.type_list;
import lego_assemble.utils.transforms;
import lego_assemble.utils.usd_envs;
import lego_assemble.utils.conversions;
import lego_assemble.vendor;

#include <cassert>

using namespace lego_assemble;

namespace {

using nlohmann::ordered_json;

static bool almost_equal_array3(const std::array<double, 3> &a,
                                const std::array<double, 3> &b,
                                double eps = 1e-9) {
	for (int i = 0; i < 3; ++i) {
		if (std::abs(a[i] - b[i]) > eps) {
			return false;
		}
	}
	return true;
}

static bool almost_equal_array4(const std::array<double, 4> &a,
                                const std::array<double, 4> &b,
                                double eps = 1e-9) {
	for (int i = 0; i < 4; ++i) {
		if (std::abs(a[i] - b[i]) > eps) {
			return false;
		}
	}
	return true;
}

// -------------------- basic struct JSON roundtrips --------------------

static void test_brick_serializer_roundtrip() {
	BrickColor color{255, 128, 0};
	BrickPart brick{2, 4, BrickHeightPerPlate, color};

	BrickSerializer ser;
	ordered_json j = ser.to_json(brick);

	assert(j.at("L").get<BrickUnit>() == 2);
	assert(j.at("W").get<BrickUnit>() == 4);
	assert(j.at("H").get<PlateUnit>() == BrickHeightPerPlate);
	auto color_j = j.at("color").get<BrickColor>();
	assert(color_j == color);

	BrickPart restored = ser.from_json(j);
	assert(restored == brick);
}

static void test_json_part_roundtrip() {
	JsonPart p{
	    .id = 42,
	    .type = "custom_type",
	    .payload = ordered_json{{"foo", 1}, {"bar", "baz"}},
	};

	ordered_json j;
	to_json(j, p);

	assert(j.at("id").get<std::int64_t>() == 42);
	assert(j.at("type").get<std::string>() == "custom_type");
	assert(j.at("payload") == p.payload);

	JsonPart q{};
	from_json(j, q);

	assert(q.id == p.id);
	assert(q.type == p.type);
	assert(q.payload == p.payload);
}

static void test_json_connection_roundtrip() {
	JsonConnection c{
	    .stud_id = 1,
	    .stud_iface = 10,
	    .hole_id = 2,
	    .hole_iface = 20,
	    .offset = {3, -4},
	    .yaw = 1,
	};

	ordered_json j;
	to_json(j, c);

	assert(j.at("stud_id").get<std::int64_t>() == c.stud_id);
	assert(j.at("stud_iface").get<std::int64_t>() == c.stud_iface);
	assert(j.at("hole_id").get<std::int64_t>() == c.hole_id);
	assert(j.at("hole_iface").get<std::int64_t>() == c.hole_iface);
	auto off = j.at("offset").get<std::array<std::int64_t, 2>>();
	assert(off == c.offset);
	assert(j.at("yaw").get<std::int64_t>() == c.yaw);

	JsonConnection d{};
	from_json(j, d);

	assert(d.stud_id == c.stud_id);
	assert(d.stud_iface == c.stud_iface);
	assert(d.hole_id == c.hole_id);
	assert(d.hole_iface == c.hole_iface);
	assert(d.offset == c.offset);
	assert(d.yaw == c.yaw);
}

static void test_json_pose_hint_roundtrip() {
	JsonPoseHint h{
	    .part = 7,
	    .pos = {1.0, -2.0, 3.5},
	    .rot = {1.0, 0.0, 0.0, 0.0}, // identity quaternion wxyz
	};

	ordered_json j;
	to_json(j, h);

	assert(j.at("part").get<std::int64_t>() == h.part);
	auto pos = j.at("pos").get<std::array<double, 3>>();
	auto rot = j.at("rot").get<std::array<double, 4>>();
	assert(pos == h.pos);
	assert(rot == h.rot);

	JsonPoseHint k{};
	from_json(j, k);

	assert(k.part == h.part);
	assert(k.pos == h.pos);
	assert(k.rot == h.rot);
}

static void test_json_topology_roundtrip_and_optional_fields() {
	JsonPart part{
	    .id = 1,
	    .type = "brick",
	    .payload = ordered_json{{"L", 2}, {"W", 4}, {"H", 3}},
	};
	JsonConnection conn{
	    .stud_id = 1,
	    .stud_iface = 10,
	    .hole_id = 2,
	    .hole_iface = 20,
	    .offset = {0, 0},
	    .yaw = 0,
	};
	JsonPoseHint hint{
	    .part = 1,
	    .pos = {0.0, 0.0, 0.0},
	    .rot = {1.0, 0.0, 0.0, 0.0},
	};

	JsonTopology topo;
	topo.parts.push_back(part);
	topo.connections.push_back(conn);
	topo.pose_hints.push_back(hint);

	ordered_json j;
	to_json(j, topo);

	assert(j.at("schema").get<std::string>() ==
	       "lego_assemble/lego_topology@1");
	assert(j.at("parts").size() == 1);
	assert(j.at("connections").size() == 1);
	assert(j.at("pose_hints").size() == 1);

	JsonTopology topo2{};
	from_json(j, topo2);

	assert(topo2.parts.size() == 1);
	assert(topo2.connections.size() == 1);
	assert(topo2.pose_hints.size() == 1);
	assert(topo2.parts[0].id == part.id);
	assert(topo2.parts[0].type == part.type);
	assert(topo2.parts[0].payload == part.payload);
	assert(topo2.connections[0].stud_id == conn.stud_id);
	assert(topo2.connections[0].hole_id == conn.hole_id);
	assert(topo2.pose_hints[0].part == hint.part);

	// Missing fields must be treated as empty vectors.
	ordered_json j_partial = ordered_json::object();
	j_partial["schema"] = "lego_assemble/lego_topology@1";
	j_partial["parts"] = ordered_json::array(
	    {ordered_json{{"id", 2}, {"type", "other"}, {"payload", {}}}});

	JsonTopology topo3{};
	from_json(j_partial, topo3);
	assert(topo3.parts.size() == 1);
	assert(topo3.connections.empty());
	assert(topo3.pose_hints.empty());

	// Object with only schema: all optional vectors remain empty.
	ordered_json j_empty = ordered_json::object();
	j_empty["schema"] = "lego_assemble/lego_topology@1";
	JsonTopology topo4{};
	from_json(j_empty, topo4);
	assert(topo4.parts.empty());
	assert(topo4.connections.empty());
	assert(topo4.pose_hints.empty());
}

// -------------------- TopologySerializer with LegoGraph --------------------

using PartsGraph = PartList<BrickPart>;
using GraphG = LegoGraph<PartsGraph>;

static BrickPart make_brick(BrickUnit L, BrickUnit W, PlateUnit H,
                            BrickColor color) {
	return BrickPart(L, W, H, color);
}

static void build_simple_brick_graph(GraphG &g) {
	BrickColor red{255, 0, 0};
	BrickColor green{0, 255, 0};

	// Two bricks with a single connection between them.
	auto pid0 =
	    g.add_part<BrickPart>(std::tuple<>{}, 2, 4, BrickHeightPerPlate, red);
	auto pid1 =
	    g.add_part<BrickPart>(std::tuple<>{}, 2, 4, BrickHeightPerPlate, green);
	assert(pid0.has_value() && pid1.has_value());

	ConnectionSegment cs{}; // default offset (0,0), yaw=0
	InterfaceRef stud{*pid0, BrickPart::StudId};
	InterfaceRef hole{*pid1, BrickPart::HoleId};
	assert(g.connect(stud, hole, std::tuple<>{}, cs));
}

static void test_topology_serializer_export_graph_default_filters() {
	GraphG g;
	build_simple_brick_graph(g);

	TopologySerializer<BrickSerializer> ser;
	JsonTopology topo = ser.export_graph(g);

	assert(topo.parts.size() == 2);
	assert(topo.connections.size() == 1);
	assert(topo.pose_hints.empty());

	// Parts: ids 0 and 1, type string "brick" and payload matching BrickSerializer.
	std::unordered_map<std::int64_t, JsonPart> parts_by_id;
	for (const auto &p : topo.parts) {
		parts_by_id.emplace(p.id, p);
	}
	assert(parts_by_id.size() == 2);

	for (PartId pid : {PartId{0}, PartId{1}}) {
		auto it = parts_by_id.find(static_cast<std::int64_t>(pid));
		assert(it != parts_by_id.end());
		const JsonPart &jp = it->second;
		assert(jp.type == "brick");

		using PW = SimplePartWrapper<BrickPart>;
		const PW *pw = g.parts().get<PW>(pid);
		assert(pw);
		const BrickPart &bp = pw->wrapped();
		ordered_json expected = BrickSerializer{}.to_json(bp);
		assert(jp.payload == expected);
	}

	const JsonConnection &jc = topo.connections[0];
	assert(jc.stud_id == 0);
	assert(jc.hole_id == 1);
	assert(jc.stud_iface == static_cast<std::int64_t>(BrickPart::StudId));
	assert(jc.hole_iface == static_cast<std::int64_t>(BrickPart::HoleId));
	const std::array<std::int64_t, 2> expected_offset{0, 0};
	assert(jc.offset == expected_offset);
	assert(jc.yaw == 0);
}

static void test_topology_serializer_export_graph_with_filters() {
	GraphG g;
	build_simple_brick_graph(g);

	TopologySerializer<BrickSerializer> ser;

	// Keep only part 0, and drop all connections.
	JsonTopology topo = ser.export_graph(
	    g, [](PartId pid) { return pid == PartId{0}; },
	    [](ConnSegId) { return false; });

	assert(topo.parts.size() == 1);
	assert(topo.connections.empty());
	assert(topo.pose_hints.empty());
	assert(topo.parts[0].id == 0);
}

// -------------------- TopologySerializer with UsdLegoGraph --------------------

using PartsUsd = PartList<BrickPart>;
using PartAuthors = type_list<PrototypeBrickAuthor>;
using PartParsers = type_list<BrickParser>;
using UsdGraphG = UsdLegoGraph<PartsUsd, PartAuthors, PartParsers>;

static pxr::UsdStageRefPtr make_stage() {
	pxr::UsdStageRefPtr stage = pxr::UsdStage::CreateInMemory();
	// create default /World prim
	auto world_prim = pxr::SdfCreatePrimInLayer(
	    stage->GetEditTarget().GetLayer(), pxr::SdfPath("/World"));
	world_prim->SetSpecifier(pxr::SdfSpecifierDef);
	world_prim->SetTypeName(pxr::UsdGeomTokens->Xform);
	return stage;
}

static void create_env_root(const pxr::UsdStageRefPtr &stage,
                            std::int64_t env_id) {
	if (env_id == kNoEnv) {
		return;
	}
	auto layer = stage->GetEditTarget().GetLayer();
	pxr::SdfPath EnvRootPath("/World/envs");
	if (!layer->GetPrimAtPath(EnvRootPath)) {
		auto class_root = pxr::SdfCreatePrimInLayer(layer, EnvRootPath);
		class_root->SetSpecifier(pxr::SdfSpecifierDef);
		class_root->SetTypeName(pxr::UsdGeomTokens->Scope);
	}
	auto env_name = std::format("env_{}", env_id);
	auto env_path = EnvRootPath.AppendChild(pxr::TfToken(env_name));
	if (!layer->GetPrimAtPath(env_path)) {
		auto env_prim = pxr::SdfCreatePrimInLayer(layer, env_path);
		env_prim->SetSpecifier(pxr::SdfSpecifierDef);
		env_prim->SetTypeName(pxr::UsdGeomTokens->Xform);
	}
}

static void test_topology_serializer_export_usd_graph_env_and_pose_hints() {
	auto stage = make_stage();
	BrickColor red{255, 0, 0};
	BrickColor green{0, 255, 0};

	std::int64_t env_id = 5;
	create_env_root(stage, env_id);

	UsdGraphG g(stage);

	BrickPart brickA = make_brick(2, 4, BrickHeightPerPlate, red);
	BrickPart brickB = make_brick(2, 4, BrickHeightPerPlate, green);

	auto pathA_opt = g.add_part(env_id, brickA);
	auto pathB_opt = g.add_part(env_id, brickB);
	assert(pathA_opt.has_value() && pathB_opt.has_value());
	[[maybe_unused]] auto [ignored_pidA, pathA] = *pathA_opt;
	[[maybe_unused]] auto [ignored_pidB, pathB] = *pathB_opt;

	const PartId *pidA =
	    g.topology().parts().project_key<pxr::SdfPath, PartId>(pathA);
	const PartId *pidB =
	    g.topology().parts().project_key<pxr::SdfPath, PartId>(pathB);
	assert(pidA && pidB);

	InterfaceRef stud_if{*pidA, BrickPart::StudId};
	InterfaceRef hole_if{*pidB, BrickPart::HoleId};
	ConnectionSegment cs{};
	auto connPath_opt = g.connect(stud_if, hole_if, cs, AlignPolicy::None);
	assert(connPath_opt.has_value());

	TopologySerializer<BrickSerializer> ser;
	JsonTopology topo = ser.export_usd_graph(g, env_id);

	// Only env_id's parts and connections should be present.
	assert(topo.parts.size() == 2);
	assert(topo.connections.size() == 1);

	// There is exactly one connected component with a valid pose hint.
	assert(topo.pose_hints.size() == 1);
	const JsonPoseHint &hint = topo.pose_hints[0];

	PartId root_pid = static_cast<PartId>(hint.part);
	auto pose_opt = g.part_pose_relative_to_env(root_pid);
	assert(pose_opt.has_value());
	const auto &[q, t] = *pose_opt;

	auto expected_pos = as_array<double, 3>(t);
	auto expected_rot = as_array<double>(q);
	assert(hint.pos == expected_pos);
	assert(hint.rot == expected_rot);
}

static void test_topology_serializer_export_usd_graph_env_filtering() {
	auto stage = make_stage();
	BrickColor red{255, 0, 0};
	BrickColor green{0, 255, 0};

	std::int64_t env_keep = 3;
	std::int64_t env_other = 4;
	create_env_root(stage, env_keep);
	create_env_root(stage, env_other);

	UsdGraphG g(stage);

	// Two parts in env_keep
	BrickPart brickA = make_brick(2, 4, BrickHeightPerPlate, red);
	BrickPart brickB = make_brick(2, 4, BrickHeightPerPlate, green);
	auto pathA_opt = g.add_part(env_keep, brickA);
	auto pathB_opt = g.add_part(env_keep, brickB);
	assert(pathA_opt.has_value() && pathB_opt.has_value());
	[[maybe_unused]] auto [ignored_pidA2, pathA] = *pathA_opt;
	[[maybe_unused]] auto [ignored_pidB2, pathB] = *pathB_opt;

	const PartId *pidA =
	    g.topology().parts().project_key<pxr::SdfPath, PartId>(pathA);
	const PartId *pidB =
	    g.topology().parts().project_key<pxr::SdfPath, PartId>(pathB);
	assert(pidA && pidB);

	InterfaceRef stud_if{*pidA, BrickPart::StudId};
	InterfaceRef hole_if{*pidB, BrickPart::HoleId};
	ConnectionSegment cs{};
	assert(g.connect(stud_if, hole_if, cs, AlignPolicy::None).has_value());

	// One part in a different env; should be excluded when exporting env_keep.
	BrickPart brickC = make_brick(2, 4, BrickHeightPerPlate, red);
	auto pathC_opt = g.add_part(env_other, brickC);
	assert(pathC_opt.has_value());

	TopologySerializer<BrickSerializer> ser;

	JsonTopology topo_keep = ser.export_usd_graph(g, env_keep);
	assert(topo_keep.parts.size() == 2);
	assert(topo_keep.connections.size() == 1);
	assert(topo_keep.pose_hints.size() == 1);

	JsonTopology topo_other = ser.export_usd_graph(g, env_other);
	assert(topo_other.parts.size() == 1);
	assert(topo_other.connections.empty());
	assert(topo_other.pose_hints.size() == 1);

	JsonTopology topo_none = ser.export_usd_graph(g, 999);
	assert(topo_none.parts.empty());
	assert(topo_none.connections.empty());
	assert(topo_none.pose_hints.empty());
}

// -------------------- TopologySerializer import() --------------------

// Unknown part types should be skipped, and any connections involving them
// must not be imported.
static void
test_topology_serializer_import_unknown_part_type_skips_parts_and_connections() {
	auto stage = make_stage();
	std::int64_t env_id = 7;
	create_env_root(stage, env_id);

	UsdGraphG g(stage);
	TopologySerializer<BrickSerializer> ser;

	BrickColor color{10, 20, 30};
	BrickPart brick = make_brick(2, 4, BrickHeightPerPlate, color);
	BrickSerializer brick_ser;

	JsonTopology topo;

	// Known brick part
	JsonPart p_known{
	    .id = 1,
	    .type = std::string(BrickSerializer::TypeString),
	    .payload = brick_ser.to_json(brick),
	};
	topo.parts.push_back(p_known);

	// Unknown part type; should be ignored by import()
	JsonPart p_unknown{
	    .id = 2,
	    .type = "unknown_type",
	    .payload = ordered_json{{"foo", 42}},
	};
	topo.parts.push_back(p_unknown);

	// Connection referencing both known and unknown part ids.
	// Since the unknown endpoint cannot be imported, the connection must be
	// skipped entirely.
	JsonConnection conn{
	    .stud_id = 1,
	    .stud_iface = static_cast<std::int64_t>(BrickPart::StudId),
	    .hole_id = 2,
	    .hole_iface = static_cast<std::int64_t>(BrickPart::HoleId),
	    .offset = {0, 0},
	    .yaw = 0,
	};
	topo.connections.push_back(conn);

	ser.import(topo, g, env_id);

	// Only the known brick part should be present.
	assert(g.topology().parts().size() == 1);
	// No connections should have been created.
	assert(g.topology().connection_segments().size() == 0);
	assert(g.topology().connection_bundles().size() == 0);
	// Import should not create any unrealized parts or connections.
	assert(g.unrealized_parts().empty());
	assert(g.unrealized_connections().empty());
}

// Round-trip invariant: export_usd_graph -> import -> export_usd_graph again
// should preserve parts, connections, and pose hints (up to reindexing of
// PartIds). When a non-identity env transform is supplied to import(), the
// pose hints in the re-exported topology should be transformed accordingly.
static void
test_topology_serializer_export_import_usd_graph_roundtrip_with_env_transform() {
	auto stage_src = make_stage();
	BrickColor red{255, 0, 0};
	BrickColor green{0, 255, 0};

	std::int64_t env_src = 3;
	create_env_root(stage_src, env_src);

	UsdGraphG g_src(stage_src);

	// Use bricks with distinct payloads so we can match parts across
	// round-trips by (type, payload).
	BrickPart brickA = make_brick(2, 4, BrickHeightPerPlate, red);
	BrickPart brickB = make_brick(1, 2, BrickHeightPerPlate, green);

	auto pathA_opt = g_src.add_part(env_src, brickA);
	auto pathB_opt = g_src.add_part(env_src, brickB);
	assert(pathA_opt.has_value() && pathB_opt.has_value());
	auto [pidA_src, pathA] = *pathA_opt;
	auto [pidB_src, pathB] = *pathB_opt;
	(void)pathA;
	(void)pathB;

	InterfaceRef stud_if{pidA_src, BrickPart::StudId};
	InterfaceRef hole_if{pidB_src, BrickPart::HoleId};
	ConnectionSegment cs{};
	assert(g_src.connect(stud_if, hole_if, cs, AlignPolicy::None).has_value());

	TopologySerializer<BrickSerializer> ser;
	JsonTopology topo = ser.export_usd_graph(g_src, env_src);

	assert(topo.parts.size() == 2);
	assert(topo.connections.size() == 1);
	assert(topo.pose_hints.size() == 1);

	// Import into a different environment on a fresh stage, applying a
	// non-trivial env transform.
	auto stage_dst = make_stage();
	std::int64_t env_dst = 10;
	create_env_root(stage_dst, env_dst);

	UsdGraphG g_dst(stage_dst);

	Eigen::Quaterniond q_env_ref = Eigen::Quaterniond::Identity();
	Eigen::Vector3d t_env_ref(0.5, -1.0, 2.0);
	Transformd T_env_ref{q_env_ref, t_env_ref};

	ser.import(topo, g_dst, env_dst, T_env_ref);

	JsonTopology topo_rt = ser.export_usd_graph(g_dst, env_dst);

	// Same number of logical entities after round-trip.
	assert(topo_rt.parts.size() == topo.parts.size());
	assert(topo_rt.connections.size() == topo.connections.size());
	assert(topo_rt.pose_hints.size() == topo.pose_hints.size());

	// Build a mapping from source JsonPart id -> destination JsonPart id by
	// matching (type, payload). Part ids may be renumbered but payloads are
	// preserved.
	std::unordered_map<std::int64_t, std::int64_t> src_to_dst;
	std::unordered_set<std::int64_t> used_dst;

	for (const JsonPart &p_src : topo.parts) {
		bool found = false;
		for (const JsonPart &p_dst : topo_rt.parts) {
			if (p_dst.type == p_src.type && p_dst.payload == p_src.payload &&
			    !used_dst.contains(p_dst.id)) {
				src_to_dst[p_src.id] = p_dst.id;
				used_dst.insert(p_dst.id);
				found = true;
				break;
			}
		}
		assert(found);
	}
	assert(src_to_dst.size() == topo.parts.size());

	// Check that each exported connection is preserved under the part-id
	// reindexing.
	struct ConnKey {
		std::int64_t stud_id;
		std::int64_t stud_iface;
		std::int64_t hole_id;
		std::int64_t hole_iface;
		std::array<std::int64_t, 2> offset;
		std::int64_t yaw;
	};

	std::vector<ConnKey> conns_rt;
	conns_rt.reserve(topo_rt.connections.size());
	for (const JsonConnection &jc : topo_rt.connections) {
		conns_rt.push_back(ConnKey{
		    .stud_id = jc.stud_id,
		    .stud_iface = jc.stud_iface,
		    .hole_id = jc.hole_id,
		    .hole_iface = jc.hole_iface,
		    .offset = jc.offset,
		    .yaw = jc.yaw,
		});
	}

	for (const JsonConnection &jc_src : topo.connections) {
		auto it_stud = src_to_dst.find(jc_src.stud_id);
		auto it_hole = src_to_dst.find(jc_src.hole_id);
		assert(it_stud != src_to_dst.end());
		assert(it_hole != src_to_dst.end());

		std::int64_t stud_mapped = it_stud->second;
		std::int64_t hole_mapped = it_hole->second;

		bool found = false;
		for (const ConnKey &ck : conns_rt) {
			if (ck.stud_id == stud_mapped && ck.hole_id == hole_mapped &&
			    ck.stud_iface == jc_src.stud_iface &&
			    ck.hole_iface == jc_src.hole_iface &&
			    ck.offset == jc_src.offset && ck.yaw == jc_src.yaw) {
				found = true;
				break;
			}
		}
		assert(found);
	}

	// Pose hints: new hints should correspond to old hints transformed by
	// T_env_ref (up to numerical tolerance), with part ids remapped.
	for (const JsonPoseHint &hint_src : topo.pose_hints) {
		auto it = src_to_dst.find(hint_src.part);
		assert(it != src_to_dst.end());
		std::int64_t part_mapped = it->second;

		// Reconstruct {}^{ref}T_part from the source pose hint.
		Eigen::Quaterniond q_src = as<Eigen::Quaterniond>(hint_src.rot); // wxyz
		Eigen::Vector3d t_src = as<Eigen::Vector3d>(hint_src.pos);
		Transformd T_ref_part{q_src, t_src};

		// Expected env-local transform in the destination env:
		// {}^{env_dst}T_part = T_env_ref * {}^{ref}T_part
		Transformd T_env_part_expected = SE3d{}.project(T_env_ref * T_ref_part);
		auto expected_pos =
		    as_array<double, 3>(T_env_part_expected.second); // translation
		auto expected_rot =
		    as_array<double>(T_env_part_expected.first); // quaternion wxyz

		bool found = false;
		for (const JsonPoseHint &hint_dst : topo_rt.pose_hints) {
			if (hint_dst.part != part_mapped) {
				continue;
			}
			assert(almost_equal_array3(hint_dst.pos, expected_pos));
			assert(almost_equal_array4(hint_dst.rot, expected_rot));
			found = true;
			break;
		}
		assert(found);
	}
}

} // namespace

int main() {
	test_brick_serializer_roundtrip();
	test_json_part_roundtrip();
	test_json_connection_roundtrip();
	test_json_pose_hint_roundtrip();
	test_json_topology_roundtrip_and_optional_fields();
	test_topology_serializer_export_graph_default_filters();
	test_topology_serializer_export_graph_with_filters();
	test_topology_serializer_export_usd_graph_env_and_pose_hints();
	test_topology_serializer_export_usd_graph_env_filtering();
	test_topology_serializer_import_unknown_part_type_skips_parts_and_connections();
	test_topology_serializer_export_import_usd_graph_roundtrip_with_env_transform();
	return 0;
}
