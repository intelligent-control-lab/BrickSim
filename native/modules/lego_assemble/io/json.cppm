export module lego_assemble.io.json;

import std;
import lego_assemble.core.specs;
import lego_assemble.core.connections;
import lego_assemble.core.graph;
import lego_assemble.usd.usd_graph;
import lego_assemble.utils.type_list;
import lego_assemble.utils.poly_store;
import lego_assemble.utils.usd_envs;
import lego_assemble.utils.conversions;
import lego_assemble.utils.transforms;
import lego_assemble.utils.c4_rotation;
import lego_assemble.vendor.nlohmann_json;
import lego_assemble.vendor.pxr;
import lego_assemble.vendor.carb;
import lego_assemble.vendor.eigen;

namespace lego_assemble {

export template <typename T>
concept PartSerializer = requires {
	typename T::PartType;
	requires PartLike<typename T::PartType>;
	{ T::TypeString } -> std::convertible_to<std::string_view>;
} && requires(T t, const typename T::PartType &part) {
	{ t.to_json(part) } -> std::same_as<nlohmann::ordered_json>;
} && requires(T t, const nlohmann::ordered_json &j) {
	{ t.from_json(j) } -> std::same_as<typename T::PartType>;
};

export struct BrickSerializer {
	using PartType = BrickPart;
	static constexpr std::string_view TypeString = "brick";

	// Example JSON:
	// {
	//   "L": 2,
	//   "W": 4,
	//   "H": 3,
	//   "color": [255, 0, 0]
	// }

	nlohmann::ordered_json to_json(const BrickPart &part) const {
		return {
		    {"L", part.L()},
		    {"W", part.W()},
		    {"H", part.H()},
		    {"color", part.color()},
		};
	}

	BrickPart from_json(const nlohmann::ordered_json &j) const {
		BrickUnit L{};
		BrickUnit W{};
		PlateUnit H{};
		BrickColor color{};

		j.at("L").get_to(L);
		j.at("W").get_to(W);
		j.at("H").get_to(H);
		j.at("color").get_to(color);

		return BrickPart(L, W, H, color);
	}
};

static_assert(PartSerializer<BrickSerializer>);

static constexpr auto SchemaString = "lego_assemble/lego_topology@1";

export struct JsonPart {
	std::int64_t id;
	std::string type;
	nlohmann::ordered_json payload;
};

export void to_json(nlohmann::ordered_json &j, const JsonPart &part) {
	j = nlohmann::ordered_json{
	    {"id", part.id},
	    {"type", part.type},
	    {"payload", part.payload},
	};
}

export void from_json(const nlohmann::ordered_json &j, JsonPart &part) {
	j.at("id").get_to(part.id);
	j.at("type").get_to(part.type);
	j.at("payload").get_to(part.payload);
}

export struct JsonConnection {
	std::int64_t stud_id;
	std::int64_t stud_iface;
	std::int64_t hole_id;
	std::int64_t hole_iface;
	std::array<std::int64_t, 2> offset;
	std::int64_t yaw;
};

export void to_json(nlohmann::ordered_json &j, const JsonConnection &conn) {
	j = nlohmann::ordered_json{
	    {"stud_id", conn.stud_id}, {"stud_iface", conn.stud_iface},
	    {"hole_id", conn.hole_id}, {"hole_iface", conn.hole_iface},
	    {"offset", conn.offset},   {"yaw", conn.yaw},
	};
}

export void from_json(const nlohmann::ordered_json &j, JsonConnection &conn) {
	j.at("stud_id").get_to(conn.stud_id);
	j.at("stud_iface").get_to(conn.stud_iface);
	j.at("hole_id").get_to(conn.hole_id);
	j.at("hole_iface").get_to(conn.hole_iface);
	j.at("offset").get_to(conn.offset);
	j.at("yaw").get_to(conn.yaw);
}

export struct JsonPoseHint {
	std::int64_t part;
	std::array<double, 3> pos;
	std::array<double, 4> rot; // wxyz
};

export void to_json(nlohmann::ordered_json &j, const JsonPoseHint &hint) {
	j = nlohmann::ordered_json{
	    {"part", hint.part},
	    {"pos", hint.pos},
	    {"rot", hint.rot},
	};
}

export void from_json(const nlohmann::ordered_json &j, JsonPoseHint &hint) {
	j.at("part").get_to(hint.part);
	j.at("pos").get_to(hint.pos);
	j.at("rot").get_to(hint.rot);
}

export struct JsonTopology {
	std::vector<JsonPart> parts;
	std::vector<JsonConnection> connections;
	std::vector<JsonPoseHint> pose_hints;
};

export void to_json(nlohmann::ordered_json &j, const JsonTopology &topology) {
	j = nlohmann::ordered_json{
	    {"schema", SchemaString},
	    {"parts", topology.parts},
	    {"connections", topology.connections},
	    {"pose_hints", topology.pose_hints},
	};
}

export void from_json(const nlohmann::ordered_json &j, JsonTopology &topology) {
	if (j.contains("parts")) {
		j.at("parts").get_to(topology.parts);
	}
	if (j.contains("connections")) {
		j.at("connections").get_to(topology.connections);
	}
	if (j.contains("pose_hints")) {
		j.at("pose_hints").get_to(topology.pose_hints);
	}
}

template <class T> using GetPartType = typename T::PartType;

export template <PartSerializer... Serializers> class TopologySerializer {
  public:
	using SerializerList = type_list<Serializers...>;
	using PartTypeList = SerializerList::template map<GetPartType>;
	static_assert(
	    PartTypeList::unique,
	    "TopologySerializer: all Serializers must have unique PartType");
	template <PartLike P>
	    requires PartTypeList::template
	contains<P> using SerializerFor =
	    SerializerList::template at<PartTypeList::template index_of<P>>;

	// For LegoGraph
	template <class Graph>

	JsonTopology export_graph(const Graph &g, auto &&part_filter,
	                          auto &&conn_filter) const
	    requires(
	        PartTypeList::template is_superset_of<
	            typename Graph::PartTypeList> &&
	        std::same_as<std::invoke_result_t<decltype(part_filter), PartId>,
	                     bool> &&
	        std::same_as<std::invoke_result_t<decltype(conn_filter), ConnSegId>,
	                     bool>)
	{
		JsonTopology result;

		// Serialize parts
		[&]<PartWrapperLike... PWs>(type_list<PWs...>) {
			(
			    [&]<PartWrapperLike PW>(
			        const pmr_vector_storage<PW, PartId> &storage) {
				    using P = typename PW::wrapped_type;
				    using PS = SerializerFor<P>;

				    // TODO: a bug in clang (?) causes us to be unable to use zip
				    // for (const auto &[pid, pw] : std::views::zip(storage.ids, storage.data)) {
				    for (std::size_t i = 0; i < storage.ids.size(); ++i) {
					    PartId pid = storage.ids[i];
					    if (!part_filter(pid)) {
						    continue;
					    }
					    const PW &pw = storage.data[i];

					    const P &part = pw.wrapped();
					    result.parts.push_back({
					        .id = static_cast<std::int64_t>(pid),
					        .type = std::string{PS::TypeString},
					        .payload = PS{}.to_json(part),
					    });
				    }
			    }(g.parts().template storage_for<PWs>()),
			    ...);
		}(typename Graph::WrappedPartList{});

		// Serialize connections
		static constexpr std::size_t IdxConnSegId =
		    Graph::ConnSegKeys::template index_of<ConnSegId>;
		static constexpr std::size_t IdxConnSegRef =
		    Graph::ConnSegKeys::template index_of<ConnSegRef>;
		for (const auto &[cs_keys, csw] : g.connection_segments().view()) {
			ConnSegId csid = std::get<IdxConnSegId>(cs_keys);
			if (!conn_filter(csid)) {
				continue;
			}
			const ConnSegRef &cs_ref = std::get<IdxConnSegRef>(cs_keys);
			const auto &[stud_ref, hole_ref] = cs_ref;
			const auto &[stud_pid, stud_ifid] = stud_ref;
			const auto &[hole_pid, hole_ifid] = hole_ref;
			const ConnectionSegment &cs = csw.wrapped();
			result.connections.push_back({
			    .stud_id = static_cast<std::int64_t>(stud_pid),
			    .stud_iface = static_cast<std::int64_t>(stud_ifid),
			    .hole_id = static_cast<std::int64_t>(hole_pid),
			    .hole_iface = static_cast<std::int64_t>(hole_ifid),
			    .offset = as_array<std::int64_t, 2>(cs.offset),
			    .yaw = static_cast<std::int64_t>(cs.yaw),
			});
		}

		return result;
	}

	// For LegoGraph
	template <class Graph>
	JsonTopology export_graph(const Graph &g) const
	    requires(
	        PartTypeList::template is_superset_of<typename Graph::PartTypeList>)
	{
		return export_graph(
		    g, [&](PartId) { return true; }, [&](ConnSegId) { return true; });
	}

	// For UsdLegoGraph
	template <class Graph>
	    requires(PartTypeList::template is_superset_of<
	             typename Graph::TopologyGraph::PartTypeList>)
	JsonTopology export_usd_graph(const Graph &g,
	                              std::int64_t env_id = kNoEnv) const {
		JsonTopology result = export_graph(
		    g.topology(),
		    [&](PartId pid) {
			    if (auto e = g.part_env_id(pid)) {
				    return *e == env_id;
			    } else {
				    return false;
			    }
		    },
		    [&](ConnSegId csid) {
			    if (auto e = g.conn_env_id(csid)) {
				    return *e == env_id;
			    } else {
				    return false;
			    }
		    });

		for (auto cc : g.topology().components()) {
			PartId pid = cc.root();
			auto e = g.part_env_id(pid);
			if (!e || *e != env_id) {
				continue;
			}
			auto pose = g.part_pose_relative_to_env(pid);
			if (!pose) {
				continue;
			}
			const auto &[q, t] = *pose;
			result.pose_hints.push_back(JsonPoseHint{
			    .part = static_cast<std::int64_t>(pid),
			    .pos = as_array<double, 3>(t),
			    .rot = as_array<double>(q),
			});
		}
		return result;
	}

	template <class UsdGraph>
	void import(const JsonTopology &topology, UsdGraph &g,
	            std::int64_t env_id = kNoEnv,
	            const Transformd &T_env_ref = SE3d{}.identity()) const {
		// 1) Import parts
		std::unordered_map<std::int64_t, PartId> id_map;
		for (const JsonPart &jp : topology.parts) {
			bool matched =
			    visit_by_type_string(jp.type, [&](auto &&serializer) {
				    using PS = std::decay_t<decltype(serializer)>;
				    using P = typename PS::PartType;
				    P part = serializer.from_json(jp.payload);
				    auto added =
				        g.template add_part<P>(env_id, std::move(part));
				    if (!added) {
					    log_warn("TopologySerializer: failed to import part id "
					             "{} of type {}",
					             jp.id, jp.type);
					    return;
				    }
				    const auto &[pid, path] = *added;
				    id_map[jp.id] = pid;
			    });
			if (!matched) {
				log_warn(
				    "TopologySerializer: unknown part type string {} for part "
				    "id {}",
				    jp.type, jp.id);
			}
		}

		// 2) Import connections
		for (const JsonConnection &jc : topology.connections) {
			auto it_stud = id_map.find(jc.stud_id);
			auto it_hole = id_map.find(jc.hole_id);
			if (it_stud == id_map.end() || it_hole == id_map.end()) {
				// One of the endpoints failed to import; skip
				continue;
			}
			PartId stud_pid = it_stud->second;
			PartId hole_pid = it_hole->second;

			InterfaceRef stud_if{stud_pid,
			                     static_cast<InterfaceId>(jc.stud_iface)};
			InterfaceRef hole_if{hole_pid,
			                     static_cast<InterfaceId>(jc.hole_iface)};
			ConnectionSegment conn_seg{
			    .offset = as<Eigen::Vector2i>(jc.offset),
			    .yaw = to_c4(static_cast<int>(jc.yaw)),
			};
			auto connected = g.connect(stud_if, hole_if, std::move(conn_seg),
			                           AlignPolicy::None);
			if (!connected) {
				log_warn("TopologySerializer: failed to import connection "
				         "between {}:{} and {}:{}",
				         jc.stud_id, jc.stud_iface, jc.hole_id, jc.hole_iface);
			}
		}

		// 3) Apply pose hints
		for (const JsonPoseHint &jph : topology.pose_hints) {
			auto it = id_map.find(jph.part);
			if (it == id_map.end()) {
				// part failed to import; skip
				continue;
			}
			PartId pid = it->second;
			Transformd T_ref_part{as<Eigen::Quaterniond>(jph.rot),
			                      as<Eigen::Vector3d>(jph.pos)};
			Transformd T_env_part = T_env_ref * T_ref_part;
			g.set_component_transform(pid, T_env_part);
		}
	}

  private:
	template <typename Fn>
	static constexpr bool
	visit_by_type_string(const std::string_view type_string, Fn &&fn)
	    requires(std::invocable<Fn, Serializers> && ...)
	{
		return ([&]<PartSerializer S>(S &&serializer) {
			if (serializer.TypeString == type_string) {
				std::invoke(std::forward<Fn>(fn), std::forward<S>(serializer));
				return true;
			} else {
				return false;
			}
		}(Serializers{}) ||
		        ...);
	}
};

} // namespace lego_assemble
