export module lego_assemble.usd.usd_graph;

import std;
import lego_assemble.core.specs;
import lego_assemble.core.graph;
import lego_assemble.core.connections;
import lego_assemble.usd.allocator;
import lego_assemble.usd.author;
import lego_assemble.usd.parse;
import lego_assemble.usd.specs;
import lego_assemble.utils.conversions;
import lego_assemble.utils.metric_system;
import lego_assemble.utils.transforms;
import lego_assemble.utils.type_list;
import lego_assemble.utils.poly_store;
import lego_assemble.utils.unique_set;
import lego_assemble.utils.sdf_path_map;
import lego_assemble.utils.usd_envs;
import lego_assemble.utils.sdf;
import lego_assemble.vendor;

namespace lego_assemble {

export enum class AlignPolicy {
	None,       // just connect, no pose changes
	MoveStudCC, // move stud's component to match hole
	MoveHoleCC, // move hole's component to match stud
};

export template <PartLike P> struct UsdPartWrapper : SimplePartWrapper<P> {
	template <class... Args>
	explicit UsdPartWrapper(std::vector<InterfaceColliderPair> colliders,
	                        Args &&...args)
	    : SimplePartWrapper<P>(std::forward<Args>(args)...),
	      colliders_{std::move(colliders)} {}

	std::span<const InterfaceColliderPair> colliders() const {
		return colliders_;
	}

  private:
	std::vector<InterfaceColliderPair> colliders_;
};

template <class T> using GetPartType = T::PartType;

export template <class Parts, class PartAuthors, class PartParsers,
                 class Hooks = NoHooks>
class UsdLegoGraph;

export template <class... Ps, class... PAs, class... PPs, class Hooks>
class UsdLegoGraph<type_list<Ps...>, type_list<PAs...>, type_list<PPs...>,
                   Hooks> {
  public:
	using PartAuthorList = type_list<PAs...>;
	using PartParserList = type_list<PPs...>;
	static_assert((PartAuthor<PAs> && ...),
	              "UsdLegoGraph: all PAs... must satisfy PartAuthor concept");
	using PartAuthorPartTypes = PartAuthorList::template map<GetPartType>;
	static_assert(PartAuthorPartTypes::unique,
	              "UsdLegoGraph: all PAs... must have unique PartType");
	static_assert((PartParser<PPs> && ...),
	              "UsdLegoGraph: all PPs... must satisfy PartParser concept");
	using PartParserPartTypes = PartParserList::template map<GetPartType>;
	static_assert(PartParserPartTypes::unique,
	              "UsdLegoGraph: all PPs... must have unique PartType");

	using TopologyGraph =
	    LegoGraph<type_list<Ps...>, UsdPartWrapper, pmr_vector_storage,
	              type_list<pxr::SdfPath>, type_list<pxr::SdfPath::Hash>,
	              type_list<std::equal_to<>>, SimpleWrapper<ConnectionSegment>,
	              type_list<pxr::SdfPath>, type_list<pxr::SdfPath::Hash>,
	              type_list<std::equal_to<>>, SimpleWrapper<ConnectionBundle>,
	              Hooks>;
	using Self = UsdLegoGraph<type_list<Ps...>, type_list<PAs...>,
	                          type_list<PPs...>, Hooks>;

	// Re-export hook detection traits
	template <class P>
	static constexpr bool HasOnPartAddedHook =
	    TopologyGraph::template HasOnPartAddedHook<P>;

	static constexpr bool HasAllOnPartAddedHooks =
	    TopologyGraph::HasAllOnPartAddedHooks;

	template <class P>
	static constexpr bool HasOnPartRemovingHook =
	    TopologyGraph::template HasOnPartRemovingHook<P>;

	static constexpr bool HasAllOnPartRemovingHooks =
	    TopologyGraph::HasAllOnPartRemovingHooks;

	static constexpr bool HasOnBundleCreatedHook =
	    TopologyGraph::HasOnBundleCreatedHook;

	static constexpr bool HasOnBundleRemovingHook =
	    TopologyGraph::HasOnBundleRemovingHook;

	static constexpr bool HasOnConnectedHook =
	    TopologyGraph::HasOnConnectedHook;

	static constexpr bool HasOnDisconnectingHook =
	    TopologyGraph::HasOnDisconnectingHook;

	template <PartLike P>
	    requires PartAuthorPartTypes::template
	contains<P> using PartAuthorFor =
	    PartAuthorList::template at<PartAuthorPartTypes::template index_of<P>>;

	template <PartLike P>
	    requires PartParserPartTypes::template
	contains<P> using PartParserFor =
	    PartParserList::template at<PartParserPartTypes::template index_of<P>>;

	explicit UsdLegoGraph(
	    pxr::UsdStageRefPtr stage,
	    std::pmr::memory_resource *mr = std::pmr::get_default_resource())
	    : topology_(nullptr, mr), stage_(std::move(stage)), allocator_(stage_) {
		notice_sink_ = std::make_unique<UsdNoticeSink>(this);
		initial_sync();
		notice_sink_->connect();
	}
	~UsdLegoGraph() = default;
	UsdLegoGraph(const UsdLegoGraph &) = delete;
	UsdLegoGraph &operator=(const UsdLegoGraph &) = delete;
	UsdLegoGraph(UsdLegoGraph &&) = delete;
	UsdLegoGraph &operator=(UsdLegoGraph &&) = delete;

	// Direct edit to topology is disallowed.
	const TopologyGraph &topology() const {
		return topology_;
	}

	Hooks *get_hooks() noexcept {
		return topology_.get_hooks();
	}

	void set_hooks(Hooks *hooks) noexcept {
		topology_.set_hooks(hooks);
	}

	std::vector<pxr::SdfPath> unrealized_parts() const {
		std::vector<pxr::SdfPath> result;
		for (const auto &[path, part_info] : part_path_table_) {
			if (!part_info.pid.has_value()) {
				result.push_back(path);
			}
		}
		return result;
	}

	std::vector<pxr::SdfPath> unrealized_connections() const {
		std::vector<pxr::SdfPath> result;
		for (const auto &[path, conn_info] : conn_path_table_) {
			if (!conn_info.csid.has_value()) {
				result.push_back(path);
			}
		}
		return result;
	}

	template <PartLike P>
	    requires PartAuthorPartTypes::template
	contains<P> std::optional<std::tuple<PartId, pxr::SdfPath>>
	add_part(std::int64_t env_id, P part) {
		using Author = PartAuthorFor<P>;
		pxr::SdfChangeBlock _changes;
		auto [path, colliders] =
		    allocator_.allocate_part_managed<Author>(env_id, part);
		std::optional<PartId> part_id = topology_.template add_part<P>(
		    std::forward_as_tuple(path), std::move(colliders), std::move(part));
		if (!part_id) [[unlikely]] {
			// rollback
			allocator_.deallocate_managed_part(path);
			log_warn("UsdLegoGraph: failed to add part for prim {}",
			         path.GetText());
			return std::nullopt;
		}

		UsdPartInfo &part_info = part_path_table_[path];
		part_info.pid = *part_id;
		try_realize_unrealized_conns_for(path, part_info);
		return {{*part_id, path}};
	}

	bool remove_part(const pxr::SdfPath &path) {
		pxr::SdfChangeBlock _changes;
		// Remove the part from USD
		bool part_removed_from_usd = allocator_.deallocate_managed_part(path);
		if (!part_removed_from_usd) {
			// Part not exists, or
			// this is not a managed part, it can't be removed
			return false;
		}
		auto part_info_it = part_path_table_.find(path);
		if (part_info_it != part_path_table_.end()) {
			UsdPartInfo &part_info = part_info_it->second;
			// Make a copy because we may modify the original
			OrderedVecSet<pxr::SdfPath> unrealized_conns =
			    part_info.unrealized_conns;
			if (part_info.pid.has_value()) {
				// Remove relevant realized connections from both topology and USD
				PartId pid = *part_info.pid;
				auto remove_and_deallocate_realized_conn =
				    [&](const pxr::SdfPath &conn_path) {
					    // Now, remove from USD
					    bool usd_removed =
					        allocator_.deallocate_managed_conn(conn_path);
					    if (usd_removed) [[likely]] {
						    // Success because it's a managed one
						    // Remove from bookkeeping tables
						    bool removed = conn_path_table_.erase(conn_path);
						    if (!removed) [[unlikely]] {
							    log_error("UsdLegoGraph: failed to erase "
							              "connection "
							              "path {} from connection path "
							              "table during part removal of "
							              "path {}",
							              conn_path.GetText(), path.GetText());
						    }
					    } else [[unlikely]] {
						    // If it's an unmanaged one, keep it in bookkeeping table
						    // And mark it as unrealized
						    auto conn_info_it =
						        conn_path_table_.find(conn_path);
						    if (conn_info_it == conn_path_table_.end())
						        [[unlikely]] {
							    log_error("UsdLegoGraph: failed to find "
							              "connection path {} in connection "
							              "path table during part removal of "
							              "path {}",
							              conn_path.GetText(), path.GetText());
							    return;
						    }
						    UsdConnInfo &conn_info = conn_info_it->second;
						    conn_info.csid = std::nullopt;
						    // Add to unrealized_conns to both involving parts
						    // Because the part was realized before, these entries must exist
						    // in part_path_table_ already, no insert happens here.
						    part_path_table_[conn_info.hole]
						        .unrealized_conns.add(conn_path);
						    part_path_table_[conn_info.stud]
						        .unrealized_conns.add(conn_path);
					    }
				    };
				std::optional<std::vector<pxr::SdfPath>>
				    affected_realized_conns = collect_connections(pid);
				if (!affected_realized_conns) [[unlikely]] {
					log_error("UsdLegoGraph: failed collect connections for {} "
					          "during part removal of path {}",
					          pid, path.GetText());
				}
				// Now remove the part from topology
				bool removed = topology_.remove_part(pid).has_value();
				if (!removed) [[unlikely]] {
					// This should not happen, actually
					log_error("UsdLegoGraph: failed to remove part id {} "
					          "from topology during part removal of path {}",
					          pid, path.GetText());
					return false;
				}
				// Now remove relevant realized connections from USD
				if (affected_realized_conns) [[likely]] {
					for (const pxr::SdfPath &conn_path :
					     *affected_realized_conns) {
						remove_and_deallocate_realized_conn(conn_path);
					}
				}
				// Unset pid
				part_info.pid = std::nullopt;
				// So far, all realized connections are removed from both topology and USD.
				// The part is removed from topology
				// Fall through to remove unrealized connections and finally the part prim itself.
			}
			if (part_info.unrealized_conns.empty()) {
				// No unrealized connections left, remove the part entry itself
				bool removed = part_path_table_.erase(path);
				if (!removed) [[unlikely]] {
					log_error("UsdLegoGraph: failed to erase part path {} "
					          "from part path table during part removal",
					          path.GetText());
					return false;
				}
			} else {
				// Remove unrealized connections involving this part.
				// We are skipping unrealized connections that are added during the above process,
				// They are the unmanaged ones, and still can't be removed.
				// This entry will be removed in remove_unrealized_conn_from_part
				// if all unrealized connections are gone.
				for (const pxr::SdfPath &conn_path : unrealized_conns) {
					// Remove it from USD
					bool usd_removed =
					    allocator_.deallocate_managed_conn(conn_path);
					if (usd_removed) [[likely]] {
						// Success because it's a managed one
						// Then remove from bookkeeping tables
						auto conn_info_it = conn_path_table_.find(conn_path);
						if (conn_info_it == conn_path_table_.end())
						    [[unlikely]] {
							log_error("UsdLegoGraph: failed to find "
							          "connection path {} in connection "
							          "path table during part removal of "
							          "path {}",
							          conn_path.GetText(), path.GetText());
							continue;
						}
						UsdConnInfo &conn_info = conn_info_it->second;
						remove_unrealized_conn_and_cleanup(conn_info.stud,
						                                   conn_path);
						remove_unrealized_conn_and_cleanup(conn_info.hole,
						                                   conn_path);
						bool removed = conn_path_table_.erase(conn_path);
						if (!removed) [[unlikely]] {
							log_error("UsdLegoGraph: failed to erase "
							          "connection path {} from connection "
							          "path table during part removal of "
							          "path {}",
							          conn_path.GetText(), path.GetText());
							continue;
						}
					} else [[unlikely]] {
						// If it's an unmanaged one, keep it in bookkeeping table
						// Nothing more to do
					}
					// One endpoint is gone
					conns_to_retry_.erase(conn_path);
				}
			}
		}
		// We delay conn_to_retry_ re-processing to usd notice handling
		return true;
	}

	std::optional<std::tuple<ConnSegId, pxr::SdfPath>>
	connect(const InterfaceRef &stud_if, const InterfaceRef &hole_if,
	        ConnectionSegment conn_seg,
	        AlignPolicy align_policy = AlignPolicy::MoveHoleCC) {
		auto [stud_pid, stud_ifid] = stud_if;
		auto [hole_pid, hole_ifid] = hole_if;
		const pxr::SdfPath *stud_path_ptr =
		    topology_.parts().template project_key<PartId, pxr::SdfPath>(
		        stud_pid);
		const pxr::SdfPath *hole_path_ptr =
		    topology_.parts().template project_key<PartId, pxr::SdfPath>(
		        hole_pid);
		if (!stud_path_ptr || !hole_path_ptr) {
			return std::nullopt;
		}
		const pxr::SdfPath &stud_path = *stud_path_ptr;
		const pxr::SdfPath &hole_path = *hole_path_ptr;
		pxr::SdfChangeBlock _changes;

		bool connected_before = topology_.is_connected(stud_pid, hole_pid);
		if (!connected_before && align_policy != AlignPolicy::None) {
			auto stud_spec = topology_.get_interface_spec(stud_if);
			auto hole_spec = topology_.get_interface_spec(hole_if);
			if (!stud_spec || !hole_spec) {
				log_warn("UsdLegoGraph: failed to get interface specs when "
				         "connecting stud {} to hole {}",
				         stud_pid, hole_pid);
				return std::nullopt;
			}
			Transformd T_stud_hole =
			    conn_seg.compute_transform(*stud_spec, *hole_spec);

			if (align_policy == AlignPolicy::MoveStudCC) {
				auto T_env_hole = part_pose_relative_to_env(hole_pid);
				if (!T_env_hole) {
					log_warn(
					    "UsdLegoGraph: failed to get env-relative pose "
					    "for hole part {} when connecting stud {} to hole {}",
					    hole_pid, stud_pid, hole_pid);
					return std::nullopt;
				}
				auto T_env_stud = (*T_env_hole) * inverse(T_stud_hole);
				set_component_transform(stud_pid, T_env_stud);

			} else if (align_policy == AlignPolicy::MoveHoleCC) {
				auto T_env_stud = part_pose_relative_to_env(stud_pid);
				if (!T_env_stud) {
					log_warn(
					    "UsdLegoGraph: failed to get env-relative pose "
					    "for stud part {} when connecting stud {} to hole {}",
					    stud_pid, stud_pid, hole_pid);
					return std::nullopt;
				}
				auto T_env_hole = (*T_env_stud) * T_stud_hole;
				set_component_transform(hole_pid, T_env_hole);
			}
		}
		pxr::SdfPath conn_path = allocator_.allocate_conn_managed(
		    stud_path, stud_ifid, hole_path, hole_ifid, conn_seg);
		std::optional<ConnSegId> csid = topology_.connect(
		    stud_if, hole_if, std::make_tuple(conn_path), std::move(conn_seg));
		if (!csid) [[unlikely]] {
			// rollback
			allocator_.deallocate_managed_conn(conn_path);
			log_warn("UsdLegoGraph: failed to add connection for prim {}",
			         conn_path.GetText());
			return std::nullopt;
		}

		UsdConnInfo &conn_info = conn_path_table_[conn_path];
		conn_info.csid = *csid;
		conn_info.stud = stud_path;
		conn_info.hole = hole_path;
		return {{*csid, conn_path}};
	}

	bool disconnect(const pxr::SdfPath &conn_path) {
		pxr::SdfChangeBlock _changes;
		// Remove the connection from USD
		bool conn_removed_from_usd =
		    allocator_.deallocate_managed_conn(conn_path);
		if (!conn_removed_from_usd) {
			// Connection not exists, or
			// this is not a managed connection, it can't be removed
			return false;
		}
		auto conn_info_it = conn_path_table_.find(conn_path);
		if (conn_info_it != conn_path_table_.end()) {
			UsdConnInfo &conn_info = conn_info_it->second;
			if (conn_info.csid.has_value()) {
				// This is a realized connection
				ConnSegId csid = *conn_info.csid;
				bool disconnected = topology_.disconnect(csid).has_value();
				if (!disconnected) [[unlikely]] {
					// This should not happen, actually
					log_error("UsdLegoGraph: failed to disconnect "
					          "connection seg id {} from topology during "
					          "connection removal of path {}",
					          csid, conn_path.GetText());
					return false;
				}
				// Unset csid
				conn_info.csid = std::nullopt;
			} else {
				// This is an unrealized connection
				// Remove from unrealized_conns of both involving parts
				remove_unrealized_conn_and_cleanup(conn_info.stud, conn_path);
				remove_unrealized_conn_and_cleanup(conn_info.hole, conn_path);
				conns_to_retry_.erase(conn_path);
			}
			// Finally, remove the conn entry itself
			bool removed = conn_path_table_.erase(conn_path);
			if (!removed) [[unlikely]] {
				log_error("UsdLegoGraph: failed to erase connection path {} "
				          "from connection path table during connection "
				          "removal",
				          conn_path.GetText());
			}
		}
		return true;
	}

	std::optional<std::int64_t> part_env_id(PartId pid) const {
		const pxr::SdfPath *prim_path_ptr =
		    topology_.parts().template project_key<PartId, pxr::SdfPath>(pid);
		if (!prim_path_ptr) {
			return std::nullopt;
		}
		return envIdFromPath(*prim_path_ptr);
	}

	std::optional<std::int64_t> conn_env_id(ConnSegId csid) const {
		const pxr::SdfPath *conn_path_ptr =
		    topology_.connection_segments()
		        .template project<ConnSegId, pxr::SdfPath>(csid);
		if (!conn_path_ptr) {
			return std::nullopt;
		}
		return envIdFromPath(*conn_path_ptr);
	}

	// Returns {}^{env}T_part in SI units (meters), if the part has
	// a prim path that can be resolved into an env root.
	std::optional<Transformd> part_pose_relative_to_env(PartId pid) const {
		// 1. Map part id -> prim path
		const pxr::SdfPath *prim_path_ptr =
		    topology_.parts().template project_key<PartId, pxr::SdfPath>(pid);
		if (!prim_path_ptr) {
			// No USD prim for this part
			return std::nullopt;
		}
		const pxr::SdfPath &prim_path = *prim_path_ptr;

		// 2. Determine env id and env root path (/World or /World/envs/env_X)
		auto env_id_opt = envIdFromPath(prim_path);
		if (!env_id_opt) {
			// Part is not under a known env root
			return std::nullopt;
		}
		std::int64_t env_id = *env_id_opt;
		pxr::SdfPath env_path = pathForEnv(env_id);

		// 3. Look up prims on the stage
		pxr::UsdPrim prim = stage_->GetPrimAtPath(prim_path);
		pxr::UsdPrim env_prim = stage_->GetPrimAtPath(env_path);
		if (!prim || !env_prim) {
			return std::nullopt;
		}

		// 4. Compose env->part matrix in STAGE UNITS using USD's xform ops.
		pxr::GfMatrix4d M_env_part = ComputeRelativeTransform(prim, env_prim);

		// 5. Convert matrix to SE(3) in STAGE UNITS
		auto T_stage = as<Transformd>(M_env_part);

		// 6. Convert translation from stage units to meters (SI).
		return MetricSystem(stage_).to_m(T_stage);
	}

	// Set the transform of a part w.r.t. its env in USD.
	// And propagate the transforms to its connected component.
	std::size_t set_component_transform(PartId u0, const Transformd &T_u0) {
		// Look up the anchor part's prim path.
		const pxr::SdfPath *u0_path_ptr =
		    topology_.parts().template project_key<PartId, pxr::SdfPath>(u0);
		if (!u0_path_ptr) {
			log_warn("UsdLegoGraph::set_component_transform: anchor part {} "
			         "has no associated SdfPath",
			         u0);
			return 0;
		}
		const pxr::SdfPath &u0_path = *u0_path_ptr;

		// Resolve env root for this part.
		auto env_id_opt = envIdFromPath(u0_path);
		if (!env_id_opt.has_value()) {
			log_warn("UsdLegoGraph::set_component_transform: anchor prim {} is "
			         "outside any known env",
			         u0_path.GetText());
			return 0;
		}
		std::int64_t env_id = *env_id_opt;
		pxr::SdfPath env_path = pathForEnv(env_id);

		pxr::UsdPrim env_prim = stage_->GetPrimAtPath(env_path);
		if (!env_prim) {
			log_error(
			    "UsdLegoGraph::set_component_transform: env prim {} not found",
			    env_path.GetText());
			return 0;
		}

		MetricSystem metrics(stage_);
		auto layer = stage_->GetEditTarget().GetLayer();

		std::size_t updated_count = 0;
		pxr::SdfChangeBlock _changes;
		for (auto [v, T_u0_v] : topology_.component_view(u0).transforms()) {
			// For each part v in the connected component, we have:
			//   T_u0    : {}^{env}T_{u0} (env <- u0), in meters
			//   T_u0_v  : {}^{u0}T_v    (u0 <- v), relative graph transform
			// So the desired env-local pose for v is:
			//   T_env_v = T_u0 * T_u0_v  (env <- v), in meters.
			Transformd T_env_v = SE3d{}.project(T_u0 * T_u0_v);

			// Convert to stage units.
			Transformd T_env_v_stage = metrics.from_m(T_env_v);

			// Map PartId -> SdfPath.
			const pxr::SdfPath *v_path_ptr =
			    topology_.parts().template project_key<PartId, pxr::SdfPath>(v);
			if (!v_path_ptr) {
				log_error("UsdLegoGraph::set_component_transform: part {} has "
				          "no SdfPath mapping",
				          v);
				continue;
			}
			const pxr::SdfPath &v_path = *v_path_ptr;

			// All parts in a connected component are expected to live in the
			// same env. If not, skip and warn.
			auto v_env_id_opt = envIdFromPath(v_path);
			if (!v_env_id_opt.has_value() || v_env_id_opt.value() != env_id) {
				log_warn("UsdLegoGraph::set_component_transform: part {} at "
				         "path {} is in a different env; skipping",
				         v, v_path.GetText());
				continue;
			}

			pxr::UsdPrim v_prim = stage_->GetPrimAtPath(v_path);
			if (!v_prim) {
				log_warn("UsdLegoGraph::set_component_transform: prim {} not "
				         "found on stage; skipping",
				         v_path.GetText());
				continue;
			}

			// Compute the current env->parent transform for v's parent.
			pxr::UsdPrim parent_prim = v_prim.GetParent();
			pxr::GfMatrix4d M_env_parent(1.0);

			if (parent_prim && parent_prim != env_prim) {
				M_env_parent = ComputeRelativeTransform(parent_prim, env_prim);
			}

			// Desired env->v matrix in stage units.
			pxr::GfTransform env_v_tf =
			    as<pxr::GfTransform>(T_env_v_stage.first, T_env_v_stage.second);
			pxr::GfMatrix4d M_env_v = env_v_tf.GetMatrix();

			// Solve for parent->v local transform:
			//   M_env_v = M_env_parent * M_parent_v
			// => M_parent_v = M_env_parent^{-1} * M_env_v
			pxr::GfMatrix4d M_parent_v = M_env_parent.GetInverse() * M_env_v;

			// Decompose into translate + rotate (+ unit scale), and author
			// using separate xformOps
			pxr::GfVec3d translate = M_parent_v.ExtractTranslation();
			pxr::GfQuatd orient = M_parent_v.ExtractRotationQuat();
			pxr::GfVec3f scale(1.0f, 1.0f, 1.0f);

			pxr::SdfPrimSpecHandle prim_spec = layer->GetPrimAtPath(v_path);
			if (!prim_spec) {
				prim_spec = pxr::SdfCreatePrimInLayer(layer, v_path);
				prim_spec->SetSpecifier(pxr::SdfSpecifierDef);
			}

			SetAttr<pxr::GfVec3d>(prim_spec, xformOpTranslate, translate);
			SetAttr<pxr::GfQuatd>(prim_spec, xformOpOrient, orient);
			SetAttr<pxr::GfVec3f>(prim_spec, xformOpScale, scale);
			SetAttr<pxr::VtTokenArray>(
			    prim_spec, pxr::UsdGeomTokens->xformOpOrder,
			    {xformOpTranslate, xformOpOrient, xformOpScale});

			updated_count++;
		}

		return updated_count;
	}

	LegoAllocator &allocator() noexcept {
		return allocator_;
	}

  private:
	class UsdNoticeSink : public pxr::TfWeakBase {
	  public:
		explicit UsdNoticeSink(Self *owner) : owner_(owner) {}
		~UsdNoticeSink() {
			disconnect();
		}
		UsdNoticeSink(const UsdNoticeSink &) = delete;
		UsdNoticeSink &operator=(const UsdNoticeSink &) = delete;
		UsdNoticeSink(UsdNoticeSink &&) = delete;
		UsdNoticeSink &operator=(UsdNoticeSink &&) = delete;

		void connect() {
			pxr::TfWeakPtr<UsdNoticeSink> weak_this =
			    pxr::TfCreateWeakPtr(this);
			if (!objects_key_) {
				objects_key_ = pxr::TfNotice::Register(
				    weak_this, &UsdNoticeSink::on_objects_changed,
				    owner_->stage_);
			}
		}

		void disconnect() {
			if (objects_key_) {
				pxr::TfNotice::Revoke(*objects_key_);
				objects_key_ = std::nullopt;
			}
		}

		void on_objects_changed(
		    const pxr::UsdNotice::ObjectsChanged &notice,
		    [[maybe_unused]] const pxr::UsdStageWeakPtr &sender) {
			owner_->resync_tree(notice.GetResyncedPaths(),
			                    notice.GetChangedInfoOnlyPaths());
		}

	  private:
		Self *owner_;
		std::optional<pxr::TfNotice::Key> objects_key_;
	};

	struct UsdPartInfo {
		std::optional<PartId> pid;
		OrderedVecSet<pxr::SdfPath> unrealized_conns;
	};

	struct UsdConnInfo {
		std::optional<ConnSegId> csid;
		pxr::SdfPath stud;
		pxr::SdfPath hole;
	};

	TopologyGraph topology_;
	pxr::UsdStageRefPtr stage_;
	LegoAllocator allocator_;

	std::unique_ptr<UsdNoticeSink> notice_sink_;

	// Entry exists iif.
	// 1. This is a realized part prim in USD.
	//    In this case pid has value; OR
	// 2. This is not a realized part prim in USD, AND
	// 	  there are unrealized connections involving this path.
	//    In this case pid has no value.
	std::map<pxr::SdfPath, UsdPartInfo> part_path_table_;

	// Entry exists iif. Connection prim exists in USD.
	// 1. Connection is realized, in which case csid has value; or
	// 2. Connection is not realized, in which case csid has no value.
	//    In this case, the involving parts exist in part_path_table_, AND
	//	  this connection path exists in ALL involving parts' unrealized_conns.
	//    Note: this connection can be unrealized even when both involving parts are realized.
	//          For example, (but not limited) if the connection causes inconsistent transforms.
	std::map<pxr::SdfPath, UsdConnInfo> conn_path_table_;

	// Connections that:
	// 1. are not realized; AND
	// 2. both involving parts are realized.
	// These connections are to be retried for realization every time.
	std::unordered_set<pxr::SdfPath, pxr::SdfPath::Hash> conns_to_retry_;

	template <class Fn>
	bool try_parse_part(const pxr::UsdPrim &prim, Fn &&fn)
	    requires(std::invocable<Fn, PartPrimParseResult<Ps>> && ...)
	{
		return ([&]<PartParser PP>(PP &&parser) {
			if (auto result = parser(prim)) {
				std::invoke(std::forward<Fn>(fn), std::move(result.value()));
				return true;
			} else {
				return false;
			}
		}(PPs{}) ||
		        ...);
	}

	void initial_sync() {
		// The graph is assumed to be empty when called.
		for (pxr::UsdPrim prim : stage_->Traverse()) {
			try_parse_part(prim, [&]<PartLike P>(
			                         PartPrimParseResult<P> result) {
				pxr::SdfPath path = prim.GetPath();
				auto &[part, colliders] = result;
				std::optional<PartId> part_id = topology_.template add_part<P>(
				    std::forward_as_tuple(path), std::move(colliders),
				    std::move(part));
				if (part_id) [[likely]] {
					part_path_table_[path].pid = *part_id;
				} else [[unlikely]] {
					log_warn("UsdLegoGraph: failed to add part for prim {}",
					         path.GetText());
				}
			});
		}

		for (pxr::UsdPrim prim : stage_->Traverse()) {
			if (std::optional<ConnectionPrimParseResult> conn =
			        parse_connection_prim(prim)) {
				pxr::SdfPath path = prim.GetPath();
				UsdConnInfo &conn_info = conn_path_table_[path];
				conn_info.stud = conn->stud_path;
				conn_info.hole = conn->hole_path;
				UsdPartInfo &stud_info = part_path_table_[conn->stud_path];
				UsdPartInfo &hole_info = part_path_table_[conn->hole_path];
				bool realizable =
				    stud_info.pid.has_value() && hole_info.pid.has_value();
				bool realized = false;
				if (realizable) {
					InterfaceRef stud_if_ref{*stud_info.pid,
					                         conn->stud_interface};
					InterfaceRef hole_if_ref{*hole_info.pid,
					                         conn->hole_interface};
					std::optional<ConnSegId> csid = topology_.connect(
					    stud_if_ref, hole_if_ref, std::forward_as_tuple(path),
					    std::move(conn->conn_seg));
					if (csid) [[likely]] {
						conn_info.csid = *csid;
						realized = true;
					} else [[unlikely]] {
						conns_to_retry_.insert(path);
						log_warn("UsdLegoGraph: failed to add connection for "
						         "prim {}",
						         path.GetText());
					}
				}
				if (!realized) {
					stud_info.unrealized_conns.add(path);
					hole_info.unrealized_conns.add(path);
				}
			}
		}
	}

	void resync_tree(
	    pxr::UsdNotice::ObjectsChanged::PathRange resynced_paths,
	    pxr::UsdNotice::ObjectsChanged::PathRange changed_info_only_paths) {
		// dirty_conns is the super set of connections that need to be re-evaluated after parts resync
		std::unordered_set<pxr::SdfPath, pxr::SdfPath::Hash> dirty_conns;
		// ==== Parts resync ====
		auto remove_part_and_invalidate_conns =
		    [&]<PartLike P> [[nodiscard]] (
		        PartId pid, const UsdPartWrapper<P> &pw) -> bool /* success */ {
			std::vector<pxr::SdfPath> affected_conn_paths =
			    collect_connections(pw);

			bool removed = topology_.remove_part(pid).has_value();
			if (!removed) [[unlikely]] {
				return false;
			}

			for (const pxr::SdfPath &conn_path : affected_conn_paths) {
				auto conn_info_it = conn_path_table_.find(conn_path);
				if (conn_info_it == conn_path_table_.end()) [[unlikely]] {
					log_error(
					    "UsdLegoGraph: failed to find connection path {} in "
					    "connection path table during resync",
					    conn_path.GetText());
					continue;
				}
				UsdConnInfo &conn_info = conn_info_it->second;
				// Unset invalid csid
				conn_info.csid = std::nullopt;
				// Add to involving parts' unrealized_conns
				// Because the part was realized before, these entries must exist
				// in part_path_table_ already, no insert happens here.
				part_path_table_[conn_info.stud].unrealized_conns.add(
				    conn_path);
				part_path_table_[conn_info.hole].unrealized_conns.add(
				    conn_path);
			}
			dirty_conns.insert_range(std::move(affected_conn_paths));
			return true;
		};
		auto do_part_modified = [&]<PartLike Old, PartLike New> [[nodiscard]] (
		                            const pxr::SdfPath &path,
		                            UsdPartInfo &part_info, PartId old_pid,
		                            New &&new_part,
		                            InterfaceCollidersVector &&colliders,
		                            const UsdPartWrapper<Old> &old_pw)
		    -> bool /* erase_part_info */ {
			// Delete the old
			bool deleted = remove_part_and_invalidate_conns(old_pid, old_pw);
			if (!deleted) [[unlikely]] {
				log_warn("UsdLegoGraph: failed to remove modified part "
				         "id {} for path {}",
				         old_pid, path.GetText());
				return false;
			}
			// Add the new part
			std::optional<PartId> new_pid = topology_.template add_part<New>(
			    std::forward_as_tuple(path), std::move(colliders),
			    std::move(new_part));
			if (new_pid) [[likely]] {
				part_info.pid = *new_pid;
			} else [[unlikely]] {
				part_info.pid = std::nullopt;
				log_warn("UsdLegoGraph: failed to add modified part for "
				         "prim {}",
				         path.GetText());
				if (part_info.unrealized_conns.empty()) {
					return true;
				}
			}
			return false;
		};
		auto do_part_added =
		    [&]<PartLike P> [[nodiscard]] (const pxr::SdfPath &path,
		                                   UsdPartInfo &part_info, P &&new_part,
		                                   InterfaceCollidersVector &&colliders)
		    -> bool /* erase_part_info */ {
			std::optional<PartId> new_pid = topology_.template add_part<P>(
			    std::forward_as_tuple(path), std::move(colliders),
			    std::move(new_part));
			if (new_pid) [[likely]] {
				part_info.pid = *new_pid;
				dirty_conns.insert_range(part_info.unrealized_conns);
			} else [[unlikely]] {
				log_warn("UsdLegoGraph: failed to add part for prim {}",
				         path.GetText());
				if (part_info.unrealized_conns.empty()) {
					return true;
				}
			}
			return false;
		};
		auto do_part_remove =
		    [&] [[nodiscard]] (
		        const pxr::SdfPath &path,
		        UsdPartInfo &part_info) -> bool /* erase_part_info */ {
			PartId old_pid = *part_info.pid;
			bool erase_part_info = false;
			bool visited = topology_.parts().visit(
			    old_pid, [&]<PartLike P>(const UsdPartWrapper<P> &old_pw) {
				    // Delete the old
				    bool deleted =
				        remove_part_and_invalidate_conns(old_pid, old_pw);
				    if (!deleted) [[unlikely]] {
					    log_warn("UsdLegoGraph: failed to remove deleted part "
					             "id {} for path {}",
					             old_pid, path.GetText());
					    return;
				    }
				    part_info.pid = std::nullopt;
				    if (part_info.unrealized_conns.empty()) {
					    erase_part_info = true;
				    }
			    });
			if (!visited) [[unlikely]] {
				log_error("UsdLegoGraph: failed to find part id {} during "
				          "resync",
				          old_pid);
			}
			return erase_part_info;
		};
		// This process all paths that exist in USD now
		auto process_part_prim = [&](const pxr::UsdPrim &prim) {
			pxr::SdfPath path = prim.GetPath();
			bool recognized = try_parse_part(prim, [&]<PartLike P>(
			                                           PartPrimParseResult<P>
			                                               result) {
				auto &[part, colliders] = result;
				UsdPartInfo &part_info = part_path_table_[path];
				bool erase_part_info = false;
				if (part_info.pid.has_value()) {
					// Part already exists.
					// It is unmodified iif.
					// 1. it is of the same type; and
					// 2. its data is unchanged.
					PartId pid = *part_info.pid;
					bool pid_exists = topology_.parts().alive(pid);
					if (!pid_exists) [[unlikely]] {
						log_error(
						    "UsdLegoGraph: part id {} for path {} not found in "
						    "parts storage",
						    pid, path.GetText());
						return;
					}
					const UsdPartWrapper<P> *existing_part =
					    topology_.parts().template get<UsdPartWrapper<P>>(pid);
					if (existing_part) {
						if (existing_part->wrapped() == part) {
							// Unmodified
							// No action needed.
						} else {
							// Modified, same type
							erase_part_info = do_part_modified(
							    path, part_info, pid, std::move(part),
							    std::move(colliders), *existing_part);
						}
					} else {
						// Modified, different type
						topology_.parts().visit(pid, [&](const auto &old_pw) {
							erase_part_info = do_part_modified(
							    path, part_info, pid, std::move(part),
							    std::move(colliders), old_pw);
						});
					}
				} else {
					// Before: part NOT recognized.
					// Now: part recognized.
					erase_part_info = do_part_added(
					    path, part_info, std::move(part), std::move(colliders));
				}
				if (erase_part_info) [[unlikely]] {
					part_path_table_.erase(path);
				}
			});
			if (!recognized) {
				auto part_info_it = part_path_table_.find(path);
				if (part_info_it != part_path_table_.end()) {
					UsdPartInfo &part_info = part_info_it->second;
					bool erase_part_info;
					if (part_info.pid.has_value()) {
						// Before: part recognized.
						// Now: part NOT recognized.
						erase_part_info = do_part_remove(path, part_info);
					} else {
						// Before: part NOT recognized.
						// Now: part still NOT recognized.
						// No action needed.
						erase_part_info = part_info.unrealized_conns.empty();
					}
					if (erase_part_info) {
						part_path_table_.erase(part_info_it);
					}
				}
			}
		};
		for (const pxr::SdfPath &root_path_ : resynced_paths) {
			pxr::SdfPath root_path = root_path_.GetPrimPath();
			pxr::UsdPrim root_prim = stage_->GetPrimAtPath(root_path);
			for (pxr::UsdPrim prim : pxr::UsdPrimRange(root_prim)) {
				if (!prim.IsValid()) {
					continue;
				}
				process_part_prim(prim);
			}
		}
		for (const pxr::SdfPath &path_ : changed_info_only_paths) {
			pxr::SdfPath path = path_.GetPrimPath();
			process_part_prim(stage_->GetPrimAtPath(path));
		}
		{
			// Then process paths that are NOT in USD
			std::vector<pxr::SdfPath> part_paths_to_erase;
			for (const pxr::SdfPath &root_path_ : resynced_paths) {
				pxr::SdfPath root_path = root_path_.GetPrimPath();
				for (auto &[path, part_info] :
				     subtree_range(part_path_table_, root_path)) {
					if (stage_->GetPrimAtPath(path).IsValid()) {
						continue;
					}
					bool erase_part_info;
					if (part_info.pid.has_value()) {
						// Before: part recognized.
						// Now: part NOT recognized.
						erase_part_info = do_part_remove(path, part_info);
					} else {
						// Before: part NOT recognized.
						// Now: part still NOT recognized.
						// No action needed.
						erase_part_info = part_info.unrealized_conns.empty();
					}
					if (erase_part_info) {
						part_paths_to_erase.push_back(path);
					}
				}
			}
			for (const pxr::SdfPath &path : part_paths_to_erase) {
				part_path_table_.erase(path);
			}
		}
		// ==== Connections resync ====
		std::unordered_map<pxr::SdfPath, ConnectionPrimParseResult,
		                   pxr::SdfPath::Hash>
		    conn_to_realize;
		auto do_conn_add = [&](const pxr::SdfPath &path,
		                       ConnectionPrimParseResult &&conn) {
			UsdConnInfo &conn_info = conn_path_table_[path];
			conn_info.stud = conn.stud_path;
			conn_info.hole = conn.hole_path;
			// Try to realize later
			conn_to_realize.emplace(path, std::move(conn));
			// Add to involving parts' unrealized_conns
			part_path_table_[conn_info.stud].unrealized_conns.add(path);
			part_path_table_[conn_info.hole].unrealized_conns.add(path);
		};
		auto do_conn_remove = [&](const pxr::SdfPath &path,
		                          UsdConnInfo &conn_info) {
			if (conn_info.csid.has_value()) {
				// Delete realized connection
				ConnSegId csid = *conn_info.csid;
				bool disconnected = topology_.disconnect(csid).has_value();
				if (!disconnected) [[unlikely]] {
					log_warn("UsdLegoGraph: failed to disconnect conn "
					         "seg id {} for deleted connection prim {}",
					         csid, path.GetText());
					return;
				}
				conn_info.csid = std::nullopt;
			} else {
				// Delete unrealized connection
				remove_unrealized_conn_and_cleanup(conn_info.stud, path);
				remove_unrealized_conn_and_cleanup(conn_info.hole, path);
			}
		};
		auto do_conn_update = [&](const pxr::SdfPath &path,
		                          UsdConnInfo &conn_info,
		                          ConnectionPrimParseResult &&conn) {
			if (conn_info.csid.has_value()) {
				// Update realized connection
				ConnSegId csid = *conn_info.csid;
				const ConnSegRef *csref =
				    topology_.connection_segments()
				        .template project<ConnSegId, ConnSegRef>(csid);
				if (!csref) [[unlikely]] {
					log_error("UsdLegoGraph: failed to find conn seg ref "
					          "for conn seg id {} during resync",
					          csid);
					return;
				}
				const SimpleWrapper<ConnectionSegment> *csw =
				    topology_.connection_segments().find(csid);
				if (!csw) [[unlikely]] {
					log_error("UsdLegoGraph: failed to find conn seg "
					          "wrapper for conn seg id {} during resync",
					          csid);
					return;
				}
				const auto &[stud_ifref, hole_ifref] = *csref;
				const auto &[stud_pid, stud_ifid] = stud_ifref;
				const auto &[hole_pid, hole_ifid] = hole_ifref;
				bool unmodified = (conn.stud_path == conn_info.stud) &&
				                  (conn.hole_path == conn_info.hole) &&
				                  (conn.stud_interface == stud_ifid) &&
				                  (conn.hole_interface == hole_ifid) &&
				                  (conn.conn_seg == csw->wrapped());
				if (unmodified) {
					// Unmodified
					// No action needed.
				} else {
					// Modified
					// Disconnect the old
					bool disconnected = topology_.disconnect(csid).has_value();
					if (!disconnected) [[unlikely]] {
						log_warn("UsdLegoGraph: failed to disconnect conn "
						         "seg id {} for modified connection prim {}",
						         csid, path.GetText());
						return;
					}
					conn_info.csid = std::nullopt;
					conn_info.stud = conn.stud_path;
					conn_info.hole = conn.hole_path;
					// Add to involving parts' unrealized_conns
					part_path_table_[conn_info.stud].unrealized_conns.add(path);
					part_path_table_[conn_info.hole].unrealized_conns.add(path);
					// Try to realize later
					conn_to_realize.emplace(path, std::move(conn));
				}
			} else {
				// Update unrealized connection
				if (conn_info.stud != conn.stud_path ||
				    conn_info.hole != conn.hole_path) {
					// Remove from old parts' unrealized_conns
					remove_unrealized_conn_and_cleanup(conn_info.stud, path);
					remove_unrealized_conn_and_cleanup(conn_info.hole, path);
					// Add to new parts' unrealized_conns
					part_path_table_[conn.stud_path].unrealized_conns.add(path);
					part_path_table_[conn.hole_path].unrealized_conns.add(path);
					// Update in conn_info
					conn_info.stud = conn.stud_path;
					conn_info.hole = conn.hole_path;
				}
				// Try to realize later
				conn_to_realize.emplace(path, std::move(conn));
			}
		};
		auto preprocess_conn = [&](const pxr::SdfPath &path,
		                           const pxr::UsdPrim &prim) {
			auto conn_info_it = conn_path_table_.find(path);
			std::optional<ConnectionPrimParseResult> conn;
			if (prim.IsValid()) {
				conn = parse_connection_prim(prim);
			}
			if (conn) {
				if (conn_info_it == conn_path_table_.end()) {
					// New connection
					do_conn_add(path, std::move(*conn));
				} else {
					// Existing connection
					UsdConnInfo &conn_info = conn_info_it->second;
					do_conn_update(path, conn_info, std::move(*conn));
				}
			} else {
				if (conn_info_it != conn_path_table_.end()) {
					// Deleted connection
					UsdConnInfo &conn_info = conn_info_it->second;
					do_conn_remove(path, conn_info);
					conn_path_table_.erase(conn_info_it);
				}
			}
		};
		dirty_conns.insert_range(std::move(conns_to_retry_));
		conns_to_retry_.clear();
		for (const pxr::SdfPath &path : dirty_conns) {
			preprocess_conn(path, stage_->GetPrimAtPath(path));
		}
		for (const pxr::SdfPath &root_path_ : resynced_paths) {
			pxr::SdfPath root_path = root_path_.GetPrimPath();
			pxr::UsdPrim root_prim = stage_->GetPrimAtPath(root_path);
			for (pxr::UsdPrim prim : pxr::UsdPrimRange(root_prim)) {
				if (!prim.IsValid()) {
					continue;
				}
				pxr::SdfPath path = prim.GetPath();
				if (dirty_conns.contains(path)) {
					continue;
				}
				preprocess_conn(path, prim);
			}
		}
		for (const pxr::SdfPath &path_ : changed_info_only_paths) {
			pxr::SdfPath path = path_.GetPrimPath();
			if (dirty_conns.contains(path)) {
				continue;
			}
			pxr::UsdPrim prim = stage_->GetPrimAtPath(path);
			preprocess_conn(path, prim);
		}
		{
			std::vector<pxr::SdfPath> conn_paths_to_erase;
			for (const pxr::SdfPath &root_path_ : resynced_paths) {
				pxr::SdfPath root_path = root_path_.GetPrimPath();
				for (auto &[path, conn_info] :
				     subtree_range(conn_path_table_, root_path)) {
					if (dirty_conns.contains(path)) {
						continue;
					}
					if (stage_->GetPrimAtPath(path).IsValid()) {
						continue;
					}
					do_conn_remove(path, conn_info);
					conn_paths_to_erase.push_back(path);
				}
			}
			for (const pxr::SdfPath &path : conn_paths_to_erase) {
				conn_path_table_.erase(path);
			}
		}
		// Finally, try to realize pending connections
		for (auto &[path, conn] : conn_to_realize) {
			const PartId *stud_id_ptr =
			    topology_.parts().template project_key<pxr::SdfPath, PartId>(
			        conn.stud_path);
			const PartId *hole_id_ptr =
			    topology_.parts().template project_key<pxr::SdfPath, PartId>(
			        conn.hole_path);
			if (!stud_id_ptr || !hole_id_ptr) {
				continue;
			}
			PartId stud_pid = *stud_id_ptr;
			PartId hole_pid = *hole_id_ptr;
			InterfaceRef stud_if_ref{stud_pid, conn.stud_interface};
			InterfaceRef hole_if_ref{hole_pid, conn.hole_interface};
			std::optional<ConnSegId> csid = topology_.connect(
			    stud_if_ref, hole_if_ref, std::forward_as_tuple(path),
			    std::move(conn.conn_seg));
			if (!csid) [[unlikely]] {
				conns_to_retry_.insert(path);
				log_warn("UsdLegoGraph: failed to realize connection for "
				         "prim {} during resync",
				         path.GetText());
				continue;
			}
			conn_path_table_[path].csid = *csid;
			// Remove from involving parts' unrealized_conns
			part_path_table_[conn.stud_path].unrealized_conns.remove(path);
			part_path_table_[conn.hole_path].unrealized_conns.remove(path);
		}
	}

	void try_realize_unrealized_conns_for(const pxr::SdfPath &part_path,
	                                      UsdPartInfo &part_info) {
		OrderedVecSet<pxr::SdfPath> unrealized_conns =
		    part_info.unrealized_conns;
		for (const pxr::SdfPath &conn_path : unrealized_conns) {
			pxr::UsdPrim conn_prim = stage_->GetPrimAtPath(conn_path);
			if (!conn_prim.IsValid()) [[unlikely]] {
				log_error("UsdLegoGraph: connection prim {} not found but "
				          "listed in {}'s unrealized connections",
				          conn_path.GetText(), part_path.GetText());
				continue;
			}
			std::optional<ConnectionPrimParseResult> conn =
			    parse_connection_prim(conn_prim);
			if (!conn) [[unlikely]] {
				log_error("UsdLegoGraph: failed to parse connection prim {} "
				          "but listed in {}'s unrealized connections",
				          conn_path.GetText(), part_path.GetText());
				continue;
			}
			const PartId *stud_id_ptr =
			    topology_.parts().template project_key<pxr::SdfPath, PartId>(
			        conn->stud_path);
			const PartId *hole_id_ptr =
			    topology_.parts().template project_key<pxr::SdfPath, PartId>(
			        conn->hole_path);
			if (!stud_id_ptr || !hole_id_ptr) {
				// still not realizable, keep it unrealized
				continue;
			}
			PartId stud_pid = *stud_id_ptr;
			PartId hole_pid = *hole_id_ptr;
			InterfaceRef stud_if{stud_pid, conn->stud_interface};
			InterfaceRef hole_if{hole_pid, conn->hole_interface};
			auto csid = topology_.connect(stud_if, hole_if,
			                              std::forward_as_tuple(conn_path),
			                              std::move(conn->conn_seg));
			if (!csid) [[unlikely]] {
				conns_to_retry_.insert(conn_path);
				log_warn("UsdLegoGraph: failed to realize connection for "
				         "prim {} during part addition",
				         conn_path.GetText());
				continue;
			}
			// Success: update conn_path_table_ and clear from unrealized sets
			UsdConnInfo &cinfo = conn_path_table_[conn_path];
			cinfo.csid = *csid;
			cinfo.stud = conn->stud_path;
			cinfo.hole = conn->hole_path;
			part_path_table_[conn->stud_path].unrealized_conns.remove(
			    conn_path);
			part_path_table_[conn->hole_path].unrealized_conns.remove(
			    conn_path);
			conns_to_retry_.erase(conn_path);
		}
	}

	void remove_unrealized_conn_and_cleanup(const pxr::SdfPath &part_path,
	                                        const pxr::SdfPath &conn_path) {
		auto it = part_path_table_.find(part_path);
		if (it == part_path_table_.end()) {
			return;
		}
		UsdPartInfo &info = it->second;
		info.unrealized_conns.remove(conn_path);
		if (!info.pid.has_value() && info.unrealized_conns.empty()) {
			// This path is neither a realized part nor referenced by any
			// unrealized connections -> drop the entry entirely.
			part_path_table_.erase(it);
		}
	}

	template <PartLike P>
	std::vector<pxr::SdfPath> collect_connections(const UsdPartWrapper<P> &pw) {
		std::vector<pxr::SdfPath> conn_paths;
		auto add_conn_path = [&](const OrderedVecSet<ConnSegId> &csids) {
			for (ConnSegId csid : csids) {
				const pxr::SdfPath *path_ptr =
				    topology_.connection_segments()
				        .template project<ConnSegId, pxr::SdfPath>(csid);
				if (!path_ptr) {
					log_error(
					    "UsdLegoGraph: failed to find connection path for "
					    "conn seg id {}",
					    csid);
					continue;
				}
				conn_paths.push_back(*path_ptr);
			}
		};
		add_conn_path(pw.incomings());
		add_conn_path(pw.outgoings());
		return conn_paths;
	}

	std::optional<std::vector<pxr::SdfPath>> collect_connections(PartId pid) {
		std::optional<std::vector<pxr::SdfPath>> conn_paths;
		topology_.parts().visit(pid,
		                        [&]<PartLike P>(const UsdPartWrapper<P> &pw) {
			                        conn_paths = collect_connections(pw);
		                        });
		return conn_paths;
	}
};

} // namespace lego_assemble
