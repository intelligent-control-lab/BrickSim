export module lego_assemble.core.graph;

import std;
import lego_assemble.core.specs;
import lego_assemble.core.connections;
import lego_assemble.utils.type_list;
import lego_assemble.utils.poly_store;
import lego_assemble.utils.multi_key_map;
import lego_assemble.utils.pair;
import lego_assemble.utils.unordered_pair;
import lego_assemble.utils.unique_set;
import lego_assemble.utils.dynamic_graph;
import lego_assemble.utils.typed_id;
import lego_assemble.utils.transforms;

namespace lego_assemble {

export template <class T>
concept WrapperLike = requires(T t) {
	typename T::wrapped_type;
	{ t.wrapped() } -> std::same_as<typename T::wrapped_type &>;
} && requires(const T t) {
	{ t.wrapped() } -> std::same_as<const typename T::wrapped_type &>;
};

export template <class T> struct SimpleWrapper {
	using wrapped_type = T;
	T wrapped_;

	constexpr SimpleWrapper()
	    requires std::default_initializable<T>
	= default;
	constexpr SimpleWrapper(const SimpleWrapper &) = default;
	constexpr SimpleWrapper(SimpleWrapper &&) = default;
	constexpr SimpleWrapper &operator=(const SimpleWrapper &) = default;
	constexpr SimpleWrapper &operator=(SimpleWrapper &&) = default;

	template <class... Args>
	    requires(std::constructible_from<T, Args...> &&
	             (!std::same_as<std::remove_cvref_t<Args>, SimpleWrapper<T>> &&
	              ...))
	explicit constexpr SimpleWrapper(Args &&...args)
	    : wrapped_(std::forward<Args>(args)...) {}

	// Remove trailing PMR
	template <class... Args>
	    requires(!std::constructible_from<T, Args...> && sizeof...(Args) > 0 &&
	             std::same_as<std::remove_cvref_t<last_type<Args...>>,
	                          std::pmr::memory_resource *> &&
	             type_list<Args...>::template drop_back<
	                 1>::template can_construct<T>)
	explicit constexpr SimpleWrapper(Args &&...args)
	    : wrapped_(std::make_from_tuple<T>(
	          select_forward_as_tuple<
	              typename type_list<Args...>::template drop_back_seq<1>>(
	              std::forward<Args>(args)...))) {}

	template <class Self> constexpr auto &&wrapped(this Self &&self) noexcept {
		return std::forward<Self>(self).wrapped_;
	}
};
static_assert(WrapperLike<SimpleWrapper<int>>);

export using PartId = std::uint64_t;
export using ConnSegId = std::uint64_t;

// An interface reference: (part id, interface id)
export using InterfaceRef = std::pair<PartId, InterfaceId>;
export using InterfaceRefHash =
    PairHash<PartId, std::hash<PartId>, InterfaceId, std::hash<InterfaceId>>;

// A connection segment reference: (stud interface ref, hole interface ref)
export using ConnSegRef = std::pair<InterfaceRef, InterfaceRef>;
export using ConnSegRefHash = PairHash<InterfaceRef, InterfaceRefHash>;

export template <class T, class V>
concept AdjacentContainerLike =
    std::is_lvalue_reference_v<T> && !std::is_const_v<T> &&
    UniqueSetLike<std::remove_reference_t<T>, V>;

export template <class T>
concept PartWrapperLike =
    WrapperLike<T> && PartLike<typename T::wrapped_type> && requires(T t) {
	    // this part is the hole
	    { t.incomings() } -> AdjacentContainerLike<ConnSegId>;
	    // this part is the stud
	    { t.outgoings() } -> AdjacentContainerLike<ConnSegId>;
	    // all connecting parts
	    { t.neighbor_parts() } -> AdjacentContainerLike<PartId>;
    };

export template <class T>
concept ConnSegWrapperLike =
    WrapperLike<T> && std::same_as<typename T::wrapped_type, ConnectionSegment>;

export template <PartLike T> struct SimplePartWrapper : SimpleWrapper<T> {
	// We use vectors because usually the connections are few
	// ids are sorted in ascending order so binary search can be used
	OrderedVecSet<ConnSegId> incomings_;
	OrderedVecSet<ConnSegId> outgoings_;
	OrderedVecSet<PartId> neighbor_parts_;

	template <class... Args>
	explicit SimplePartWrapper(Args &&...args)
	    : SimpleWrapper<T>(std::forward<Args>(args)...),
	      incomings_(tail_mr(args...)), outgoings_(tail_mr(args...)),
	      neighbor_parts_(tail_mr(args...)) {}

	template <class Self>
	constexpr auto &&incomings(this Self &&self) noexcept {
		return std::forward<Self>(self).incomings_;
	}

	template <class Self>
	constexpr auto &&outgoings(this Self &&self) noexcept {
		return std::forward<Self>(self).outgoings_;
	}

	template <class Self>
	constexpr auto &&neighbor_parts(this Self &&self) noexcept {
		return std::forward<Self>(self).neighbor_parts_;
	}
};

export using ConnectionEndpoint = UnorderedPair<PartId>;

export struct ConnectionBundle {
	OrderedVecSet<ConnSegId> conn_seg_ids;
	Transformd T_a_b;
	Transformd T_b_a;
	explicit ConnectionBundle(
	    std::pmr::memory_resource *r = std::pmr::get_default_resource())
	    : conn_seg_ids{r} {}
};
template <class T>
concept ConnBundleWrapperLike =
    WrapperLike<T> && std::same_as<typename T::wrapped_type, ConnectionBundle>;

// For indexing vertices in the dynamic graph
export using DgVertexId = TypedId<struct DgVertexTag, std::uint32_t>;

struct NoHooks {};

export template <
    class Ps, template <class> class PartWrapper = SimplePartWrapper,
    template <class, class> class PartUnderlyingStorage = pmr_vector_storage,
    class PartExtraKeys = type_list<>, class PartExtraKeysHash = type_list<>,
    class PartExtraKeysEq = type_list<>,
    class ConnSegWrapper = SimpleWrapper<ConnectionSegment>,
    class ConnSegExtraKeys = type_list<>,
    class ConnSegExtraKeysHash = type_list<>,
    class ConnSegExtraKeysEq = type_list<>,
    class ConnBundleWrapper = SimpleWrapper<ConnectionBundle>,
    class Hooks = NoHooks, class DynamicGraph = HolmDeLichtenbergThorup>
class LegoGraph;

export template <class... Ps, template <class> class PartWrapper,
                 template <class, class> class PartUnderlyingStorage,
                 class... PEKs, class... PEKHs, class... PEKEqs,
                 class ConnSegWrapper, class... CSEKs, class... CSEKHs,
                 class... CSEKEqs, class ConnBundleWrapper, class Hooks,
                 class DynamicGraph>
class LegoGraph<type_list<Ps...>, PartWrapper, PartUnderlyingStorage,
                type_list<PEKs...>, type_list<PEKHs...>, type_list<PEKEqs...>,
                ConnSegWrapper, type_list<CSEKs...>, type_list<CSEKHs...>,
                type_list<CSEKEqs...>, ConnBundleWrapper, Hooks, DynamicGraph> {
	static_assert((PartLike<Ps> && ...),
	              "LegoGraph: all Ps... must satisfy PartLike concept");
	static_assert(unique_types<Ps...>,
	              "LegoGraph: part types Ps... must be unique");
	static_assert(sizeof...(Ps) >= 1,
	              "LegoGraph: at least one part type required");
	static_assert((PartWrapperLike<PartWrapper<Ps>> && ...),
	              "LegoGraph: PartWrapper<T> must satisfy PartWrapperLike "
	              "concept for all part types T");
	static_assert(ConnSegWrapperLike<ConnSegWrapper>,
	              "LegoGraph: ConnSegWrapper must satisfy "
	              "ConnSegWrapperLike concept");
	static_assert((StorageLike<PartUnderlyingStorage<PartWrapper<Ps>, PartId>,
	                           PartWrapper<Ps>, PartId> &&
	               ...),
	              "LegoGraph: PartUnderlyingStorage<T,Id> must satisfy "
	              "StorageLike concept for all part wrapper types T");
	static_assert(
	    DynamicGraphLike<DynamicGraph>,
	    "LegoGraph: DynamicGraph must satisfy DynamicGraphLike concept");
	static_assert(std::is_class_v<Hooks>,
	              "LegoGraph: Hooks must be a class type");

  public:
	using PartTypeList = PartList<Ps...>;
	using WrappedPartList = type_list<PartWrapper<Ps>...>;
	using PartKeys = type_list<PartId, DgVertexId, PEKs...>;
	using PartKeysHash =
	    type_list<std::hash<PartId>, std::hash<DgVertexId>, PEKHs...>;
	using PartKeysEq = type_list<std::equal_to<>, std::equal_to<>, PEKEqs...>;
	using PartStore =
	    PolyStore<PartKeys, WrappedPartList, PartUnderlyingStorage,
	              PartKeysHash, PartKeysEq>;
	using ConnSegKeys = type_list<ConnSegId, ConnSegRef, CSEKs...>;
	using ConnSegKeysHash =
	    type_list<std::hash<ConnSegId>, ConnSegRefHash, CSEKHs...>;
	using ConnSegKeysEq =
	    type_list<std::equal_to<>, std::equal_to<>, CSEKEqs...>;
	using ConnSegStore = MultiKeyMap<ConnSegKeys, ConnSegWrapper,
	                                 ConnSegKeysHash, ConnSegKeysEq>;
	using ConnBundleStore =
	    std::pmr::unordered_map<ConnectionEndpoint, ConnBundleWrapper>;

	template <class P>
	    requires PartTypeList::template
	contains<P> static constexpr bool HasOnPartAddedHook =
	    requires(Hooks &hooks, PartId pid, PartWrapper<P> &pw) {
		    {
			    // Called after a new part is added
			    hooks.template on_part_added<P>(pid, pw)
		    } -> std::same_as<void>;
	    };

	static constexpr bool HasAllOnPartAddedHooks =
	    (HasOnPartAddedHook<Ps> && ...);

	template <class P>
	    requires PartTypeList::template
	contains<P> static constexpr bool HasOnPartRemovingHook =
	    requires(Hooks &hooks, PartId pid, PartWrapper<P> &pw) {
		    {
			    // Called before a part is removed.
			    // All connections involving this part are still present.
			    hooks.template on_part_removing<P>(pid, pw)
		    } -> std::same_as<void>;
	    };

	static constexpr bool HasAllOnPartRemovingHooks =
	    (HasOnPartRemovingHook<Ps> && ...);

	static constexpr bool HasOnConnectedHook =
	    requires(Hooks &hooks, ConnSegId csid, const ConnSegRef &csref,
	             const InterfaceSpec &stud_spec, const InterfaceSpec &hole_spec,
	             ConnSegWrapper &csw, ConnBundleWrapper &cbw) {
		    {
			    // Called after a new connection segment is created.
			    hooks.on_connected(csid, csref, stud_spec, hole_spec, csw, cbw)
		    } -> std::same_as<void>;
	    };

	static constexpr bool HasOnDisconnectingHook =
	    requires(Hooks &hooks, ConnSegId csid, const ConnSegRef &csref,
	             ConnSegWrapper &csw, ConnBundleWrapper &cbw) {
		    {
			    // Called before a connection segment is removed.
			    // When called from explicit disconnect,
			    // the connection segment is still present in the graph and in cbw.
			    // When called from part removal,
			    // all connection segments involving the part are still present in
			    // the graph and in cbw,
			    // and this is called for each such connection segment,
			    // and it's called before on_part_removing for that part.
			    hooks.on_disconnecting(csid, csref, csw, cbw)
		    } -> std::same_as<void>;
	    };

	static constexpr bool HasOnBundleCreatedHook = requires(
	    Hooks &hooks, const ConnectionEndpoint &ep, ConnBundleWrapper &cbw) {
		{
			// Called after a new connection bundle is created.
			// The connection segment causing the bundle creation is added to graph
			// and in cbw. cbw's transform has been set up.
			// This is called before on_connected.
			hooks.on_bundle_created(ep, cbw)
		} -> std::same_as<void>;
	};

	static constexpr bool HasOnBundleRemovingHook = requires(
	    Hooks &hooks, const ConnectionEndpoint &ep, ConnBundleWrapper &cbw) {
		{
			// Called before a connection bundle is removed.
			// If this is caused by a single disconnection, that connection segment
			// is still present in the graph and in cbw,
			// and it's called after on_disconnecting.
			// If this is caused by part removal, the part and all relevant
			// connection segments are still present in the graph and in cbw,
			// and it's called before on_part_removing for that part.
			hooks.on_bundle_removing(ep, cbw)
		} -> std::same_as<void>;
	};

	explicit LegoGraph(
	    Hooks *hooks = nullptr,
	    std::pmr::memory_resource *r = std::pmr::get_default_resource())
	    : res_{r}, parts_{r}, conn_segs_{r}, conn_bundles_(r), hooks_{hooks},
	      dynamic_graph_(std::size_t(0), r) {}

	const PartStore &parts() const noexcept {
		return parts_;
	}

	const ConnSegStore &connection_segments() const noexcept {
		return conn_segs_;
	}

	const ConnBundleStore &connection_bundles() const noexcept {
		return conn_bundles_;
	}

	const DynamicGraph &dynamic_graph() const noexcept {
		return dynamic_graph_;
	}

	template <class InputId, class OutputId = InputId>
	std::generator<std::pair<OutputId, OutputId>>
	part_path(const InputId &u, const InputId &v) const
	    requires(PartKeys::template contains<InputId> &&
	             PartKeys::template contains<OutputId>)
	{
		const auto *u_dgid =
		    parts_.template project_key<InputId, DgVertexId>(u);
		const auto *v_dgid =
		    parts_.template project_key<InputId, DgVertexId>(v);
		if (!u_dgid || !v_dgid) {
			co_return;
		}
		for (auto [p_vid, q_vid] :
		     dynamic_graph_.path(u_dgid->value(), v_dgid->value())) {
			const auto *p_id =
			    parts_.template project_key<DgVertexId, OutputId>(
			        DgVertexId(p_vid));
			const auto *q_id =
			    parts_.template project_key<DgVertexId, OutputId>(
			        DgVertexId(q_vid));
			if (p_id && q_id) {
				co_yield std::make_pair(*p_id, *q_id);
			} else {
				std::unreachable();
			}
		}
	}

	template <class Id>
	std::optional<Transformd> lookup_transform(const Id &u, const Id &v) const
	    requires(PartKeys::template contains<Id>)
	{
		Transformd T = SE3d{}.identity();
		bool has_path = false;
		for (auto [a_pid, b_pid] : part_path<Id, PartId>(u, v)) {
			const auto it = conn_bundles_.find({a_pid, b_pid});
			if (it == conn_bundles_.end()) {
				std::unreachable();
			}
			const auto &bundle = it->second.wrapped();
			const auto &T_a_b = a_pid < b_pid ? bundle.T_a_b : bundle.T_b_a;
			T = T * T_a_b;
			has_path = true;
		}
		if (has_path) {
			return T;
		}
		// Is u equal to v and valid?
		if (u == v && parts_.alive(u)) {
			return T;
		} else {
			return std::nullopt;
		}
	}

	std::optional<InterfaceSpec>
	get_interface_spec(const InterfaceRef &iref) const {
		const auto &[part_id, interface_id] = iref;
		std::optional<InterfaceSpec> result;
		parts_.visit(part_id, [&](const auto &pw) {
			const auto &part = pw.wrapped();
			result = get_interface_at(part, interface_id);
		});
		return result;
	}

	template <class P, class... PEKArgs, class... PWArgs>
	    requires(in_pack<P, Ps...> && sizeof...(PEKArgs) == sizeof...(PEKs) &&
	             ((std::same_as<PEKs, std::remove_cvref_t<PEKArgs>>) && ... &&
	              true) &&
	             std::constructible_from<PartWrapper<P>, PWArgs...,
	                                     std::pmr::memory_resource *>)
	std::optional<PartId> add_part(std::tuple<PEKArgs...> &&keys,
	                               PWArgs &&...args) {
		PartId id = next_part_id_;
		DgVertexId dgid{dynamic_graph_.add_vertex()};
		if (!parts_.template emplace<PartWrapper<P>>(
		        std::tuple_cat(std::make_tuple(id, dgid),
		                       std::forward<std::tuple<PEKArgs...>>(keys)),
		        std::forward<PWArgs>(args)..., res_)) {
			// rollback
			dynamic_graph_.erase_vertex(dgid.value());
			return std::nullopt;
		}
		next_part_id_++;
		call_on_part_added<P>(id);
		return id;
	}

	template <class PK>
	    requires(PartKeys::template contains<PK>)
	std::optional<PartId> remove_part(const PK &key) {
		const PartId *pid_ptr = parts_.template project_key<PK, PartId>(key);
		if (!pid_ptr) {
			return std::nullopt;
		}
		PartId pid = *pid_ptr;

		// Remove connections
		bool visited = parts_.visit(pid, [&]<PartLike P>(PartWrapper<P> &pw) {
			call_on_disconnecting_for_all<P>(pid, pw);
			call_on_bundle_removing_for_all<P>(pid, pw);
			call_on_part_removing<P>(pid, pw);

			for (ConnSegId csid : pw.incomings()) {
				// Delete from the other side
				// This is hole, so other side is stud
				const ConnSegRef *csref_ptr =
				    conn_segs_.template project<ConnSegId, ConnSegRef>(csid);
				if (!csref_ptr) {
					std::unreachable();
				}
				const auto &[stud_if_ref, hole_if_ref] = *csref_ptr;
				const auto &[stud_pid, stud_ifid] = stud_if_ref;
				bool visited = parts_.visit(stud_pid, [&](auto &stud_pw) {
					if (!stud_pw.outgoings().remove(csid)) {
						std::unreachable();
					}
				});
				if (!visited) {
					std::unreachable();
				}
				if (!conn_segs_.erase_by_key(csid)) {
					std::unreachable();
				}
			}
			for (ConnSegId csid : pw.outgoings()) {
				// Delete from the other side
				// This is stud, so other side is hole
				const ConnSegRef *csref_ptr =
				    conn_segs_.template project<ConnSegId, ConnSegRef>(csid);
				if (!csref_ptr) {
					std::unreachable();
				}
				const auto &[stud_if_ref, hole_if_ref] = *csref_ptr;
				const auto &[hole_pid, hole_ifid] = hole_if_ref;
				bool visited = parts_.visit(hole_pid, [&](auto &hole_pw) {
					if (!hole_pw.incomings().remove(csid)) {
						std::unreachable();
					}
				});
				if (!visited) {
					std::unreachable();
				}
				if (!conn_segs_.erase_by_key(csid)) {
					std::unreachable();
				}
			}
			for (PartId npid : pw.neighbor_parts()) {
				bool visited = parts_.visit(npid, [&](auto &npw) {
					if (!npw.neighbor_parts().remove(pid)) {
						std::unreachable();
					}
				});
				if (!visited) {
					std::unreachable();
				}
				if (!conn_bundles_.erase({pid, npid})) {
					std::unreachable();
				}
			}
		});
		if (!visited) {
			std::unreachable();
		}

		// Remove from dynamic graph
		const DgVertexId *dgid =
		    parts_.template project_key<PartId, DgVertexId>(pid);
		if (!(dgid && dynamic_graph_.erase_vertex(dgid->value()))) {
			std::unreachable();
		}

		// Finally remove the part itself
		if (!parts_.erase_by_key(key)) {
			std::unreachable();
		}
		return pid;
	}

	template <class... CSEKArgs, class... CSWArgs>
	    requires(sizeof...(CSEKArgs) == sizeof...(CSEKs) &&
	             ((std::same_as<CSEKs, std::remove_cvref_t<CSEKArgs>>) && ... &&
	              true) &&
	             std::constructible_from<ConnSegWrapper, CSWArgs...,
	                                     std::pmr::memory_resource *>)
	std::optional<ConnSegId>
	connect(const InterfaceRef &stud_if, const InterfaceRef &hole_if,
	        std::tuple<CSEKArgs...> &&keys, CSWArgs &&...args) {
		ConnSegRef csref{stud_if, hole_if};
		if (conn_segs_.contains(csref)) {
			return std::nullopt;
		}
		std::optional<InterfaceSpec> stud_spec = get_interface_spec(stud_if);
		std::optional<InterfaceSpec> hole_spec = get_interface_spec(hole_if);
		if (!stud_spec || !hole_spec) {
			return std::nullopt;
		}
		if (!(stud_spec->type == InterfaceType::Stud &&
		      hole_spec->type == InterfaceType::Hole)) {
			return std::nullopt;
		}

		const auto &[stud_pid, stud_ifid] = stud_if;
		const auto &[hole_pid, hole_ifid] = hole_if;
		if (stud_pid == hole_pid) {
			return std::nullopt;
		}

		ConnSegWrapper csw(std::forward<CSWArgs>(args)..., res_);
		Transformd new_transform = SE3d{}.project(
		    csw.wrapped().compute_transform(*stud_spec, *hole_spec));

		ConnectionEndpoint conn_endpoint{stud_pid, hole_pid};
		auto conn_bundle_it = conn_bundles_.find(conn_endpoint);
		bool bundle_exists = conn_bundle_it != conn_bundles_.end();
		if (bundle_exists) {
			auto &bundle = conn_bundle_it->second.wrapped();
			Transformd existent_direct_transform =
			    stud_pid < hole_pid ? bundle.T_a_b : bundle.T_b_a;
			if (!SE3d{}.almost_equal(existent_direct_transform,
			                         new_transform)) {
				return std::nullopt;
			}
		}
		auto existent_transform = lookup_transform(stud_pid, hole_pid);
		if (existent_transform) {
			if (!SE3d{}.almost_equal(*existent_transform, new_transform)) {
				return std::nullopt;
			}
		}
		if (bundle_exists && !existent_transform) {
			std::unreachable();
		}

		ConnSegId csid = next_conn_seg_id_;
		if (!conn_segs_.insert(
		        std::tuple_cat(std::make_tuple(csid, csref),
		                       std::forward<std::tuple<CSEKArgs...>>(keys)),
		        std::move(csw))) {
			return std::nullopt;
		}
		next_conn_seg_id_++;

		if (!bundle_exists) {
			auto [new_it, inserted] =
			    conn_bundles_.emplace(conn_endpoint, ConnBundleWrapper(res_));
			if (!inserted) {
				std::unreachable();
			}
			conn_bundle_it = new_it;
			ConnectionBundle &bundle = conn_bundle_it->second.wrapped();
			if (stud_pid < hole_pid) {
				bundle.T_a_b = new_transform;
				bundle.T_b_a = inverse(new_transform);
			} else {
				bundle.T_b_a = new_transform;
				bundle.T_a_b = inverse(new_transform);
			}

			const DgVertexId *stud_dgid_ptr =
			    parts_.template project_key<PartId, DgVertexId>(stud_pid);
			const DgVertexId *hole_dgid_ptr =
			    parts_.template project_key<PartId, DgVertexId>(hole_pid);
			if (stud_dgid_ptr && hole_dgid_ptr) {
				if (!dynamic_graph_.add_edge(stud_dgid_ptr->value(),
				                             hole_dgid_ptr->value())) {
					std::unreachable();
				}
			} else {
				std::unreachable();
			}
		}
		ConnectionBundle &bundle = conn_bundle_it->second.wrapped();
		if (!bundle.conn_seg_ids.add(csid)) {
			std::unreachable();
		}

		bool stud_visited = parts_.visit(stud_pid, [&](auto &stud_pw) {
			if (!stud_pw.outgoings().add(csid)) {
				std::unreachable();
			}
			bool neighbor_already_exists =
			    !stud_pw.neighbor_parts().add(hole_pid);
			if (bundle_exists != neighbor_already_exists) {
				std::unreachable();
			}
		});
		if (!stud_visited) {
			std::unreachable();
		}

		bool hole_visited = parts_.visit(hole_pid, [&](auto &hole_pw) {
			if (!hole_pw.incomings().add(csid)) {
				std::unreachable();
			}
			bool neighbor_already_exists =
			    !hole_pw.neighbor_parts().add(stud_pid);
			if (bundle_exists != neighbor_already_exists) {
				std::unreachable();
			}
		});
		if (!hole_visited) {
			std::unreachable();
		}
		if (!bundle_exists) {
			call_on_bundle_created(conn_endpoint, conn_bundle_it->second);
		}
		call_on_connected(csid, csref, *stud_spec, *hole_spec,
		                  conn_bundle_it->second);
		return csid;
	}

	template <class ConnId>
	    requires(ConnSegKeys::template contains<ConnId>)
	std::optional<ConnSegId> disconnect(const ConnId &conn_id) {
		const ConnSegId *csid_ptr =
		    conn_segs_.template project<ConnId, ConnSegId>(conn_id);
		if (!csid_ptr) {
			return std::nullopt;
		}
		ConnSegId csid = *csid_ptr;

		const ConnSegRef *csref_ptr =
		    conn_segs_.template project<ConnId, ConnSegRef>(conn_id);
		if (!csref_ptr) {
			std::unreachable();
		}
		const ConnSegRef &csref = *csref_ptr;

		const auto &[stud_if_ref, hole_if_ref] = csref;
		const auto &[stud_pid, stud_ifid] = stud_if_ref;
		const auto &[hole_pid, hole_ifid] = hole_if_ref;

		ConnectionEndpoint conn_endpoint{stud_pid, hole_pid};
		auto conn_bundle_it = conn_bundles_.find(conn_endpoint);
		if (conn_bundle_it == conn_bundles_.end()) {
			std::unreachable();
		}
		ConnectionBundle &bundle = conn_bundle_it->second.wrapped();

		call_on_disconnecting(csid, csref, conn_bundle_it->second);
		bool part_disconnected = bundle.conn_seg_ids.size() == 1;
		if (part_disconnected) {
			call_on_bundle_removing(conn_endpoint, conn_bundle_it->second);
		}

		if (!bundle.conn_seg_ids.remove(csid)) {
			std::unreachable();
		}

		if (part_disconnected) {
			if (!conn_bundles_.erase(conn_endpoint)) {
				std::unreachable();
			}
			const DgVertexId *stud_dgid_ptr =
			    parts_.template project_key<PartId, DgVertexId>(stud_pid);
			const DgVertexId *hole_dgid_ptr =
			    parts_.template project_key<PartId, DgVertexId>(hole_pid);
			if (stud_dgid_ptr && hole_dgid_ptr) {
				if (!dynamic_graph_.erase_edge(stud_dgid_ptr->value(),
				                               hole_dgid_ptr->value())) {
					std::unreachable();
				}
			} else {
				std::unreachable();
			}
		}

		bool stud_visited = parts_.visit(stud_pid, [&](auto &stud_pw) {
			if (!stud_pw.outgoings().remove(csid)) {
				std::unreachable();
			}
			if (part_disconnected) {
				if (!stud_pw.neighbor_parts().remove(hole_pid)) {
					std::unreachable();
				}
			}
		});
		if (!stud_visited) {
			std::unreachable();
		}

		bool hole_visited = parts_.visit(hole_pid, [&](auto &hole_pw) {
			if (!hole_pw.incomings().remove(csid)) {
				std::unreachable();
			}
			if (part_disconnected) {
				if (!hole_pw.neighbor_parts().remove(stud_pid)) {
					std::unreachable();
				}
			}
		});
		if (!hole_visited) {
			std::unreachable();
		}

		if (!conn_segs_.erase_by_key(csid)) {
			std::unreachable();
		}
		return csid;
	}

	Hooks *get_hooks() const noexcept {
		return hooks_;
	}

	void set_hooks(Hooks *hooks) noexcept {
		hooks_ = hooks;
	}

  private:
	std::pmr::memory_resource *res_;
	PartStore parts_;
	PartId next_part_id_ = 0;
	ConnSegStore conn_segs_;
	ConnSegId next_conn_seg_id_ = 0;
	ConnBundleStore conn_bundles_;
	Hooks *hooks_;
	DynamicGraph dynamic_graph_;

	template <class P>
	    requires PartTypeList::template
	contains<P> void call_on_part_added(PartId id) {
		if constexpr (HasOnPartAddedHook<P>) {
			if (!hooks_) {
				return;
			}
			PartWrapper<P> *pw_ptr = parts_.template get<PartWrapper<P>>(id);
			if (!pw_ptr) {
				std::unreachable();
			}
			hooks_->template on_part_added<P>(id, *pw_ptr);
		}
	}

	template <class P>
	    requires PartTypeList::template
	contains<P> void call_on_part_removing(PartId id, PartWrapper<P> &pw) {
		if constexpr (HasOnPartRemovingHook<P>) {
			if (!hooks_) {
				return;
			}
			hooks_->template on_part_removing<P>(id, pw);
		}
	}

	void call_on_connected(ConnSegId csid, const ConnSegRef &csref,
	                       const InterfaceSpec &stud_spec,
	                       const InterfaceSpec &hole_spec,
	                       ConnBundleWrapper &cbw) {
		if constexpr (HasOnConnectedHook) {
			if (!hooks_) {
				return;
			}
			ConnSegWrapper *csw_ptr = conn_segs_.find(csid);
			if (!csw_ptr) {
				std::unreachable();
			}
			hooks_->on_connected(csid, csref, stud_spec, hole_spec, *csw_ptr,
			                     cbw);
		}
	}

	void call_on_disconnecting(ConnSegId csid, const ConnSegRef &csref,
	                           ConnBundleWrapper &cbw) {
		if constexpr (HasOnDisconnectingHook) {
			if (!hooks_) {
				return;
			}
			ConnSegWrapper *csw_ptr = conn_segs_.find(csid);
			if (!csw_ptr) {
				std::unreachable();
			}
			hooks_->on_disconnecting(csid, csref, *csw_ptr, cbw);
		}
	}

	template <class P>
	    requires PartTypeList::template
	contains<P> void call_on_disconnecting_for_all(PartId pid,
	                                               PartWrapper<P> &pw) {
		if constexpr (HasOnDisconnectingHook) {
			if (!hooks_) {
				return;
			}
			for (ConnSegId csid : pw.incomings()) {
				// This is hole, so other side is stud
				const ConnSegRef *csref_ptr =
				    conn_segs_.template project<ConnSegId, ConnSegRef>(csid);
				if (!csref_ptr) {
					std::unreachable();
				}
				const auto &[stud_if_ref, hole_if_ref] = *csref_ptr;
				const auto &[stud_pid, stud_ifid] = stud_if_ref;
				ConnectionEndpoint ep{stud_pid, pid};
				auto conn_bundle_it = conn_bundles_.find(ep);
				if (conn_bundle_it == conn_bundles_.end()) {
					std::unreachable();
				}
				auto csw_ptr = conn_segs_.find(csid);
				if (!csw_ptr) {
					std::unreachable();
				}
				hooks_->on_disconnecting(csid, *csref_ptr, *csw_ptr,
				                         conn_bundle_it->second);
			}
			for (ConnSegId csid : pw.outgoings()) {
				// This is stud, so other side is hole
				const ConnSegRef *csref_ptr =
				    conn_segs_.template project<ConnSegId, ConnSegRef>(csid);
				if (!csref_ptr) {
					std::unreachable();
				}
				const auto &[stud_if_ref, hole_if_ref] = *csref_ptr;
				const auto &[hole_pid, hole_ifid] = hole_if_ref;
				ConnectionEndpoint ep{pid, hole_pid};
				auto conn_bundle_it = conn_bundles_.find(ep);
				if (conn_bundle_it == conn_bundles_.end()) {
					std::unreachable();
				}
				auto csw_ptr = conn_segs_.find(csid);
				if (!csw_ptr) {
					std::unreachable();
				}
				hooks_->on_disconnecting(csid, *csref_ptr, *csw_ptr,
				                         conn_bundle_it->second);
			}
		}
	}

	void call_on_bundle_created(const ConnectionEndpoint &ep,
	                            ConnBundleWrapper &cbw) {
		if constexpr (HasOnBundleCreatedHook) {
			if (!hooks_) {
				return;
			}
			hooks_->on_bundle_created(ep, cbw);
		}
	}

	void call_on_bundle_removing(const ConnectionEndpoint &ep,
	                             ConnBundleWrapper &cbw) {
		if constexpr (HasOnBundleRemovingHook) {
			if (!hooks_) {
				return;
			}
			hooks_->on_bundle_removing(ep, cbw);
		}
	}

	template <class P>
	    requires PartTypeList::template
	contains<P> void call_on_bundle_removing_for_all(PartId pid,
	                                                 PartWrapper<P> &pw) {
		if constexpr (HasOnBundleRemovingHook) {
			if (!hooks_) {
				return;
			}
			for (PartId npid : pw.neighbor_parts()) {
				ConnectionEndpoint ep{pid, npid};
				auto conn_bundle_it = conn_bundles_.find(ep);
				if (conn_bundle_it == conn_bundles_.end()) {
					std::unreachable();
				}
				hooks_->on_bundle_removing(ep, conn_bundle_it->second);
			}
		}
	}
};

} // namespace lego_assemble
