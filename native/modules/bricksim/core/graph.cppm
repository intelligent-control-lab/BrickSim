export module bricksim.core.graph;

import std;
import bricksim.core.specs;
import bricksim.core.connections;
import bricksim.utils.type_list;
import bricksim.utils.poly_store;
import bricksim.utils.multi_key_map;
import bricksim.utils.pair;
import bricksim.utils.unordered_pair;
import bricksim.utils.unique_set;
import bricksim.utils.dynamic_graph;
import bricksim.utils.typed_id;
import bricksim.utils.transforms;
import bricksim.utils.memory;

namespace bricksim {

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
	    : SimpleWrapper<T>(std::forward<Args>(args)...) {}

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
};
template <class T>
concept ConnBundleWrapperLike =
    WrapperLike<T> && std::same_as<typename T::wrapped_type, ConnectionBundle>;

// For indexing vertices in the dynamic graph
export using DgVertexId = TypedId<struct DgVertexTag, std::uint32_t>;

export struct NoHooks {};

export template <
    class Ps, template <class> class PartWrapper = SimplePartWrapper,
    class PartExtraKeys = type_list<>, class PartExtraKeysHash = type_list<>,
    class PartExtraKeysEq = type_list<>,
    class ConnSegWrapper = SimpleWrapper<ConnectionSegment>,
    class ConnSegExtraKeys = type_list<>,
    class ConnSegExtraKeysHash = type_list<>,
    class ConnSegExtraKeysEq = type_list<>,
    class ConnBundleWrapper = SimpleWrapper<ConnectionBundle>,
    class Hooks = NoHooks, class DynamicGraph = HolmDeLichtenbergThorup>
class LegoGraph;

export template <class... Ps, template <class> class PartWrapper, class... PEKs,
                 class... PEKHs, class... PEKEqs, class ConnSegWrapper,
                 class... CSEKs, class... CSEKHs, class... CSEKEqs,
                 class ConnBundleWrapper, class Hooks, class DynamicGraph>
class LegoGraph<type_list<Ps...>, PartWrapper, type_list<PEKs...>,
                type_list<PEKHs...>, type_list<PEKEqs...>, ConnSegWrapper,
                type_list<CSEKs...>, type_list<CSEKHs...>,
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
	static_assert(
	    DynamicGraphLike<DynamicGraph>,
	    "LegoGraph: DynamicGraph must satisfy DynamicGraphLike concept");
	static_assert(std::is_class_v<Hooks>,
	              "LegoGraph: Hooks must be a class type");

  public:
	using Self = LegoGraph<type_list<Ps...>, PartWrapper, type_list<PEKs...>,
	                       type_list<PEKHs...>, type_list<PEKEqs...>,
	                       ConnSegWrapper, type_list<CSEKs...>,
	                       type_list<CSEKHs...>, type_list<CSEKEqs...>,
	                       ConnBundleWrapper, Hooks, DynamicGraph>;
	using PartTypeList = PartList<Ps...>;
	using WrappedPartList = type_list<PartWrapper<Ps>...>;
	using PartExtraKeys = type_list<PEKs...>;
	using PartKeys = type_list<PartId, DgVertexId, PEKs...>;
	using PartKeysHash =
	    type_list<std::hash<PartId>, std::hash<DgVertexId>, PEKHs...>;
	using PartKeysEq = type_list<std::equal_to<>, std::equal_to<>, PEKEqs...>;
	using PartStore =
	    PolyStore<PartKeys, WrappedPartList, PartKeysHash, PartKeysEq>;
	using WrappedConnSeg = ConnSegWrapper;
	using ConnSegExtraKeys = type_list<CSEKs...>;
	using ConnSegKeys = type_list<ConnSegId, ConnSegRef, CSEKs...>;
	using ConnSegKeysHash =
	    type_list<std::hash<ConnSegId>, ConnSegRefHash, CSEKHs...>;
	using ConnSegKeysEq =
	    type_list<std::equal_to<>, std::equal_to<>, CSEKEqs...>;
	using ConnSegStore = MultiKeyMap<ConnSegKeys, ConnSegWrapper,
	                                 ConnSegKeysHash, ConnSegKeysEq>;
	using WrappedConnBundle = ConnBundleWrapper;
	using ConnBundleStore =
	    std::unordered_map<ConnectionEndpoint, ConnBundleWrapper>;
	using PartEntry = PartStore::entry_reference;
	using PartConstEntry = PartStore::const_entry_reference;
	using ConnSegEntry = ConnSegStore::entry_type &;
	using ConnSegConstEntry = const ConnSegStore::entry_type &;
	using ConnBundleEntry = ConnBundleStore::value_type &;
	using ConnBundleConstEntry = const ConnBundleStore::value_type &;

	static constexpr bool HasOnPartAddedHook =
	    requires(Hooks &hooks, PartEntry entry) {
		    {
			    // Called after a new part is added
			    hooks.on_part_added(entry)
		    } -> std::same_as<void>;
	    };

	static constexpr bool HasOnPartRemovingHook =
	    requires(Hooks &hooks, PartEntry entry) {
		    {
			    // Called before a part is removed.
			    // All connections involving this part are still present.
			    hooks.on_part_removing(entry)
		    } -> std::same_as<void>;
	    };

	static constexpr bool HasOnPartRemovedHook =
	    requires(Hooks &hooks, PartId pid) {
		    {
			    // Called after a part is removed.
			    // All connections involving this part have been removed.
			    // The part isself is also gone.
			    hooks.on_part_removed(pid)
		    } -> std::same_as<void>;
	    };

	static constexpr bool HasOnConnectedHook = requires(
	    Hooks &hooks, ConnSegEntry cs_entry, ConnBundleEntry cb_entry,
	    const InterfaceSpec &stud_spec, const InterfaceSpec &hole_spec) {
		{
			// Called after a new connection segment is created.
			hooks.on_connected(cs_entry, cb_entry, stud_spec, hole_spec)
		} -> std::same_as<void>;
	};

	static constexpr bool HasOnDisconnectingHook = requires(
	    Hooks &hooks, ConnSegEntry cs_entry, ConnBundleEntry cb_entry) {
		{
			// Called before a connection segment is removed.
			// When called from explicit disconnect,
			// the connection segment is still present in the graph and in cbw.
			// When called from part removal,
			// all connection segments involving the part are still present in
			// the graph and in cbw,
			// and this is called for each such connection segment,
			// and it's called before on_part_removing for that part.
			hooks.on_disconnecting(cs_entry, cb_entry)
		} -> std::same_as<void>;
	};

	static constexpr bool HasOnDisconnectedHook =
	    requires(Hooks &hooks, ConnSegId csid, const ConnSegRef &csref) {
		    {
			    // Called after a connection segment is removed.
			    // When called from explicit disconnect,
			    // the connection segment is already gone from the graph and from cbw.
			    // When called from part removal,
			    // all connection segments involving the part,
			    // and the part itself, are already gone from the graph and from cbw.
			    // This is called for each such connection segment.
			    // And this is called before on_part_removed for that part.
			    hooks.on_disconnected(csid, csref)
		    } -> std::same_as<void>;
	    };

	static constexpr bool HasOnBundleCreatedHook =
	    requires(Hooks &hooks, ConnBundleEntry cb_entry) {
		    {
			    // Called after a new connection bundle is created.
			    // The connection segment causing the bundle creation is added to graph
			    // and in cbw. cbw's transform has been set up.
			    // This is called before on_connected.
			    hooks.on_bundle_created(cb_entry)
		    } -> std::same_as<void>;
	    };

	static constexpr bool HasOnBundleRemovingHook =
	    requires(Hooks &hooks, ConnBundleEntry cb_entry) {
		    {
			    // Called before a connection bundle is removed.
			    // If this is caused by a single disconnection, that connection segment
			    // is still present in the graph and in cbw,
			    // and it's called after on_disconnecting.
			    // If this is caused by part removal, the part and all relevant
			    // connection segments are still present in the graph and in cbw,
			    // and it's called before on_part_removing for that part.
			    hooks.on_bundle_removing(cb_entry)
		    } -> std::same_as<void>;
	    };

	static constexpr bool HasOnBundleRemovedHook =
	    requires(Hooks &hooks, const ConnectionEndpoint &ep) {
		    {
			    // Called after a connection bundle is removed.
			    // If this is caused by a single disconnection, that connection segment
			    // is already gone from the graph and from cbw.
			    // If this is caused by part removal, the part and all relevant
			    // connection segments/bundles are already gone from the graph and from cbw.
			    hooks.on_bundle_removed(ep)
		    } -> std::same_as<void>;
	    };

  public:
	class ComponentView {
	  public:
		[[nodiscard]] PartId root() const {
			return root_;
		}

		[[nodiscard]] std::size_t size() const {
			DgVertexId dgid = g_->parts_.template key_of<DgVertexId>(root_);
			return g_->dynamic_graph_.component_view(dgid.value()).size();
		}

		// Yields PartIds in the component
		std::generator<PartId> vertices() const {
			DgVertexId root_dgid =
			    g_->parts_.template key_of<DgVertexId>(root_);

			for (vertex_id vid :
			     g_->dynamic_graph_.component_view(root_dgid.value())
			         .vertices()) {
				co_yield g_->parts_.template key_of<PartId>(DgVertexId{vid});
			}
		}

		// Yields pairs of PartIds representing edges in the component
		// Optional boolean: tree_only (default false returns all edges, true returns spanning forest)
		std::generator<std::pair<PartId, PartId>>
		edges(bool tree_only = false) const {
			DgVertexId root_dgid =
			    g_->parts_.template key_of<DgVertexId>(root_);

			auto view = g_->dynamic_graph_.component_view(root_dgid.value());

			// Select generator based on flag
			auto edge_gen = tree_only ? view.tree_edges() : view.edges();

			for (auto [u_vid, v_vid] : edge_gen) {
				co_yield {
				    g_->parts_.template key_of<PartId>(DgVertexId{u_vid}),
				    g_->parts_.template key_of<PartId>(DgVertexId{v_vid}),
				};
			}
		}

		// Calculates transforms on the fly by traversing the HLT spanning tree.
		// Complexity: O(N) where N is component size. No cycle checks needed.
		aligned_generator<std::pair<PartId, Transformd>> transforms() const {
			DgVertexId root_dgid =
			    g_->parts_.template key_of<DgVertexId>(root_);

			// 1. Yield Identity for root
			Transformd identity = SE3d{}.identity();
			co_yield {root_, identity};

			// 2. Cache for calculated transforms relative to root
			std::unordered_map<PartId, Transformd> cache;
			cache.reserve(size()); // Hint size
			cache.emplace(root_, identity);

			// 3. Traverse the Spanning Tree (F0) from DynamicGraph
			//    tree_edges() is guaranteed to yield edges {parent, child} in a BFS/traversal order
			//    stemming from the view's root.
			auto view = g_->dynamic_graph_.component_view(root_dgid.value());

			for (auto [parent_vid, child_vid] : view.tree_edges()) {
				PartId parent_id =
				    g_->parts_.template key_of<PartId>(DgVertexId{parent_vid});
				PartId child_id =
				    g_->parts_.template key_of<PartId>(DgVertexId{child_vid});

				const Transformd &T_root_parent = cache.at(parent_id);

				// Retrieve connection transform T_parent_child
				const auto &cbw = g_->conn_bundles_.at({parent_id, child_id});
				const ConnectionBundle &bundle = cbw.wrapped();
				const Transformd &T_parent_child =
				    (parent_id < child_id) ? bundle.T_a_b : bundle.T_b_a;

				// Compose: T_root_child = T_root_parent * T_parent_child
				Transformd T_root_child = T_root_parent * T_parent_child;

				// Yield and Cache
				cache.emplace(child_id, T_root_child);
				co_yield {child_id, T_root_child};
			}
		}

		std::generator<ConnSegConstEntry> connection_segments() const {
			DgVertexId root_dgid =
			    g_->parts_.template key_of<DgVertexId>(root_);

			for (vertex_id vid :
			     g_->dynamic_graph_.component_view(root_dgid.value())
			         .vertices()) {
				for (ConnSegId csid :
				     g_->parts_.visit(DgVertexId{vid}, [](const auto &pw) {
					     return pw.incomings();
				     })) {
					co_yield g_->connection_segments().entry_of(csid);
				}
			}
		}

	  private:
		const Self *g_;
		PartId root_;

		ComponentView(const LegoGraph &g, PartId root) : g_(&g), root_(root) {}

		friend Self;
	};

	explicit LegoGraph(Hooks *hooks = nullptr) : hooks_{hooks} {}

	const PartStore &parts() const noexcept {
		return parts_;
	}

	decltype(auto) part_at(this auto &&self, const auto &key)
	    requires(
	        PartKeys::template contains<std::remove_cvref_t<decltype(key)>>)
	{
		return self.parts_.value_of(key);
	}

	const ConnSegStore &connection_segments() const noexcept {
		return conn_segs_;
	}

	decltype(auto) connection_segment_at(this auto &&self, const auto &key)
	    requires(
	        ConnSegKeys::template contains<std::remove_cvref_t<decltype(key)>>)
	{
		return self.conn_segs_.value_of(key);
	}

	const ConnBundleStore &connection_bundles() const noexcept {
		return conn_bundles_;
	}

	decltype(auto) connection_bundle_at(this auto &&self,
	                                    const ConnectionEndpoint &ep) {
		return self.conn_bundles_.at(ep);
	}

	const DynamicGraph &dynamic_graph() const noexcept {
		return dynamic_graph_;
	}

	// Requires u and v are valid
	bool is_connected(PartId u, PartId v) const {
		DgVertexId u_dgid = parts_.template key_of<DgVertexId>(u);
		DgVertexId v_dgid = parts_.template key_of<DgVertexId>(v);
		return dynamic_graph_.connected(u_dgid.value(), v_dgid.value());
	}

	// Requires u and v are valid
	template <class InputId = PartId, class OutputId = InputId>
	std::generator<std::pair<OutputId, OutputId>>
	part_path(const InputId &u, const InputId &v) const
	    requires(PartKeys::template contains<InputId> &&
	             PartKeys::template contains<OutputId>)
	{
		DgVertexId u_dgid = parts_.template key_of<DgVertexId>(u);
		DgVertexId v_dgid = parts_.template key_of<DgVertexId>(v);
		for (auto [p_vid, q_vid] :
		     dynamic_graph_.path(u_dgid.value(), v_dgid.value())) {
			co_yield {
			    parts_.template key_of<OutputId>(DgVertexId{p_vid}),
			    parts_.template key_of<OutputId>(DgVertexId{q_vid}),
			};
		}
	}

	// Requires u and v are valid
	template <class Id = PartId>
	std::optional<Transformd> lookup_transform(const Id &u, const Id &v) const
	    requires(PartKeys::template contains<Id>)
	{
		Transformd T = SE3d{}.identity();
		bool has_path = false;
		for (auto [a_pid, b_pid] : part_path<Id, PartId>(u, v)) {
			const auto &cbw = conn_bundles_.at({a_pid, b_pid});
			const auto &bundle = cbw.wrapped();
			const auto &T_a_b = a_pid < b_pid ? bundle.T_a_b : bundle.T_b_a;
			T = T * T_a_b;
			has_path = true;
		}
		if (has_path || u == v) {
			return T;
		}
		return std::nullopt;
	}

	// Requires root is valid
	ComponentView component_view(PartId root) const {
		if (!parts_.template contains<PartId>(root)) {
			throw std::out_of_range(
			    "LegoGraph::component_view: root part id not found");
		}
		return ComponentView{*this, root};
	}

	std::generator<ComponentView> components() const {
		for (auto dg_view : dynamic_graph_.components()) {
			PartId root_pid =
			    parts_.template key_of<PartId>(DgVertexId{dg_view.root()});
			co_yield ComponentView{*this, root_pid};
		}
	}

	std::optional<InterfaceSpec>
	find_interface_spec(const InterfaceRef &iref) const {
		const auto &[part_id, interface_id] = iref;
		std::optional<std::optional<InterfaceSpec>> res =
		    parts_.try_visit(part_id, [&](const auto &pw) {
			    return pw.wrapped().get_interface(interface_id);
		    });
		if (res) {
			return std::move(*res);
		} else {
			return std::nullopt;
		}
	}

	// Throws if not found
	InterfaceSpec interface_spec_at(const InterfaceRef &iref) const {
		const auto &[part_id, interface_id] = iref;
		return parts_.visit(part_id, [&](const auto &pw) {
			auto iface = pw.wrapped().get_interface(interface_id);
			if (!iface) {
				throw std::out_of_range(
				    "LegoGraph::interface_spec_at: interface id not found");
			}
			return std::move(*iface);
		});
	}

	template <in_pack<Ps...> P, class... Args>
	    requires(sizeof...(Args) >= sizeof...(PEKs) &&
	             type_list<Args...>::template take_front<
	                 sizeof...(PEKs)>::template convertible_to<PEKs...> &&
	             type_list<Args...>::template drop_front<
	                 sizeof...(PEKs)>::template can_construct<PartWrapper<P>>)
	std::optional<PartId> add_part(Args &&...args) {
		PartId id = next_part_id_;
		DgVertexId dgid{dynamic_graph_.add_vertex()};
		if (!parts_.template emplace<PartWrapper<P>>(id, dgid, args...)) {
			// rollback
			dynamic_graph_.erase_vertex(dgid.value());
			return std::nullopt;
		}
		next_part_id_++;

		if constexpr (HasOnPartAddedHook) {
			if (hooks_) {
				hooks_->on_part_added(parts_.entry_of(id));
			}
		}

		return id;
	}

	// Returns nullopt if part not found
	template <class PK>
	    requires(PartKeys::template contains<PK>)
	std::optional<PartId> remove_part(const PK &key) {
		auto entry_opt = parts_.find(key);
		if (!entry_opt) {
			return std::nullopt;
		}
		PartEntry entry = *entry_opt;
		PartId pid = entry.template key<PartId>();

		std::optional<std::vector<std::tuple<ConnSegId, ConnSegRef>>>
		    removed_cs;
		if constexpr (HasOnDisconnectedHook) {
			if (hooks_) {
				removed_cs.emplace();
			}
		}

		std::optional<std::vector<ConnectionEndpoint>> removed_bundles;
		if constexpr (HasOnBundleRemovedHook) {
			if (hooks_) {
				removed_bundles.emplace();
			}
		}

		// Remove connections
		entry.visit([&]<PartLike P>(PartWrapper<P> &pw) {
			// Call disconnecting for all connection segments involving this part
			if constexpr (HasOnDisconnectingHook) {
				if (hooks_) {
					auto call_disconnecting = [this](ConnSegId csid) {
						ConnSegEntry cs_entry = conn_segs_.entry_of(csid);
						const auto &[stud_if_ref, hole_if_ref] =
						    cs_entry.template key<ConnSegRef>();
						const auto &[stud_pid, stud_ifid] = stud_if_ref;
						const auto &[hole_pid, hole_ifid] = hole_if_ref;
						ConnectionEndpoint ep{stud_pid, hole_pid};
						ConnBundleEntry cb_entry = *conn_bundles_.find(ep);
						hooks_->on_disconnecting(cs_entry, cb_entry);
					};
					for (ConnSegId csid : pw.incomings()) {
						call_disconnecting(csid);
					}
					for (ConnSegId csid : pw.outgoings()) {
						call_disconnecting(csid);
					}
				}
			}

			// Call bundle removing for all bundles involving this part
			if constexpr (HasOnBundleRemovingHook) {
				if (hooks_) {
					for (PartId npid : pw.neighbor_parts()) {
						ConnectionEndpoint ep{pid, npid};
						ConnBundleEntry cb_entry = *conn_bundles_.find(ep);
						hooks_->on_bundle_removing(cb_entry);
					}
				}
			}

			// Call part removing hook
			if constexpr (HasOnPartRemovingHook) {
				if (hooks_) {
					hooks_->on_part_removing(entry);
				}
			}

			for (ConnSegId csid : pw.incomings()) {
				// Delete from the other side
				// This is hole, so other side is stud
				ConnSegRef csref = conn_segs_.template key_of<ConnSegRef>(csid);
				const auto &[stud_if_ref, hole_if_ref] = csref;
				const auto &[stud_pid, stud_ifid] = stud_if_ref;
				parts_.visit(stud_pid, [&](auto &stud_pw) {
					stud_pw.outgoings().remove(csid);
				});
				conn_segs_.erase(csid);
				if (removed_cs) {
					removed_cs->emplace_back(csid, std::move(csref));
				}
			}
			for (ConnSegId csid : pw.outgoings()) {
				// Delete from the other side
				// This is stud, so other side is hole
				ConnSegRef csref = conn_segs_.template key_of<ConnSegRef>(csid);
				const auto &[stud_if_ref, hole_if_ref] = csref;
				const auto &[hole_pid, hole_ifid] = hole_if_ref;
				parts_.visit(hole_pid, [&](auto &hole_pw) {
					hole_pw.incomings().remove(csid);
				});
				conn_segs_.erase(csid);
				if (removed_cs) {
					removed_cs->emplace_back(csid, std::move(csref));
				}
			}
			for (PartId npid : pw.neighbor_parts()) {
				parts_.visit(
				    npid, [&](auto &npw) { npw.neighbor_parts().remove(pid); });
				conn_bundles_.erase({pid, npid});
				if (removed_bundles) {
					removed_bundles->emplace_back(pid, npid);
				}
			}
		});

		// Remove from dynamic graph
		DgVertexId dgid = entry.template key<DgVertexId>();
		dynamic_graph_.erase_vertex(dgid.value());

		// Finally remove the part itself
		parts_.erase(pid);
		// entry is now invalid

		// Call hooks
		if constexpr (HasOnDisconnectedHook) {
			if (removed_cs) {
				for (const auto &[csid, csref] : *removed_cs) {
					hooks_->on_disconnected(csid, csref);
				}
			}
		}
		if constexpr (HasOnBundleRemovedHook) {
			if (removed_bundles) {
				for (const auto &ep : *removed_bundles) {
					hooks_->on_bundle_removed(ep);
				}
			}
		}
		if constexpr (HasOnPartRemovedHook) {
			if (hooks_) {
				hooks_->on_part_removed(pid);
			}
		}

		return pid;
	}

	// Returns nullopt if connection segment could not be created
	// due to invalid parts, interfaces, self-connection, transform mismatch,
	// or existing connection segment between the same interfaces
	template <class... Args>
	    requires(sizeof...(Args) >= sizeof...(CSEKs) &&
	             type_list<Args...>::template take_front<
	                 sizeof...(CSEKs)>::template convertible_to<CSEKs...> &&
	             type_list<Args...>::template drop_front<
	                 sizeof...(CSEKs)>::template can_construct<ConnSegWrapper>)
	std::optional<ConnSegId> connect(const InterfaceRef &stud_if,
	                                 const InterfaceRef &hole_if,
	                                 Args &&...args) {
		ConnSegRef csref{stud_if, hole_if};
		if (conn_segs_.contains(csref)) {
			// already connected
			return std::nullopt;
		}
		std::optional<InterfaceSpec> stud_spec = find_interface_spec(stud_if);
		std::optional<InterfaceSpec> hole_spec = find_interface_spec(hole_if);
		if (!stud_spec || !hole_spec) {
			// part or interface not found
			return std::nullopt;
		}
		if (!(stud_spec->type == InterfaceType::Stud &&
		      hole_spec->type == InterfaceType::Hole)) {
			// invalid interface types
			return std::nullopt;
		}

		const auto &[stud_pid, stud_ifid] = stud_if;
		const auto &[hole_pid, hole_ifid] = hole_if;
		if (stud_pid == hole_pid) {
			// self-connection not allowed
			return std::nullopt;
		}

		auto csw = std::make_from_tuple<ConnSegWrapper>(
		    select_forward_as_tuple<typename type_list<
		        Args...>::template drop_front_seq<sizeof...(CSEKs)>>(
		        std::forward<Args>(args)...));

		if (!csw.wrapped().compute_overlap(*stud_spec, *hole_spec).is_valid()) {
			// no overlap
			return std::nullopt;
		}

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

		ConnSegId csid = next_conn_seg_id_;
		if (!conn_segs_.emplace(
		        std::tuple_cat(std::make_tuple(csid, csref),
		                       select_forward_as_tuple<
		                           std::make_index_sequence<sizeof...(CSEKs)>>(
		                           std::forward<Args>(args)...)),
		        std::move(csw))) {
			return std::nullopt;
		}
		next_conn_seg_id_++;

		if (!bundle_exists) {
			auto [new_it, _] =
			    conn_bundles_.emplace(conn_endpoint, ConnBundleWrapper{});
			conn_bundle_it = new_it;
			ConnectionBundle &bundle = conn_bundle_it->second.wrapped();
			if (stud_pid < hole_pid) {
				bundle.T_a_b = new_transform;
				bundle.T_b_a = inverse(new_transform);
			} else {
				bundle.T_b_a = new_transform;
				bundle.T_a_b = inverse(new_transform);
			}

			DgVertexId stud_dgid = parts_.template key_of<DgVertexId>(stud_pid);
			DgVertexId hole_dgid = parts_.template key_of<DgVertexId>(hole_pid);
			dynamic_graph_.add_edge(stud_dgid.value(), hole_dgid.value());
		}
		ConnectionBundle &bundle = conn_bundle_it->second.wrapped();
		bundle.conn_seg_ids.add(csid);

		parts_.visit(stud_pid, [&](auto &stud_pw) {
			stud_pw.outgoings().add(csid);
			stud_pw.neighbor_parts().add(hole_pid);
		});

		parts_.visit(hole_pid, [&](auto &hole_pw) {
			hole_pw.incomings().add(csid);
			hole_pw.neighbor_parts().add(stud_pid);
		});
		if constexpr (HasOnBundleCreatedHook) {
			if (!bundle_exists && hooks_) {
				hooks_->on_bundle_created(*conn_bundle_it);
			}
		}
		if constexpr (HasOnConnectedHook) {
			if (hooks_) {
				hooks_->on_connected(conn_segs_.entry_of(csid), *conn_bundle_it,
				                     *stud_spec, *hole_spec);
			}
		}
		return csid;
	}

	// Returns nullopt if connection segment not found
	// Otherwise returns the disconnected ConnSegId
	template <class ConnId>
	    requires(ConnSegKeys::template contains<ConnId>)
	std::optional<ConnSegId> disconnect(const ConnId &conn_id) {
		auto *cs_entry_ptr = conn_segs_.find(conn_id);
		if (!cs_entry_ptr) {
			return std::nullopt;
		}
		ConnSegEntry cs_entry = *cs_entry_ptr;
		ConnSegId csid = cs_entry.template key<ConnSegId>();
		ConnSegRef csref = cs_entry.template key<ConnSegRef>();

		const auto &[stud_if_ref, hole_if_ref] = csref;
		const auto &[stud_pid, stud_ifid] = stud_if_ref;
		const auto &[hole_pid, hole_ifid] = hole_if_ref;
		PartEntry stud_entry = parts_.entry_of(stud_pid);
		PartEntry hole_entry = parts_.entry_of(hole_pid);

		ConnectionEndpoint conn_endpoint{stud_pid, hole_pid};
		ConnBundleEntry cb_entry = *conn_bundles_.find(conn_endpoint);
		ConnectionBundle &bundle = cb_entry.second.wrapped();

		if constexpr (HasOnDisconnectingHook) {
			if (hooks_) {
				hooks_->on_disconnecting(cs_entry, cb_entry);
			}
		}

		bool part_disconnected = bundle.conn_seg_ids.size() == 1;
		if (part_disconnected) {
			if constexpr (HasOnBundleRemovingHook) {
				if (hooks_) {
					hooks_->on_bundle_removing(cb_entry);
				}
			}
		}

		bundle.conn_seg_ids.remove(csid);

		if (part_disconnected) {
			conn_bundles_.erase(conn_endpoint);
			auto stud_dgid = stud_entry.template key<DgVertexId>();
			auto hole_dgid = hole_entry.template key<DgVertexId>();
			dynamic_graph_.erase_edge(stud_dgid.value(), hole_dgid.value());
		}

		stud_entry.visit([&](auto &stud_pw) {
			stud_pw.outgoings().remove(csid);
			if (part_disconnected) {
				stud_pw.neighbor_parts().remove(hole_pid);
			}
		});

		hole_entry.visit([&](auto &hole_pw) {
			hole_pw.incomings().remove(csid);
			if (part_disconnected) {
				hole_pw.neighbor_parts().remove(stud_pid);
			}
		});

		conn_segs_.erase(csid);

		// Call hooks
		if constexpr (HasOnDisconnectedHook) {
			if (hooks_) {
				hooks_->on_disconnected(csid, csref);
			}
		}
		if constexpr (HasOnBundleRemovedHook) {
			if (hooks_ && part_disconnected) {
				hooks_->on_bundle_removed(conn_endpoint);
			}
		}
		return csid;
	}

	Hooks *get_hooks() noexcept {
		return hooks_;
	}

	void set_hooks(Hooks *hooks) noexcept {
		hooks_ = hooks;
	}

  private:
	PartStore parts_;
	PartId next_part_id_ = 0;
	ConnSegStore conn_segs_;
	ConnSegId next_conn_seg_id_ = 0;
	ConnBundleStore conn_bundles_;
	Hooks *hooks_;
	DynamicGraph dynamic_graph_;
};

} // namespace bricksim
