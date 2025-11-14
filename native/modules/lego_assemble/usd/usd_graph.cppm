export module lego_assemble.usd.usd_graph;

import std;
import lego_assemble.core.specs;
import lego_assemble.core.graph;
import lego_assemble.core.connections;
import lego_assemble.usd.allocator;
import lego_assemble.usd.author;
import lego_assemble.usd.parse;
import lego_assemble.utils.type_list;
import lego_assemble.utils.poly_store;
import lego_assemble.utils.unique_set;
import lego_assemble.vendor.pxr;
import lego_assemble.vendor.carb;

namespace lego_assemble {

using PEKs = type_list<pxr::SdfPath>;
using PEKHs = type_list<pxr::SdfPath::Hash>;
using PEKEqs = type_list<std::equal_to<>>;

using CSEKs = type_list<pxr::SdfPath>;
using CSEKHs = type_list<pxr::SdfPath::Hash>;
using CSEKEqs = type_list<std::equal_to<>>;

template <class T> using GetPartType = typename T::PartType;

export template <class Parts, class PartAuthors, class PartParsers>
class UsdLegoGraph;

export template <class... Ps, class... PAs, class... PPs>
class UsdLegoGraph<type_list<Ps...>, type_list<PAs...>, type_list<PPs...>>
    : public LegoGraph<type_list<Ps...>, SimplePartWrapper, pmr_vector_storage,
                       PEKs, PEKHs, PEKEqs, SimpleWrapper<ConnectionSegment>,
                       CSEKs, CSEKHs, CSEKEqs,
                       SimpleWrapper<ConnectionBundle>> {
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

	using Base =
	    LegoGraph<type_list<Ps...>, SimplePartWrapper, pmr_vector_storage, PEKs,
	              PEKHs, PEKEqs, SimpleWrapper<ConnectionSegment>, CSEKs,
	              CSEKHs, CSEKEqs, SimpleWrapper<ConnectionBundle>>;
	using Self =
	    UsdLegoGraph<type_list<Ps...>, type_list<PAs...>, type_list<PPs...>>;

	template <PartLike P>
	    requires PartAuthorPartTypes::template
	contains<P> using PartAuthorFor = typename PartAuthorList::template at<
	    PartAuthorPartTypes::template index_of<P>>;

	template <PartLike P>
	    requires PartParserPartTypes::template
	contains<P> using PartParserFor = typename PartParserList::template at<
	    PartParserPartTypes::template index_of<P>>;

	explicit UsdLegoGraph(
	    pxr::UsdStageRefPtr stage,
	    std::pmr::memory_resource *mr = std::pmr::get_default_resource())
	    : Base(mr), stage_(std::move(stage)) {
		notice_sink_ = std::make_unique<UsdNoticeSink>(*this);
		initial_sync();
		notice_sink_->connect();
	}

	~UsdLegoGraph() {}

  protected:
	friend Base;

	class UsdNoticeSink : public pxr::TfWeakBase {
	  public:
		explicit UsdNoticeSink(Self &owner) : owner_(owner) {}
		~UsdNoticeSink() {
			disconnect();
		}

		void connect() {
			pxr::TfWeakPtr<UsdNoticeSink> weak_this =
			    pxr::TfCreateWeakPtr(this);
			if (!objects_key_) {
				objects_key_ = pxr::TfNotice::Register(
				    weak_this, &UsdNoticeSink::on_objects_changed,
				    owner_.stage_);
			}
		}

		void disconnect() {
			if (objects_key_) {
				pxr::TfNotice::Revoke(*objects_key_);
				objects_key_ = std::nullopt;
			}
		}

		void on_objects_changed(const pxr::UsdNotice::ObjectsChanged &notice,
		                        const pxr::UsdStageWeakPtr &sender) {
			owner_.resync_tree(notice.GetResyncedPaths(),
			                   notice.GetChangedInfoOnlyPaths());
		}

	  private:
		Self &owner_;
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

	pxr::UsdStageRefPtr stage_;
	std::unique_ptr<UsdNoticeSink> notice_sink_;

	// Entry exists iif.
	// 1. This is a realized part prim in USD.
	//    In this case pid has value; OR
	// 2. This is not a realized part prim in USD, AND
	// 	  there are unrealized connections involving this path.
	//    In this case pid has no value.
	pxr::SdfPathTable<UsdPartInfo> part_path_table_;

	// Entry exists iif. Connection prim exists in USD.
	// 1. Connection is realized, in which case csid has value; or
	// 2. Connection is not realized, in which case csid has no value.
	//    In this case, the involving parts exist in part_path_table_, AND
	//	  this connection path exists in ALL involving parts' unrealized_conns.
	//    Note: this connection can be unrealized even when both involving parts are realized.
	//          For example, (but not limited) if the connection causes inconsistent transforms.
	pxr::SdfPathTable<UsdConnInfo> conn_path_table_;

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
			try_parse_part(
			    prim, [&]<PartLike P>(PartPrimParseResult<P> result) {
				    pxr::SdfPath path = prim.GetPath();
				    auto &[part, colliders] = result;
				    std::optional<PartId> part_id = this->template add_part<P>(
				        std::forward_as_tuple(path), std::move(part));
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
					std::optional<ConnSegId> csid = this->connect(
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
		        PartId pid,
		        const SimplePartWrapper<P> &pw) -> bool /* success */ {
			std::vector<pxr::SdfPath> affected_conn_paths;
			affected_conn_paths.reserve(pw.incomings().size() +
			                            pw.outgoings().size());
			auto add_conn_path = [&](const OrderedVecSet<ConnSegId> &csids) {
				for (ConnSegId csid : csids) {
					const pxr::SdfPath *path_ptr =
					    this->conn_segs_
					        .template project<ConnSegId, pxr::SdfPath>(csid);
					if (!path_ptr) {
						log_error(
						    "UsdLegoGraph: failed to find connection path "
						    "for conn seg id {} during part removal",
						    csid);
						continue;
					}
					affected_conn_paths.push_back(*path_ptr);
				}
			};
			add_conn_path(pw.incomings());
			add_conn_path(pw.outgoings());

			bool removed = this->remove_part(pid).has_value();
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
		                            const SimplePartWrapper<Old> &old_pw)
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
			std::optional<PartId> new_pid = this->template add_part<New>(
			    std::forward_as_tuple(path), std::move(new_part));
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
		auto do_part_added = [&]<PartLike P> [[nodiscard]] (
		                         const pxr::SdfPath &path,
		                         UsdPartInfo &part_info,
		                         P &&new_part) -> bool /* erase_part_info */ {
			std::optional<PartId> new_pid = this->template add_part<P>(
			    std::forward_as_tuple(path), std::move(new_part));
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
			bool visited = this->parts().visit(
			    old_pid, [&]<PartLike P>(const SimplePartWrapper<P> &old_pw) {
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
					bool pid_exists = this->parts().alive(pid);
					if (!pid_exists) [[unlikely]] {
						log_error(
						    "UsdLegoGraph: part id {} for path {} not found in "
						    "parts storage",
						    pid, path.GetText());
						return;
					}
					const SimplePartWrapper<P> *existing_part =
					    this->parts().template get<SimplePartWrapper<P>>(pid);
					if (existing_part) {
						if (existing_part->wrapped() == part) {
							// Unmodified
							// No action needed.
						} else {
							// Modified, same type
							erase_part_info = do_part_modified(
							    path, part_info, pid, std::move(part),
							    *existing_part);
						}
					} else {
						// Modified, different type
						this->parts().visit(pid, [&](const auto &old_pw) {
							erase_part_info = do_part_modified(
							    path, part_info, pid, std::move(part), old_pw);
						});
					}
				} else {
					// Before: part NOT recognized.
					// Now: part recognized.
					erase_part_info =
					    do_part_added(path, part_info, std::move(part));
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
				auto [first, last] =
				    part_path_table_.FindSubtreeRange(root_path);
				for (auto it = first; it != last; ++it) {
					const pxr::SdfPath &path = it->first;
					if (stage_->GetPrimAtPath(path).IsValid()) {
						continue;
					}
					UsdPartInfo &part_info = it->second;
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
		auto remove_unrealized_from_part = [&](const pxr::SdfPath &part_path,
		                                       const pxr::SdfPath &conn_path) {
			auto part_info_it = part_path_table_.find(part_path);
			if (part_info_it == part_path_table_.end()) {
				return;
			}
			UsdPartInfo &part_info = part_info_it->second;
			part_info.unrealized_conns.remove(conn_path);
			if (part_info.unrealized_conns.empty() &&
			    !part_info.pid.has_value()) {
				part_path_table_.erase(part_info_it);
			}
		};
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
				bool disconnected = this->disconnect(csid).has_value();
				if (!disconnected) [[unlikely]] {
					log_warn("UsdLegoGraph: failed to disconnect conn "
					         "seg id {} for deleted connection prim {}",
					         csid, path.GetText());
					return;
				}
				conn_info.csid = std::nullopt;
			} else {
				// Delete unrealized connection
				remove_unrealized_from_part(conn_info.stud, path);
				remove_unrealized_from_part(conn_info.hole, path);
			}
		};
		auto do_conn_update = [&](const pxr::SdfPath &path,
		                          UsdConnInfo &conn_info,
		                          ConnectionPrimParseResult &&conn) {
			if (conn_info.csid.has_value()) {
				// Update realized connection
				ConnSegId csid = *conn_info.csid;
				const ConnSegRef *csref =
				    this->connection_segments()
				        .template project<ConnSegId, ConnSegRef>(csid);
				if (!csref) [[unlikely]] {
					log_error("UsdLegoGraph: failed to find conn seg ref "
					          "for conn seg id {} during resync",
					          csid);
					return;
				}
				const SimpleWrapper<ConnectionSegment> *csw =
				    this->connection_segments().find(csid);
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
					bool disconnected = this->disconnect(csid).has_value();
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
					remove_unrealized_from_part(conn_info.stud, path);
					remove_unrealized_from_part(conn_info.hole, path);
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
				auto [first, last] =
				    conn_path_table_.FindSubtreeRange(root_path);
				for (auto it = first; it != last; ++it) {
					const pxr::SdfPath &path = it->first;
					if (dirty_conns.contains(path)) {
						continue;
					}
					if (stage_->GetPrimAtPath(path).IsValid()) {
						continue;
					}
					UsdConnInfo &conn_info = it->second;
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
			    this->parts().template project_key<pxr::SdfPath, PartId>(
			        conn.stud_path);
			const PartId *hole_id_ptr =
			    this->parts().template project_key<pxr::SdfPath, PartId>(
			        conn.hole_path);
			if (!stud_id_ptr || !hole_id_ptr) {
				continue;
			}
			PartId stud_pid = *stud_id_ptr;
			PartId hole_pid = *hole_id_ptr;
			InterfaceRef stud_if_ref{stud_pid, conn.stud_interface};
			InterfaceRef hole_if_ref{hole_pid, conn.hole_interface};
			std::optional<ConnSegId> csid = this->connect(
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
};

} // namespace lego_assemble
