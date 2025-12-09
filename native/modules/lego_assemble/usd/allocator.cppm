export module lego_assemble.usd.allocator;

import std;
import lego_assemble.core.specs;
import lego_assemble.core.connections;
import lego_assemble.usd.tokens;
import lego_assemble.usd.author;
import lego_assemble.usd.specs;
import lego_assemble.utils.usd_envs;
import lego_assemble.utils.sdf;
import lego_assemble.vendor;

namespace lego_assemble {

const pxr::SdfPath LegoStatePath("/_LegoState");
const pxr::SdfPath LegoStateNextPartIdProp =
    LegoStatePath.AppendProperty(pxr::TfToken("next_part_id"));
const pxr::SdfPath LegoStateNextConnIdProp =
    LegoStatePath.AppendProperty(pxr::TfToken("next_conn_id"));

const pxr::TfToken kManaged("managed");
const pxr::TfToken kParts("Parts");
const pxr::TfToken kConns("Conns");

export class LegoAllocator {
  public:
	explicit LegoAllocator(pxr::UsdStageRefPtr stage)
	    : stage_{std::move(stage)} {}

	template <PartAuthor PA>
	std::tuple<pxr::SdfPath, InterfaceCollidersVector>
	allocate_part_managed(std::int64_t env_id,
	                      const typename PA::PartType &part) {
		auto path = allocate_part_path(env_id);
		auto colliders = PA{}(stage_, path, part);
		return {path, colliders};
	}

	template <PartAuthor PA>
	std::tuple<pxr::SdfPath, InterfaceCollidersVector>
	allocate_part_unmanaged(const pxr::SdfPath &path,
	                        const typename PA::PartType &part) {
		auto env_id = env_id_from_path(path);
		if (!env_id.has_value()) [[unlikely]] {
			throw std::invalid_argument(
			    "Cannot allocate unmanaged part outside of an environment.");
		}
		auto colliders = PA{}(stage_, path, part);
		return {path, colliders};
	}

	pxr::SdfPath allocate_conn_managed(const pxr::SdfPath &stud,
	                                   InterfaceId stud_if,
	                                   const pxr::SdfPath &hole,
	                                   InterfaceId hole_if,
	                                   const ConnectionSegment &conn_seg) {
		auto stud_env = env_id_from_path(stud);
		auto hole_env = env_id_from_path(hole);
		if (!stud_env.has_value() || !hole_env.has_value() ||
		    stud_env.value() != hole_env.value()) [[unlikely]] {
			throw std::invalid_argument(
			    "Cannot allocate connection between interfaces in different "
			    "environments.");
		}
		auto env_id = stud_env.value();
		auto path = allocate_conn_path(env_id);
		author_connection(stage_, path, stud, stud_if, hole, hole_if, conn_seg);
		return path;
	}

	void allocate_conn_unmanaged(const pxr::SdfPath &conn_path,
	                             const pxr::SdfPath &stud, InterfaceId stud_if,
	                             const pxr::SdfPath &hole, InterfaceId hole_if,
	                             const ConnectionSegment &conn_seg) {
		auto conn_env = env_id_from_path(conn_path);
		auto stud_env = env_id_from_path(stud);
		auto hole_env = env_id_from_path(hole);
		if (!conn_env.has_value() || !stud_env.has_value() ||
		    !hole_env.has_value() || conn_env.value() != stud_env.value() ||
		    stud_env.value() != hole_env.value()) [[unlikely]] {
			throw std::invalid_argument(
			    "Cannot allocate connection between interfaces in different "
			    "environments.");
		}
		author_connection(stage_, conn_path, stud, stud_if, hole, hole_if,
		                  conn_seg);
	}

	bool deallocate_managed_part(const pxr::SdfPath &path) {
		return deallocate_managed(path, kParts);
	}

	bool deallocate_managed_conn(const pxr::SdfPath &path) {
		return deallocate_managed(path, kConns);
	}

	bool deallocate_managed_all(std::int64_t env_id) {
		auto layer = stage_->GetEditTarget().GetLayer();
		auto managed_root = get_managed_root(env_id);
		auto managed_root_prim = layer->GetPrimAtPath(managed_root);
		if (!managed_root_prim) {
			return false;
		}
		pxr::SdfChangeBlock _changes;
		managed_root_prim->SetNameChildren({});
		return true;
	}

  private:
	pxr::UsdStageRefPtr stage_;

	bool deallocate_managed(const pxr::SdfPath &path,
	                        const pxr::TfToken &group) {
		auto env_id = env_id_from_path(path);
		if (!env_id.has_value()) [[unlikely]] {
			return false;
		}
		auto managed_group = get_managed_group(env_id.value(), group);
		if (!path.HasPrefix(managed_group)) [[unlikely]] {
			return false;
		}
		auto layer = stage_->GetEditTarget().GetLayer();
		if (auto prim = layer->GetPrimAtPath(path)) {
			if (auto parent = prim->GetNameParent()) [[likely]] {
				pxr::SdfChangeBlock _changes;
				parent->RemoveNameChild(prim);
				return true;
			}
		}
		return false;
	}

	pxr::SdfPath ensure_managed_root(std::int64_t env_id) {
		auto layer = stage_->GetEditTarget().GetLayer();
		auto managed_root = get_managed_root(env_id);
		if (layer->GetPrimAtPath(managed_root)) [[likely]] {
			return managed_root;
		}

		pxr::SdfChangeBlock _changes;
		auto managed_root_prim = pxr::SdfCreatePrimInLayer(layer, managed_root);
		managed_root_prim->SetSpecifier(pxr::SdfSpecifierDef);
		managed_root_prim->SetTypeName(pxr::UsdGeomTokens->Scope);
		return managed_root;
	}

	pxr::SdfPath ensure_managed_group(std::int64_t env_id,
	                                  const pxr::TfToken &group) {
		auto layer = stage_->GetEditTarget().GetLayer();
		auto managed_group = get_managed_group(env_id, group);
		if (layer->GetPrimAtPath(managed_group)) [[likely]] {
			return managed_group;
		}

		pxr::SdfChangeBlock _changes;
		auto managed_root = ensure_managed_root(env_id);
		auto managed_group_prim =
		    pxr::SdfCreatePrimInLayer(layer, managed_group);
		managed_group_prim->SetSpecifier(pxr::SdfSpecifierDef);
		managed_group_prim->SetTypeName(pxr::UsdGeomTokens->Scope);
		return managed_group;
	}

	pxr::SdfPath get_managed_root(std::int64_t env_id) {
		return path_for_env(env_id).AppendChild(kManaged);
	}

	pxr::SdfPath get_managed_group(std::int64_t env_id,
	                               const pxr::TfToken &group) {
		return get_managed_root(env_id).AppendChild(group);
	}

	pxr::SdfPath allocate_part_path(std::int64_t env_id) {
		return allocate_path(LegoStateNextPartIdProp, kParts, "Part_{}",
		                     env_id);
	}

	pxr::SdfPath allocate_conn_path(std::int64_t env_id) {
		return allocate_path(LegoStateNextConnIdProp, kConns, "Conn_{}",
		                     env_id);
	}

	pxr::SdfPath allocate_path(const pxr::SdfPath &id_prop,
	                           const pxr::TfToken &group,
	                           std::format_string<std::int64_t> fname,
	                           std::int64_t env_id) {
		auto layer = stage_->GetEditTarget().GetLayer();
		pxr::SdfChangeBlock _changes;

		std::int64_t next_id;
		auto attr = layer->GetAttributeAtPath(id_prop);
		if (attr) [[likely]] {
			next_id = attr->GetDefaultValue().Get<std::int64_t>();
		} else {
			auto state_prim = layer->GetPrimAtPath(LegoStatePath);
			if (!state_prim) {
				state_prim = pxr::SdfCreatePrimInLayer(layer, LegoStatePath);
				state_prim->SetSpecifier(pxr::SdfSpecifierDef);
			}
			attr =
			    pxr::SdfAttributeSpec::New(state_prim, id_prop.GetNameToken(),
			                               pxr::SdfValueTypeNames->Int64,
			                               pxr::SdfVariabilityUniform, true);
			next_id = 0;
		}

		pxr::SdfPath parent = ensure_managed_group(env_id, group);
		pxr::SdfPath path;
		while (layer->GetPrimAtPath(
		    path = parent.AppendChild(
		        pxr::TfToken(std::format(fname, std::int64_t{next_id}))))) {
			next_id++;
		}

		if (!attr->SetDefaultValue(pxr::VtValue(next_id + 1))) [[unlikely]] {
			throw std::runtime_error("Failed to update next_id.");
		}
		return path;
	}
};

} // namespace lego_assemble
