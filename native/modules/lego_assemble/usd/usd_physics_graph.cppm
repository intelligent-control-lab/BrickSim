export module lego_assemble.usd.usd_physics_graph;

import std;
import lego_assemble.core.specs;
import lego_assemble.core.graph;
import lego_assemble.core.connections;
import lego_assemble.core.assembly;
import lego_assemble.physx.physics_graph;
import lego_assemble.utils.type_list;
import lego_assemble.vendor.physx;
import lego_assemble.vendor.pxr;

namespace lego_assemble {

using PEKs = type_list<pxr::SdfPath>;
using PEKHs = type_list<pxr::SdfPath::Hash>;
using PEKEqs = type_list<std::equal_to<>>;

template <bool SyncConns>
using CSEKs =
    std::conditional_t<SyncConns, type_list<pxr::SdfPath>, type_list<>>;
template <bool SyncConns>
using CSEKHs =
    std::conditional_t<SyncConns, type_list<pxr::SdfPath::Hash>, type_list<>>;
template <bool SyncConns>
using CSEKEqs =
    std::conditional_t<SyncConns, type_list<std::equal_to<>>, type_list<>>;

export template <bool SyncConns, class Ps>
class UsdPhysicsLegoGraph final
    : public PhysicsLegoGraph<Ps, PEKs, PEKHs, PEKEqs, CSEKs<SyncConns>,
                              CSEKHs<SyncConns>, CSEKEqs<SyncConns>> {
  public:
	using Base = PhysicsLegoGraph<Ps, PEKs, PEKHs, PEKEqs, CSEKs<SyncConns>,
	                              CSEKHs<SyncConns>, CSEKEqs<SyncConns>>;
	using Self = UsdPhysicsLegoGraph<SyncConns, Ps>;
};

} // namespace lego_assemble
