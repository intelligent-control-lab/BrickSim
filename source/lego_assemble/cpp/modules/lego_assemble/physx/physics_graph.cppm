export module lego_assemble.physx.physics_graph;

import std;
import lego_assemble.core.specs;
import lego_assemble.core.graph;
import lego_assemble.core.connections;
import lego_assemble.utils.type_list;
import lego_assemble.utils.poly_store;
import lego_assemble.vendor.physx;

namespace lego_assemble {

export template <
    class Ps, template <class> class PartWrapper = SimplePartWrapper,
    class PartExtraKeys = type_list<>, class PartExtraKeysHash = type_list<>,
    class PartExtraKeysEq = type_list<>,
    class ConnSegWrapper = SimpleWrapper<ConnectionSegment>,
    class ConnSegExtraKeys = type_list<>,
    class ConnSegExtraKeysHash = type_list<>,
    class ConnSegExtraKeysEq = type_list<>,
    class ConnBundleWrapper = SimpleWrapper<ConnectionBundle>>
class PhysicsLegoGraph final
    : public LegoGraph<Ps, PartWrapper, pmr_vector_storage, PartExtraKeys,
                       PartExtraKeysHash, PartExtraKeysEq, ConnSegWrapper,
                       ConnSegExtraKeys, ConnSegExtraKeysHash,
                       ConnSegExtraKeysEq, ConnBundleWrapper> {
  public:
	using Base = LegoGraph<Ps, PartWrapper, pmr_vector_storage, PartExtraKeys,
	                       PartExtraKeysHash, PartExtraKeysEq, ConnSegWrapper,
	                       ConnSegExtraKeys, ConnSegExtraKeysHash,
	                       ConnSegExtraKeysEq, ConnBundleWrapper>;

	explicit PhysicsLegoGraph(
	    std::pmr::memory_resource *r = std::pmr::get_default_resource())
	    : Base(r) {}

	using Base::add_part;
	using Base::connect;
	using Base::disconnect;
	using Base::remove_part;
};

} // namespace lego_assemble
