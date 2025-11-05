import std;
import lego_assemble.utils.type_list;
import lego_assemble.utils.multi_key_map;

#include <cassert>

using lego_assemble::MultiKeyMap;
using lego_assemble::type_list;

static int smoke_basic_two_keys() {
	std::pmr::unsynchronized_pool_resource pool;
	using Keys = type_list<int, std::string>;
	using Map = MultiKeyMap<Keys, std::string>;

	Map m{&pool};
	assert(m.resource() == &pool);
	assert(m.size() == 0);
	assert(m.empty());
	assert(!m.contains(1));
	assert(!m.contains(std::string{"A"}));

	// ---- insert (lvalue keys) success
	Map::tuple_type k1{1, "A"};
	assert(m.insert(k1, "v1"));
	assert(m.size() == 1);
	assert(!m.empty());
	{
		// find (non-const) hit/miss
		auto *p1 = m.find(1);
		assert(p1 && *p1 == "v1");
		auto *p2 = m.find(std::string{"A"});
		assert(p2 && *p2 == "v1");
		auto *p3 = m.find(999); // miss
		assert(p3 == nullptr);
	}

	// const find (const overload) hit/miss
	{
		const Map &cm = m;
		const auto *cp1 = cm.find(1);
		assert(cp1 && *cp1 == "v1");
		const auto *cp2 = cm.find(std::string{"nope"});
		assert(cp2 == nullptr);
	}

	// find_keys hit/miss
	{
		const auto *tk = m.find_keys(1);
		assert(tk != nullptr);
		assert(std::get<0>(*tk) == 1 && std::get<1>(*tk) == "A");
		assert(m.find_keys(std::string{"zzz"}) == nullptr);
	}

	// project hit/miss
	{
		const auto *to = m.project<int, std::string>(1);
		assert(to && *to == "A");
		assert((m.project<int, std::string>(999) == nullptr));
	}

	// ---- insertion failures due to per-key collisions
	// collide on first key
	assert(!m.insert(Map::tuple_type{1, "B"}, "vX"));
	// collide on second key
	assert(!m.insert(Map::tuple_type{2, "A"}, "vX"));
	// collide on both
	assert(!m.insert(Map::tuple_type{1, "A"}, "vX"));
	assert(m.size() == 1);

	// ---- insert (rvalue keys) success for a disjoint tuple
	assert(m.insert(Map::tuple_type{2, "B"}, "v2"));
	assert(m.size() == 2);
	assert(m.contains(2));
	assert(m.contains(std::string{"B"}));

	// ---- replace_value hit/miss
	assert(m.replace_value(2, std::string{"v2b"}));
	assert(*m.find(2) == "v2b");
	assert(!m.replace_value(std::string{"NOPE"}, std::string{"x"}));

	// ---- value_ptr hit/miss
	{
		auto *vp = m.value_ptr(std::string{"B"});
		assert(vp && *vp == "v2b");
		*vp = "v2c";
		assert(*m.find(2) == "v2c");
		assert(m.value_ptr(std::string{"missing"}) == nullptr);
	}

	// ---- emplace with const lvalue keys
	const Map::tuple_type k3{3, "C"};
	assert(m.emplace(k3, "v3"));
	assert(m.size() == 3);

	// ---- emplace with rvalue keys
	assert(m.emplace(Map::tuple_type{4, "D"}, "v4"));
	assert(m.size() == 4);
	assert(m.contains(std::string{"D"}));

	// ---- erase_by_key: victim == last (erase the last inserted: "D")
	assert(m.erase_by_key(std::string{"D"})); // victim==last branch
	assert(!m.contains(std::string{"D"}));
	assert(m.size() == 3);

	// ---- erase_by_key: victim != last (erase the first inserted: key 1)
	assert(m.erase_by_key(1)); // victim!=last branch triggers repoint+move
	assert(!m.contains(1));
	assert(!m.contains(std::string{"A"}));
	// The remaining items should still be findable
	assert(m.find(2) && *m.find(2) == "v2c");
	assert(m.find(3) && *m.find(3) == "v3");

	// ---- erase by tuple (uses first key)
	assert(m.erase(Map::tuple_type{2, "B"}));
	assert(!m.contains(2));
	assert(m.size() == 1);

	// ---- erase_by_key miss
	assert(!m.erase_by_key(std::string{"__nope__"}));
	assert(m.size() == 1);

	// ---- reserve (no visible effect besides not crashing)
	auto prev_size = m.size();
	m.reserve(32);
	assert(m.size() == prev_size);

	// ---- view (span) sanity
	{
		// add two more
		assert(m.insert(Map::tuple_type{5, "E"}, "v5"));
		assert(m.insert(Map::tuple_type{6, "F"}, "v6"));
		auto s = m.view();
		assert(s.size() == m.size());
		// assert that all present keys are reachable via view
		std::size_t hits = 0;
		for (const auto &entry : s) {
			const auto &keys = entry.first;
			const auto &val = entry.second;
			assert(m.find(std::get<0>(keys)) != nullptr);
			assert(m.find(std::get<1>(keys)) != nullptr);
			assert(m.find(std::get<0>(keys)) == m.find(std::get<1>(keys)));
			assert(*(m.find(std::get<0>(keys))) == val);
			++hits;
		}
		assert(hits == s.size());
	}

	// ---- clear
	m.clear();
	assert(m.size() == 0);
	assert(m.empty());
	assert(!m.contains(5));
	assert(!m.contains(std::string{"F"}));

	return 0;
}

static int piecewise_and_move_only() {
	// Piecewise construction (mapped_type requires multi-arg constructor)
	std::pmr::unsynchronized_pool_resource pool1;
	using KeysP = type_list<int, std::string>;
	using MapP = MultiKeyMap<KeysP, std::pair<int, int>>;
	MapP mp{&pool1};
	assert(mp.emplace_piecewise(std::piecewise_construct,
	                            std::forward_as_tuple(10, std::string{"X"}),
	                            std::forward_as_tuple(7, 8)));
	{
		const MapP &cmp = mp;
		const auto *vp = cmp.find(10);
		assert((vp && *vp == std::pair<int, int>(7, 8)));
		assert(cmp.find(std::string{"X"}) != nullptr);
		assert(cmp.find(999) == nullptr);
	}
	// also touch erase_by_key / victim==last path here
	assert(mp.erase_by_key(10));
	assert(mp.size() == 0);

	// Move-only mapped_type coverage
	std::pmr::unsynchronized_pool_resource pool2;
	using KeysU = type_list<int, std::string>;
	using MapU = MultiKeyMap<KeysU, std::unique_ptr<int>>;
	MapU mu{&pool2};

	// insert with lvalue keys
	MapU::tuple_type ku1{7, "G"};
	assert(mu.insert(ku1, std::make_unique<int>(7)));
	{
		auto *p = mu.find(7);
		assert(p && **p == 7);
	}
	// replace_value (move-only)
	assert(mu.replace_value(std::string{"G"}, std::make_unique<int>(77)));
	{
		auto *p = mu.find(7);
		assert(p && **p == 77);
	}
	// emplace with rvalue keys (move-only arg)
	assert(mu.emplace(MapU::tuple_type{8, "H"}, std::make_unique<int>(88)));
	{
		const MapU &cmu = mu;
		auto *p = cmu.find(std::string{"H"});
		assert(p && **p == 88);
	}

	// value_ptr miss
	assert(mu.value_ptr(std::string{"ZZ"}) == nullptr);

	// erase_by_key paths
	// ensure victim!=last by erasing key 7 (first inserted)
	assert(mu.erase_by_key(7));
	// ensure victim==last by erasing the remaining last
	assert(mu.erase_by_key(std::string{"H"}));
	assert(mu.size() == 0);
	return 0;
}

int main() {
	if (int r = smoke_basic_two_keys())
		return r;
	if (int r = piecewise_and_move_only())
		return r;
	std::cout << "All MultiKeyMap tests passed.\n";
	return 0;
}
