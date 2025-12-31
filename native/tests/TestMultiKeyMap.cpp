import std;
import lego_assemble.utils.type_list;
import lego_assemble.utils.multi_key_map;

#include <cassert>

using namespace lego_assemble;

static int smoke_basic_two_keys() {
	using Keys = type_list<int, std::string>;
	using Map = MultiKeyMap<Keys, std::string>;

	Map m;
	assert(m.size() == 0);
	assert(m.empty());
	assert(!m.contains(1));
	assert(!m.contains(std::string{"A"}));

	// ---- insert (lvalue keys) success
	Map::keys_type k1{1, "A"};
	assert(m.insert(k1, "v1"));
	assert(m.size() == 1);
	assert(!m.empty());
	{
		// find (non-const) hit/miss
		auto *p1 = m.find(1);
		assert(p1 && p1->value() == "v1");

		auto *p2 = m.find(std::string{"A"});
		assert(p2 && p2->value() == "v1");

		auto *p3 = m.find(999); // miss
		assert(p3 == nullptr);
	}

	// const find (const overload) hit/miss
	{
		const Map &cm = m;
		const auto *cp1 = cm.find(1);
		assert(cp1 && cp1->value() == "v1");

		const auto *cp2 = cm.find(std::string{"nope"});
		assert(cp2 == nullptr);
	}

	// find_key (replacement for project/find_keys)
	{
		// Check finding specific key
		const auto *int_k = m.find_key<int>(std::string{"A"});
		assert(int_k && *int_k == 1);

		// Check full tuple access via find
		const auto *entry = m.find(1);
		assert(entry != nullptr);
		assert(std::get<0>(entry->keys()) == 1 &&
		       std::get<1>(entry->keys()) == "A");

		assert(m.find_key<int>(std::string{"zzz"}) == nullptr);
	}

	// ---- insertion failures due to per-key collisions
	// collide on first key
	assert(!m.insert(Map::keys_type{1, "B"}, "vX"));
	// collide on second key
	assert(!m.insert(Map::keys_type{2, "A"}, "vX"));
	// collide on both
	assert(!m.insert(Map::keys_type{1, "A"}, "vX"));
	assert(m.size() == 1);

	// ---- insert (rvalue keys) success for a disjoint tuple
	assert(m.insert(Map::keys_type{2, "B"}, "v2"));
	assert(m.size() == 2);
	assert(m.contains(2));
	assert(m.contains(std::string{"B"}));

	// ---- replace_value (Removed API -> Use find + assign)
	{
		auto *entry = m.find(2);
		assert(entry);
		entry->value() = "v2b";
		assert(m.find(2)->value() == "v2b");
	}
	// miss
	assert(m.find(std::string{"NOPE"}) == nullptr);

	// ---- value_ptr (Removed API -> Use find + value())
	{
		auto *entry = m.find(std::string{"B"});
		assert(entry && entry->value() == "v2b");
		entry->value() = "v2c";
		assert(m.find(2)->value() == "v2c");
	}

	// ---- emplace with const lvalue keys
	const Map::keys_type k3{3, "C"};
	// API Change: emplace(Tuple, Args...)
	assert(m.emplace(k3, "v3"));
	assert(m.size() == 3);

	// ---- emplace with rvalue keys
	assert(m.emplace(Map::keys_type{4, "D"}, "v4"));
	assert(m.size() == 4);
	assert(m.contains(std::string{"D"}));

	// ---- erase: victim == last (erase the last inserted: "D")
	// API Change: renamed to erase
	assert(m.erase(std::string{"D"}));
	assert(!m.contains(std::string{"D"}));
	assert(m.size() == 3);

	// ---- erase: victim != last (erase the first inserted: key 1)
	assert(m.erase(1));
	assert(!m.contains(1));
	assert(!m.contains(std::string{"A"}));

	// The remaining items should still be findable
	assert(m.find(2) && m.find(2)->value() == "v2c");
	assert(m.find(3) && m.find(3)->value() == "v3");

	// ---- erase by tuple (Removed -> Erase by explicit key)
	assert(m.erase(2));
	assert(!m.contains(2));
	assert(m.size() == 1);

	// ---- erase miss
	assert(!m.erase(std::string{"__nope__"}));
	assert(m.size() == 1);

	// ---- reserve (no visible effect besides not crashing)
	auto prev_size = m.size();
	m.reserve(32);
	assert(m.size() == prev_size);

	// ---- view (span) sanity
	{
		// add two more
		assert(m.insert(Map::keys_type{5, "E"}, "v5"));
		assert(m.insert(Map::keys_type{6, "F"}, "v6"));

		auto s = m.view();
		assert(s.size() == m.size());

		std::size_t hits = 0;
		for (const auto &[keys, val] : s) {
			// Check lookup
			assert(m.find(std::get<0>(keys)) != nullptr);
			assert(m.find(std::get<1>(keys)) != nullptr);

			// Pointer identity
			assert(m.find(std::get<0>(keys)) == m.find(std::get<1>(keys)));

			// Value equality
			assert(m.find(std::get<0>(keys))->value() == val);
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
	using KeysP = type_list<int, std::string>;
	using MapP = MultiKeyMap<KeysP, std::pair<int, int>>;
	MapP mp;

	// New emplace naturally supports it: emplace(TupleKeys, Arg1, Arg2...)
	assert(mp.emplace(std::make_tuple(10, std::string{"X"}), 7, 8));

	{
		const MapP &cmp = mp;
		const auto *vp = cmp.find(10);
		assert((vp && vp->value() == std::pair<int, int>(7, 8)));
		assert(cmp.find(std::string{"X"}) != nullptr);
		assert(cmp.find(999) == nullptr);
	}

	// erase
	assert(mp.erase(10));
	assert(mp.size() == 0);

	// Move-only mapped_type coverage
	using KeysU = type_list<int, std::string>;
	using MapU = MultiKeyMap<KeysU, std::unique_ptr<int>>;
	MapU mu;

	// insert with lvalue keys
	MapU::keys_type ku1{7, "G"};
	assert(mu.insert(ku1, std::make_unique<int>(7)));
	{
		auto *p = mu.find(7);
		assert(p && *p->value() == 7);
	}

	// replace_value (move-only) -> explicit assignment
	{
		auto *entry = mu.find(std::string{"G"});
		assert(entry);
		entry->value() = std::make_unique<int>(77);
		assert(*entry->value() == 77);
	}

	// emplace with rvalue keys (move-only arg)
	assert(mu.emplace(MapU::keys_type{8, "H"}, std::make_unique<int>(88)));
	{
		const MapU &cmu = mu;
		auto *p = cmu.find(std::string{"H"});
		assert(p && *p->value() == 88);
	}

	// miss
	assert(mu.find(std::string{"ZZ"}) == nullptr);

	// erase paths
	assert(mu.erase(7));
	assert(mu.erase(std::string{"H"}));
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
