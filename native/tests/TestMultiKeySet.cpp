import std;
import lego_assemble.utils.type_list;
import lego_assemble.utils.multi_key_set;

using lego_assemble::MultiKeySet;
using lego_assemble::type_list;

#include <cassert>

// --------- Counting key wrappers (distinct types for unique_types<Ks...>) ----------
template <int Tag> struct Key {
	int v{};
	inline static int copies = 0;
	inline static int moves = 0;
	inline static int assigns = 0;
	inline static int move_assigns = 0;

	static void reset_counts() {
		copies = moves = assigns = move_assigns = 0;
	}

	Key() = default;
	explicit Key(int x) : v(x) {}
	Key(const Key &o) : v(o.v) {
		++copies;
	}
	Key(Key &&o) noexcept : v(o.v) {
		++moves;
	}
	Key &operator=(const Key &o) {
		v = o.v;
		++assigns;
		return *this;
	}
	Key &operator=(Key &&o) noexcept {
		v = o.v;
		++move_assigns;
		return *this;
	}

	friend bool operator==(const Key &, const Key &) = default;
};

template <int Tag> struct std::hash<Key<Tag>> {
	size_t operator()(Key<Tag> const &k) const noexcept {
		return std::hash<int>{}(k.v);
	}
};

// --------- Aliases ----------
using K1 = Key<1>;
using K2 = Key<2>;
using K3 = Key<3>;
using Keys = type_list<K1, K2, K3>;
using Set = MultiKeySet<Keys>;

// Helper to sum copy counts across all key types
static int copies_sum() {
	return K1::copies + K2::copies + K3::copies;
}

// --------- Tests ----------
int main() {
	Set s;

	// Reset counters
	K1::reset_counts();
	K2::reset_counts();
	K3::reset_counts();

	// Reserve branch
	s.reserve(8);
	assert(s.size() == 0);
	assert(s.empty());

	// Insert 3 distinct tuples (exercise emplace rvalues)
	assert(s.emplace(K1{1}, K2{10}, K3{100}));
	assert(s.emplace(K1{2}, K2{20}, K3{200}));
	assert(s.emplace(K1{3}, K2{30}, K3{300}));
	assert(s.size() == 3);
	assert(!s.empty());

	// contains / find branches
	{
		const K1 k1a{1}, k1b{2}, k1c{3};
		const K2 k2b{20};
		const K3 k3c{300};

		assert(s.contains(k1a));
		assert(s.contains(k1b));
		assert(s.contains(k1c));
		assert(s.find(k2b) != nullptr);
		assert(s.find(k3c) != nullptr);

		// project From->To
		auto to12 = s.find_key<K2>(k1b); // 2 -> 20
		assert(to12 && to12->v == 20);
		auto to13 = s.find_key<K3>(k1c); // 3 -> 300
		assert(to13 && to13->v == 300);

		// miss branches
		assert(s.find(K2{999}) == nullptr);
		assert(!s.contains(K3{42}));
		assert((s.find_key<K1>(K2{999}) == nullptr));
	}

	// Duplicate insertion must fail if ANY key collides (check_all_unique_)
	assert(!s.emplace(K1{1}, K2{999}, K3{998}));   // collide first key
	assert(!s.emplace(K1{999}, K2{20}, K3{998}));  // collide second key
	assert(!s.emplace(K1{999}, K2{998}, K3{300})); // collide third key
	assert(s.size() == 3);

	// ----- erase: victim == last branch -----
	{
		const K2 k2_last{30}; // last entry is (3,30,300)
		const int copies_before = copies_sum();
		assert(s.erase(k2_last)); // should remove last directly
		assert(s.size() == 2);
		assert(copies_sum() == copies_before); // NO extra copies on erase
		// removed keys gone
		assert(!s.contains(K1{3}));
		assert(!s.contains(K2{30}));
		assert(!s.contains(K3{300}));
	}

	// ----- erase: victim != last (hole fill) branch -----
	{
		// Currently two tuples remain: (1,10,100) at idx 0 and (2,20,200) at idx 1(last)
		// Erase the first by a key -> triggers swap-with-last path
		const int copies_before = copies_sum();
		assert(s.erase(K1{1})); // remove victim=0, move last into 0
		assert(copies_sum() == copies_before); // still NO extra copies on erase
		assert(s.size() == 1);

		// The moved tuple (2,20,200) must still be findable at the new index
		assert(s.contains(K1{2}));
		assert(s.contains(K2{20}));
		assert(s.contains(K3{200}));
		auto p = s.find(K1{2});
		assert(p && std::get<0>(*p).v == 2 && std::get<1>(*p).v == 20 &&
		       std::get<2>(*p).v == 200);

		// The victim's keys must be gone
		assert(!s.contains(K1{1}));
		assert(!s.contains(K2{10}));
		assert(!s.contains(K3{100}));
	}

	// ----- span
	{
		// Rebuild two elements
		assert(s.emplace(K1{7}, K2{70}, K3{700}));
		assert(s.size() == 2);
		assert(s.view().size() == 2);
	}

	// clear branch
	s.clear();
	assert(s.empty());
	assert(s.size() == 0);

	return 0;
}
