import std;
import lego_assemble.utils.type_list;
import lego_assemble.utils.poly_store;

#include <cassert>

using namespace std;
using namespace lego_assemble;

// ----------------------- meta smoke tests ------------------------
namespace {
template <class... Ts> using TL = type_list<Ts...>;

template <class X> struct wrap {
	using type = X;
}; // for transform_t demo
template <class X> using wrap_t = typename wrap<X>::type;

// A trivial custom storage policy to verify the T -> Storage<T> mapping.
template <class T, class Id> struct TaggedStorage {
	using value_type = T;
	pmr::vector<T> data;
	pmr::vector<Id> ids;
	int tag = 42;
	explicit TaggedStorage(pmr::memory_resource *r) : data{r}, ids{r} {}
};

// Counting PMR to ensure allocations go through our resource.
struct counting_resource : pmr::memory_resource {
	size_t alloc_calls = 0, dealloc_calls = 0;
	size_t alloc_bytes = 0, dealloc_bytes = 0;
	pmr::memory_resource *upstream = pmr::get_default_resource();

	counting_resource() = default;

  private:
	void *do_allocate(size_t bytes, size_t align) override {
		alloc_calls++;
		alloc_bytes += bytes;
		return upstream->allocate(bytes, align);
	}
	void do_deallocate(void *p, size_t bytes, size_t align) override {
		dealloc_calls++;
		dealloc_bytes += bytes;
		upstream->deallocate(p, bytes, align);
	}
	bool
	do_is_equal(const pmr::memory_resource &other) const noexcept override {
		return this == &other;
	}
};
} // namespace

// --------------------------- test types ---------------------------
struct Player {
	int hp;
	std::string name;
};
struct Enemy {
	float x, y;
};
struct Chest {
	int gold;
};

using MyTypes = TL<Player, Enemy, Chest>;
using Store = PolyStore<TL<int>, MyTypes>; // default pmr_vector_storage
using TStore =
    PolyStore<TL<int>, MyTypes, TaggedStorage>; // custom storage policy
using SingleStore = PolyStore<TL<int>, TL<Player>>;

// Verify StorageLike concept for the default storage.
static_assert(StorageLike<pmr_vector_storage<Player, int>, Player, int>);

// Verify meta utilities.
static_assert(MyTypes::size == 3);
static_assert(in_pack<Player, Player, Enemy, Chest>);
static_assert(!in_pack<double, Player, Enemy, Chest>);
static_assert(index_in_pack<Player, Player, Enemy, Chest> == 0);
static_assert(index_in_pack<Enemy, Player, Enemy, Chest> == 1);
static_assert(index_in_pack<Chest, Player, Enemy, Chest> == 2);
static_assert(unique_types<Player, Enemy, Chest>);

// transform_t demo: wrap<T> -> type_list<wrap_t<Player>, wrap_t<Enemy>, wrap_t<Chest>>
using Wrapped = MyTypes::map<wrap_t>;
static_assert(
    std::same_as<Wrapped, TL<wrap_t<Player>, wrap_t<Enemy>, wrap_t<Chest>>>);

// Helper to exercise the const overload of get()
template <class T, class S> const T *cget(const S &s, typename S::id_type id) {
	return s.template get<T>(id);
}

// --------------------------- test body ---------------------------
int main() {
	// ---------- resource wiring ----------
	counting_resource counter;
	{
		Store store{&counter};
		assert(store.resource() == &counter);

		// ---------- reserve ----------
		store.reserve<Enemy>(16);
		assert(store.storage_for<Enemy>().data.capacity() >= 16);
		assert(store.storage_for<Enemy>().ids.capacity() >= 16);

		// ---------- inserts (monotonic ids) ----------
		assert(store.emplace<Player>(0, 100, "Alice"));
		assert(store.emplace<Enemy>(1, 1.0f, 2.0f));
		assert(store.emplace<Chest>(2, 250));
		assert(store.emplace<Enemy>(3, 3.0f, 4.0f));
		assert(store.emplace<Enemy>(4, 5.0f, 6.0f));

		// size() across categories
		assert(store.size() == 5);

		// ---------- typed lookup ----------
		{
			auto *p = store.get<Player>(0);
			assert(p && p->hp == 100 && p->name == "Alice");

			// type mismatch should return nullptr
			assert(store.get<Enemy>(0) == nullptr);

			// const overload
			const Store &cs = store;
			const Player *cp = cs.get<Player>(0);
			assert(cp && cp->hp == 100);
		}

		// ---------- view<T> & contiguous iteration ----------
		{
			auto enemies = store.view<Enemy>();
			assert(enemies.size() == 3);
			float sumx = 0;
			for (auto &e : enemies)
				sumx += e.x;
			assert(sumx == (1.0f + 3.0f + 5.0f));
		}
		{
			const auto enemies = std::as_const(store).view<Enemy>();
			static_assert(std::same_as<std::remove_cvref_t<decltype(enemies)>,
			                           std::span<const Enemy>>);
			assert(enemies.size() == 3);
		}

		// ---------- visit (type-erased) ----------
		{
			bool ok1 = store.visit(1, [](auto &obj) {
				using T = std::decay_t<decltype(obj)>;
				if constexpr (std::same_as<T, Enemy>) {
					obj.x += 0.5f;
				}
			});
			assert(ok1);
			assert(store.get<Enemy>(1)->x == 1.5f);

			bool ok2 = store.visit(999999, [](auto &) {});
			assert(!ok2);
		}

		// ---------- erase: victim == last branch ----------
		{
			// current enemies indices: [1, 3, 4] in that order.
			// erase the last one's id (4) → victim == last
			bool erased = store.erase(4);
			assert(erased);
			assert(store.get<Enemy>(4) == nullptr);
			assert(!store.alive(4));
			assert(store.size() == 4);
		}

		// ---------- erase: victim != last branch (swap-remove + directory fix-up) ----------
		{
			// capture the id of the current last enemy (which is 3 after the previous erase)
			auto &Eblk = store.storage_for<Enemy>();
			assert(Eblk.data.size() == 2);
			int moved_id = Eblk.ids.back(); // this should be 3

			// erase the first enemy's id (1) → victim = 0, last = 1, move last into 0
			bool erased = store.erase(1);
			assert(erased);
			assert(store.get<Enemy>(1) == nullptr);
			assert(!store.alive(1));
			assert(store.size() == 3);

			// directory must now map moved_id to slot 0
			assert(Eblk.ids.size() == 1);
			assert(Eblk.ids[0] == moved_id);
			Enemy *moved_ptr = store.get<Enemy>(moved_id);
			assert(moved_ptr == &Eblk.data[0]); // points to the moved slot
		}

		// ---------- erase: id not found ----------
		assert(!store.erase(123456u)); // never existed
		assert(!store.erase(1));       // already erased above

		// ---------- alive() ----------
		assert(store.alive(0));
		assert(!store.alive(1));

		// ---------- storage_for<T> (mutating through storage) ----------
		{
			auto &Pblk = store.storage_for<Player>();
			assert(Pblk.data.size() == 1 && Pblk.ids.size() == 1 &&
			       Pblk.ids[0] == 0);
			Pblk.data[0].hp = 77;
			assert(store.get<Player>(0)->hp == 77);
		}

		// ---------- clear (reset to empty) ----------
		store.clear();
		assert(store.size() == 0);
		assert(!store.alive(0)); // directory emptied
		assert(store.emplace<Player>(0, 1, "X"));
	}

	// We should have seen some allocations routed through the counting_resource.
	assert(counter.alloc_calls > 0);

	// ---------- custom storage policy (T -> Storage<T>) ----------
	{
		TStore store; // uses TaggedStorage<T,Id>
		assert(store.emplace<Player>(0, 5, "P"));
		assert(store.emplace<Enemy>(99, 9.0f, 9.0f));
		assert(store.size() == 2);

		// The custom storage exists and carries its extra state.
		auto &eblk = store.storage_for<Enemy>();
		assert(eblk.tag == 42);
		assert(eblk.ids.size() == 1 && eblk.ids[0] == 99);

		// Normal API still works
		assert(store.get<Player>(0)->hp == 5);
		assert(store.erase(99));
		assert(!store.alive(99));
		assert(store.size() == 1);
	}

	// ---------- normal use scenario ----------
	{
		Store s;
		assert(s.emplace<Player>(11, 120, "Knight"));
		assert(s.emplace<Enemy>(22, 10.f, 20.f));
		s.visit(22, [](auto &obj) {
			if constexpr (std::same_as<std::decay_t<decltype(obj)>, Enemy>) {
				obj.x += 1.f;
				obj.y += 2.f;
			}
		});
		for (auto &en : s.view<Enemy>()) {
			assert(en.x == 11.f && en.y == 22.f);
		}
		assert(s.erase(11));
		assert(!s.alive(11));
		assert(s.size() == 1);
	}

	// ---------- single-type store ----------
	{
		SingleStore s;
		assert(s.emplace<Player>(1, 50, "Solo"));
		assert(s.size() == 1);
		auto *p = s.get<Player>(1);
		assert(p && p->hp == 50 && p->name == "Solo");
	}

	return 0;
}
