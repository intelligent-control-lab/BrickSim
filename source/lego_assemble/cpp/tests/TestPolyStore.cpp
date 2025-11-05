import std;
import lego_assemble.utils.type_list;
import lego_assemble.utils.poly_store;
import lego_assemble.utils.multi_key_map;

#include <cassert>

using namespace std;
using namespace lego_assemble;
using namespace std::string_literals;

namespace {
template <class... Ts> using TL = type_list<Ts...>;

// Counting PMR to ensure allocations go through our resource.
struct counting_resource : pmr::memory_resource {
	size_t alloc_calls = 0, dealloc_calls = 0;
	size_t alloc_bytes = 0, dealloc_bytes = 0;
	pmr::memory_resource *upstream = pmr::get_default_resource();

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

// Small tagged storage to verify T -> Storage<T, Anchor> mapping & anchor ids.
template <class T, class Anchor> struct TaggedStorage {
	using value_type = T;
	pmr::vector<T> data;
	pmr::vector<Anchor> ids; // stores the anchor key
	int tag = 42;
	explicit TaggedStorage(pmr::memory_resource *r) : data{r}, ids{r} {}
};

// Test payload types
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

// Helper to exercise const overloads of get()
template <class T, class S, class K> const T *cget(const S &s, const K &k) {
	return s.template get<T>(k);
}

} // namespace

int main() {
	// ------------ Multi-key store aliases ------------
	using Keys = TL<uint32_t, std::string, uint64_t>;

	using Types = TL<Player, Enemy, Chest>;
	using MStore = PolyStore<Keys, Types, pmr_vector_storage>;
	using TStore = PolyStore<Keys, Types, TaggedStorage>;

	// ------------ Resource wiring ------------
	counting_resource counter;
	{
		MStore store{&counter};
		assert(store.resource() == &counter);

		// reserve<T>
		store.reserve<Enemy>(8);
		assert(store.storage_for<Enemy>().data.capacity() >= 8);
		assert(store.storage_for<Enemy>().ids.capacity() >= 8);

		// ------------ Inserts covering all emplace paths ------------
		// 1) emplace
		bool ins_p = store.emplace<Player>(
		    std::forward_as_tuple(uint32_t(1001u), "alice"s, uint64_t(0xA1ull)),
		    100, "Alice"s);
		assert(ins_p);

		// 2) emplace (separate key args)
		bool ins_e1 = store.emplace<Enemy>(uint32_t(2001u), "goblin"s,
		                                     uint64_t(0xB1ull), 1.0f, 2.0f);
		assert(ins_e1);

		// 3) emplace
		bool ins_c1 = store.emplace<Chest>(
		    MStore::keys_tuple{uint32_t(3001u), "box-1"s, uint64_t(0xC1ull)},
		    250);
		assert(ins_c1);

		// More enemies to exercise erase-branches
		bool ins_e2 = store.emplace<Enemy>(uint32_t(2002u), "orc"s,
		                                     uint64_t(0xB2ull), 3.0f, 4.0f);
		bool ins_e3 = store.emplace<Enemy>(uint32_t(2003u), "troll"s,
		                                     uint64_t(0xB3ull), 5.0f, 6.0f);
		assert(ins_e2 && ins_e3);

		// Duplicate keys should be rejected (directory insert fails)
		bool dup_sameT = store.emplace<Player>(uint32_t(1001u), "alice"s,
		                                         uint64_t(0xA1ull), 1, "Dup"s);
		bool dup_diffT = store.emplace<Enemy>(uint32_t(1001u), "alice"s,
		                                        uint64_t(0xA1ull), 9.9f, 9.9f);
		assert(!dup_sameT && !dup_diffT);

		// size across categories: 1P + 3E + 1C
		assert(store.size() == 5);

		// ------------ typed get<T>(K) ------------
		// success via anchor
		{
			Player *pp = store.get<Player>(uint32_t(1001u));
			assert(pp && pp->hp == 100 && pp->name == "Alice");
			// type mismatch (found record, wrong T)
			assert(store.get<Enemy>(uint32_t(1001u)) == nullptr);
			// const overload
			const MStore &cs = store;
			const Player *cpp = cget<Player>(cs, uint32_t(1001u));
			assert(cpp && cpp->hp == 100);
		}
		// success via non-anchor key
		{
			Enemy *pe = store.get<Enemy>(std::string{"goblin"});
			assert(pe && pe->x == 1.0f && pe->y == 2.0f);

			const Chest *pc = store.get<Chest>(uint64_t(0xC1ull));
			assert(pc && pc->gold == 250);
		}
		// not found
		assert(store.get<Player>(uint32_t(999999u)) == nullptr);

		// ------------ project_key<From, To> ------------
		{
			const uint32_t *id_from_name =
			    store.project_key<std::string, uint32_t>("orc"s);
			assert(id_from_name && *id_from_name == uint32_t(2002u));

			const std::string *name_from_tok =
			    store.project_key<uint64_t, std::string>(uint64_t(0xB3ull));
			assert(name_from_tok && *name_from_tok == "troll");

			assert(
			    (store.project_key<std::string, uint32_t>("zzz"s) == nullptr));
		}

		// ------------ visit<K>(key, f) ------------
		{
			bool ok1 = store.visit(uint32_t(2001u), [](auto &obj) {
				using T = std::decay_t<decltype(obj)>;
				if constexpr (std::same_as<T, Enemy>)
					obj.x += 0.5f;
			});
			assert(ok1 && store.get<Enemy>(uint32_t(2001u))->x == 1.5f);

			bool ok2 = store.visit(std::string{"box-1"}, [](auto &obj) {
				using T = std::decay_t<decltype(obj)>;
				if constexpr (std::same_as<T, Chest>)
					obj.gold += 5;
			});
			assert(ok2 && store.get<Chest>(uint64_t(0xC1ull))->gold == 255);

			bool ok3 = store.visit(uint64_t(0xDEADBEEFull), [](auto &) {});
			assert(!ok3); // missing key
		}

		// ------------ view<T> (non-const & const) ------------
		{
			auto enemies = store.view<Enemy>();
			assert(enemies.size() == 3);
			float sumx = 0;
			for (auto &e : enemies)
				sumx += e.x;
			assert(sumx == (1.5f + 3.0f + 5.0f)); // e1 was bumped by +0.5

			const auto cenemies = std::as_const(store).view<Enemy>();
			static_assert(std::same_as<std::remove_cvref_t<decltype(cenemies)>,
			                           std::span<const Enemy>>);
			assert(cenemies.size() == 3);
		}

		// ------------ erase_by_key: victim == last ------------
		{
			// enemy order in block: [e1(2001), e2(2002), e3(2003)] → erase last by anchor
			bool erased = store.erase_by_key(uint32_t(2003u));
			assert(erased);
			assert(store.get<Enemy>(uint32_t(2003u)) == nullptr);
			assert(!store.alive(uint32_t(2003u)));
			// all keys of the removed record should be gone
			assert(!store.alive(std::string{"troll"}));
			assert(!store.alive(uint64_t(0xB3ull)));
			assert(store.size() == 4);
		}

		// ------------ erase_by_key: victim != last (swap-remove + directory fix) ------------
		{
			auto &Eblk = store.storage_for<Enemy>();
			assert(Eblk.data.size() == 2);
			uint32_t moved_anchor = Eblk.ids.back(); // should be 2002 (e2)

			// erase first enemy by a NON-ANCHOR key → victim=0, last=1
			bool erased = store.erase_by_key(std::string{"goblin"});
			assert(erased);
			assert(store.get<Enemy>(uint32_t(2001u)) == nullptr);
			assert(!store.alive(uint32_t(2001u)));
			assert(store.size() == 3);

			// After swap, directory must map moved_anchor to slot 0
			assert(Eblk.ids.size() == 1 && Eblk.ids[0] == moved_anchor);
			Enemy *moved_ptr_by_anchor = store.get<Enemy>(moved_anchor);
			assert(moved_ptr_by_anchor == &Eblk.data[0]);
			// non-anchor key of the moved record must also resolve
			Enemy *moved_ptr_by_name = store.get<Enemy>(std::string{"orc"});
			assert(moved_ptr_by_name == &Eblk.data[0]);
		}

		// ------------ erase_by_key: key not found ------------
		assert(!store.erase_by_key(uint32_t(0xFFFFFFFFu)));

		// ------------ erase(keys_tuple) (delegates to anchor) ------------
		{
			assert(store.alive(uint32_t(1001u)) &&
			       store.alive(std::string{"alice"}) &&
			       store.alive(uint64_t(0xA1ull)));
			bool ok =
			    store.erase({uint32_t(1001u), "alice"s, uint64_t(0xA1ull)});
			assert(ok);
			assert(!store.alive(uint32_t(1001u)));
			assert(!store.alive(std::string{"alice"}));
			assert(!store.alive(uint64_t(0xA1ull)));
			assert(store.size() == 2); // 1 Enemy (2002) + 1 Chest
		}

		// ------------ storage_for<T> mutation ------------
		{
			auto &Eblk = store.storage_for<Enemy>();
			assert(Eblk.data.size() == 1 && Eblk.ids.size() == 1 &&
			       Eblk.ids[0] == uint32_t(2002u));
			Eblk.data[0].x = 42.0f;
			assert(store.get<Enemy>(std::string{"orc"})->x == 42.0f);
		}

		// ------------ clear() ------------
		store.clear();
		assert(store.size() == 0);
		assert(!store.alive(uint32_t(2002u)));
		assert(!store.alive(std::string{"orc"}));
		assert(!store.alive(uint64_t(0xB2ull)));

		// After clear, reusing old keys should succeed
		bool re = store.emplace<Enemy>(uint32_t(2001u), "goblin"s,
		                                 uint64_t(0xB1ull), 9.0f, 9.0f);
		assert(re);
	}

	// We should have seen allocations routed through the counting_resource.
	assert(counter.alloc_calls > 0);

	// ------------ Custom storage policy smoke (multi-key) ------------
	{
		TStore store{&counter};
		bool ok = store.emplace<Enemy>(uint32_t(7001u), "z1"s,
		                                 uint64_t(0xEE11ull), 1.0f, 1.0f);
		assert(ok);
		auto &blk = store.storage_for<Enemy>();
		assert(blk.tag == 42);
		assert(blk.ids.size() == 1 &&
		       blk.ids[0] == uint32_t(7001u)); // anchor stored
		assert(store.get<Enemy>(uint32_t(7001u)));
	}

	return 0;
}
