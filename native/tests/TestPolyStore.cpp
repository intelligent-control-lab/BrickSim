import std;
import lego_assemble.utils.type_list;
import lego_assemble.utils.poly_store;

#include <cassert>

using namespace std;
using namespace lego_assemble;

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

} // namespace

int main() {
	// ----------------- basic meta sanity on type_list -----------------
	{
		using Types = TL<Player, Enemy, Chest>;
		static_assert(Types::size == 3);
		static_assert(in_pack<Player, Player, Enemy, Chest>);
		static_assert(!in_pack<double, Player, Enemy, Chest>);
		static_assert(index_in_pack<Player, Player, Enemy, Chest> == 0);
		static_assert(index_in_pack<Enemy, Player, Enemy, Chest> == 1);
		static_assert(index_in_pack<Chest, Player, Enemy, Chest> == 2);
		static_assert(unique_types<Player, Enemy, Chest>);
	}

	counting_resource counter;

	// ====================== multi-key PolyStore ======================
	{
		using Keys = TL<std::uint32_t, std::string, std::uint64_t>;
		using Types = TL<Player, Enemy, Chest>;
		using Store = PolyStore<Keys, Types>;

		Store store{&counter};

		// Reserve APIs (not deeply observable, but we want them exercised)
		store.reserve_for_type<Enemy>(8);
		store.reserve(4, 8, 4); // Player, Enemy, Chest

		// -------- emplace: tuple-of-keys + value args --------
		bool ins_p = store.emplace<Player>(
		    std::forward_as_tuple(std::uint32_t(1001u), "alice"s,
		                          std::uint64_t(0xA1ull)),
		    100, "Alice"s);
		assert(ins_p);

		// -------- emplace: separate key args + value args --------
		bool ins_e1 = store.emplace<Enemy>(std::uint32_t(2001u), "goblin"s,
		                                   std::uint64_t(0xB1ull), 1.0f, 2.0f);
		assert(ins_e1);

		// -------- emplace: keys_type directly + value args --------
		bool ins_c1 = store.emplace<Chest>(
		    Store::keys_type{std::uint32_t(3001u), "box-1"s,
		                     std::uint64_t(0xC1ull)},
		    250);
		assert(ins_c1);

		// more enemies to exercise erase paths
		bool ins_e2 = store.emplace<Enemy>(std::uint32_t(2002u), "orc"s,
		                                   std::uint64_t(0xB2ull), 3.0f, 4.0f);
		bool ins_e3 = store.emplace<Enemy>(std::uint32_t(2003u), "troll"s,
		                                   std::uint64_t(0xB3ull), 5.0f, 6.0f);
		assert(ins_e2 && ins_e3);

		// duplicate key set should be rejected, regardless of value type
		bool dup_sameT = store.emplace<Player>(
		    std::uint32_t(1001u), "alice"s, std::uint64_t(0xA1ull), 1, "Dup"s);
		bool dup_diffT = store.emplace<Enemy>(
		    std::uint32_t(1001u), "alice"s, std::uint64_t(0xA1ull), 9.0f, 9.0f);
		assert(!dup_sameT && !dup_diffT);

		// total size and per-type size
		assert(store.size() == 5);
		assert(store.size_of_type<Player>() == 1);
		assert(store.size_of_type<Enemy>() == 3);
		assert(store.size_of_type<Chest>() == 1);
		assert(!store.empty());

		// ---------------- find_value / value_of ----------------
		{
			// correct type
			Player *pp = store.find_value<Player>(std::uint32_t(1001u));
			assert(pp && pp->hp == 100 && pp->name == "Alice");

			const Chest &c =
			    std::as_const(store).value_of<Chest>(std::uint64_t(0xC1ull));
			assert(c.gold == 250);

			// wrong type: find_value returns nullptr or throws; value_of throws
			bool threw = false;
			try {
				(void)store.value_of<Enemy>(std::uint32_t(1001u));
			} catch (...) {
				threw = true;
			}
			assert(threw);

			// not found
			assert(store.find_value<Player>(std::uint32_t(999999u)) == nullptr);
		}

		// ---------------- key_of / keys_of (projection) ----------------
		{
			std::uint32_t id_from_name =
			    store.key_of<std::uint32_t>("orc"s); // From name -> id
			assert(id_from_name == 2002u);

			std::string name_from_tok =
			    store.key_of<std::string>(std::uint64_t(0xB3ull));
			assert(name_from_tok == "troll");

			const auto &keys = store.keys_of(std::uint32_t(3001u));
			assert(std::get<0>(keys) == 3001u);
			assert(std::get<1>(keys) == "box-1");
			assert(std::get<2>(keys) == std::uint64_t(0xC1ull));
		}

		// ---------------- contains ----------------
		{
			assert(store.contains(std::uint32_t(1001u)));
			assert(store.contains(std::string{"goblin"}));
			assert(store.contains(std::uint64_t(0xB1ull)));
			assert(!store.contains(std::uint32_t(999999u)));
		}

		// ---------------- value_view<T> (non-const & const) ----------------
		{
			auto enemies = store.value_view<Enemy>();
			assert(enemies.size() == 3);
			float sumx = 0.f;
			for (auto &e : enemies)
				sumx += e.x;
			assert(sumx == (1.0f + 3.0f + 5.0f));

			const auto &cstore = std::as_const(store);
			auto cenemies = cstore.value_view<Enemy>();
			static_assert(std::same_as<std::remove_cvref_t<decltype(cenemies)>,
			                           std::span<const Enemy>>);
			assert(cenemies.size() == 3);
		}

		// ---------------- entry_of / EntryRef ----------------
		{
			auto e = store.entry_of(std::uint32_t(2002u));
			assert(e.is<Enemy>());
			assert(e.key<std::uint32_t>() == 2002u);
			Enemy &ref = e.value<Enemy>();
			ref.x = 42.0f;
			assert(store.value_of<Enemy>(std::uint32_t(2002u)).x == 42.0f);

			// const_entry_reference from const store
			auto ce = std::as_const(store).entry_of(std::uint64_t(0xB2ull));
			assert(ce.is<Enemy>());
			const Enemy &eref = ce.value<Enemy>();
			assert(eref.x == 42.0f);

			// EntryRef::visit
			int visited_type = 0;
			e.visit([&](auto &obj) {
				using T = std::remove_cvref_t<decltype(obj)>;
				if constexpr (std::same_as<T, Enemy>)
					visited_type = 1;
			});
			assert(visited_type == 1);
		}

		// ---------------- try_visit (void visitor) ----------------
		{
			bool ok1 = store.try_visit(std::uint32_t(2001u), [](auto &obj) {
				using T = std::remove_cvref_t<decltype(obj)>;
				if constexpr (std::same_as<T, Enemy>) {
					obj.x += 0.5f;
				}
			});
			assert(ok1);
			assert(store.value_of<Enemy>(std::uint32_t(2001u)).x == 1.5f);

			bool ok_missing =
			    store.try_visit(std::uint64_t(0xDEADBEEFull), [](auto &) {});
			assert(!ok_missing);
		}

		// ---------------- try_visit (value visitor) ----------------
		{
			auto r = store.try_visit(
			    std::uint32_t(2002u), [](auto &obj) -> std::string {
				    using T = std::remove_cvref_t<decltype(obj)>;
				    if constexpr (std::same_as<T, Enemy>)
					    return "Enemy";
				    else if constexpr (std::same_as<T, Player>)
					    return "Player";
				    else
					    return "Other";
			    });
			assert(r.has_value() && *r == "Enemy");

			auto r_missing =
			    store.try_visit(std::uint32_t(999999u),
			                    [](auto &) -> std::string { return "x"; });
			assert(!r_missing.has_value());
		}

		// ---------------- visit (throwing on missing) ----------------
		{
			bool threw_missing = false;
			try {
				store.visit(std::uint32_t(999999u), [](auto &) {});
			} catch (const std::out_of_range &) {
				threw_missing = true;
			}
			assert(threw_missing);

			// valid visit mutating y
			store.visit(std::uint32_t(2002u), [](auto &obj) {
				using T = std::remove_cvref_t<decltype(obj)>;
				if constexpr (std::same_as<T, Enemy>) {
					obj.y += 1.0f;
				}
			});
			assert(store.value_of<Enemy>(std::uint32_t(2002u)).y == 5.0f);
		}

		// ---------------- erase: victim == last branch ----------------
		{
			// Enemies: [e1(2001), e2(2002), e3(2003)] → erase last by anchor
			bool erased = store.erase(std::uint32_t(2003u));
			assert(erased);
			assert(!store.contains(std::uint32_t(2003u)));
			assert(!store.contains(std::string{"troll"}));
			assert(!store.contains(std::uint64_t(0xB3ull)));
			assert(store.size() == 4);
			assert(store.size_of_type<Enemy>() == 2);
		}

		// ---------------- erase: victim != last (swap-remove) ----------------
		{
			// Now Enemy storage holds [2001, 2002]; erase by non-anchor key of 2001
			bool erased = store.erase(std::string{"goblin"});
			assert(erased);
			assert(!store.contains(std::string{"goblin"}));
			assert(!store.contains(std::uint32_t(2001u)));
			assert(store.size_of_type<Enemy>() == 1);
			assert(store.size() == 3);

			// 2002 must still be alive and accessible
			assert(store.contains(std::uint32_t(2002u)));
			Enemy &e = store.value_of<Enemy>(std::uint32_t(2002u));
			(void)e;
		}

		// ---------------- erase: key not found ----------------
		{
			assert(!store.erase(std::uint32_t(0xFFFFFFFFu)));
		}

		// ---------------- clear_type<Enemy> ----------------
		{
			store.clear_type<Enemy>();
			assert(store.size_of_type<Enemy>() == 0);
			// Player & Chest should remain
			assert(store.contains(std::uint32_t(1001u)));
			assert(store.contains(std::uint64_t(0xC1ull)));
			assert(store.size() == 2);
		}

		// ---------------- clear() ----------------
		{
			store.clear();
			assert(store.size() == 0);
			assert(store.empty());
			assert(!store.contains(std::uint32_t(1001u)));
			assert(!store.contains(std::uint64_t(0xC1ull)));

			// After clear, keys can be reused
			bool re = store.emplace<Enemy>(
			    std::forward_as_tuple(std::uint32_t(2002u), "orc"s,
			                          std::uint64_t(0xB2ull)),
			    9.0f, 9.0f);
			assert(re);
			assert(store.size() == 1);
			auto vs = store.value_view<Enemy>();
			assert(vs.size() == 1);
		}
	}

	// We should have seen allocations routed through counting_resource
	assert(counter.alloc_calls > 0);

	// ====================== single-key PolyStore ======================
	{
		using MyTypes = TL<Player, Enemy, Chest>;
		using Keys = TL<int>;
		using Store = PolyStore<Keys, MyTypes>;
		using SingleStore = PolyStore<Keys, TL<Player>>;

		// -------- multi-type single-key store --------
		{
			Store store{&counter};

			store.reserve_for_type<Enemy>(16);

			// simple emplace using key + value args
			assert(store.emplace<Player>(0, 100, "Alice"));
			assert(store.emplace<Enemy>(1, 1.0f, 2.0f));
			assert(store.emplace<Chest>(2, 250));
			assert(store.emplace<Enemy>(3, 3.0f, 4.0f));
			assert(store.emplace<Enemy>(4, 5.0f, 6.0f));

			assert(store.size() == 5);
			assert(store.size_of_type<Enemy>() == 3);

			// typed lookup
			{
				auto *p = store.find_value<Player>(0);
				assert(p && p->hp == 100 && p->name == "Alice");

				const Store &cs = store;
				const Player *cp = cs.find_value<Player>(0);
				assert(cp && cp->hp == 100);

				assert(store.find_value<Enemy>(0) == nullptr);
			}

			// value_view<Enemy>
			{
				auto enemies = store.value_view<Enemy>();
				assert(enemies.size() == 3);
				float sumx = 0;
				for (auto &e : enemies)
					sumx += e.x;
				assert(sumx == (1.0f + 3.0f + 5.0f));
			}

			// visit / try_visit
			{
				bool ok1 = store.try_visit(1, [](auto &obj) {
					using T = std::remove_cvref_t<decltype(obj)>;
					if constexpr (std::same_as<T, Enemy>)
						obj.x += 0.5f;
				});
				assert(ok1);
				assert(store.value_of<Enemy>(1).x == 1.5f);

				bool ok_missing = store.try_visit(999999, [](auto &) {});
				assert(!ok_missing);

				bool threw_missing = false;
				try {
					store.visit(999999, [](auto &) {});
				} catch (const std::out_of_range &) {
					threw_missing = true;
				}
				assert(threw_missing);
			}

			// erase branches
			{
				// enemies ids: [1,3,4] → erase last
				bool erased = store.erase(4);
				assert(erased);
				assert(!store.contains(4));
				assert(store.size() == 4);

				// now enemies ids: [1,3]; erase first → swap-remove
				bool erased2 = store.erase(1);
				assert(erased2);
				assert(!store.contains(1));
				assert(store.size() == 3);
			}

			// clear & reuse
			{
				store.clear();
				assert(store.size() == 0);
				assert(store.empty());
				assert(store.emplace<Player>(0, 1, "X"));
				assert(store.size() == 1);
			}
		}

		// -------- single-type single-key store --------
		{
			SingleStore s;
			assert(s.emplace<Player>(1, 50, "Solo"));
			assert(s.size() == 1);
			Player &p = s.value_of<Player>(1);
			assert(p.hp == 50 && p.name == "Solo");
		}
	}

	return 0;
}
