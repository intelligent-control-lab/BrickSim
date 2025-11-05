export module lego_assemble.utils.poly_store;

import std;
import lego_assemble.utils.type_list;
import lego_assemble.utils.multi_key_map; // new multi-key index

namespace lego_assemble {

// ========================== default storage policy ==========================
// Storage<T, Key> must expose:
//   using value_type = T;
//   std::pmr::vector<T>   data;
//   std::pmr::vector<Key> ids;   // stores Id in single-key mode; anchor key in multi-key mode
//   explicit Storage(std::pmr::memory_resource*);
export template <class T, class Key> struct pmr_vector_storage {
	using value_type = T;
	using id_type = Key;
	std::pmr::vector<T> data;
	std::pmr::vector<Key> ids;
	explicit pmr_vector_storage(std::pmr::memory_resource *r)
	    : data{r}, ids{r} {}
};

// Light validation for a Storage<T, Key>
export template <class S, class T, class Key>
concept StorageLike = requires(S s, std::pmr::memory_resource *r) {
	typename S::value_type;
	requires std::same_as<typename S::value_type, T>;
	{ s.data.size() } -> std::convertible_to<std::size_t>;
	{ s.ids.size() } -> std::convertible_to<std::size_t>;
	S{r};
};

export template <class Ks, class Ts,
                 template <class, class> class Storage = pmr_vector_storage,
                 class Hs = typename Ks::template map<std::hash>,
                 class Es = typename Ks::template map<std::equal_to>>
class PolyStore;

export template <class... Ks, class... Ts,
                 template <class, class> class Storage, class... Hs,
                 class... Es>
class PolyStore<type_list<Ks...>, type_list<Ts...>, Storage, type_list<Hs...>,
                type_list<Es...>> {
	static_assert(sizeof...(Ks) >= 1,
	              "PolyStore: at least one key type required");
	static_assert(unique_types<Ks...>, "PolyStore: key types must be unique");
	static_assert(unique_types<Ts...>,
	              "PolyStore: TypesList must contain unique types");
	// Directory value encodes {type_index, slot_index} within per-type blocks
	struct Loc1 {
		std::uint32_t index;
	};
	struct LocN {
		std::uint32_t type;
		std::uint32_t index;
	};
	static constexpr bool single_type = sizeof...(Ts) == 1;
	using Loc = std::conditional_t<single_type, Loc1, LocN>;

	using KeysList = type_list<Ks...>;
	using HashList = type_list<Hs...>;
	using EqList = type_list<Es...>;
	using Anchor = Ks...[0]; // first key type as anchor

	static_assert((StorageLike<Storage<Ts, Anchor>, Ts, Anchor> && ...),
	              "PolyStore: Storage<T,Anchor> must satisfy StorageLike");

	// per-type storage (ids[] stores the anchor key)
	using Blocks = std::tuple<Storage<Ts, Anchor>...>;

	// multi-key directory
	using Dir =
	    MultiKeyMap<type_list<Ks...>, Loc, type_list<Hs...>, type_list<Es...>>;

  public:
	using keys_tuple = std::tuple<Ks...>;

	explicit PolyStore(
	    std::pmr::memory_resource *r = std::pmr::get_default_resource())
	    : res_{r}, blocks_{Storage<Ts, Anchor>(r)...}, dir_{r} {}

	// -------- creation --------

	template <class T, class... KArgs, class... Args>
	    requires(in_pack<T, Ts...> && sizeof...(KArgs) == KeysList::size &&
	             (std::same_as<std::remove_cvref_t<KArgs>, Ks> && ...) &&
	             std::constructible_from<T, Args...>)
	[[nodiscard]] bool emplace(std::tuple<KArgs...> &&keys, Args &&...args) {
		constexpr std::size_t Kidx = index_in_pack<T, Ts...>;
		auto &blk = std::get<Kidx>(blocks_);
		const std::uint32_t idx = static_cast<std::uint32_t>(blk.data.size());
		Loc loc;
		if constexpr (single_type) {
			loc = Loc{idx};
		} else {
			loc = Loc{static_cast<std::uint32_t>(Kidx), idx};
		}

		// copy first key as anchor
		Anchor anchor = std::get<0>(keys);

		// Try to insert into directory first to validate uniqueness.
		if (!dir_.insert(std::move(keys), loc))
			return false;

		blk.data.emplace_back(std::forward<Args>(args)...);
		// anchor = first key in tuple
		blk.ids.emplace_back(std::move(anchor));
		return true;
	}

	template <class T, class... AllArgs>
	    requires(in_pack<T, Ts...> && sizeof...(AllArgs) >= KeysList::size &&
	             (type_list<AllArgs...>::template take_front<
	                 KeysList::size>::template same_as_remove_cvref<Ks...>) &&
	             (type_list<AllArgs...>::template drop_front<
	                 KeysList::size>::template can_construct<T>))
	[[nodiscard]] bool emplace(AllArgs &&...all_args) {
		using TL = type_list<AllArgs...>;
		return select_invoke<
		    typename TL::template drop_front_seq<KeysList::size>>(
		    [&]<class... Args>(Args &&...args) {
			    return emplace<T>(
			        select_forward_as_tuple<
			            typename TL::template front_seq<KeysList::size>>(
			            std::forward<AllArgs>(all_args)...),
			        std::forward<Args>(args)...);
		    },
		    std::forward<AllArgs>(all_args)...);
	}

	// -------- lookup (typed) --------
	template <class T, class K>
	    requires(in_pack<T, Ts...> && in_pack<std::remove_cvref_t<K>, Ks...>)
	[[nodiscard]] T *get(const K &key) noexcept {
		const Loc *p = dir_.find(std::as_const(key));
		if (!p)
			return nullptr;
		const Loc loc = *p;
		if constexpr (single_type) {
			auto &blk = std::get<0>(blocks_);
			return &blk.data[loc.index];
		} else {
			constexpr std::size_t Kidx = index_in_pack<T, Ts...>;
			if (loc.type != Kidx)
				return nullptr;
			auto &blk = std::get<Kidx>(blocks_);
			return &blk.data[loc.index];
		}
	}
	template <class T, class K>
	    requires(in_pack<T, Ts...> && in_pack<std::remove_cvref_t<K>, Ks...>)
	[[nodiscard]] const T *get(const K &key) const noexcept {
		return const_cast<PolyStore *>(this)->template get<T>(key);
	}

	// Project keys (handy utility wired to the directory)
	template <class From, class To>
	    requires(in_pack<From, Ks...> && in_pack<To, Ks...>)
	[[nodiscard]] const To *project_key(const From &from) const noexcept {
		return dir_.template project<From, To>(from);
	}

	// -------- type-erased visit --------
	template <class Self, class K, class F>
	    requires(in_pack<std::remove_cvref_t<K>, Ks...>)
	bool visit(this Self &self, const K &key, F &&f) {
		const Loc *p = self.dir_.find(std::as_const(key));
		if (!p)
			return false;
		const Loc loc = *p;
		if constexpr (single_type) {
			auto &blk = std::get<0>(self.blocks_);
			std::invoke(std::forward<F>(f), blk.data[loc.index]);
			return true;
		} else {
			return dispatch_by_type_(loc.type, [&](auto I_const) {
				constexpr std::size_t I = decltype(I_const)::value;
				auto &blk = std::get<I>(self.blocks_);
				std::invoke(std::forward<F>(f), blk.data[loc.index]);
			});
		}
	}

	// -------- delete (swap-remove) by any key --------
	template <class K>
	    requires(in_pack<std::remove_cvref_t<K>, Ks...>)
	bool erase_by_key(const K &key) {
		const Loc *p = dir_.find(std::as_const(key));
		if (!p)
			return false;
		const Loc loc = *p;

		if constexpr (single_type) {
			auto &blk = std::get<0>(blocks_);
			const std::size_t last = blk.data.size() - 1;
			const std::size_t victim = loc.index;
			const Anchor moved_anchor = blk.ids[last];

			if (victim != last) {
				blk.data[victim] = std::move(blk.data[last]);
				blk.ids[victim] = moved_anchor;
				dir_.template replace_value<Anchor>(
				    moved_anchor, Loc{static_cast<std::uint32_t>(victim)});
			}
			blk.data.pop_back();
			blk.ids.pop_back();
			(void)dir_.erase_by_key(std::as_const(key));
			return true;

		} else {
			return dispatch_by_type_(loc.type, [&](auto I_const) {
				constexpr std::size_t I = decltype(I_const)::value;
				auto &blk = std::get<I>(blocks_);
				const std::size_t last = blk.data.size() - 1;
				const std::size_t victim = loc.index;
				const Anchor moved_anchor = blk.ids[last];

				if (victim != last) {
					// move last into victim
					blk.data[victim] = std::move(blk.data[last]);
					blk.ids[victim] = moved_anchor;
					// update directory for the moved record (via its anchor key)
					dir_.template replace_value<Anchor>(
					    moved_anchor, Loc{static_cast<std::uint32_t>(I),
					                      static_cast<std::uint32_t>(victim)});
				}

				// remove victim from block
				blk.data.pop_back();
				blk.ids.pop_back();

				// finally remove victim from directory by the provided key
				(void)dir_.erase_by_key(std::as_const(key));
			});
		}
	}

	// Erase by full keys tuple
	bool erase(const keys_tuple &keys) {
		// erase_by_key via anchor (first key) is enough, but any key works:
		return erase_by_key(std::get<0>(keys));
	}

	// -------- queries & views --------
	template <class K>
	    requires(in_pack<std::remove_cvref_t<K>, Ks...>)
	[[nodiscard]] bool alive(const K &key) const noexcept {
		return dir_.contains(std::as_const(key));
	}

	template <class T>
	    requires(in_pack<T, Ts...>)
	[[nodiscard]] std::span<T> view() noexcept {
		return std::span<T>(std::get<index_in_pack<T, Ts...>>(blocks_).data);
	}
	template <class T>
	    requires(in_pack<T, Ts...>)
	[[nodiscard]] std::span<const T> view() const noexcept {
		const auto &blk = std::get<index_in_pack<T, Ts...>>(blocks_);
		return std::span<const T>(blk.data);
	}

	template <class T>
	    requires(in_pack<T, Ts...>)
	void reserve(std::size_t n) {
		auto &blk = std::get<index_in_pack<T, Ts...>>(blocks_);
		blk.data.reserve(n);
		blk.ids.reserve(n);
	}

	[[nodiscard]] std::size_t size() const {
		return (std::get<Storage<Ts, Anchor>>(blocks_).data.size() + ... + 0u);
	}

	// Memory control knobs
	void clear() {
		(std::get<Storage<Ts, Anchor>>(blocks_).data.clear(), ...);
		(std::get<Storage<Ts, Anchor>>(blocks_).ids.clear(), ...);
		dir_.clear();
	}

	std::pmr::memory_resource *resource() const noexcept {
		return res_;
	}

	template <class T>
	    requires(in_pack<T, Ts...>)
	auto &storage_for() noexcept {
		return std::get<index_in_pack<T, Ts...>>(blocks_);
	}
	template <class T>
	    requires(in_pack<T, Ts...>)
	const auto &storage_for() const noexcept {
		return std::get<index_in_pack<T, Ts...>>(blocks_);
	}

  private:
	// runtime -> compile-time dispatch over type index
	template <class F, std::size_t... Is>
	static bool dispatch_by_type_impl_(std::uint32_t t, F &&fn,
	                                   std::index_sequence<Is...>) {
		bool done = false;
		(void)std::initializer_list<int>{
		    (t == Is ? (fn(std::integral_constant<std::size_t, Is>{}),
		                done = true, 0)
		             : 0)...};
		return done;
	}
	template <class F> static bool dispatch_by_type_(std::uint32_t t, F &&fn) {
		return dispatch_by_type_impl_(
		    t, std::forward<F>(fn), std::make_index_sequence<sizeof...(Ts)>{});
	}

  private:
	std::pmr::memory_resource *res_;
	Blocks blocks_;
	Dir dir_;
};

} // namespace lego_assemble
