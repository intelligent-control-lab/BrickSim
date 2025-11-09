export module lego_assemble.utils.multi_key_set;

import std;
import lego_assemble.utils.hash;
import lego_assemble.utils.type_list;

namespace lego_assemble {

export template <class KeysList,
                 class HashList = typename KeysList::template map<std::hash>,
                 class EqList = typename KeysList::template map<std::equal_to>>
class MultiKeySet;

export template <class... Ks, class... Hs, class... Es>
class MultiKeySet<type_list<Ks...>, type_list<Hs...>, type_list<Es...>> {
  public:
	using KeysList = type_list<Ks...>;
	using HashList = type_list<Hs...>;
	using EqList = type_list<Es...>;

	static_assert(KeysList::size > 0,
	              "MultiKeySet requires at least one key type");
	static_assert(unique_types<Ks...>, "MultiKeySet key types must be unique");
	static_assert(HashList::size == KeysList::size,
	              "MultiKeySet HashList size must match KeysList size");
	static_assert(EqList::size == KeysList::size,
	              "MultiKeySet EqList size must match KeysList size");
	static_assert((hash_function<Hs, Ks> && ...),
	              "MultiKeySet HashList must satisfy hash_function concept");
	static_assert(
	    (std::equivalence_relation<Es, const Ks &, const Ks &> && ...),
	    "MultiKeySet EqList must satisfy equivalence_relation concept");

	using size_type = std::uint32_t;
	using tuple_type = std::tuple<Ks...>;

	explicit MultiKeySet(
	    std::pmr::memory_resource *r = std::pmr::get_default_resource())
	    : res_{r}, store_{r}, maps_{make_map<Ks, Hs, Es>(r)...} {}

	// Insert / emplace unique tuple of keys. Returns false if any key collides.
	template <class... Args>
	    requires std::constructible_from<tuple_type, Args...>
	[[nodiscard]] bool emplace(Args &&...args) {
		return insert(tuple_type{std::forward<Args>(args)...});
	}
	[[nodiscard]] bool insert(const tuple_type &t) {
		return insert_impl_(t);
	}
	[[nodiscard]] bool insert(tuple_type &&t) {
		return insert_impl_(std::move(t));
	}

	// Find full record by any key; nullptr if absent
	template <class K>
	    requires in_pack<K, Ks...>
	[[nodiscard]] const tuple_type *find(const K &k) const noexcept {
		const auto &m = map_for_<K>();
		if (auto it = m.find(k); it != m.end())
			return &store_[it->second];
		return nullptr;
	}
	template <class K>
	    requires in_pack<K, Ks...>
	[[nodiscard]] bool contains(const K &k) const noexcept {
		return map_for_<K>().contains(k);
	}

	// Project a key value to another key within the same tuple
	template <class From, class To>
	    requires(in_pack<From, Ks...> && in_pack<To, Ks...>)
	[[nodiscard]] const To *project(const From &from) const noexcept {
		if (auto p = find<From>(from)) {
			return &std::get<index_in_pack<To, Ks...>>(*p);
		}
		return nullptr;
	}

	// Erase by any single key
	template <class K>
	    requires in_pack<K, Ks...>
	bool erase_by_key(const K &k) {
		auto &m = map_for_<K>();
		auto it = m.find(k);
		if (it == m.end())
			return false;
		return erase_at_index_(it->second);
	}

	// Erase by tuple (uses first key)
	bool erase(const tuple_type &t) {
		return erase_by_key(std::get<0>(t));
	}

	// Introspection / iteration
	[[nodiscard]] std::size_t size() const noexcept {
		return store_.size();
	}
	[[nodiscard]] bool empty() const noexcept {
		return store_.empty();
	}
	[[nodiscard]] std::span<const tuple_type> view() const noexcept {
		return {store_.data(), store_.size()};
	}

	// Memory knobs
	void reserve(std::size_t n) {
		store_.reserve(n);
		reserve_maps_(n);
	}
	void clear() {
		store_.clear();
		clear_maps_();
	}
	std::pmr::memory_resource *resource() const noexcept {
		return res_;
	}

  private:
	using index_type = size_type;

	template <class K>
	using MapAlloc =
	    std::pmr::polymorphic_allocator<std::pair<const K, index_type>>;

	template <class K, class H, class E>
	using KeyMap = std::unordered_map<K, index_type, H, E, MapAlloc<K>>;

	template <class K, class H, class E>
	static KeyMap<K, H, E> make_map(std::pmr::memory_resource *r) {
		return KeyMap<K, H, E>(0, H{}, E{}, MapAlloc<K>{r});
	}

	using Maps = std::tuple<KeyMap<Ks, Hs, Es>...>;

	template <class K> auto &map_for_() noexcept {
		constexpr std::size_t I = index_in_pack<K, Ks...>;
		return std::get<I>(maps_);
	}
	template <class K> const auto &map_for_() const noexcept {
		constexpr std::size_t I = index_in_pack<K, Ks...>;
		return std::get<I>(maps_);
	}

	template <class TupleLike> [[nodiscard]] bool insert_impl_(TupleLike &&t) {
		if (!check_all_unique_(t))
			return false; // enforce N-way uniqueness

		const index_type idx = static_cast<index_type>(store_.size());
		store_.push_back(std::forward<TupleLike>(t));
		insert_all_maps_(store_.back(), idx);
		return true;
	}

	[[nodiscard]] bool erase_at_index_(index_type victim) {
		const index_type last = static_cast<index_type>(store_.size() - 1);
		if (victim == last) {
			// Just erase the last element's keys and pop.
			erase_all_maps_(store_.back());
			store_.pop_back();
			return true;
		} else {
			// 1) Repoint the "last" element's keys from `last` -> `victim`.
			//    Do this BEFORE moving, so keys are still intact for lookups.
			update_all_maps_to_(store_.back(), victim);
			// 2) Remove the victim's keys from all maps while they still exist in place.
			erase_all_maps_(store_[victim]);
			// 3) Move the last tuple into the hole and pop.
			store_[victim] = std::move(store_.back());
			store_.pop_back();
			return true;
		}
	}

	[[nodiscard]] bool check_all_unique_(const tuple_type &t) const {
		return check_all_unique_impl_(
		    t, std::make_index_sequence<KeysList::size>{});
	}
	template <std::size_t... Is>
	[[nodiscard]] bool
	check_all_unique_impl_(const tuple_type &t,
	                       std::index_sequence<Is...>) const {
		return (!std::get<Is>(maps_).contains(std::get<Is>(t)) && ...);
	}

	void insert_all_maps_(const tuple_type &t, index_type index) {
		insert_all_maps_impl_(t, index,
		                      std::make_index_sequence<KeysList::size>{});
	}
	template <std::size_t... Is>
	void insert_all_maps_impl_(const tuple_type &t, index_type index,
	                           std::index_sequence<Is...>) {
		(std::get<Is>(maps_).emplace(std::get<Is>(t), index), ...);
	}

	void erase_all_maps_(const tuple_type &t) {
		erase_all_maps_impl_(t, std::make_index_sequence<KeysList::size>{});
	}
	template <std::size_t... Is>
	void erase_all_maps_impl_(const tuple_type &t, std::index_sequence<Is...>) {
		(std::get<Is>(maps_).erase(std::get<Is>(t)), ...);
	}

	void update_all_maps_to_(const tuple_type &t, index_type new_index) {
		update_all_maps_to_impl_(t, new_index,
		                         std::make_index_sequence<KeysList::size>{});
	}
	template <std::size_t... Is>
	void update_all_maps_to_impl_(const tuple_type &t, index_type new_index,
	                              std::index_sequence<Is...>) {
		(([](auto &map, const auto &key, index_type i) {
			 auto it = map.find(key); // should exist
			 // assert(it != map.end());
			 it->second = i;
		 }(std::get<Is>(maps_), std::get<Is>(t), new_index)),
		 ...);
	}

	void clear_maps_() {
		clear_maps_impl_(std::make_index_sequence<KeysList::size>{});
	}
	template <std::size_t... Is>
	void clear_maps_impl_(std::index_sequence<Is...>) {
		(std::get<Is>(maps_).clear(), ...);
	}

	void reserve_maps_(std::size_t n) {
		reserve_maps_impl_(n, std::make_index_sequence<KeysList::size>{});
	}
	template <std::size_t... Is>
	void reserve_maps_impl_(std::size_t n, std::index_sequence<Is...>) {
		(std::get<Is>(maps_).reserve(n), ...);
	}

  private:
	std::pmr::memory_resource *res_;
	std::pmr::vector<tuple_type>
	    store_; // stores only the N unique keys (one tuple per record)
	Maps maps_; // one unique index per key
};

} // namespace lego_assemble
