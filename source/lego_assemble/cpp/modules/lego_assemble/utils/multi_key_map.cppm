export module lego_assemble.utils.multi_key_map;

import std;
import lego_assemble.utils.hash;
import lego_assemble.utils.type_list;

namespace lego_assemble {

// Primary template
export template <class KeysList, class Mapped,
                 class HashList = typename KeysList::template map<std::hash>,
                 class EqList = typename KeysList::template map<std::equal_to>>
class MultiKeyMap;

// Partial specialization for type_list<Ks...>
export template <class... Ks, class M, class... Hs, class... Es>
class MultiKeyMap<type_list<Ks...>, M, type_list<Hs...>, type_list<Es...>> {
  public:
	using KeysList = type_list<Ks...>;
	using HashList = type_list<Hs...>;
	using EqList = type_list<Es...>;
	using mapped_type = M;
	using size_type = std::uint32_t;
	using tuple_type = std::tuple<Ks...>;
	// Match std::map naming: pair of (keys tuple, mapped value)
	using value_type = std::pair<tuple_type, mapped_type>;

	static_assert(KeysList::size > 0,
	              "MultiKeyMap requires at least one key type");
	static_assert(unique_types<Ks...>, "MultiKeyMap key types must be unique");
	static_assert(HashList::size == KeysList::size,
	              "MultiKeyMap HashList size must match KeysList size");
	static_assert(EqList::size == KeysList::size,
	              "MultiKeyMap EqList size must match KeysList size");
	static_assert((hash_function<Hs, Ks> && ...),
	              "MultiKeyMap HashList must satisfy hash_function concept");
	static_assert(
	    (std::equivalence_relation<Es, const Ks &, const Ks &> && ...),
	    "MultiKeyMap EqList must satisfy equivalence_relation concept");

	explicit MultiKeyMap(
	    std::pmr::memory_resource *r = std::pmr::get_default_resource())
	    : res_{r}, store_{r}, maps_{make_map<Ks, Hs, Es>(r)...} {}

	// --- Insertion ---

	// Insert tuple + value. Returns false if any key collides.
	template <class V>
	    requires std::constructible_from<mapped_type, V &&>
	[[nodiscard]] bool insert(const tuple_type &keys, V &&v) {
		return insert_impl_(keys, std::forward<V>(v));
	}
	template <class V>
	    requires std::constructible_from<mapped_type, V &&>
	[[nodiscard]] bool insert(tuple_type &&keys, V &&v) {
		return insert_impl_(std::move(keys), std::forward<V>(v));
	}

	// Emplace value from args while providing the keys tuple.
	// Example: map.emplace(keys_tuple, v_ctor_arg1, v_ctor_arg2, ...)
	template <class... VArgs>
	    requires std::constructible_from<mapped_type, VArgs...>
	[[nodiscard]] bool emplace(const tuple_type &keys, VArgs &&...vargs) {
		return insert_impl_(keys, mapped_type{std::forward<VArgs>(vargs)...});
	}
	template <class... VArgs>
	    requires std::constructible_from<mapped_type, VArgs...>
	[[nodiscard]] bool emplace(tuple_type &&keys, VArgs &&...vargs) {
		return insert_impl_(std::move(keys),
		                    mapped_type{std::forward<VArgs>(vargs)...});
	}

	// Piecewise construction of (keys..., value...) like std::pair.
	// Example:
	//   map.emplace_piecewise(std::piecewise_construct,
	//                         std::forward_as_tuple(k1,k2,...),
	//                         std::forward_as_tuple(v_args...));
	template <class... KArgs, class... VArgs>
	    requires(std::constructible_from<tuple_type, KArgs...> &&
	             std::constructible_from<mapped_type, VArgs...>)
	[[nodiscard]] bool emplace_piecewise(std::piecewise_construct_t,
	                                     std::tuple<KArgs...> keys_args,
	                                     std::tuple<VArgs...> val_args) {
		tuple_type keys =
		    std::make_from_tuple<tuple_type>(std::move(keys_args));
		mapped_type val =
		    std::make_from_tuple<mapped_type>(std::move(val_args));
		return insert_impl_(std::move(keys), std::move(val));
	}

	// --- Lookup ---

	// Find value by any key; nullptr if absent
	template <class K>
	    requires in_pack<K, Ks...>
	[[nodiscard]] mapped_type *find(const K &k) noexcept {
		auto &m = map_for_<K>();
		if (auto it = m.find(k); it != m.end())
			return &store_[it->second].second;
		return nullptr;
	}
	template <class K>
	    requires in_pack<K, Ks...>
	[[nodiscard]] const mapped_type *find(const K &k) const noexcept {
		const auto &m = map_for_<K>();
		if (auto it = m.find(k); it != m.end())
			return &store_[it->second].second;
		return nullptr;
	}

	// Retrieve the full keys tuple by any key; nullptr if absent
	template <class K>
	    requires in_pack<K, Ks...>
	[[nodiscard]] const tuple_type *find_keys(const K &k) const noexcept {
		const auto &m = map_for_<K>();
		if (auto it = m.find(k); it != m.end())
			return &store_[it->second].first;
		return nullptr;
	}

	// Project a key value to another key within the same tuple
	template <class From, class To>
	    requires(in_pack<From, Ks...> && in_pack<To, Ks...>)
	[[nodiscard]] const To *project(const From &from) const noexcept {
		if (auto p = find_keys<From>(from)) {
			return &std::get<index_in_pack<To, Ks...>>(*p);
		}
		return nullptr;
	}

	// Contains by any single key
	template <class K>
	    requires in_pack<K, Ks...>
	[[nodiscard]] bool contains(const K &k) const noexcept {
		return map_for_<K>().contains(k);
	}

	// --- Value updates ---

	// Replace the mapped value addressed by any key. Returns false if missing.
	template <class K, class V>
	    requires in_pack<K, Ks...> && std::constructible_from<mapped_type, V &&>
	bool replace_value(const K &k, V &&v) {
		auto &m = map_for_<K>();
		auto it = m.find(k);
		if (it == m.end())
			return false;
		store_[it->second].second = mapped_type{std::forward<V>(v)};
		return true;
	}

	// Get a mutable pointer to the mapped value by any key; nullptr if missing.
	template <class K>
	    requires in_pack<K, Ks...>
	[[nodiscard]] mapped_type *value_ptr(const K &k) noexcept {
		auto &m = map_for_<K>();
		if (auto it = m.find(k); it != m.end())
			return &store_[it->second].second;
		return nullptr;
	}

	// --- Erase ---

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

	// Erase by keys tuple (uses first key)
	bool erase(const tuple_type &keys) {
		return erase_by_key(std::get<0>(keys));
	}

	// --- Introspection / iteration ---

	[[nodiscard]] std::size_t size() const noexcept {
		return store_.size();
	}
	[[nodiscard]] bool empty() const noexcept {
		return store_.empty();
	}

	// View all entries as (tuple_of_keys, mapped_value)
	[[nodiscard]] std::span<const value_type> view() const noexcept {
		return {store_.data(), store_.size()};
	}

	// --- Memory knobs ---

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

	template <class TupleLike, class VLike>
	[[nodiscard]] bool insert_impl_(TupleLike &&keys_like, VLike &&v_like) {
		// 0) Check uniqueness using the tuple-like directly.
		if (!check_all_unique_(keys_like))
			return false;

		const index_type idx = static_cast<index_type>(store_.size());

		// 1) Construct in-place once.
		store_.emplace_back(std::forward<TupleLike>(keys_like),
		                    std::forward<VLike>(v_like));

		// 2) Index using the keys now living in store_.back()
		insert_all_maps_(store_.back().first, idx);
		return true;
	}

	[[nodiscard]] bool erase_at_index_(index_type victim) {
		const index_type last = static_cast<index_type>(store_.size() - 1);
		if (victim == last) {
			// Just erase the last element's keys and pop.
			erase_all_maps_(store_.back().first);
			store_.pop_back();
			return true;
		} else {
			// 1) Repoint the "last" element's keys *before* moving it.
			update_all_maps_to_(store_.back().first, victim);
			// 2) Erase the victim's keys while they're still in place.
			erase_all_maps_(store_[victim].first);
			// 3) Move the last pair into the hole and pop.
			store_[victim] = std::move(store_.back());
			store_.pop_back();
			return true;
		}
	}

	template <class TupleLike>
	[[nodiscard]] bool check_all_unique_(const TupleLike &t) const {
		return check_all_unique_impl_(
		    t, std::make_index_sequence<KeysList::size>{});
	}
	template <class TupleLike, std::size_t... Is>
	[[nodiscard]] bool
	check_all_unique_impl_(const TupleLike &t,
	                       std::index_sequence<Is...>) const {
		return (!std::get<Is>(maps_).contains(std::get<Is>(t)) && ...);
	}

	void insert_all_maps_(const tuple_type &keys, index_type index) {
		insert_all_maps_impl_(keys, index,
		                      std::make_index_sequence<KeysList::size>{});
	}
	template <std::size_t... Is>
	void insert_all_maps_impl_(const tuple_type &keys, index_type index,
	                           std::index_sequence<Is...>) {
		(std::get<Is>(maps_).emplace(std::get<Is>(keys), index), ...);
	}

	void erase_all_maps_(const tuple_type &keys) {
		erase_all_maps_impl_(keys, std::make_index_sequence<KeysList::size>{});
	}
	template <std::size_t... Is>
	void erase_all_maps_impl_(const tuple_type &keys,
	                          std::index_sequence<Is...>) {
		(std::get<Is>(maps_).erase(std::get<Is>(keys)), ...);
	}

	void update_all_maps_to_(const tuple_type &keys, index_type new_index) {
		update_all_maps_to_impl_(keys, new_index,
		                         std::make_index_sequence<KeysList::size>{});
	}
	template <std::size_t... Is>
	void update_all_maps_to_impl_(const tuple_type &keys, index_type new_index,
	                              std::index_sequence<Is...>) {
		(([](auto &map, const auto &key, index_type i) {
			 auto it = map.find(key); // should exist
			 // assert(it != map.end());
			 it->second = i;
		 }(std::get<Is>(maps_), std::get<Is>(keys), new_index)),
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
	std::pmr::vector<value_type>
	    store_; // stores pairs of (N unique keys tuple, mapped value)
	Maps maps_; // one unique index per key
};

} // namespace lego_assemble
