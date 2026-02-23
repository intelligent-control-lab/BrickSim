export module bricksim.utils.multi_key_map;

import std;
import bricksim.utils.hash;
import bricksim.utils.type_list;

namespace bricksim {

export template <class KeysList, class Mapped> class MultiKeyMapEntry;

export template <class... Ks, class M>
class MultiKeyMapEntry<type_list<Ks...>, M> {
  public:
	using keys_type = std::tuple<Ks...>;
	using value_type = M;

	MultiKeyMapEntry() = default;
	MultiKeyMapEntry(MultiKeyMapEntry &&) = default;
	MultiKeyMapEntry(const MultiKeyMapEntry &) = default;
	MultiKeyMapEntry &operator=(const MultiKeyMapEntry &) = delete;

	template <class KeyTuple, class ValueTuple>
	MultiKeyMapEntry(std::piecewise_construct_t, KeyTuple &&key_args,
	                 ValueTuple &&value_args)
	    : keys_(std::make_from_tuple<keys_type>(
	          std::forward<KeyTuple>(key_args))),
	      value_(std::make_from_tuple<value_type>(
	          std::forward<ValueTuple>(value_args))) {}

	const keys_type &keys() const noexcept {
		return keys_;
	}
	template <in_pack<Ks...> K> const K &key() const noexcept {
		return std::get<index_in_pack<K, Ks...>>(keys_);
	}
	value_type &value() noexcept {
		return value_;
	}
	const value_type &value() const noexcept {
		return value_;
	}

	template <std::size_t I> decltype(auto) get() noexcept {
		if constexpr (I == 0) {
			return std::as_const(keys_);
		} else if constexpr (I == 1) {
			return (value_);
		}
	}

	template <std::size_t I> decltype(auto) get() const noexcept {
		if constexpr (I == 0)
			return (keys_);
		else if constexpr (I == 1)
			return (value_);
	}

  private:
	template <class, class, class, class> friend class MultiKeyMap;

	keys_type keys_;
	value_type value_;
	MultiKeyMapEntry &operator=(MultiKeyMapEntry &&) = default;
};

export template <class KeysList, class Mapped,
                 class HashList = typename KeysList::template map<std::hash>,
                 class EqList = typename KeysList::template map<std::equal_to>>
class MultiKeyMap;

// No thread safety guarantee
// No strong exception guarantee
export template <class... Ks, class M, class... Hs, class... Es>
class MultiKeyMap<type_list<Ks...>, M, type_list<Hs...>, type_list<Es...>> {
  public:
	using KeysList = type_list<Ks...>;
	using HashList = type_list<Hs...>;
	using EqList = type_list<Es...>;

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

	using keys_type = std::tuple<Ks...>;
	using value_type = M;
	using entry_type = MultiKeyMapEntry<type_list<Ks...>, M>;

	explicit MultiKeyMap() : maps_{make_map<Ks, Hs, Es>()...} {}

	template <tuple_of_size<sizeof...(Ks)> KeysTuple, class... VArgs>
	    requires((std::convertible_to<
	                  std::tuple_element_t<index_in_pack<Ks, Ks...>,
	                                       std::remove_cvref_t<KeysTuple>>,
	                  Ks> &&
	              ...) &&
	             (std::constructible_from<value_type, VArgs...>))
	bool emplace(KeysTuple &&kargs, VArgs &&...vargs) {
		if ((map_for_<Ks>().contains(
		         std::get<index_in_pack<Ks, Ks...>>(kargs)) ||
		     ...)) {
			return false;
		}
		const index_type idx = static_cast<index_type>(records_.size());
		records_.emplace_back(
		    std::piecewise_construct, std::forward<KeysTuple>(kargs),
		    std::forward_as_tuple(std::forward<VArgs>(vargs)...));
		std::apply(
		    [this, idx](const Ks &...keys) {
			    (map_for_<Ks>().emplace(keys, idx), ...);
		    },
		    records_.back().keys());
		return true;
	}

	template <class... Args>
	    requires(sizeof...(Args) >= sizeof...(Ks) &&
	             (type_list<Args...>::template take_front<
	                 sizeof...(Ks)>::template convertible_to<Ks...>) &&
	             (type_list<Args...>::template drop_front<
	                 sizeof...(Ks)>::template can_construct<value_type>))
	bool emplace(Args &&...args) {
		using ArgsList = type_list<Args...>;
		return select_invoke<
		    typename ArgsList::template drop_front_seq<sizeof...(Ks)>>(
		    [&]<class... VArgs>(VArgs &&...vargs) {
			    return emplace(
			        select_forward_as_tuple<
			            typename ArgsList::template front_seq<sizeof...(Ks)>>(
			            std::forward<Args>(args)...),
			        std::forward<VArgs>(vargs)...);
		    },
		    std::forward<Args>(args)...);
	}

	bool insert(const keys_type &keys, const value_type &value) {
		return emplace(keys, value);
	}

	bool insert(const keys_type &keys, value_type &&value) {
		return emplace(keys, std::move(value));
	}

	bool insert(keys_type &&keys, value_type &&value) {
		return emplace(std::move(keys), std::move(value));
	}

	entry_type *find(const in_pack<Ks...> auto &k) {
		const auto &map = map_for_<decltype(k)>();
		auto it = map.find(k);
		if (it == map.end())
			return nullptr;
		return &records_[it->second];
	}
	const entry_type *find(const in_pack<Ks...> auto &k) const {
		const auto &map = map_for_<decltype(k)>();
		auto it = map.find(k);
		if (it == map.end())
			return nullptr;
		return &records_[it->second];
	}
	template <in_pack<Ks...> To>
	const To *find_key(const in_pack<Ks...> auto &k) const {
		if (const auto *record = find(k)) {
			return &record->template key<To>();
		}
		return nullptr;
	}
	const keys_type *find_keys(const in_pack<Ks...> auto &k) const {
		if (const auto *record = find(k)) {
			return &record->keys();
		}
		return nullptr;
	}
	value_type *find_value(const in_pack<Ks...> auto &k) {
		if (auto *record = find(k)) {
			return &record->value();
		}
		return nullptr;
	}
	const value_type *find_value(const in_pack<Ks...> auto &k) const {
		if (const auto *record = find(k)) {
			return &record->value();
		}
		return nullptr;
	}

	entry_type &entry_of(const in_pack<Ks...> auto &k) {
		if (auto *ptr = find(k)) {
			return *ptr;
		}
		throw std::out_of_range("MultiKeyMap::entry_of: key not found");
	}
	const entry_type &entry_of(const in_pack<Ks...> auto &k) const {
		if (const auto *ptr = find(k)) {
			return *ptr;
		}
		throw std::out_of_range("MultiKeyMap::entry_of: key not found");
	}
	template <in_pack<Ks...> To>
	const To &key_of(const in_pack<Ks...> auto &k) const {
		if (const auto *ptr = find_key<To>(k)) {
			return *ptr;
		}
		throw std::out_of_range("MultiKeyMap::key_of: key not found");
	}
	const keys_type &keys_of(const in_pack<Ks...> auto &k) const {
		if (const auto *ptr = find_keys(k)) {
			return *ptr;
		}
		throw std::out_of_range("MultiKeyMap::keys_of: key not found");
	}
	value_type &value_of(const in_pack<Ks...> auto &k) {
		if (auto *ptr = find_value(k)) {
			return *ptr;
		}
		throw std::out_of_range("MultiKeyMap::value_of: key not found");
	}
	const value_type &value_of(const in_pack<Ks...> auto &k) const {
		if (const auto *ptr = find_value(k)) {
			return *ptr;
		}
		throw std::out_of_range("MultiKeyMap::value_of: key not found");
	}

	bool contains(const in_pack<Ks...> auto &k) const {
		return map_for_<decltype(k)>().contains(k);
	}

	bool erase(const in_pack<Ks...> auto &k) {
		auto &m = map_for_<decltype(k)>();
		auto it = m.find(k);
		if (it == m.end())
			return false;
		erase_at_index_(it->second);
		return true;
	}

	std::size_t size() const noexcept {
		return records_.size();
	}
	bool empty() const noexcept {
		return records_.empty();
	}

	std::span<entry_type> view() noexcept {
		return {records_.data(), records_.size()};
	}
	std::span<const entry_type> view() const noexcept {
		return {records_.data(), records_.size()};
	}

	void reserve(std::size_t n) {
		records_.reserve(n);
		(map_for_<Ks>().reserve(n), ...);
	}

	void clear() {
		records_.clear();
		(map_for_<Ks>().clear(), ...);
	}

  private:
	using index_type = std::uint32_t;

	template <class K, class H, class E>
	using KeyMap = std::unordered_map<K, index_type, H, E>;

	using Maps = std::tuple<KeyMap<Ks, Hs, Es>...>;

	template <class K, class H, class E> static KeyMap<K, H, E> make_map() {
		return KeyMap<K, H, E>(0, H{}, E{});
	}

	template <class K>
	    requires in_pack<std::remove_cvref_t<K>, Ks...>
	auto &map_for_() noexcept {
		return std::get<index_in_pack<std::remove_cvref_t<K>, Ks...>>(maps_);
	}
	template <class K>
	    requires in_pack<std::remove_cvref_t<K>, Ks...>
	const auto &map_for_() const noexcept {
		return std::get<index_in_pack<std::remove_cvref_t<K>, Ks...>>(maps_);
	}

	void erase_at_index_(index_type victim) {
		const index_type last = static_cast<index_type>(records_.size() - 1);

		std::apply(
		    [this](const Ks &...keys) { (map_for_<Ks>().erase(keys), ...); },
		    records_[victim].keys());

		if (victim != last) {
			records_[victim] = std::move(records_[last]);
			std::apply(
			    [this, victim](const Ks &...keys) {
				    ((map_for_<Ks>().find(keys)->second = victim), ...);
			    },
			    records_[victim].keys());
		}

		records_.pop_back();
	}

  private:
	std::vector<entry_type> records_;
	Maps maps_;
};

} // namespace bricksim

// Structured Binding Support
namespace std {

export template <class KeysList, class Mapped>
struct tuple_size<bricksim::MultiKeyMapEntry<KeysList, Mapped>>
    : std::integral_constant<std::size_t, 2> {};

export template <class KeysList, class Mapped>
struct tuple_size<const bricksim::MultiKeyMapEntry<KeysList, Mapped>>
    : std::integral_constant<std::size_t, 2> {};

export template <std::size_t I, class KeysList, class Mapped>
struct tuple_element<I, bricksim::MultiKeyMapEntry<KeysList, Mapped>> {
	using EntryType = bricksim::MultiKeyMapEntry<KeysList, Mapped>;
	using type =
	    std::conditional_t<I == 0, const typename EntryType::keys_type, Mapped>;
};

export template <std::size_t I, class KeysList, class Mapped>
struct tuple_element<I,
                     const bricksim::MultiKeyMapEntry<KeysList, Mapped>> {
	using EntryType = bricksim::MultiKeyMapEntry<KeysList, Mapped>;
	using type = std::conditional_t<I == 0, const typename EntryType::keys_type,
	                                const Mapped>;
};

} // namespace std
