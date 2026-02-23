export module bricksim.utils.multi_key_set;

import std;
import bricksim.utils.hash;
import bricksim.utils.type_list;

namespace bricksim {

export template <class KeysList,
                 class HashList = typename KeysList::template map<std::hash>,
                 class EqList = typename KeysList::template map<std::equal_to>>
class MultiKeySet;

// No thread safety guarantee
// No strong exception guarantee
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

	using entry_type = std::tuple<Ks...>;

	explicit MultiKeySet() : maps_{make_map<Ks, Hs, Es>()...} {}

	template <class... Args>
	    requires(sizeof...(Args) == sizeof...(Ks) &&
	             (std::convertible_to<Args, Ks> && ...))
	bool emplace(Args &&...args) {
		if ((map_for_<Ks>().contains(args) || ...)) {
			return false;
		}
		const index_type idx = static_cast<index_type>(records_.size());
		records_.emplace_back(std::forward<Args>(args)...);
		std::apply(
		    [this, idx](const Ks &...keys) {
			    (map_for_<Ks>().emplace(keys, idx), ...);
		    },
		    records_.back());
		return true;
	}

	bool insert(const entry_type &t) {
		return std::apply(
		    [this](const Ks &...keys) { return emplace(keys...); }, t);
	}

	bool insert(entry_type &&t) {
		return std::apply(
		    [this](Ks &&...keys) { return emplace(std::move(keys)...); },
		    std::move(t));
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
		if (auto record = find(k)) {
			return &std::get<index_in_pack<To, Ks...>>(*record);
		}
		return nullptr;
	}

	const entry_type &entry_of(const in_pack<Ks...> auto &k) const {
		if (auto *ptr = find(k)) {
			return *ptr;
		}
		throw std::out_of_range("MultiKeySet::entry_of: key not found");
	}

	template <in_pack<Ks...> To>
	const To &key_of(const in_pack<Ks...> auto &k) const {
		if (auto *ptr = find_key<To>(k)) {
			return *ptr;
		}
		throw std::out_of_range("MultiKeySet::key_of: key not found");
	}

	bool contains(const in_pack<Ks...> auto &k) const {
		return map_for_<decltype(k)>().contains(k);
	}

	bool erase(const in_pack<Ks...> auto &k) {
		auto &map = map_for_<decltype(k)>();
		auto it = map.find(k);
		if (it == map.end())
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

	std::span<const entry_type> view() const & noexcept {
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

	std::vector<entry_type> records_;
	Maps maps_;

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
		    records_[victim]);

		if (victim != last) {
			records_[victim] = std::move(records_[last]);
			std::apply(
			    [this, victim](const Ks &...keys) {
				    ((map_for_<Ks>().find(keys)->second = victim), ...);
			    },
			    records_[victim]);
		}

		records_.pop_back();
	}
};

} // namespace bricksim
