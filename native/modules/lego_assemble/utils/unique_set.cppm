export module lego_assemble.utils.unique_set;

import std;

namespace lego_assemble {

// ---------- Concept: UniqueSetLike ----------
// A PMR-constructible container of unique T with iteration, size,
// contains(), add(), remove() returning bool.
export template <class S, class T>
concept UniqueSetLike =
    std::ranges::forward_range<const S> &&
    std::same_as<std::ranges::range_value_t<S>, T> &&
    std::constructible_from<S, std::pmr::memory_resource *> &&
    requires(S s, const S cs, const T &x) {
	    { cs.size() } -> std::convertible_to<std::size_t>;
	    { cs.contains(x) } -> std::same_as<bool>;
	    { s.add(x) } -> std::same_as<bool>;    // true if inserted
	    { s.remove(x) } -> std::same_as<bool>; // true if erased
	    { cs.begin() } -> std::forward_iterator;
	    { cs.end() } -> std::sentinel_for<decltype(cs.begin())>;
    };

// ---------- OrderedVecSet: sorted unique vector<T> ----------
export template <class T, class Cmp = std::less<T>> class OrderedVecSet {
	static_assert(std::strict_weak_order<Cmp, const T &, const T &>);
	std::pmr::vector<T> data_;
	[[no_unique_address]] Cmp cmp_{};

	// equality under strict weak ordering
	bool equiv_(const T &a, const T &b) const {
		return !cmp_(a, b) && !cmp_(b, a);
	}
	typename std::pmr::vector<T>::const_iterator
	lower_bound_(const T &x) const {
		return std::lower_bound(data_.begin(), data_.end(), x, cmp_);
	}
	typename std::pmr::vector<T>::iterator lower_bound_(const T &x) {
		return std::lower_bound(data_.begin(), data_.end(), x, cmp_);
	}

  public:
	using value_type = T;
	using const_iterator = std::pmr::vector<T>::const_iterator;

	OrderedVecSet(
	    std::pmr::memory_resource *r = std::pmr::get_default_resource(),
	    Cmp cmp = {})
	    : data_{r}, cmp_(std::move(cmp)) {}

	// iteration (const range)
	const_iterator begin() const noexcept {
		return data_.begin();
	}
	const_iterator end() const noexcept {
		return data_.end();
	}

	std::size_t size() const noexcept {
		return data_.size();
	}
	bool empty() const noexcept {
		return data_.empty();
	}
	void clear() noexcept {
		data_.clear();
	}

	// optional perf knobs
	void reserve(std::size_t n) {
		data_.reserve(n);
	}

	bool contains(const T &x) const {
		auto it = lower_bound_(x);
		return it != data_.end() && equiv_(*it, x);
	}

	bool add(const T &x) {
		auto it = lower_bound_(x);
		if (it == data_.end() || !equiv_(*it, x)) {
			data_.insert(it, x);
			return true;
		}
		return false;
	}

	bool remove(const T &x) {
		auto it = lower_bound_(x);
		if (it != data_.end() && equiv_(*it, x)) {
			data_.erase(it);
			return true;
		}
		return false;
	}
};
static_assert(UniqueSetLike<OrderedVecSet<int>, int>);

// ---------- HashSet: wraps pmr::unordered_set<T,Hash,Eq> ----------
export template <class T, class Hash = std::hash<T>,
                 class Eq = std::equal_to<T>>
class HashSet {
	std::pmr::unordered_set<T, Hash, Eq> set_;

  public:
	using value_type = T;
	using const_iterator = std::pmr::unordered_set<T, Hash, Eq>::const_iterator;

	HashSet(std::pmr::memory_resource *r = std::pmr::get_default_resource(),
	        std::size_t bucket_count = 0, Hash h = {}, Eq eq = {})
	    : set_(bucket_count, h, eq, std::pmr::polymorphic_allocator<T>{r}) {}

	const_iterator begin() const noexcept {
		return set_.begin();
	}
	const_iterator end() const noexcept {
		return set_.end();
	}

	std::size_t size() const noexcept {
		return set_.size();
	}
	bool empty() const noexcept {
		return set_.empty();
	}
	void clear() noexcept {
		set_.clear();
	}

	void reserve(std::size_t n) {
		set_.reserve(n);
	} // optional

	bool contains(const T &x) const {
		return set_.contains(x);
	}
	bool add(const T &x) {
		return set_.insert(x).second;
	}
	bool remove(const T &x) {
		return set_.erase(x) != 0;
	}
};
static_assert(UniqueSetLike<HashSet<int>, int>);

// ---------- (Optional) TreeSet: wraps pmr::set<T,Cmp> ----------
export template <class T, class Cmp = std::less<T>> class TreeSet {
	std::pmr::set<T, Cmp> set_;

  public:
	using value_type = T;
	using const_iterator = std::pmr::set<T, Cmp>::const_iterator;

	TreeSet(std::pmr::memory_resource *r = std::pmr::get_default_resource(),
	        Cmp cmp = {})
	    : set_(cmp, std::pmr::polymorphic_allocator<T>{r}) {}

	const_iterator begin() const noexcept {
		return set_.begin();
	}
	const_iterator end() const noexcept {
		return set_.end();
	}

	std::size_t size() const noexcept {
		return set_.size();
	}
	bool empty() const noexcept {
		return set_.empty();
	}
	void clear() noexcept {
		set_.clear();
	}

	bool contains(const T &x) const {
		return set_.contains(x);
	}
	bool add(const T &x) {
		return set_.insert(x).second;
	}
	bool remove(const T &x) {
		return set_.erase(x) != 0;
	}
};
static_assert(UniqueSetLike<TreeSet<int>, int>);

} // namespace lego_assemble
