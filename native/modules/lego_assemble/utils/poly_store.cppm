export module lego_assemble.utils.poly_store;

import std;
import lego_assemble.utils.type_list;
import lego_assemble.utils.multi_key_map;

namespace lego_assemble {

export template <class Ks, class Ts,
                 class Hs = typename Ks::template map<std::hash>,
                 class Es = typename Ks::template map<std::equal_to>>
class PolyStore;

export template <class... Ks, class... Ts, class... Hs, class... Es>
class PolyStore<type_list<Ks...>, type_list<Ts...>, type_list<Hs...>,
                type_list<Es...>> {
	static_assert(sizeof...(Ks) >= 1,
	              "PolyStore: at least one key type required");
	static_assert(sizeof...(Ts) >= 1,
	              "PolyStore: at least one value type required");
	static_assert(unique_types<Ks...>, "PolyStore: key types must be unique");
	static_assert(unique_types<Ts...>,
	              "PolyStore: TypesList must contain unique types");

  public:
	using KeysList = type_list<Ks...>;
	using HashList = type_list<Hs...>;
	using EqList = type_list<Es...>;

	using keys_type = std::tuple<Ks...>;
	using type_index_t = std::uint32_t;

  private:
	using value_index_t = std::uint32_t;

	struct Location {
		type_index_t type;
		value_index_t index;
	};

	template <type_index_t I>
	using type_index_constant = std::integral_constant<type_index_t, I>;

	template <std::size_t I>
	using type_at = std::tuple_element_t<I, std::tuple<Ts...>>;

	template <in_pack<Ts...> T>
	static constexpr type_index_t index_of_type =
	    static_cast<type_index_t>(index_in_pack<T, Ts...>);

	using Anchor = std::tuple_element_t<0, std::tuple<Ks...>>;

	template <class T> struct Storage {
		std::vector<T> data;
		std::vector<Anchor> ids;
	};

	using Storages = std::tuple<Storage<Ts>...>;
	using Directory = MultiKeyMap<type_list<Ks...>, Location, type_list<Hs...>,
	                              type_list<Es...>>;

	template <type_index_t Begin, type_index_t End, class Fn>
	static decltype(auto) dispatch_recursive_(type_index_t t, Fn &&fn) {
		if constexpr (End - Begin == 1) {
			return fn(type_index_constant<Begin>{});
		} else {
			constexpr type_index_t Mid = Begin + (End - Begin) / 2;
			if (t < Mid) {
				return dispatch_recursive_<Begin, Mid>(t, std::forward<Fn>(fn));
			} else {
				return dispatch_recursive_<Mid, End>(t, std::forward<Fn>(fn));
			}
		}
	}
	template <class Fn>
	static decltype(auto) dispatch_by_type_(type_index_t type_idx, Fn &&fn) {
		if (type_idx >= static_cast<type_index_t>(sizeof...(Ts))) {
			std::unreachable();
		}
		return dispatch_recursive_<static_cast<type_index_t>(0),
		                           static_cast<type_index_t>(sizeof...(Ts))>(
		    type_idx, std::forward<Fn>(fn));
	}

  public:
	template <bool Const> class EntryRef {
		friend class PolyStore;
		friend class EntryRef<!Const>;

	  public:
		using DirEntryPtr = const typename Directory::entry_type *;
		using ValuePtr = std::conditional_t<Const, const void *, void *>;
		template <in_pack<Ts...> T>
		using RefT = std::conditional_t<Const, const T &, T &>;
		template <in_pack<Ts...> T>
		using PtrT = std::conditional_t<Const, const T *, T *>;

	  private:
		DirEntryPtr dir_entry_;
		ValuePtr val_;

		EntryRef(DirEntryPtr dir_entry, ValuePtr val)
		    : dir_entry_{dir_entry}, val_{val} {}

	  public:
		template <bool OtherConst>
		    requires(Const && !OtherConst)
		EntryRef(const EntryRef<OtherConst> &other)
		    : dir_entry_(other.dir_entry_), val_(other.val_) {}

		const keys_type &keys() const noexcept {
			return dir_entry_->keys();
		}
		template <in_pack<Ks...> K> const K &key() const noexcept {
			return dir_entry_->template key<K>();
		}
		type_index_t type_index() const noexcept {
			return dir_entry_->value().type;
		}
		template <in_pack<Ts...> T> bool is() const noexcept {
			return type_index() == index_of_type<T>;
		}
		template <in_pack<Ts...> T> RefT<T> value() const {
			if (!is<T>()) {
				throw std::bad_cast();
			}
			return *static_cast<PtrT<T>>(val_);
		}
		template <in_pack<Ts...> T> PtrT<T> value_if() const noexcept {
			return is<T>() ? static_cast<PtrT<T>>(val_) : nullptr;
		}
		template <class Fn> decltype(auto) visit(Fn &&fn) const {
			return dispatch_by_type_(
			    type_index(), [&]<class I>(I) -> decltype(auto) {
				    using T = type_at<I::value>;
				    return std::invoke(std::forward<Fn>(fn),
				                       *static_cast<PtrT<T>>(val_));
			    });
		}
	};

	using entry_reference = EntryRef<false>;
	using const_entry_reference = EntryRef<true>;

	explicit PolyStore() : storages_{Storage<Ts>()...} {}

	template <in_pack<Ts...> T, tuple_of_size<sizeof...(Ks)> KeysTuple,
	          class... Args>
	    requires((std::convertible_to<
	                  std::tuple_element_t<index_in_pack<Ks, Ks...>,
	                                       std::remove_cvref_t<KeysTuple>>,
	                  Ks> &&
	              ...) &&
	             std::constructible_from<T, Args...>)
	bool emplace(KeysTuple &&kargs, Args &&...args) {
		auto &storage = std::get<index_of_type<T>>(storages_);
		const value_index_t idx =
		    static_cast<value_index_t>(storage.data.size());
		Location loc{index_of_type<T>, idx};

		// Use std::as_const to force a COPY of the first key.
		// Even if kargs is an rvalue, we read it as const& here so the
		// data stays valid for the directory move later.
		Anchor anchor = std::get<0>(std::as_const(kargs));

		// Try to insert into directory first to validate uniqueness.
		if (!directory_.emplace(std::forward<KeysTuple>(kargs), loc)) {
			return false;
		}

		storage.data.emplace_back(std::forward<Args>(args)...);
		storage.ids.emplace_back(std::move(anchor));
		return true;
	}

	template <in_pack<Ts...> T, class... Args>
	    requires(sizeof...(Args) >= sizeof...(Ks) &&
	             (type_list<Args...>::template take_front<
	                 sizeof...(Ks)>::template convertible_to<Ks...>) &&
	             (type_list<Args...>::template drop_front<
	                 sizeof...(Ks)>::template can_construct<T>))
	bool emplace(Args &&...args) {
		using ArgsList = type_list<Args...>;
		return select_invoke<
		    typename ArgsList::template drop_front_seq<sizeof...(Ks)>>(
		    [&]<class... VArgs>(VArgs &&...vargs) {
			    return emplace<T>(
			        select_forward_as_tuple<
			            typename ArgsList::template front_seq<sizeof...(Ks)>>(
			            std::forward<Args>(args)...),
			        std::forward<VArgs>(vargs)...);
		    },
		    std::forward<Args>(args)...);
	}

	auto find(this auto &&self, const in_pack<Ks...> auto &k)
	    -> std::optional<std::conditional_t<
	        std::is_const_v<std::remove_reference_t<decltype(self)>>,
	        const_entry_reference, entry_reference>> {
		using ValuePtr = std::conditional_t<
		    std::is_const_v<std::remove_reference_t<decltype(self)>>,
		    const void *, void *>;
		const auto *dir_entry = self.directory_.find(k);
		if (!dir_entry) {
			return std::nullopt;
		}
		const Location &loc = dir_entry->value();
		ValuePtr ptr = dispatch_by_type_(loc.type, [&]<class I>(I) -> ValuePtr {
			return &std::get<I::value>(self.storages_).data[loc.index];
		});
		return {{dir_entry, ptr}};
	}

	template <in_pack<Ks...> To>
	const To *find_key(const in_pack<Ks...> auto &k) const {
		return directory_.template find_key<To>(k);
	}
	const keys_type *find_keys(const in_pack<Ks...> auto &k) const {
		return directory_.find_keys(k);
	}
	template <in_pack<Ts...> T>
	auto find_value(this auto &&self, const in_pack<Ks...> auto &k)
	    -> std::conditional_t<
	        std::is_const_v<std::remove_reference_t<decltype(self)>>, const T *,
	        T *> {
		const auto *dir_entry = self.directory_.find(k);
		if (!dir_entry) {
			return nullptr;
		}
		const Location &loc = dir_entry->value();
		if (loc.type != index_of_type<T>) {
			return nullptr;
		}
		auto &storage = std::get<index_of_type<T>>(self.storages_);
		return &storage.data[loc.index];
	}

	auto entry_of(this auto &&self, const in_pack<Ks...> auto &k)
	    -> std::conditional_t<
	        std::is_const_v<std::remove_reference_t<decltype(self)>>,
	        const_entry_reference, entry_reference> {
		if (auto opt = self.find(k)) {
			return *opt;
		}
		throw std::out_of_range("PolyStore::entry_of: key not found");
	}
	template <in_pack<Ks...> To>
	const To &key_of(const in_pack<Ks...> auto &k) const {
		if (const auto *ptr = find_key<To>(k)) {
			return *ptr;
		}
		throw std::out_of_range("PolyStore::key_of: key not found");
	}
	const keys_type &keys_of(const in_pack<Ks...> auto &k) const {
		if (const auto *ptr = find_keys(k)) {
			return *ptr;
		}
		throw std::out_of_range("PolyStore::keys_of: key not found");
	}
	template <in_pack<Ts...> T>
	auto value_of(this auto &&self, const in_pack<Ks...> auto &k)
	    -> std::conditional_t<
	        std::is_const_v<std::remove_reference_t<decltype(self)>>, const T &,
	        T &> {
		if (auto *ptr = self.template find_value<T>(k)) {
			return *ptr;
		}
		throw std::out_of_range(
		    "PolyStore::value_of: key not found or type mismatch");
	}

	template <class Fn>
	decltype(auto) visit(this auto &&self, const in_pack<Ks...> auto &k,
	                     Fn &&fn) {
		const auto *dir_entry = self.directory_.find(k);
		if (!dir_entry) {
			throw std::out_of_range("PolyStore::visit: key not found");
		}
		const Location &loc = dir_entry->value();
		return dispatch_by_type_(loc.type, [&]<class I>(I) -> decltype(auto) {
			auto &storage = std::get<I::value>(self.storages_);
			return std::invoke(std::forward<Fn>(fn), storage.data[loc.index]);
		});
	}

	template <class Fn>
	decltype(auto) try_visit(this auto &&self, const in_pack<Ks...> auto &k,
	                         Fn &&fn) {
		constexpr bool Const =
		    std::is_const_v<std::remove_reference_t<decltype(self)>>;
		using ExampleArg =
		    std::conditional_t<Const, const type_at<0> &, type_at<0> &>;
		using InvokeResult = std::invoke_result_t<Fn, ExampleArg>;

		if constexpr (std::is_void_v<InvokeResult>) {
			// void -> bool
			const auto *dir_entry = self.directory_.find(k);
			if (!dir_entry) {
				return false;
			}
			const Location &loc = dir_entry->value();
			self.dispatch_by_type_(loc.type, [&]<class I>(I) {
				auto &storage = std::get<I::value>(self.storages_);
				std::invoke(std::forward<Fn>(fn), storage.data[loc.index]);
			});
			return true;

		} else if constexpr (!std::is_reference_v<InvokeResult>) {
			// value -> std::optional<value>
			const auto *dir_entry = self.directory_.find(k);
			if (!dir_entry) {
				return std::optional<InvokeResult>{};
			}
			const Location &loc = dir_entry->value();
			return self.dispatch_by_type_(loc.type, [&]<class I>(I) {
				auto &storage = std::get<I::value>(self.storages_);
				return std::optional<InvokeResult>{
				    std::invoke(std::forward<Fn>(fn), storage.data[loc.index])};
			});

		} else {
			// reference -> not supported
			static_assert(
			    false,
			    "PolyStore::try_visit: returning references is not supported");
		}
	}

	bool contains(const in_pack<Ks...> auto &k) const {
		return directory_.contains(k);
	}

	bool erase(const in_pack<Ks...> auto &k) {
		auto *dir_entry = directory_.find(k);
		if (!dir_entry) {
			return false;
		}
		Location loc = dir_entry->value();
		directory_.erase(k);
		dispatch_by_type_(loc.type, [&]<class I>(I) {
			auto &storage = std::get<I::value>(storages_);
			const std::size_t last = storage.data.size() - 1;
			const std::size_t victim = loc.index;
			if (victim != last) {
				storage.data[victim] = std::move(storage.data[last]);
				storage.ids[victim] = std::move(storage.ids[last]);
				Location &moved_loc = directory_.value_of(storage.ids[victim]);
				moved_loc.index = static_cast<value_index_t>(victim);
			}
			storage.data.pop_back();
			storage.ids.pop_back();
		});
		return true;
	}

	std::size_t size() const noexcept {
		return directory_.size();
	}

	template <in_pack<Ts...> T> std::size_t size_of_type() const noexcept {
		const auto &storage = std::get<index_of_type<T>>(storages_);
		return storage.data.size();
	}

	bool empty() const noexcept {
		return directory_.empty();
	}

	template <in_pack<Ts...> T>
	auto value_view(this auto &&self) noexcept -> std::span<std::conditional_t<
	    std::is_const_v<std::remove_reference_t<decltype(self)>>, const T, T>> {
		auto &storage = std::get<index_of_type<T>>(self.storages_);
		return {storage.data.data(), storage.data.size()};
	}

	template <in_pack<Ts...> T>
	auto entries_of_type(this auto &&self) -> std::generator<
	    std::tuple<const keys_type &,
	               std::conditional_t<
	                   std::is_const_v<std::remove_reference_t<decltype(self)>>,
	                   const T &, T &>>> {
		auto &storage = std::get<index_of_type<T>>(self.storages_);
		for (std::size_t i = 0; i < storage.data.size(); ++i) {
			const Anchor &anchor = storage.ids[i];
			const keys_type &keys = self.directory_.keys_of(anchor);
			auto &value = storage.data[i];
			co_yield {keys, value};
		}
	}

	template <class Fn> void for_each(this auto &&self, Fn &&fn) {
		(
		    [&](auto &storage) {
			    for (std::size_t i = 0; i < storage.data.size(); ++i) {
				    const Anchor &anchor = storage.ids[i];
				    const keys_type &keys = self.directory_.keys_of(anchor);
				    auto &value = storage.data[i];
				    std::invoke(fn, keys, value);
			    }
		    }(std::get<index_of_type<Ts>>(self.storages_)),
		    ...);
	}

	template <in_pack<Ts...> T> void reserve_for_type(std::size_t n) {
		auto &storage = std::get<index_of_type<T>>(storages_);
		storage.data.reserve(n);
		storage.ids.reserve(n);
		directory_.reserve(n);
	}

	void reserve(std::convertible_to<std::size_t> auto... ns) {
		static_assert(sizeof...(ns) == sizeof...(Ts),
		              "PolyStore::reserve: number of arguments must equal to "
		              "number of types");
		std::size_t sum = 0;
		((std::get<index_of_type<Ts>>(storages_).data.reserve(ns),
		  std::get<index_of_type<Ts>>(storages_).ids.reserve(ns), sum += ns),
		 ...);
		directory_.reserve(sum);
	}

	template <in_pack<Ts...> T> void clear_type() {
		auto &storage = std::get<index_of_type<T>>(storages_);
		for (const Anchor &anchor : storage.ids) {
			directory_.erase(anchor);
		}
		storage.data.clear();
		storage.ids.clear();
	}

	void clear() {
		(std::get<index_of_type<Ts>>(storages_).data.clear(), ...);
		(std::get<index_of_type<Ts>>(storages_).ids.clear(), ...);
		directory_.clear();
	}

  private:
	Storages storages_;
	Directory directory_;
};

} // namespace lego_assemble
