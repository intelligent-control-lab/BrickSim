export module lego_assemble.utils.type_list;

import std;

namespace lego_assemble {

export template <class... Ts> using last_type = Ts...[sizeof...(Ts) - 1];

export template <class... Ts> decltype(auto) last_value(Ts &&...args) {
	static_assert(sizeof...(args) > 0);
	return args...[sizeof...(args) - 1];
}

export template <class... Args>
constexpr std::pmr::memory_resource *tail_mr(Args &&...args) {
	if constexpr (sizeof...(args) > 0 &&
	              std::same_as<std::remove_cvref_t<last_type<Args...>>,
	                           std::pmr::memory_resource *>) {
		return last_value(std::forward<Args>(args)...);
	} else {
		return std::pmr::get_default_resource();
	}
}

export template <class T, class... Ts>
concept in_pack = (std::same_as<T, Ts> || ... || false);

export template <class... Ts> constexpr bool unique_types = true;
export template <class T, class... Ts>
constexpr bool unique_types<T, Ts...> =
    (!std::is_same_v<T, Ts> && ...) && unique_types<Ts...>;

export template <class T, class... Ts>
    requires unique_types<Ts...> && in_pack<T, Ts...>
constexpr std::size_t index_in_pack =
    []<std::size_t... Is>(std::index_sequence<Is...>) consteval {
	    return ((std::same_as<T, Ts> ? Is : 0u) + ...);
    }(std::index_sequence_for<Ts...>{});

template <std::size_t N, class Seq> struct index_shift_t;
template <std::size_t N, std::size_t... Is>
struct index_shift_t<N, std::index_sequence<Is...>> {
	using type = std::index_sequence<(Is + N)...>;
};
export template <std::size_t N, class Seq>
using index_shift = typename index_shift_t<N, Seq>::type;

export template <class Seq, class F, class... Args>
constexpr decltype(auto) select_invoke(F &&f, Args &&...args) {
	return [&]<std::size_t... Is>(std::index_sequence<Is...>) {
		static_assert(((Is < sizeof...(Args)) && ...), "Index out of bounds");
		return std::invoke(std::forward<F>(f),
		                   std::forward<Args...[Is]>(args...[Is])...);
	}(Seq{});
}

export template <class Seq, class... Args>
constexpr decltype(auto) select_forward_as_tuple(Args &&...args) {
	return [&]<std::size_t... Is>(std::index_sequence<Is...>) {
		static_assert(((Is < sizeof...(Args)) && ...), "Index out of bounds");
		return std::forward_as_tuple(std::forward<Args...[Is]>(args...[Is])...);
	}(Seq{});
}

export template <class T>
concept tuple_like =
    requires { typename std::tuple_size<std::remove_cvref_t<T>>::type; };

export template <class T, std::size_t N>
concept tuple_of_size =
    tuple_like<T> && (std::tuple_size_v<std::remove_cvref_t<T>> == N);

// TODO: Workaround for https://github.com/clangd/clangd/issues/2529
template <std::size_t N, class... Ts> struct _type_list_at {
	using type = Ts...[N];
};

export template <class... Ts> struct type_list {
  private:
	// Helper for select
	template <class Seq> struct select_seq_t;
	template <std::size_t... Is>
	struct select_seq_t<std::index_sequence<Is...>> {
		static_assert(((Is < sizeof...(Ts)) && ...), "Index out of bounds");
		using type = type_list<Ts...[Is]...>;
	};

  public:
	static constexpr std::size_t size = sizeof...(Ts);

	static constexpr bool unique = unique_types<Ts...>;

	template <class U>
	static constexpr std::size_t index_of = index_in_pack<U, Ts...>;

	template <std::size_t N>
	    requires(N < size)
	// using at = Ts...[N];
	using at = _type_list_at<N, Ts...>::type;

	template <class... Us>
	static constexpr bool same_as = (std::same_as<Ts, Us> && ...);

	template <class... Us>
	static constexpr bool convertible_to = (std::convertible_to<Ts, Us> && ...);

	template <class U>
	static constexpr bool is_subset_of = (U::template contains<Ts> && ...);

	template <class U>
	static constexpr bool is_superset_of =
	    U::template is_subset_of<type_list<Ts...>>;

	template <class... Us>
	static constexpr bool same_as_remove_cvref =
	    (std::same_as<std::remove_cvref_t<Ts>, std::remove_cvref_t<Us>> && ...);

	template <class U>
	static constexpr bool can_construct = std::constructible_from<U, Ts...>;

	template <template <class> class F> using map = type_list<F<Ts>...>;

	template <template <class, class...> class F, class... Us>
	using map_n = type_list<F<Ts, Us...>...>;

	template <class... Us> using append = type_list<Ts..., Us...>;
	template <class... Us> using prepend = type_list<Us..., Ts...>;

	template <class U> static constexpr bool contains = in_pack<U, Ts...>;

	template <std::size_t... Is> using select = type_list<Ts[Is]...>;

	template <class Seq> using select_seq = typename select_seq_t<Seq>::type;

	template <std::size_t N>
	    requires(N <= size)
	using front_seq = std::make_index_sequence<N>;

	template <std::size_t N>
	    requires(N <= size)
	using take_front = select_seq<front_seq<N>>;

	template <std::size_t N>
	    requires(N <= size)
	using back_seq = index_shift<size - N, std::make_index_sequence<N>>;

	template <std::size_t N>
	    requires(N <= size)
	using take_back = select_seq<back_seq<N>>;

	template <std::size_t N>
	    requires(N <= size)
	using drop_front_seq = index_shift<N, std::make_index_sequence<size - N>>;

	template <std::size_t N>
	    requires(N <= size)
	using drop_front = select_seq<drop_front_seq<N>>;

	template <std::size_t N>
	    requires(N <= size)
	using drop_back_seq = std::make_index_sequence<size - N>;

	template <std::size_t N>
	    requires(N <= size)
	using drop_back = select_seq<drop_back_seq<N>>;
};

template <class... Lists> struct type_list_concat_t;
template <> struct type_list_concat_t<> {
	using type = type_list<>;
};
template <class... Ts> struct type_list_concat_t<type_list<Ts...>> {
	using type = type_list<Ts...>;
};
template <class... Ts, class... Us, class... Rest>
struct type_list_concat_t<type_list<Ts...>, type_list<Us...>, Rest...> {
	using type =
	    typename type_list_concat_t<type_list<Ts..., Us...>, Rest...>::type;
};
export template <class... Lists>
using type_list_concat = typename type_list_concat_t<Lists...>::type;

} // namespace lego_assemble
