export module bricksim.utils.dynamic_graph;

import std;
import bricksim.utils.unordered_pair;

namespace bricksim {

// --------- Common types / concepts ---------

// uint32_t is enough
export using vertex_id = std::uint32_t;

export template <class CV>
concept ComponentViewLike = requires(CV cv) {
	{ cv.size() } -> std::same_as<std::size_t>;
	{
		// Root vertex of the component
		// This is the starting point for traversals
		cv.root()
	} -> std::same_as<vertex_id>;
	{ cv.vertices() } -> std::same_as<std::generator<vertex_id>>;
	{
		// BFS all edges in the component
		cv.edges()
	} -> std::same_as<std::generator<std::pair<vertex_id, vertex_id>>>;
	{
		// BFS all tree edges in the component
		cv.tree_edges()
	} -> std::same_as<std::generator<std::pair<vertex_id, vertex_id>>>;
};

export template <class G>
concept DynamicGraphLike = requires(G g, const G cg, vertex_id u, vertex_id v) {
	{ g.reset(0u) } -> std::same_as<void>;
	{ cg.num_vertices() } -> std::convertible_to<std::size_t>;
	{ g.add_vertex() } -> std::same_as<vertex_id>;
	{ g.erase_vertex(u) } -> std::same_as<bool>;
	{ g.add_edge(u, v) } -> std::same_as<bool>;
	{ g.erase_edge(u, v) } -> std::same_as<bool>;
	{ cg.connected(u, v) } -> std::same_as<bool>; // must be const
	{ g.clear() } -> std::same_as<void>;
	{
		cg.path(u, v)
	} -> std::same_as<std::generator<std::pair<vertex_id, vertex_id>>>;
	{ cg.component_size(u) } -> std::same_as<std::size_t>;
	typename G::component_view_type;
	requires ComponentViewLike<typename G::component_view_type>;
	{
		cg.components()
	} -> std::same_as<std::generator<typename G::component_view_type>>;
	{
		// If a non-existent vertex is given, it returns an empty view
		cg.component_view(u)
	} -> std::same_as<typename G::component_view_type>;
} && std::constructible_from<G, std::size_t>;

// --------- Naive dynamic graph (PMR, fixed-width ints, const connected) ---------

// A naive dynamic graph implementation for cross-checking
// Efficiency is not a concern here.
export class NaiveDynamicGraph {
  public:
	class ComponentView {
	  public:
		[[nodiscard]] std::size_t size() const {
			return g_->component_size(root_);
		}

		[[nodiscard]] vertex_id root() const {
			return root_;
		}

		std::generator<vertex_id> vertices() const {
			if (!g_->valid(root_))
				co_return;

			// Optimization: Use unordered_set for O(|Component|) memory/init
			// instead of O(|Graph|) vector.
			std::unordered_set<vertex_id> vis;
			std::deque<vertex_id> dq;
			std::queue<vertex_id, std::deque<vertex_id>> q(std::move(dq));

			vis.insert(root_);
			q.push(root_);

			while (!q.empty()) {
				auto x = q.front();
				q.pop();
				co_yield x;

				for (auto y : g_->adj_[x]) {
					// Only push valid, unvisited neighbors
					if (g_->alive_[y] != 0 && vis.insert(y).second) {
						q.push(y);
					}
				}
			}
		}

		std::generator<std::pair<vertex_id, vertex_id>> edges() const {
			if (!g_->valid(root_))
				co_return;

			std::unordered_set<vertex_id> vis;
			std::deque<vertex_id> dq;
			std::queue<vertex_id, std::deque<vertex_id>> q(std::move(dq));

			vis.insert(root_);
			q.push(root_);

			while (!q.empty()) {
				auto x = q.front();
				q.pop();

				for (auto y : g_->adj_[x]) {
					if (g_->alive_[y] == 0)
						continue;

					// Yield edge exactly once based on ID order
					if (x < y) {
						co_yield {x, y};
					}

					if (vis.insert(y).second) {
						q.push(y);
					}
				}
			}
		}

		std::generator<std::pair<vertex_id, vertex_id>> tree_edges() const {
			if (!g_->valid(root_))
				co_return;

			std::unordered_set<vertex_id> vis;
			std::deque<vertex_id> dq;
			std::queue<vertex_id, std::deque<vertex_id>> q(std::move(dq));

			vis.insert(root_);
			q.push(root_);

			while (!q.empty()) {
				auto x = q.front();
				q.pop();

				for (auto y : g_->adj_[x]) {
					if (g_->alive_[y] != 0 && vis.insert(y).second) {
						// {x, y} is a tree edge discovered by BFS
						co_yield {x, y};
						q.push(y);
					}
				}
			}
		}

	  private:
		const NaiveDynamicGraph *g_;
		vertex_id root_;

		ComponentView(const NaiveDynamicGraph &g, vertex_id root) noexcept
		    : g_{&g}, root_{root} {}

		friend class NaiveDynamicGraph;
	};

	using component_view_type = ComponentView;

	explicit NaiveDynamicGraph(std::size_t n0 = 0) {
		reset(n0);
	}

	void reset(std::size_t n0) {
		adj_.assign(n0, {});
		alive_.assign(n0, static_cast<std::uint8_t>(1));
		free_.clear();
		alive_count_ = static_cast<vertex_id>(n0);
	}

	[[nodiscard]] std::size_t num_vertices() const noexcept {
		return alive_count_;
	}

	vertex_id add_vertex() {
		if (!free_.empty()) {
			vertex_id id = free_.back();
			free_.pop_back();
			alive_[id] = static_cast<std::uint8_t>(1);
			adj_[id].clear();
			++alive_count_;
			return id;
		}
		vertex_id id = static_cast<vertex_id>(adj_.size());
		adj_.emplace_back();
		alive_.emplace_back(static_cast<std::uint8_t>(1));
		++alive_count_;
		return id;
	}

	[[nodiscard]] bool erase_vertex(vertex_id v) {
		if (v >= adj_.size() || alive_[v] == 0)
			return false;

		std::vector<vertex_id> neigh;
		neigh.reserve(adj_[v].size());
		for (auto u : adj_[v])
			neigh.push_back(u);
		for (auto u : neigh)
			adj_[u].erase(v);

		adj_[v].clear();
		alive_[v] = static_cast<std::uint8_t>(0);
		free_.push_back(v);
		--alive_count_;
		return true;
	}

	[[nodiscard]] bool add_edge(vertex_id u, vertex_id v) {
		if (u == v)
			return false;
		if (!valid(u) || !valid(v))
			return false;
		if (adj_[u].contains(v))
			return false;
		adj_[u].insert(v);
		adj_[v].insert(u);
		return true;
	}

	[[nodiscard]] bool erase_edge(vertex_id u, vertex_id v) {
		if (u == v)
			return false;
		if (!valid(u) || !valid(v))
			return false;
		if (!adj_[u].contains(v))
			return false;
		adj_[u].erase(v);
		adj_[v].erase(u);
		return true;
	}

	[[nodiscard]] bool connected(vertex_id s, vertex_id t) const {
		if (s == t)
			return valid(s);
		if (!valid(s) || !valid(t))
			return false;

		std::deque<vertex_id> dq;
		std::queue<vertex_id, std::deque<vertex_id>> q(std::move(dq));
		std::vector<std::uint8_t> vis(adj_.size(),
		                              static_cast<std::uint8_t>(0));

		vis[s] = static_cast<std::uint8_t>(1);
		q.push(s);
		while (!q.empty()) {
			auto x = q.front();
			q.pop();
			for (auto y : adj_[x]) {
				if (alive_[y] != 0 && !vis[y]) {
					if (y == t)
						return true;
					vis[y] = static_cast<std::uint8_t>(1);
					q.push(y);
				}
			}
		}
		return false;
	}

	void clear() {
		for (auto &s : adj_)
			s.clear();
		std::fill(alive_.begin(), alive_.end(), static_cast<std::uint8_t>(0));
		free_.clear();
		alive_count_ = 0;
	}

	std::generator<std::pair<vertex_id, vertex_id>> path(vertex_id u,
	                                                     vertex_id v) const {
		if (!valid(u) || !valid(v))
			co_return;
		if (u == v)
			co_return; // empty path

		// BFS parent map to reconstruct a simple u->v path.
		const vertex_id NIL = std::numeric_limits<vertex_id>::max();
		std::vector<std::uint8_t> seen(adj_.size(),
		                               static_cast<std::uint8_t>(0));
		std::vector<vertex_id> parent(adj_.size(), NIL);
		std::deque<vertex_id> q;

		seen[u] = static_cast<std::uint8_t>(1);
		parent[u] = u;
		q.push_back(u);

		bool found = false;
		while (!q.empty()) {
			vertex_id x = q.front();
			q.pop_front();
			for (auto y : adj_[x]) {
				if (alive_[y] == 0 || seen[y])
					continue;
				seen[y] = static_cast<std::uint8_t>(1);
				parent[y] = x;
				if (y == v) {
					found = true;
					q.clear(); // terminate BFS early
					break;
				}
				q.push_back(y);
			}
		}
		if (!found)
			co_return;

		// Reconstruct vertex path [u = p0, p1, ..., pk = v]
		std::vector<vertex_id> path;
		for (vertex_id cur = v; cur != u; cur = parent[cur])
			path.push_back(cur);
		path.push_back(u);
		std::reverse(path.begin(), path.end());

		// Invoke visitor on each consecutive edge (p_i, p_{i+1})
		for (std::size_t i = 1; i < path.size(); ++i)
			co_yield {path[i - 1], path[i]};
	}

	[[nodiscard]] std::size_t component_size(vertex_id s) const {
		if (!valid(s))
			return 0;

		std::deque<vertex_id> dq;
		std::queue<vertex_id, std::deque<vertex_id>> q(std::move(dq));
		std::vector<std::uint8_t> vis(adj_.size(),
		                              static_cast<std::uint8_t>(0));

		vis[s] = static_cast<std::uint8_t>(1);
		q.push(s);
		std::size_t cnt = 1;
		while (!q.empty()) {
			auto x = q.front();
			q.pop();
			for (auto y : adj_[x]) {
				if (alive_[y] != 0 && !vis[y]) {
					vis[y] = static_cast<std::uint8_t>(1);
					++cnt;
					q.push(y);
				}
			}
		}
		return cnt;
	}

	std::generator<component_view_type> components() const {
		std::vector<std::uint8_t> vis(adj_.size(),
		                              static_cast<std::uint8_t>(0));

		for (vertex_id s = 0; s < static_cast<vertex_id>(adj_.size()); ++s) {
			if (!valid(s) || vis[s])
				continue;

			// Mark this component so we don't yield it again.
			std::deque<vertex_id> dq;
			std::queue<vertex_id, std::deque<vertex_id>> q(std::move(dq));

			vis[s] = static_cast<std::uint8_t>(1);
			q.push(s);
			while (!q.empty()) {
				auto x = q.front();
				q.pop();
				for (auto y : adj_[x]) {
					if (alive_[y] != 0 && !vis[y]) {
						vis[y] = static_cast<std::uint8_t>(1);
						q.push(y);
					}
				}
			}

			co_yield ComponentView{*this, s};
		}
	}

	[[nodiscard]] component_view_type component_view(vertex_id u) const {
		return ComponentView{*this, u};
	}

  private:
	[[nodiscard]] bool valid(vertex_id v) const noexcept {
		return v < adj_.size() && alive_[v] != 0;
	}

	std::vector<std::unordered_set<vertex_id>> adj_;
	std::vector<std::uint8_t> alive_; // 0 or 1
	std::vector<vertex_id> free_;
	vertex_id alive_count_{0};
};

static_assert(DynamicGraphLike<NaiveDynamicGraph>);

// --------- Link-Cut Forest (PMR, fixed-width ints, const queries) ---------

class LinkCutForest {
  public:
	explicit LinkCutForest(std::size_t n = 0) {
		reset(n);
	}

	void reset(std::size_t n) {
		n_ = static_cast<vertex_id>(n);
		ch_.assign(n_ + 1u, std::array<std::uint32_t, 2>{0u, 0u});
		p_.assign(n_ + 1u, 0u);
		rev_.assign(n_ + 1u, false);
		size_.assign(n_ + 1u, static_cast<std::int64_t>(1));
		vir_.assign(n_ + 1u, static_cast<std::int64_t>(0));
	}

	void ensure_capacity(std::size_t new_n) {
		if (new_n <= n_)
			return;
		n_ = static_cast<vertex_id>(new_n);
		ch_.resize(n_ + 1u, std::array<std::uint32_t, 2>{0u, 0u});
		p_.resize(n_ + 1u, 0u);
		rev_.resize(n_ + 1u, false);
		size_.resize(n_ + 1u, static_cast<std::int64_t>(1));
		vir_.resize(n_ + 1u, static_cast<std::int64_t>(0));
	}

	void reset_node(vertex_id x0) {
		const std::uint32_t x = to1(x0);
		if (x == 0u || x > n_)
			return;
		ch_[x][0] = ch_[x][1] = 0u;
		p_[x] = 0u;
		rev_[x] = false;
		size_[x] = static_cast<std::int64_t>(1);
		vir_[x] = static_cast<std::int64_t>(0);
	}

	void make_root(vertex_id x0) const {
		const std::uint32_t x = to1(x0);
		access(x);
		splay(x);
		apply_rev(x);
		push(x);
	}

	[[nodiscard]] bool connected(vertex_id u0, vertex_id v0) const {
		if (u0 == v0)
			return true;
		const std::uint32_t u = to1(u0), v = to1(v0);
		return find_root(u) == find_root(v);
	}

	void link(vertex_id u0, vertex_id v0) {
		if (connected(u0, v0))
			return; // guard against cycles
		const std::uint32_t u = to1(u0), v = to1(v0);
		make_root(u0);
		access(v);
		splay(v);
		p_[u] = v;
		vir_[v] += get_size(u);
		pull(v);
	}

	// robust cut: tries both orientations; only severs the (u,v) edge
	void cut(vertex_id u0, vertex_id v0) {
		if (!connected(u0, v0))
			return;
		const std::uint32_t u = to1(u0), v = to1(v0);

		// Try u as root
		make_root(u0);
		access(v);
		splay(v);
		push(v);
		push(u);
		if (ch_[v][0] == u && ch_[u][1] == 0u) { // direct adjacency
			ch_[v][0] = 0u;
			p_[u] = 0u;
			pull(v); // no vir_ tweak here
			return;
		}

		// Try v as root
		make_root(v0);
		access(u);
		splay(u);
		push(u);
		push(v);
		if (ch_[u][0] == v && ch_[v][1] == 0u) { // symmetric
			ch_[u][0] = 0u;
			p_[v] = 0u;
			pull(u);
			return;
		}
		// else: (u,v) was not a tree edge – nothing to cut
	}

	[[nodiscard]] std::size_t component_size(vertex_id u0) {
		const std::uint32_t u = to1(u0);
		make_root(u0);
		access(u);
		splay(u);
		return static_cast<std::size_t>(get_size(u));
	}

	std::generator<std::pair<vertex_id, vertex_id>> path(vertex_id u0,
	                                                     vertex_id v0) const {
		// Empty path => nothing to visit.
		if (u0 == v0)
			co_return;

		// Critical correctness check: no path if not connected.
		if (!connected(u0, v0))
			co_return;

		const std::uint32_t v = to1(v0);

		// Expose the unique path u0..v0 as the splay at v.
		make_root(u0);
		access(v);
		splay(v);

		// Iterative in-order traversal of the exposed path.
		std::vector<std::uint32_t> stack;
		std::uint32_t curr = v;
		std::uint32_t prev = 0u;

		while (curr || !stack.empty()) {
			// Go left as far as possible, pushing lazy flags top-down.
			while (curr) {
				push(curr); // handle lazy reversal at curr
				stack.push_back(curr);
				curr = ch_[curr][0]; // left child (after push)
			}

			curr = stack.back();
			stack.pop_back();

			// Visit node: emit edge from previous to current.
			if (prev)
				co_yield {prev - 1u, curr - 1u}; // back to 0-based ids
			prev = curr;

			// Then traverse right subtree.
			curr = ch_[curr][1];
		}
	}

	[[nodiscard]] std::size_t component_size(vertex_id u0) const {
		const std::uint32_t u = to1(u0);
		make_root(u0);
		access(u);
		splay(u);
		return static_cast<std::size_t>(get_size(u));
	}

  private:
	// Indexing: nodes are [1..n_], 0 is "null"
	vertex_id n_{0};

	// These members are mutated even in logically-const queries (splay/access),
	// so they are marked mutable for const-correctness of 'connected'.
	mutable std::vector<std::array<std::uint32_t, 2>> ch_; // children
	mutable std::vector<std::uint32_t> p_;                 // parent
	mutable std::vector<std::uint8_t> rev_;                // path-reversal flag
	mutable std::vector<std::int64_t> size_,
	    vir_; // subtree size; virtual size

	static bool
	is_root_of_aux(const std::vector<std::array<std::uint32_t, 2>> &ch,
	               const std::vector<std::uint32_t> &p, std::uint32_t x) {
		const std::uint32_t px = p[x];
		return px == 0u || (ch[px][0] != x && ch[px][1] != x);
	}

	[[nodiscard]] static constexpr std::uint32_t to1(vertex_id x0) noexcept {
		return x0 + 1u;
	}
	[[nodiscard]] std::int64_t get_size(std::uint32_t x) const noexcept {
		return x ? size_[x] : static_cast<std::int64_t>(0);
	}

	void pull(std::uint32_t x) const {
		size_[x] =
		    static_cast<std::int64_t>(1) +
		    (ch_[x][0] ? size_[ch_[x][0]] : static_cast<std::int64_t>(0)) +
		    (ch_[x][1] ? size_[ch_[x][1]] : static_cast<std::int64_t>(0)) +
		    vir_[x];
	}

	void apply_rev(std::uint32_t x) const {
		if (x) {
			rev_[x] = !rev_[x];
			std::swap(ch_[x][0], ch_[x][1]);
		}
	}

	void push(std::uint32_t x) const {
		if (!x || !rev_[x])
			return;
		if (ch_[x][0])
			apply_rev(ch_[x][0]);
		if (ch_[x][1])
			apply_rev(ch_[x][1]);
		rev_[x] = false;
	}

	void push_path(std::uint32_t x) const {
		std::vector<std::uint32_t> stk;
		for (std::uint32_t y = x;; y = p_[y]) {
			stk.push_back(y);
			if (is_root_of_aux(ch_, p_, y))
				break;
		}
		for (std::size_t i = stk.size(); i-- > 0;)
			push(stk[i]);
	}

	void rotate(std::uint32_t x) const {
		const std::uint32_t y = p_[x], z = p_[y];
		push(y);
		push(x);
		const std::uint32_t dx = (ch_[y][1] == x) ? 1u : 0u;
		const std::uint32_t b = ch_[x][dx ^ 1u];

		if (!is_root_of_aux(ch_, p_, y)) {
			if (ch_[z][0] == y)
				ch_[z][0] = x;
			else if (ch_[z][1] == y)
				ch_[z][1] = x;
		}
		p_[x] = z;
		ch_[x][dx ^ 1u] = y;
		p_[y] = x;
		ch_[y][dx] = b;
		if (b)
			p_[b] = y;
		pull(y);
		pull(x);
	}

	void splay(std::uint32_t x) const {
		push_path(x);
		while (!is_root_of_aux(ch_, p_, x)) {
			const std::uint32_t y = p_[x], z = p_[y];
			if (!is_root_of_aux(ch_, p_, y)) {
				const bool zigzig = (ch_[z][0] == y) == (ch_[y][0] == x);
				rotate(zigzig ? y : x);
			}
			rotate(x);
		}
	}

	void access(std::uint32_t x) const {
		std::uint32_t last = 0u;
		for (std::uint32_t y = x; y; y = p_[y]) {
			splay(y);
			if (ch_[y][1])
				vir_[y] += size_[ch_[y][1]];
			ch_[y][1] = last;
			if (last)
				vir_[y] -= size_[last];
			pull(y);
			last = y;
		}
		splay(x);
	}

	std::uint32_t find_root(std::uint32_t x) const {
		access(x);
		// top-down: push *before* reading ch_[x][0]
		while (true) {
			push(x);
			if (!ch_[x][0])
				break;
			x = ch_[x][0];
		}
		splay(x);
		return x;
	}

	// Push lazy 'rev_' flags to leaves on the whole splay subtree rooted at x.
	void push_all(std::uint32_t x) const {
		if (!x)
			return;
		// iterative stack to avoid recursion depth issues
		std::vector<std::uint32_t> st;
		st.push_back(x);
		// First, collect nodes in a stack for post-order push
		std::vector<std::uint32_t> post;
		while (!st.empty()) {
			auto y = st.back();
			st.pop_back();
			post.push_back(y);
			if (ch_[y][0])
				st.push_back(ch_[y][0]);
			if (ch_[y][1])
				st.push_back(ch_[y][1]);
		}
		// Then push from root to leaves
		for (std::size_t i = post.size(); i-- > 0;)
			push(post[i]);
	}
};

// --------- Holm–de Lichtenberg–Thorup connectivity (PMR, fixed-width, const connected) ---------

export class HolmDeLichtenbergThorup {
  public:
	class ComponentView {
	  public:
		[[nodiscard]] std::size_t size() const {
			return g_->component_size(root_);
		}

		[[nodiscard]] vertex_id root() const {
			return root_;
		}

		std::generator<vertex_id> vertices() const {
			if (!g_->valid(root_))
				co_return;

			// BFS on F0 (Spanning Forest).
			// Queue stores {current_node, parent_node} to avoid backtracking.
			// No visited set needed because F0 contains no cycles.
			std::deque<std::pair<vertex_id, vertex_id>> dq;
			dq.push_back({root_, std::numeric_limits<vertex_id>::max()});

			while (!dq.empty()) {
				auto [u, p] = dq.front();
				dq.pop_front();

				co_yield u;

				for (auto v : g_->treeAdj_[u]) {
					if (v != p) {
						dq.push_back({v, u});
					}
				}
			}
		}

		// Time complexity: O(|V|)
		std::generator<std::pair<vertex_id, vertex_id>> tree_edges() const {
			if (!g_->valid(root_))
				co_return;

			std::deque<std::pair<vertex_id, vertex_id>> dq;
			dq.push_back({root_, std::numeric_limits<vertex_id>::max()});

			while (!dq.empty()) {
				auto [u, p] = dq.front();
				dq.pop_front();

				for (auto v : g_->treeAdj_[u]) {
					if (v != p) {
						// u -> v is a tree edge directed away from root
						co_yield {u, v};
						dq.push_back({v, u});
					}
				}
			}
		}

		// Time complexity: O(|V| + |E|)
		std::generator<std::pair<vertex_id, vertex_id>> edges() const {
			if (!g_->valid(root_))
				co_return;

			// Use BFS on F0 to discover all nodes, then iterate ALL lists.
			std::deque<std::pair<vertex_id, vertex_id>> dq;
			dq.push_back({root_, std::numeric_limits<vertex_id>::max()});

			while (!dq.empty()) {
				auto [u, p] = dq.front();
				dq.pop_front();

				// 1. Process Tree Edges (F0)
				for (auto v : g_->treeAdj_[u]) {
					// Yield if u < v to ensure undirected edge yielded once
					if (u < v) {
						co_yield {u, v};
					}

					// BFS Traversal logic: continue if not parent
					if (v != p) {
						dq.push_back({v, u});
					}
				}

				// 2. Process Non-Tree Edges (All levels)
				// These do not affect traversal (F0 spans the component),
				// we just yield them.
				for (std::uint32_t lvl = 0; lvl <= g_->L_; ++lvl) {
					for (auto v : g_->nonTreeAdj_[lvl][u]) {
						if (u < v) {
							co_yield {u, v};
						}
					}
				}
			}
		}

	  private:
		const HolmDeLichtenbergThorup *g_;
		vertex_id root_;

		ComponentView(const HolmDeLichtenbergThorup &g, vertex_id root) noexcept
		    : g_{&g}, root_{root} {}

		friend class HolmDeLichtenbergThorup;
	};

	using component_view_type = ComponentView;

	explicit HolmDeLichtenbergThorup(std::size_t n0 = 0) {
		reset(n0);
	}

	void reset(std::size_t n0) {
		cap_ = std::max<std::size_t>(1, std::bit_ceil(n0 ? n0 : 1));
		L_ = static_cast<std::uint32_t>(std::bit_width(cap_ - 1));

		forests_.clear();
		forests_.reserve(L_ + 1u);
		for (std::uint32_t i = 0; i <= L_; ++i)
			forests_.emplace_back(LinkCutForest{cap_});

		nonTreeAdj_.clear();
		nonTreeAdj_.reserve(L_ + 1u);
		for (std::uint32_t i = 0; i <= L_; ++i) {
			nonTreeAdj_.emplace_back();
			nonTreeAdj_.back().resize(cap_);
		}

		treeAdj_.assign(cap_, {});
		edges_.clear();
		treeLevel_.clear();

		alive_.assign(cap_, static_cast<std::uint8_t>(0));
		free_.clear();
		alive_count_ = 0;
		next_new_id_ = 0;
		for (vertex_id v = 0; v < static_cast<vertex_id>(n0); ++v) {
			alive_[v] = static_cast<std::uint8_t>(1);
			++alive_count_;
			next_new_id_ =
			    std::max(next_new_id_, static_cast<vertex_id>(v + 1u));
		}

		vis_mark_.assign(cap_, 0u);
		bfs_epoch_ = 1u;
		side_mark_.assign(cap_, 0u);
		side_epoch_ = 1u;
		epoch_ = 1u;
	}

	[[nodiscard]] std::size_t num_vertices() const noexcept {
		return alive_count_;
	}

	vertex_id add_vertex() {
		vertex_id id;
		if (!free_.empty()) {
			id = free_.back();
			free_.pop_back();
		} else {
			id = next_new_id_++;
			if (id >= cap_)
				grow_capacity(std::bit_ceil<std::size_t>(
				    static_cast<std::size_t>(id) + 1u));
		}
		ensure_vertex_slot(id);
		alive_[id] = static_cast<std::uint8_t>(1);
		++alive_count_;
		treeAdj_[id].clear();
		for (std::uint32_t i = 0; i <= L_; ++i)
			nonTreeAdj_[i][id].clear();
		for (auto &f : forests_)
			f.reset_node(id);
		return id;
	}

	bool erase_vertex(vertex_id v) {
		if (!valid(v))
			return false;

		std::unordered_set<vertex_id> neigh;
		for (auto u : treeAdj_[v])
			neigh.insert(u);
		for (std::uint32_t i = 0; i <= L_; ++i)
			for (auto u : nonTreeAdj_[i][v])
				neigh.insert(u);

		std::vector<vertex_id> buf;
		buf.reserve(neigh.size());
		for (auto u : neigh)
			buf.push_back(u);
		for (auto u : buf)
			erase_edge(v, u);

		alive_[v] = static_cast<std::uint8_t>(0);
		--alive_count_;
		for (auto &s : nonTreeAdj_)
			s[v].clear();
		treeAdj_[v].clear();
		for (auto &f : forests_)
			f.reset_node(v);
		free_.push_back(v);
		return true;
	}

	bool add_edge(vertex_id u, vertex_id v) {
		if (u == v || !valid(u) || !valid(v))
			return false;
		EdgeKey k(u, v);
		if (edges_.contains(k))
			return false;

		if (!forests_[0].connected(u, v)) {
			forests_[0].link(u, v);
			treeAdj_[u].insert(v);
			treeAdj_[v].insert(u);
			edges_.emplace(k, EdgeRec{u, v, 0u, true, 0u});
			treeLevel_[k] = 0u;
		} else {
			nonTreeAdj_[0][u].insert(v);
			nonTreeAdj_[0][v].insert(u);
			edges_.emplace(k, EdgeRec{u, v, 0u, false, 0u});
		}
		return true;
	}

	bool erase_edge(vertex_id u, vertex_id v) {
		if (u == v || !valid(u) || !valid(v))
			return false;
		EdgeKey k(u, v);
		auto it = edges_.find(k);
		if (it == edges_.end())
			return false;
		EdgeRec rec = it->second;
		edges_.erase(it);

		if (!rec.is_tree) {
			auto lev = rec.level;
			nonTreeAdj_[lev][u].erase(v);
			nonTreeAdj_[lev][v].erase(u);
			return true;
		}

		// HLT §3.2: cut from all F_j, j <= level(e)
		for (std::uint32_t j = 0; j <= rec.level && j <= L_; ++j)
			forests_[j].cut(u, v);

		treeAdj_[u].erase(v);
		treeAdj_[v].erase(u);
		treeLevel_.erase(k);

		replace_after_cut(u, v, rec.level);
		return true;
	}

	[[nodiscard]] bool connected(vertex_id u, vertex_id v) const {
		if (u == v)
			return valid(u);
		if (!valid(u) || !valid(v))
			return false;
		return forests_[0].connected(u, v);
	}

	// Optional helper for tests: all non-tree edges satisfy invariant (i).
	bool _debug_invariant_i_holds() const {
		for (const auto &kv : edges_) {
			const EdgeRec &r = kv.second;
			if (!r.is_tree) {
				if (r.level > L_)
					return false;
				if (!forests_[r.level].connected(r.u, r.v))
					return false;
			}
		}
		return true;
	}

	void clear() {
		for (auto &f : forests_)
			f.reset(cap_);
		for (auto &lvl : nonTreeAdj_)
			for (auto &s : lvl)
				s.clear();
		for (auto &s : treeAdj_)
			s.clear();
		edges_.clear();
		treeLevel_.clear();
		std::fill(alive_.begin(), alive_.end(), static_cast<std::uint8_t>(0));
		free_.clear();
		alive_count_ = 0;
		next_new_id_ = 0;
		std::fill(vis_mark_.begin(), vis_mark_.end(), 0u);
		bfs_epoch_ = 1u;
		std::fill(side_mark_.begin(), side_mark_.end(), 0u);
		side_epoch_ = 1u;
		epoch_ = 1u;
	}

	std::generator<std::pair<vertex_id, vertex_id>> path(vertex_id u,
	                                                     vertex_id v) const {
		if (!valid(u) || !valid(v))
			co_return;
		co_yield std::ranges::elements_of(forests_[0].path(u, v));
	}

	[[nodiscard]] std::size_t component_size(vertex_id u) const {
		if (!valid(u))
			return 0;
		// F_0 is a spanning forest of G; its tree size is exactly CC size.
		return forests_[0].component_size(u);
	}

	std::generator<component_view_type> components() const {
		std::vector<std::uint8_t> vis(alive_.size(),
		                              static_cast<std::uint8_t>(0));

		for (vertex_id s = 0; s < static_cast<vertex_id>(alive_.size()); ++s) {
			if (!valid(s) || vis[s])
				continue;

			std::deque<vertex_id> dq;
			std::queue<vertex_id, std::deque<vertex_id>> q(std::move(dq));

			vis[s] = static_cast<std::uint8_t>(1);
			q.push(s);

			while (!q.empty()) {
				auto x = q.front();
				q.pop();

				auto push_neighbor = [&](vertex_id y) {
					if (!valid(y) || vis[y])
						return;
					vis[y] = static_cast<std::uint8_t>(1);
					q.push(y);
				};

				for (auto y : treeAdj_[x])
					push_neighbor(y);
				for (std::uint32_t lvl = 0; lvl <= L_; ++lvl)
					for (auto y : nonTreeAdj_[lvl][x])
						push_neighbor(y);
			}

			co_yield ComponentView{*this, s};
		}
	}

	[[nodiscard]] component_view_type component_view(vertex_id u) const {
		return ComponentView{*this, u};
	}

  private:
	using EdgeKey = UnorderedPair<vertex_id>;

	struct EdgeRec {
		vertex_id u, v;
		std::uint32_t level;
		bool is_tree;
		std::uint64_t seen_epoch;
	};

	std::size_t cap_{1};
	std::uint32_t L_{0};
	std::vector<LinkCutForest> forests_;
	std::vector<std::vector<std::unordered_set<vertex_id>>> nonTreeAdj_;
	std::vector<std::unordered_set<vertex_id>> treeAdj_;
	std::unordered_map<EdgeKey, EdgeRec> edges_;
	std::unordered_map<EdgeKey, std::uint32_t> treeLevel_;
	std::vector<std::uint8_t> alive_;
	std::vector<vertex_id> free_;
	vertex_id alive_count_{0}, next_new_id_{0};

	std::uint64_t epoch_{1};
	mutable std::uint64_t bfs_epoch_{1};
	mutable std::vector<std::uint64_t> vis_mark_;
	std::uint64_t side_epoch_{1};
	std::vector<std::uint64_t> side_mark_;

	[[nodiscard]] bool valid(vertex_id v) const noexcept {
		return v < alive_.size() && alive_[v] != 0;
	}

	void ensure_vertex_slot(vertex_id id) {
		if (id >= cap_)
			grow_capacity(
			    std::bit_ceil<std::size_t>(static_cast<std::size_t>(id) + 1u));
		for (auto &f : forests_)
			f.ensure_capacity(cap_);
		if (id >= treeAdj_.size())
			treeAdj_.resize(cap_);
		for (auto &lvl : nonTreeAdj_)
			if (id >= lvl.size())
				lvl.resize(cap_);
		if (id >= alive_.size())
			alive_.resize(cap_, static_cast<std::uint8_t>(0));
		if (id >= vis_mark_.size())
			vis_mark_.resize(cap_, 0u);
		if (id >= side_mark_.size())
			side_mark_.resize(cap_, 0u);
	}

	void grow_capacity(std::size_t new_cap) {
		if (new_cap <= cap_)
			return;
		cap_ = new_cap;
		const std::uint32_t newL =
		    static_cast<std::uint32_t>(std::bit_width(cap_ - 1));

		for (auto &f : forests_)
			f.ensure_capacity(cap_);
		if (newL > L_) {
			forests_.reserve(newL + 1u);
			for (std::uint32_t i = L_ + 1u; i <= newL; ++i)
				forests_.emplace_back(LinkCutForest{cap_});
		}

		for (auto &lvl : nonTreeAdj_)
			lvl.resize(cap_);
		if (newL > L_) {
			nonTreeAdj_.reserve(newL + 1u);
			for (std::uint32_t i = L_ + 1u; i <= newL; ++i) {
				nonTreeAdj_.emplace_back();
				nonTreeAdj_.back().resize(cap_);
			}
		}

		treeAdj_.resize(cap_);
		alive_.resize(cap_, static_cast<std::uint8_t>(0));
		vis_mark_.resize(cap_, 0u);
		side_mark_.resize(cap_, 0u);
		L_ = newL;
	}

	void enumerate_Fi_component(vertex_id seed, std::uint32_t i,
	                            std::vector<vertex_id> &out) const {
		out.clear();
		if (++bfs_epoch_ == std::numeric_limits<std::uint64_t>::max()) {
			std::fill(vis_mark_.begin(), vis_mark_.end(), 0u);
			bfs_epoch_ = 1u;
		}

		std::deque<vertex_id> dq;
		std::queue<vertex_id, std::deque<vertex_id>> q(std::move(dq));

		vis_mark_[seed] = bfs_epoch_;
		q.push(seed);
		while (!q.empty()) {
			auto x = q.front();
			q.pop();
			out.push_back(x);
			for (auto y : treeAdj_[x]) {
				if (!valid(y))
					continue;
				auto it = treeLevel_.find(EdgeKey(x, y));
				if (it == treeLevel_.end() || it->second < i)
					continue;
				if (vis_mark_[y] == bfs_epoch_)
					continue;
				vis_mark_[y] = bfs_epoch_;
				q.push(y);
			}
		}
	}

	void promote_non_tree(EdgeRec &rec) {
		if (rec.level >= L_)
			return;
		const auto u = rec.u, v = rec.v;
		nonTreeAdj_[rec.level][u].erase(v);
		nonTreeAdj_[rec.level][v].erase(u);
		++rec.level;
		nonTreeAdj_[rec.level][u].insert(v);
		nonTreeAdj_[rec.level][v].insert(u);
		edges_[EdgeKey(u, v)].level = rec.level;
	}

	void make_tree_from_non_tree(EdgeRec &rec) {
		const vertex_id u = rec.u, v = rec.v;
		if (forests_[0].connected(u, v)) {
			if (rec.level < L_)
				promote_non_tree(rec);
			return;
		}
		nonTreeAdj_[rec.level][u].erase(v);
		nonTreeAdj_[rec.level][v].erase(u);
		for (std::uint32_t j = 0; j <= rec.level; ++j)
			if (!forests_[j].connected(u, v))
				forests_[j].link(u, v);

		treeAdj_[u].insert(v);
		treeAdj_[v].insert(u);
		treeLevel_[EdgeKey(u, v)] = rec.level;
		rec.is_tree = true;
		edges_[EdgeKey(u, v)] = rec;
	}

	void
	promote_tree_edges_in_small_Fi_component(const std::vector<vertex_id> &side,
	                                         std::uint32_t i) {
		if (i >= L_)
			return;
		if (++side_epoch_ == std::numeric_limits<std::uint64_t>::max()) {
			std::fill(side_mark_.begin(), side_mark_.end(), 0u);
			side_epoch_ = 1u;
		}
		for (auto v : side)
			side_mark_[v] = side_epoch_;

		for (auto x : side) {
			for (auto y : treeAdj_[x]) {
				if (x >= y)
					continue;
				if (side_mark_[y] != side_epoch_)
					continue;
				EdgeKey ek(x, y);
				auto it = treeLevel_.find(ek);
				if (it == treeLevel_.end() || it->second != i)
					continue;
				const std::uint32_t to = i + 1u;
				if (!forests_[to].connected(x, y))
					forests_[to].link(x, y);
				it->second = to;
				auto eit = edges_.find(ek);
				if (eit != edges_.end() && eit->second.is_tree)
					eit->second.level = to;
			}
		}
	}

	void replace_after_cut(vertex_id u, vertex_id v, std::uint32_t up_to) {
		if (up_to > L_)
			up_to = L_;
		// Scan levels high->low per HLT §3; invariants may be temporarily violated
		// until a replacement is actually inserted. :contentReference[oaicite:1]{index=1}
		for (std::int32_t i = static_cast<std::int32_t>(up_to); i >= 0; --i) {
			const std::size_t su =
			    forests_[static_cast<std::uint32_t>(i)].component_size(u);
			const std::size_t sv =
			    forests_[static_cast<std::uint32_t>(i)].component_size(v);
			const vertex_id s = (su <= sv) ? u : v;

			std::vector<vertex_id> side;
			enumerate_Fi_component(s, static_cast<std::uint32_t>(i), side);

			// HLT Step 2: raise level-i tree edges on the small side to i+1
			promote_tree_edges_in_small_Fi_component(
			    side, static_cast<std::uint32_t>(i));

			// scan level-i non-tree incident edges; promote internals, use first crossing replacement
			if (++epoch_ == 0)
				++epoch_; // keep epoch non-zero
			for (vertex_id x : side) {
				std::vector<vertex_id> cand;
				cand.reserve(
				    nonTreeAdj_[static_cast<std::uint32_t>(i)][x].size());
				for (auto y : nonTreeAdj_[static_cast<std::uint32_t>(i)][x])
					cand.push_back(y);

				for (vertex_id y : cand) {
					EdgeKey kk(x, y);
					auto it = edges_.find(kk);
					if (it == edges_.end())
						continue;
					EdgeRec &rec = it->second;
					if (rec.seen_epoch == epoch_)
						continue;
					rec.seen_epoch = epoch_;

					const bool y_with_s =
					    forests_[static_cast<std::uint32_t>(i)].connected(y, s);

					if (!y_with_s) {
						// Crosses the current F_i-cut: must be the replacement at this level if any.
						// In a correct HLT Replace state, F_0 is split too (F_i ⊆ F_0), so F_0.connected() must be false.
						if (!forests_[0].connected(rec.u, rec.v)) {
							make_tree_from_non_tree(
							    rec); // links in all F_j, j<=i
							return;   // replacement found at level i
						} else {
							// Unreachable in a correct implementation; do NOT promote a crossing edge.
							std::unreachable();
						}
					} else {
						// Internal to the small side: safe to promote to i+1 (HLT Step 3).
						if (rec.level < L_)
							promote_non_tree(rec);
					}
				}
			}
			// no level-i replacement; continue to i-1
		}
		// split persists (no replacement exists)
	}
};

static_assert(DynamicGraphLike<HolmDeLichtenbergThorup>);

} // namespace bricksim
