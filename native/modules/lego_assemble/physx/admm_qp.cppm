export module lego_assemble.physx.admm_solver;

import std;
import lego_assemble.utils.matrix_serialization;
import lego_assemble.vendor;

namespace lego_assemble {

export struct AdmmQpOptions {
	// Penalties (can differ; think of it as scaling A and G differently)
	double rho_eq{1.0};
	double rho_ineq{1.0};

	// Equality slack (robust feasibility):
	// If eq_slack_weight > 0, solve:
	//   min_{x,s}  1/2 x^T Q x + 1/2 eq_slack_weight * ||s||^2
	//   s.t.       A x + s = b,   G x >= 0
	// This makes the equality constraint always feasible (s absorbs mismatch).
	// Set to 0 to recover the original hard equality A x = b formulation.
	double eq_slack_weight{0.0};

	// Numerical regularization added to M's diagonal
	double diag_reg{1e-12};

	// ADMM iteration controls
	int max_iter{200};
	double abs_tol{1e-6};
	double rel_tol{1e-4};

	// Over-relaxation (1.0 = off, typical 1.2~1.8 can help)
	double alpha{1.0};

	// Warm-start behavior
	// reuse (y,z,v) between solve() calls *if you reuse the same AdmmQpState*
	bool warm_start_duals{true};
};

export void to_json(nlohmann::ordered_json &j, const AdmmQpOptions &opt) {
	j = nlohmann::ordered_json{
	    {"rho_eq", opt.rho_eq},
	    {"rho_ineq", opt.rho_ineq},
	    {"eq_slack_weight", opt.eq_slack_weight},
	    {"diag_reg", opt.diag_reg},
	    {"max_iter", opt.max_iter},
	    {"abs_tol", opt.abs_tol},
	    {"rel_tol", opt.rel_tol},
	    {"alpha", opt.alpha},
	    {"warm_start_duals", opt.warm_start_duals},
	};
}

export struct AdmmQpInfo {
	bool converged{false};
	int iterations{0};
	double r_eq_norm{};
	double r_ineq_norm{};
	double s_norm{};
	double eps_eq{};
	double eps_ineq{};
	double eps_dual{};

	// ||s|| for equality slack (0 if disabled)
	double eq_slack_norm{};
};

export void to_json(nlohmann::ordered_json &j, const AdmmQpInfo &info) {
	j = nlohmann::ordered_json{
	    {"converged", info.converged},
	    {"iterations", info.iterations},
	    {"r_eq_norm", info.r_eq_norm},
	    {"r_ineq_norm", info.r_ineq_norm},
	    {"s_norm", info.s_norm},
	    {"eps_eq", info.eps_eq},
	    {"eps_ineq", info.eps_ineq},
	    {"eps_dual", info.eps_dual},
	    {"eq_slack_norm", info.eq_slack_norm},
	};
}

export struct AdmmQpState {
	// If true, (x,y,z) are valid warm-start candidates for the next solve.
	bool has_state{false};

	// Warm-start / last solution (persistent across frames)
	Eigen::VectorXd x; // n
	Eigen::VectorXd y; // me  (scaled dual for equality)
	Eigen::VectorXd z; // mi  (scaled dual for inequality)

	// Resize to match a solver's dimensions; clears warm-start if dims changed.
	void ensure_compatible(int n, int me, int mi) {
		if (x.size() != n || y.size() != me || z.size() != mi) {
			x.setZero(n);
			y.setZero(me);
			z.setZero(mi);
			has_state = false;
		}
	}

	// Explicitly clear warm-start (keep sizes if you want).
	void clear() {
		has_state = false;
		if (x.size())
			x.setZero();
		if (y.size())
			y.setZero();
		if (z.size())
			z.setZero();
	}
};

export void to_json(nlohmann::ordered_json &j, const AdmmQpState &state) {
	j = nlohmann::ordered_json{
	    {"has_state", state.has_state},
	    {"x", matrix_to_json(state.x)},
	    {"y", matrix_to_json(state.y)},
	    {"z", matrix_to_json(state.z)},
	};
}

export class AdmmQpSolver {
  public:
	AdmmQpSolver(Eigen::SparseMatrix<double> A, Eigen::SparseMatrix<double> Q,
	             Eigen::SparseMatrix<double> G, AdmmQpOptions opt = {})
	    : A_{std::move(A)}, Q_{std::move(Q)}, G_{std::move(G)}, opt_{opt} {
		if (Q_.rows() != Q_.cols()) {
			throw std::runtime_error("AdmmQpSolver: Q must be square");
		}
		if (A_.cols() != Q_.cols()) {
			throw std::runtime_error("AdmmQpSolver: A.cols != Q.cols");
		}
		if (G_.cols() != Q_.cols()) {
			throw std::runtime_error("AdmmQpSolver: G.cols != Q.cols");
		}

		A_.makeCompressed();
		Q_.makeCompressed();
		G_.makeCompressed();

		n_ = static_cast<int>(Q_.cols());
		me_ = static_cast<int>(A_.rows());
		mi_ = static_cast<int>(G_.rows());

		AT_ = A_.transpose();
		GT_ = G_.transpose();
		AT_.makeCompressed();
		GT_.makeCompressed();

		build_and_factorize_M();
	}

	AdmmQpSolver(AdmmQpSolver &&) = default;
	AdmmQpSolver &operator=(AdmmQpSolver &&) = default;
	AdmmQpSolver(const AdmmQpSolver &other) = delete;
	AdmmQpSolver &operator=(const AdmmQpSolver &other) = delete;

	int num_vars() const {
		return n_;
	}
	int num_eq() const {
		return me_;
	}
	int num_ineq() const {
		return mi_;
	}
	const AdmmQpOptions &options() const {
		return opt_;
	}
	const Eigen::SparseMatrix<double> &Q() const {
		return Q_;
	}
	const Eigen::SparseMatrix<double> &A() const {
		return A_;
	}
	const Eigen::SparseMatrix<double> &G() const {
		return G_;
	}

	const Eigen::VectorXd &solve(const Eigen::VectorXd &b, AdmmQpState &state,
	                             const Eigen::VectorXd *x_init = nullptr,
	                             AdmmQpInfo *info_out = nullptr) const {
		if (b.size() != me_) {
			throw std::runtime_error("AdmmQpSolver::solve: b has wrong size");
		}

		state.ensure_compatible(n_, me_, mi_);

		// Working variables (persisted in state only if you want warm-start)
		Eigen::VectorXd &x = state.x;
		Eigen::VectorXd &y = state.y;
		Eigen::VectorXd &z = state.z;

		// Init x
		if (x_init) {
			if (x_init->size() != n_) {
				throw std::runtime_error(
				    "AdmmQpSolver::solve: x_init has wrong size");
			}
			x = *x_init;
		} else if (!state.has_state) {
			x.setZero();
		}

		// Init duals
		const bool warm_duals = opt_.warm_start_duals && state.has_state;
		if (!warm_duals) {
			y.setZero();
			z.setZero();
		}

		// ---- locals (allocate once per solve; you said this cost is fine) ----
		Eigen::VectorXd ATb(n_), rhs(n_), ATy(n_);
		Eigen::VectorXd Ax(me_), Ax_hat(me_);
		Eigen::VectorXd s_eq(me_), u_eq(me_);
		Eigen::VectorXd Gx(mi_), Gx_hat(mi_);
		Eigen::VectorXd v(mi_), v_prev(mi_);
		Eigen::VectorXd tmp_ineq(mi_), svec(n_);

		if (me_ > 0)
			ATb.noalias() = AT_ * b;

		// Init v consistent with (x,z)
		if (mi_ > 0) {
			Gx.noalias() = G_ * x;
			if (warm_duals)
				v = (Gx + z).cwiseMax(0.0);
			else
				v = Gx.cwiseMax(0.0);
		}

		const double rho_eq_x = rho_eq_x_; // effective coefficient for x-update
		const double rho_ineq = opt_.rho_ineq;
		const double alpha = opt_.alpha;

		AdmmQpInfo info;

		for (int it = 0; it < opt_.max_iter; ++it) {
			info.iterations = it + 1;
			if (mi_ > 0)
				v_prev = v;

			// x-update
			rhs.setZero();
			if (me_ > 0) {
				ATy.noalias() = AT_ * y;
				rhs.noalias() += rho_eq_x * (ATb - ATy);
			}
			if (mi_ > 0) {
				tmp_ineq.noalias() = v - z;
				rhs.noalias() += rho_ineq * (GT_ * tmp_ineq);
			}

			x = M_solver_->solve(rhs);
			if (M_solver_->info() != Eigen::Success) {
				throw std::runtime_error(
				    "AdmmQpSolver: M_solver_.solve failed");
			}

			// Ax, Gx
			if (me_ > 0)
				Ax.noalias() = A_ * x;
			if (mi_ > 0)
				Gx.noalias() = G_ * x;

			// Equality slack + over-relaxation on u = Ax + s
			if (me_ > 0) {
				if (use_eq_slack_) {
					// s = (rho_eq / (lambda + rho_eq)) * (b - y - Ax)
					u_eq.noalias() = b - y - Ax;         // temp = (b - y - Ax)
					s_eq.noalias() = k_eq_slack_ * u_eq; // s
					u_eq.noalias() = Ax + s_eq;          // u = Ax + s
				} else {
					s_eq.setZero();
					u_eq = Ax;
				}
				Ax_hat = alpha * u_eq + (1.0 - alpha) * b; // u_hat
			}

			// over-relaxation
			if (mi_ > 0)
				Gx_hat = alpha * Gx + (1.0 - alpha) * v_prev;

			// v-update + z-update
			if (mi_ > 0) {
				tmp_ineq.noalias() = Gx_hat + z; // t
				v = tmp_ineq.cwiseMax(0.0);
				z += (Gx_hat - v);
			}

			// y-update
			if (me_ > 0)
				y += (Ax_hat - b); // (u_hat - b)

			// convergence
			const double r_eq_norm =
			    (me_ > 0) ? ((use_eq_slack_ ? (u_eq - b) : (Ax - b)).norm())
			              : 0.0;
			const double r_ineq_norm = (mi_ > 0) ? (Gx - v).norm() : 0.0;

			double s_norm = 0.0;
			if (mi_ > 0) {
				tmp_ineq.noalias() = v - v_prev;
				svec.noalias() = (rho_ineq * (GT_ * tmp_ineq));
				s_norm = svec.norm();
			}

			double eps_eq = 0.0;
			if (me_ > 0) {
				const double u_norm = use_eq_slack_ ? u_eq.norm() : Ax.norm();
				eps_eq = opt_.abs_tol * std::sqrt(double(me_)) +
				         opt_.rel_tol * std::max(u_norm, b.norm());
			}
			double eps_ineq = 0.0;
			if (mi_ > 0) {
				eps_ineq = opt_.abs_tol * std::sqrt(double(mi_)) +
				           opt_.rel_tol * std::max(Gx.norm(), v.norm());
			}

			double eps_dual = opt_.abs_tol * std::sqrt(double(n_));
			{
				double scale = 0.0;
				if (me_ > 0)
					scale = std::max(scale, (rho_eq_x * ATy).norm());
				if (mi_ > 0)
					scale = std::max(scale, (rho_ineq * (GT_ * z)).norm());
				eps_dual += opt_.rel_tol * scale;
			}

			info.r_eq_norm = r_eq_norm;
			info.r_ineq_norm = r_ineq_norm;
			info.s_norm = s_norm;
			info.eps_eq = eps_eq;
			info.eps_ineq = eps_ineq;
			info.eps_dual = eps_dual;
			info.eq_slack_norm = (use_eq_slack_ ? s_eq.norm() : 0.0);

			const bool eq_ok = (me_ == 0) || (r_eq_norm <= eps_eq);
			const bool ineq_ok = (mi_ == 0) || (r_ineq_norm <= eps_ineq);
			const bool dual_ok = (mi_ == 0) || (s_norm <= eps_dual);

			if (eq_ok && ineq_ok && dual_ok) {
				info.converged = true;
				break;
			}
		}

		if (info_out) {
			*info_out = info;
		}

		state.has_state = true;
		return state.x;
	}

  private:
	void build_and_factorize_M() {
		// Precompute equality-slack scalars (constant for solver lifetime).
		use_eq_slack_ =
		    (me_ > 0) && (opt_.rho_eq > 0.0) && (opt_.eq_slack_weight > 0.0);
		if (use_eq_slack_) {
			const double rho = opt_.rho_eq;
			const double lam = opt_.eq_slack_weight;
			k_eq_slack_ = rho / (lam + rho);     // for s-update
			rho_eq_x_ = rho * lam / (lam + rho); // for x-update and M
		} else {
			k_eq_slack_ = 0.0;
			rho_eq_x_ = opt_.rho_eq;
		}

		// M = Q + rho_eq A^T A + rho_ineq G^T G + diag_reg I
		Eigen::SparseMatrix<double> M = Q_;

		if (me_ > 0 && rho_eq_x_ != 0.0) {
			Eigen::SparseMatrix<double> AtA = AT_ * A_;
			AtA.makeCompressed();
			M += rho_eq_x_ * AtA;
		}
		if (mi_ > 0 && opt_.rho_ineq != 0.0) {
			Eigen::SparseMatrix<double> GtG = GT_ * G_;
			GtG.makeCompressed();
			M += opt_.rho_ineq * GtG;
		}

		if (opt_.diag_reg > 0.0) {
			Eigen::SparseMatrix<double> I(n_, n_);
			I.setIdentity();
			M += opt_.diag_reg * I;
		}

		M.makeCompressed();

		M_solver_ = std::make_unique<
		    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>>>();
		M_solver_->compute(M);
		if (M_solver_->info() != Eigen::Success) {
			throw std::runtime_error("AdmmQpSolver: factorization of M failed "
			                         "(try increasing diag_reg or rho)");
		}
	}

	// Input matrices (immutable after construction)
	Eigen::SparseMatrix<double> A_;
	Eigen::SparseMatrix<double> Q_;
	Eigen::SparseMatrix<double> G_;
	Eigen::SparseMatrix<double> AT_;
	Eigen::SparseMatrix<double> GT_;

	AdmmQpOptions opt_{};
	int n_ = 0;
	int me_ = 0;
	int mi_ = 0;

	// Equality slack bookkeeping
	bool use_eq_slack_{false};
	double rho_eq_x_{0.0};   // effective rho used in x-update / factorization
	double k_eq_slack_{0.0}; // rho_eq / (eq_slack_weight + rho_eq)

	// SPD factorization for x-update (immutable after compute())
	std::unique_ptr<Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>>>
	    M_solver_;
};

} // namespace lego_assemble
