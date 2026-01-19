export module lego_assemble.physx.osqp;

import std;
import lego_assemble.physx.qdldl;
import lego_assemble.utils.matrix_serialization;
import lego_assemble.utils.logging;
import lego_assemble.vendor;

namespace lego_assemble {

using Eigen::MatrixXd;
using Eigen::SparseMatrix;
using Eigen::Triplet;
using Eigen::VectorXd;

export struct OsqpOptions {
	// --- Problem regularization / penalty knobs
	// weight on slack (equality) variables
	double lambda_eq{1.0e6};
	// small diagonal added to Q for SPD
	double diag_reg{1.0e-6};

	// --- ADMM parameters
	double rho_eq{1.0e6};
	double rho_ineq{1.0e-3};
	double sigma{1.0e-6};
	double alpha{1.6};

	// --- Termination
	int max_iter{2000};
	double abs_tol{1.0e-6};
	double rel_tol{1.0e-4};

	// --- Scaling (Ruiz equilibration)
	bool enable_row_scaling{true};
	int ruiz_iterations{10};
	double ruiz_eps{1.0e-12};
	double ruiz_scale_min{1.0e-3};
	double ruiz_scale_max{1.0e3};

	// --- Adaptive rho (OSQP-style residual balancing)
	bool enable_adaptive_rho{true};
	int adaptive_rho_interval{25};
	double adaptive_rho_tolerance{5.0};
	double adaptive_rho_min{1.0e-6};
	double adaptive_rho_max{1.0e6};

	// --- Polishing ("2nd step")
	bool enable_polish{true};
	// attempt polish after this many ADMM its (if not converged)
	int polish_trigger_iter{50};
	// active-set refinement iterations
	int polish_refine_max_iter{20};
	// regularization for quasi-definite KKT
	double polish_delta{1.0e-10};
	// violation / activity tolerance
	double polish_viol_tol{1.0e-6};

	int check_every{10};
};

export void to_json(nlohmann::ordered_json &j, const OsqpOptions &opts) {
	j = nlohmann::ordered_json{
	    {"lambda_eq", opts.lambda_eq},
	    {"diag_reg", opts.diag_reg},
	    {"rho_eq", opts.rho_eq},
	    {"rho_ineq", opts.rho_ineq},
	    {"sigma", opts.sigma},
	    {"alpha", opts.alpha},
	    {"max_iter", opts.max_iter},
	    {"abs_tol", opts.abs_tol},
	    {"rel_tol", opts.rel_tol},
	    {"enable_row_scaling", opts.enable_row_scaling},
	    {"ruiz_iterations", opts.ruiz_iterations},
	    {"ruiz_eps", opts.ruiz_eps},
	    {"ruiz_scale_min", opts.ruiz_scale_min},
	    {"ruiz_scale_max", opts.ruiz_scale_max},
	    {"enable_adaptive_rho", opts.enable_adaptive_rho},
	    {"adaptive_rho_interval", opts.adaptive_rho_interval},
	    {"adaptive_rho_tolerance", opts.adaptive_rho_tolerance},
	    {"adaptive_rho_min", opts.adaptive_rho_min},
	    {"adaptive_rho_max", opts.adaptive_rho_max},
	    {"enable_polish", opts.enable_polish},
	    {"polish_trigger_iter", opts.polish_trigger_iter},
	    {"polish_refine_max_iter", opts.polish_refine_max_iter},
	    {"polish_delta", opts.polish_delta},
	    {"polish_viol_tol", opts.polish_viol_tol},
	    {"check_every", opts.check_every},
	};
}

export void from_json(const nlohmann::ordered_json &j, OsqpOptions &opts) {
	j.at("lambda_eq").get_to(opts.lambda_eq);
	j.at("diag_reg").get_to(opts.diag_reg);
	j.at("rho_eq").get_to(opts.rho_eq);
	j.at("rho_ineq").get_to(opts.rho_ineq);
	j.at("sigma").get_to(opts.sigma);
	j.at("alpha").get_to(opts.alpha);
	j.at("max_iter").get_to(opts.max_iter);
	j.at("abs_tol").get_to(opts.abs_tol);
	j.at("rel_tol").get_to(opts.rel_tol);
	j.at("enable_row_scaling").get_to(opts.enable_row_scaling);
	j.at("ruiz_iterations").get_to(opts.ruiz_iterations);
	j.at("ruiz_eps").get_to(opts.ruiz_eps);
	j.at("ruiz_scale_min").get_to(opts.ruiz_scale_min);
	j.at("ruiz_scale_max").get_to(opts.ruiz_scale_max);
	j.at("enable_adaptive_rho").get_to(opts.enable_adaptive_rho);
	j.at("adaptive_rho_interval").get_to(opts.adaptive_rho_interval);
	j.at("adaptive_rho_tolerance").get_to(opts.adaptive_rho_tolerance);
	j.at("adaptive_rho_min").get_to(opts.adaptive_rho_min);
	j.at("adaptive_rho_max").get_to(opts.adaptive_rho_max);
	j.at("enable_polish").get_to(opts.enable_polish);
	j.at("polish_trigger_iter").get_to(opts.polish_trigger_iter);
	j.at("polish_refine_max_iter").get_to(opts.polish_refine_max_iter);
	j.at("polish_delta").get_to(opts.polish_delta);
	j.at("polish_viol_tol").get_to(opts.polish_viol_tol);
	j.at("check_every").get_to(opts.check_every);
}

export struct OsqpState {
	bool has_state{false};
	VectorXd v{};
	VectorXd z{};
	VectorXd u{};
	VectorXd x{};

	double rho_eq_{0.0};
	double rho_ineq_{0.0};
	Eigen::VectorXd rho_vec_{};
	SparseMatrix<double> KKT_upper_{}; // (n+m) x (n+m), upper triangle CSC
	QdldlSolver kkt_solver_{};

	void clear() {
		*this = {};
	}
};

export void to_json(nlohmann::ordered_json &j, const OsqpState &state) {
	j = nlohmann::ordered_json{
	    {"has_state", state.has_state},
	    {"v", matrix_to_json(state.v)},
	    {"z", matrix_to_json(state.z)},
	    {"u", matrix_to_json(state.u)},
	    {"x", matrix_to_json(state.x)},
	    {"rho_eq", state.rho_eq_},
	    {"rho_ineq", state.rho_ineq_},
	    {"rho_vec", matrix_to_json(state.rho_vec_)},
	    {"KKT_upper", matrix_to_json(state.KKT_upper_)},
	};
}

export void from_json(const nlohmann::ordered_json &j, OsqpState &state) {
	j.at("has_state").get_to(state.has_state);
	state.v = json_to_matrix<VectorXd>(j.at("v"));
	state.z = json_to_matrix<VectorXd>(j.at("z"));
	state.u = json_to_matrix<VectorXd>(j.at("u"));
	state.x = json_to_matrix<VectorXd>(j.at("x"));
	j.at("rho_eq").get_to(state.rho_eq_);
	j.at("rho_ineq").get_to(state.rho_ineq_);
	state.rho_vec_ = json_to_matrix<VectorXd>(j.at("rho_vec"));
	state.KKT_upper_ = json_to_matrix<SparseMatrix<double>>(j.at("KKT_upper"));
	state.kkt_solver_.factorize(state.KKT_upper_);
}

export struct OsqpInfo {
	bool converged{false};
	int iter{};
	double r_eq_norm{};
	double r_ineq_norm{};
	double r_kkt_norm{};
	double eps_eq{};
	double eps_ineq{};
	double eps_kkt{};
};

export void to_json(nlohmann::ordered_json &j, const OsqpInfo &info) {
	j = nlohmann::ordered_json{
	    {"converged", info.converged},   {"iter", info.iter},
	    {"r_eq_norm", info.r_eq_norm},   {"r_ineq_norm", info.r_ineq_norm},
	    {"r_kkt_norm", info.r_kkt_norm}, {"eps_eq", info.eps_eq},
	    {"eps_ineq", info.eps_ineq},     {"eps_kkt", info.eps_kkt},
	};
}

export void from_json(const nlohmann::ordered_json &j, OsqpInfo &info) {
	j.at("converged").get_to(info.converged);
	j.at("iter").get_to(info.iter);
	j.at("r_eq_norm").get_to(info.r_eq_norm);
	j.at("r_ineq_norm").get_to(info.r_ineq_norm);
	j.at("r_kkt_norm").get_to(info.r_kkt_norm);
	j.at("eps_eq").get_to(info.eps_eq);
	j.at("eps_ineq").get_to(info.eps_ineq);
	j.at("eps_kkt").get_to(info.eps_kkt);
}

export class OsqpSolver {
  public:
	explicit OsqpSolver(SparseMatrix<double> A, SparseMatrix<double> Q,
	                    SparseMatrix<double> G, const OsqpOptions &options = {})
	    : A0_(std::move(A)), Q0_(std::move(Q)), G0_(std::move(G)),
	      options_(options) {
		n_x_ = static_cast<int>(Q0_.rows());
		me_ = static_cast<int>(A0_.rows());
		mi_ = static_cast<int>(G0_.rows());
		n_ = n_x_ + me_;
		m_ = me_ + mi_;
		Q_reg_ = Q0_;
		if (options_.diag_reg > 0.0) {
			SparseMatrix<double> I(n_x_, n_x_);
			I.setIdentity();
			Q_reg_ += options_.diag_reg * I;
		}

		build_P_();
		build_C_();
		apply_ruiz_scaling_();
	}

	const SparseMatrix<double> &A() const {
		return A0_;
	}
	const SparseMatrix<double> &Q() const {
		return Q0_;
	}
	const SparseMatrix<double> &G() const {
		return G0_;
	}
	const OsqpOptions &options() const {
		return options_;
	}

	OsqpInfo solve(const VectorXd &b, OsqpState &state) const {
		if (b.size() != me_) {
			throw std::invalid_argument("b dimension mismatch");
		}
		if (state.has_state) {
			if (state.v.size() != n_ || state.z.size() != m_ ||
			    state.u.size() != m_) {
				throw std::invalid_argument("state dimension mismatch");
			}
		} else {
			state.has_state = true;
			state.v.setZero(n_);
			state.z.setZero(m_);
			state.u.setZero(m_);
			state.rho_eq_ = options_.rho_eq;
			state.rho_ineq_ = options_.rho_ineq;
			build_kkt_(state);
		}

		// Build scaled bounds l,u for z = C_scaled * v
		VectorXd l = VectorXd::Zero(m_);
		VectorXd u = VectorXd::Zero(m_);
		l.head(me_) = b;
		u.head(me_) = b;
		if (mi_ > 0) {
			l.tail(mi_).setZero();
			u.tail(mi_).setConstant(std::numeric_limits<double>::infinity());
		}

		// Apply row scaling (Ruiz) to bounds: l_s = E .* l, u_s = E .* u
		VectorXd l_s = row_scale_.cwiseProduct(l);
		VectorXd u_s = row_scale_.cwiseProduct(u);

		// Initialize z to projection of C*v
		VectorXd Cv_init = C_scaled_ * state.v;
		state.z = Cv_init.cwiseMax(l_s).cwiseMin(u_s);

		OsqpInfo info;

		VectorXd rhs(n_ + m_);
		VectorXd sol(n_ + m_);

		bool polished_once = false;

		for (int it = 1; it <= options_.max_iter; ++it) {
			// x-update: solve KKT
			rhs.head(n_) = options_.sigma * state.v;
			rhs.tail(m_) = state.z - state.u;

			sol = rhs;
			if (!state.kkt_solver_.solve_in_place(sol)) {
				info.converged = false;
				info.iter = it;
				return info;
			}

			state.v = sol.head(n_);

			// Compute C*v
			VectorXd Cv = C_scaled_ * state.v;

			// Over-relax
			VectorXd Cv_hat =
			    options_.alpha * Cv + (1.0 - options_.alpha) * state.z;

			// z-update: projection onto [l_s, u_s]
			VectorXd z_tilde = Cv_hat + state.u;
			state.z = z_tilde.cwiseMax(l_s).cwiseMin(u_s);

			// u-update
			state.u += (Cv_hat - state.z);

			// Convergence / diagnostics
			if (it % options_.check_every == 0) {
				compute_info_(b, state, info);

				if (info.r_ineq_norm <= info.eps_ineq &&
				    info.r_kkt_norm <= info.eps_kkt) {
					info.converged = true;
					info.iter = it;
					return info;
				}

				// Polishing "2nd step" to reduce iterations (try once after trigger, then
				// opportunistically on later checks if it keeps failing).
				if (options_.enable_polish &&
				    it >= options_.polish_trigger_iter) {
					bool did_polish = polish_(l_s.head(me_), state);
					polished_once = polished_once || did_polish;
					if (did_polish) {
						compute_info_(b, state, info);
						if (info.r_ineq_norm <= info.eps_ineq &&
						    info.r_kkt_norm <= info.eps_kkt) {
							info.converged = true;
							info.iter = it;
							return info;
						}
					}
				}
			}

			// Adaptive rho (residual balancing) -- update KKT factorization when rho changes.
			if (options_.enable_adaptive_rho &&
			    (it % options_.adaptive_rho_interval == 0)) {
				if (adaptive_rho_update_(state)) {
					build_kkt_(state);
				}
			}
		}

		compute_info_(b, state, info);
		info.converged = (info.r_ineq_norm <= info.eps_ineq &&
		                  info.r_kkt_norm <= info.eps_kkt);
		info.iter = options_.max_iter;
		return info;
	}

  private:
	// Problem data (original)
	SparseMatrix<double> A0_;
	SparseMatrix<double> Q0_;
	SparseMatrix<double> G0_;

	// Regularized Q
	SparseMatrix<double> Q_reg_;

	// Slack-form matrices (scaled)
	SparseMatrix<double> P_;        // (n x n) full symmetric stored
	SparseMatrix<double> C_scaled_; // (m x n)

	// Scaling maps
	VectorXd row_scale_; // (m)
	VectorXd col_scale_; // (n)
	double cost_scale_ = 1.0;

	// Sizes
	int n_x_ = 0;
	int me_ = 0;
	int mi_ = 0;
	int n_ = 0;
	int m_ = 0;

	OsqpOptions options_;

	// -------------------- Build helpers --------------------
	void build_P_() {
		// P = blkdiag(Q_reg, lambda_eq * I)
		std::vector<Triplet<double>> triplets;
		triplets.reserve(Q_reg_.nonZeros() + me_);

		// Copy Q_reg (full symmetric as provided)
		for (int k = 0; k < Q_reg_.outerSize(); ++k) {
			for (SparseMatrix<double>::InnerIterator it(Q_reg_, k); it; ++it) {
				triplets.emplace_back(it.row(), it.col(), it.value());
			}
		}

		// Slack block
		for (int i = 0; i < me_; ++i) {
			triplets.emplace_back(n_x_ + i, n_x_ + i, options_.lambda_eq);
		}

		P_.resize(n_, n_);
		P_.setFromTriplets(triplets.begin(), triplets.end());
		P_.makeCompressed();
	}

	void build_C_() {
		// C = [ A  I ]
		//     [ G  0 ]
		std::vector<Triplet<double>> triplets;
		triplets.reserve(A0_.nonZeros() + G0_.nonZeros() + me_);

		// A block
		for (int k = 0; k < A0_.outerSize(); ++k) {
			for (SparseMatrix<double>::InnerIterator it(A0_, k); it; ++it) {
				triplets.emplace_back(it.row(), it.col(), it.value());
			}
		}

		// I block for slack
		for (int i = 0; i < me_; ++i) {
			triplets.emplace_back(i, n_x_ + i, 1.0);
		}

		// G block
		for (int k = 0; k < G0_.outerSize(); ++k) {
			for (SparseMatrix<double>::InnerIterator it(G0_, k); it; ++it) {
				triplets.emplace_back(me_ + it.row(), it.col(), it.value());
			}
		}

		C_scaled_.resize(m_, n_);
		C_scaled_.setFromTriplets(triplets.begin(), triplets.end());
		C_scaled_.makeCompressed();

		// Initialize scaling to identity
		row_scale_ = VectorXd::Ones(m_);
		col_scale_ = VectorXd::Ones(n_);
		cost_scale_ = 1.0;
	}

	void apply_ruiz_scaling_() {
		if (!options_.enable_row_scaling || options_.ruiz_iterations <= 0) {
			return;
		}

		const double eps = std::max(options_.ruiz_eps, 1e-18);
		const double smin = std::max(options_.ruiz_scale_min, 1e-18);
		const double smax = std::max(options_.ruiz_scale_max, smin);

		VectorXd col_norm = VectorXd::Zero(n_);
		VectorXd row_norm = VectorXd::Zero(m_);

		for (int it = 0; it < options_.ruiz_iterations; ++it) {
			col_norm.setZero();

			// col norms of P_
			for (int j = 0; j < P_.outerSize(); ++j) {
				for (SparseMatrix<double>::InnerIterator t(P_, j); t; ++t) {
					col_norm[j] = std::max(col_norm[j], std::abs(t.value()));
				}
			}

			// col norms of C_scaled_
			for (int j = 0; j < C_scaled_.outerSize(); ++j) {
				double max_abs = 0.0;
				for (SparseMatrix<double>::InnerIterator t(C_scaled_, j); t;
				     ++t) {
					max_abs = std::max(max_abs, std::abs(t.value()));
				}
				col_norm[j] = std::max(col_norm[j], max_abs);
			}

			// d = 1/sqrt(col_norm)
			VectorXd d = (col_norm.array().max(eps)).sqrt().inverse().matrix();
			for (int j = 0; j < n_; ++j) {
				d[j] = std::clamp(d[j], smin, smax);
			}

			// Apply variable scaling to P_: P <- D * P * D
			for (int j = 0; j < P_.outerSize(); ++j) {
				for (SparseMatrix<double>::InnerIterator t(P_, j); t; ++t) {
					t.valueRef() *= d[t.row()] * d[t.col()];
				}
			}

			// Apply column scaling to C_scaled_: C <- C * D
			for (int j = 0; j < C_scaled_.outerSize(); ++j) {
				for (SparseMatrix<double>::InnerIterator t(C_scaled_, j); t;
				     ++t) {
					t.valueRef() *= d[j];
				}
			}

			col_scale_ = col_scale_.cwiseProduct(d);

			// Row scaling based on row norms of C
			row_norm.setZero();
			for (int j = 0; j < C_scaled_.outerSize(); ++j) {
				for (SparseMatrix<double>::InnerIterator t(C_scaled_, j); t;
				     ++t) {
					int r = t.row();
					row_norm[r] = std::max(row_norm[r], std::abs(t.value()));
				}
			}

			VectorXd e = (row_norm.array().max(eps)).sqrt().inverse().matrix();
			for (int r = 0; r < m_; ++r) {
				e[r] = std::clamp(e[r], smin, smax);
			}

			// Apply row scaling: C <- E * C
			for (int j = 0; j < C_scaled_.outerSize(); ++j) {
				for (SparseMatrix<double>::InnerIterator t(C_scaled_, j); t;
				     ++t) {
					t.valueRef() *= e[t.row()];
				}
			}

			row_scale_ = row_scale_.cwiseProduct(e);
		}

		// Cost scaling: scale P so max abs entry is <= 1
		double p_inf = 0.0;
		for (int j = 0; j < P_.outerSize(); ++j) {
			for (SparseMatrix<double>::InnerIterator t(P_, j); t; ++t) {
				p_inf = std::max(p_inf, std::abs(t.value()));
			}
		}

		if (p_inf > 1.0) {
			double c = 1.0 / p_inf;
			cost_scale_ *= c;
			for (int j = 0; j < P_.outerSize(); ++j) {
				for (SparseMatrix<double>::InnerIterator t(P_, j); t; ++t) {
					t.valueRef() *= c;
				}
			}
		}

		P_.makeCompressed();
		C_scaled_.makeCompressed();
	}

	void build_kkt_(OsqpState &state) const {
		// Build symmetric KKT matrix (upper triangle only):
		// [ P + sigma I, C^T ]
		// [ C,          -R^{-1} ]
		// where R = diag(rho_vec)
		std::vector<Triplet<double>> triplets;
		triplets.reserve(P_.nonZeros() + C_scaled_.nonZeros() + (n_ + m_));

		// Upper triangle of (P + sigma I)
		for (int j = 0; j < P_.outerSize(); ++j) {
			for (SparseMatrix<double>::InnerIterator it(P_, j); it; ++it) {
				if (it.row() <= it.col()) {
					triplets.emplace_back(it.row(), it.col(), it.value());
				}
			}
		}
		for (int i = 0; i < n_; ++i) {
			triplets.emplace_back(i, i, options_.sigma);
		}

		// Top-right block: C^T
		for (int j = 0; j < C_scaled_.outerSize(); ++j) {
			for (SparseMatrix<double>::InnerIterator it(C_scaled_, j); it;
			     ++it) {
				int r = it.row();
				int c = it.col();
				// place at (c, n_ + r)
				triplets.emplace_back(c, n_ + r, it.value());
			}
		}

		// Bottom-right diagonal: -1/rho
		state.rho_vec_ = VectorXd::Zero(m_);
		state.rho_vec_.head(me_).setConstant(state.rho_eq_);
		if (mi_ > 0) {
			state.rho_vec_.tail(mi_).setConstant(state.rho_ineq_);
		}
		for (int i = 0; i < m_; ++i) {
			triplets.emplace_back(n_ + i, n_ + i, -1.0 / state.rho_vec_[i]);
		}

		state.KKT_upper_.resize(n_ + m_, n_ + m_);
		state.KKT_upper_.setFromTriplets(triplets.begin(), triplets.end());
		state.KKT_upper_.makeCompressed();
		if (!state.kkt_solver_.factorize(state.KKT_upper_)) {
			throw std::runtime_error("KKT matrix is empty");
		}
	}

	// -------------------- Residuals / tolerances (original space) --------------------
	double calc_eps_eq_(const VectorXd &x, const VectorXd &s,
	                    const VectorXd &b) const {
		VectorXd Axps = A0_ * x + s;
		return options_.abs_tol +
		       options_.rel_tol * std::max(Axps.norm(), b.norm());
	}

	double calc_eps_ineq_(const VectorXd &x) const {
		VectorXd gx = G0_ * x;
		return options_.abs_tol + options_.rel_tol * gx.norm();
	}

	double calc_eps_kkt_(const VectorXd &x, const VectorXd &s,
	                     const VectorXd &y_eq, const VectorXd &y_ineq) const {
		VectorXd Qx = Q_reg_ * x;
		VectorXd ATy = A0_.transpose() * y_eq;
		VectorXd GTy;
		if (mi_ > 0) {
			GTy = G0_.transpose() * y_ineq;
		} else {
			GTy = VectorXd::Zero(n_x_);
		}
		double m1 = Qx.norm();
		double m2 = ATy.norm();
		double m3 = GTy.norm();
		double m4 = (options_.lambda_eq * s).norm();
		double m5 = y_eq.norm();
		return options_.abs_tol +
		       options_.rel_tol * std::max({m1, m2, m3, m4, m5});
	}

	void compute_info_(const VectorXd &b, OsqpState &state,
	                   OsqpInfo &info) const {
		// Map scaled variables back to original x,s
		VectorXd x_scaled = state.v.head(n_x_);
		VectorXd s_scaled = state.v.segment(n_x_, me_);
		state.x = col_scale_.head(n_x_).cwiseProduct(x_scaled);
		VectorXd s = col_scale_.segment(n_x_, me_).cwiseProduct(s_scaled);

		// Duals in original scaling:
		// y_hat = rho .* u  (scaled problem)
		// y_orig = (E .* y_hat) / cost_scale
		VectorXd y_hat = state.rho_vec_.cwiseProduct(state.u);

		VectorXd y_eq = VectorXd::Zero(me_);
		if (me_ > 0) {
			y_eq = row_scale_.head(me_).cwiseProduct(y_hat.head(me_)) /
			       cost_scale_;
		}

		VectorXd y_ineq = VectorXd::Zero(mi_);
		if (mi_ > 0) {
			y_ineq = row_scale_.tail(mi_).cwiseProduct(y_hat.tail(mi_)) /
			         cost_scale_;
		}

		// r_eq: equality residual in original space (includes slack)
		info.r_eq_norm = (A0_ * state.x + s - b).norm();

		// r_ineq: inequality violation (lower bound 0)
		if (mi_ > 0) {
			VectorXd gx = G0_ * state.x;
			info.r_ineq_norm = gx.cwiseMin(0.0).norm();
		} else {
			info.r_ineq_norm = 0.0;
		}

		// r_kkt: stationarity for [x;s]
		// grad_x = Qx + A^T y_eq + G^T y_ineq
		// grad_s = lambda*s + y_eq
		VectorXd grad_x = Q_reg_ * state.x;
		if (me_ > 0) {
			grad_x += A0_.transpose() * y_eq;
		}
		if (mi_ > 0) {
			grad_x += G0_.transpose() * y_ineq;
		}
		VectorXd grad_s = options_.lambda_eq * s + y_eq;
		info.r_kkt_norm =
		    std::sqrt(grad_x.squaredNorm() + grad_s.squaredNorm());

		// eps
		info.eps_eq = calc_eps_eq_(state.x, s, b);
		info.eps_ineq = calc_eps_ineq_(state.x);
		info.eps_kkt = calc_eps_kkt_(state.x, s, y_eq, y_ineq);
	}

	// -------------------- Adaptive rho --------------------
	bool adaptive_rho_update_(OsqpState &state) const {
		// Residuals in scaled space (infinity norms)
		VectorXd Cv = C_scaled_ * state.v;
		VectorXd r_prim = Cv - state.z;
		double r_prim_inf = r_prim.cwiseAbs().maxCoeff();

		VectorXd y_hat = state.rho_vec_.cwiseProduct(state.u);
		VectorXd r_dual = P_ * state.v + C_scaled_.transpose() * y_hat;
		double r_dual_inf = r_dual.cwiseAbs().maxCoeff();

		if (r_dual_inf < 1e-12) {
			r_dual_inf = 1e-12;
		}

		double scale = 1.0;
		if (r_prim_inf > 0.0) {
			scale = std::sqrt(r_prim_inf / r_dual_inf);
		}

		if (scale <= 0.0) {
			return false;
		}

		if (!(scale > options_.adaptive_rho_tolerance ||
		      scale < 1.0 / options_.adaptive_rho_tolerance)) {
			return false;
		}

		double new_rho_eq =
		    std::clamp(state.rho_eq_ * scale, options_.adaptive_rho_min,
		               options_.adaptive_rho_max);
		double new_rho_in =
		    std::clamp(state.rho_ineq_ * scale, options_.adaptive_rho_min,
		               options_.adaptive_rho_max);

		if (new_rho_eq == state.rho_eq_ && new_rho_in == state.rho_ineq_) {
			return false;
		}

		// Rescale u to keep y = rho*u constant
		state.u.head(me_) *= (state.rho_eq_ / new_rho_eq);
		if (mi_ > 0) {
			state.u.tail(mi_) *= (state.rho_ineq_ / new_rho_in);
		}

		state.rho_eq_ = new_rho_eq;
		state.rho_ineq_ = new_rho_in;
		state.rho_vec_.head(me_).setConstant(state.rho_eq_);
		if (mi_ > 0) {
			state.rho_vec_.tail(mi_).setConstant(state.rho_ineq_);
		}

		return true;
	}

	// -------------------- Polishing ("2nd step") --------------------
	bool polish_(const VectorXd &b_s, OsqpState &state) const {
		if (mi_ <= 0) {
			// Only equalities: polishing still helps stationarity
		}

		// Determine active set from z (scaled), lower bound is 0
		std::vector<int> active;
		active.reserve(static_cast<std::size_t>(mi_));
		for (int i = 0; i < mi_; ++i) {
			if (state.z[me_ + i] <= options_.polish_viol_tol) {
				active.push_back(i);
			}
		}

		// Map from original inequality row to row index in polish constraint matrix
		std::vector<int> row_map;
		row_map.resize(static_cast<std::size_t>(mi_), -1);

		auto rebuild_row_map = [&]() {
			std::fill(row_map.begin(), row_map.end(), -1);
			for (int k = 0; k < static_cast<int>(active.size()); ++k) {
				row_map[static_cast<std::size_t>(active[k])] = me_ + k;
			}
		};

		rebuild_row_map();

		// Choose delta: keep user value but cap to make delta*||v|| not dominate abs_tol
		double delta = std::max(options_.polish_delta, 1e-16);
		double v_inf = state.v.cwiseAbs().maxCoeff();
		double cap = options_.abs_tol / (10.0 * std::max(1.0, v_inf));
		if (cap > 0.0) {
			delta = std::min(delta, cap);
		}
		delta = std::max(delta, 1e-16);

		for (int refine = 0; refine < options_.polish_refine_max_iter;
		     ++refine) {
			int m_pol = me_ + static_cast<int>(active.size());

			// Build C_polish (scaled) with rows: equalities + active inequalities
			std::vector<Triplet<double>> C_trip;
			C_trip.reserve(A0_.nonZeros() + G0_.nonZeros() + me_);

			// Equality rows (A0 block)
			for (int j = 0; j < A0_.outerSize(); ++j) {
				for (SparseMatrix<double>::InnerIterator it(A0_, j); it; ++it) {
					int r = it.row();
					int c = it.col();
					double val = row_scale_[r] * it.value() * col_scale_[c];
					C_trip.emplace_back(r, c, val);
				}
			}

			// Slack identity in equality rows
			for (int i = 0; i < me_; ++i) {
				double val = row_scale_[i] * 1.0 * col_scale_[n_x_ + i];
				C_trip.emplace_back(i, n_x_ + i, val);
			}

			// Active inequality rows (G0 block)
			if (mi_ > 0 && !active.empty()) {
				for (int j = 0; j < G0_.outerSize(); ++j) {
					for (SparseMatrix<double>::InnerIterator it(G0_, j); it;
					     ++it) {
						int r0 = it.row();
						int out_r = row_map[static_cast<std::size_t>(r0)];
						if (out_r < 0) {
							continue;
						}
						int c = it.col();
						double val =
						    row_scale_[me_ + r0] * it.value() * col_scale_[c];
						C_trip.emplace_back(out_r, c, val);
					}
				}
			}

			SparseMatrix<double> C_pol(m_pol, n_);
			C_pol.setFromTriplets(C_trip.begin(), C_trip.end());
			C_pol.makeCompressed();

			// Build KKT (upper) for polish:
			// [ P + delta I, C_pol^T ]
			// [ C_pol,      -delta I ]
			int N = n_ + m_pol;
			std::vector<Triplet<double>> K_trip;
			K_trip.reserve(P_.nonZeros() + C_pol.nonZeros() + N);

			// Upper of P
			for (int j = 0; j < P_.outerSize(); ++j) {
				for (SparseMatrix<double>::InnerIterator itP(P_, j); itP;
				     ++itP) {
					if (itP.row() <= itP.col()) {
						K_trip.emplace_back(itP.row(), itP.col(), itP.value());
					}
				}
			}
			for (int i = 0; i < n_; ++i) {
				K_trip.emplace_back(i, i, delta);
			}

			// Top-right block: C_pol^T
			for (int j = 0; j < C_pol.outerSize(); ++j) {
				for (SparseMatrix<double>::InnerIterator itC(C_pol, j); itC;
				     ++itC) {
					int r = itC.row();
					int c = itC.col();
					K_trip.emplace_back(c, n_ + r, itC.value());
				}
			}

			// Bottom-right diag: -delta I
			for (int i = 0; i < m_pol; ++i) {
				K_trip.emplace_back(n_ + i, n_ + i, -delta);
			}

			SparseMatrix<double> K_pol(N, N);
			K_pol.setFromTriplets(K_trip.begin(), K_trip.end());
			K_pol.makeCompressed();

			QdldlSolver pol_solver;
			if (!pol_solver.factorize(K_pol)) {
				return false;
			}

			// RHS = [0; d]
			VectorXd rhs = VectorXd::Zero(N);
			rhs.segment(n_, me_) = b_s;
			// active inequality rhs is zero

			if (!pol_solver.solve_in_place(rhs)) {
				return false;
			}

			VectorXd v_pol = rhs.head(n_);
			VectorXd y_pol = rhs.tail(m_pol);

			// Check violated inequalities in original space
			VectorXd x_pol =
			    col_scale_.head(n_x_).cwiseProduct(v_pol.head(n_x_));
			VectorXd g = G0_ * x_pol;

			bool added = false;
			for (int i = 0; i < mi_; ++i) {
				if (g[i] < -options_.polish_viol_tol) {
					// add if not already active
					bool already = false;
					for (int a : active) {
						if (a == i) {
							already = true;
							break;
						}
					}
					if (!already) {
						active.push_back(i);
						added = true;
					}
				}
			}
			if (added) {
				rebuild_row_map();
				continue;
			}

			// Sign check for active inequalities:
			// For lower-bound constraints (z >= 0), OSQP-style dual satisfies y <= 0.
			bool removed_any = false;
			std::vector<int> active_filtered;
			active_filtered.reserve(active.size());
			for (int k = 0; k < static_cast<int>(active.size()); ++k) {
				double yk = y_pol[me_ + k];
				if (yk <= options_.polish_viol_tol) {
					active_filtered.push_back(active[k]);
				} else {
					removed_any = true;
				}
			}
			if (removed_any) {
				active = std::move(active_filtered);
				rebuild_row_map();
				continue;
			}

			// Accept polish result: update state.v, state.u (so y matches polish), and state.z.
			state.v = v_pol;

			VectorXd y_hat_full = VectorXd::Zero(m_);
			y_hat_full.head(me_) = y_pol.head(me_);
			for (int k = 0; k < static_cast<int>(active.size()); ++k) {
				y_hat_full[me_ + active[k]] = y_pol[me_ + k];
			}

			// u = y / rho
			state.u = y_hat_full.cwiseQuotient(state.rho_vec_);

			// z = projection of C*v
			VectorXd Cv = C_scaled_ * state.v;
			state.z.head(me_) = b_s;
			if (mi_ > 0) {
				state.z.tail(mi_) = Cv.tail(mi_).cwiseMax(0.0);
			}

			return true;
		}

		return false;
	}
};

} // namespace lego_assemble
