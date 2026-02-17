export module lego_assemble.physx.osqp;

import std;
import lego_assemble.utils.matrix_serialization;
import lego_assemble.vendor;

namespace osqp = lego_assemble::vendor::osqp;

namespace std {
template <> struct formatter<osqp::OSQPInfo> {
	constexpr format_parse_context::iterator parse(format_parse_context &ctx) {
		return ctx.begin();
	}
	auto format(const osqp::OSQPInfo &info, auto &ctx) const {
		return std::format_to(
		    ctx.out(),
		    "         status = {}\n"
		    "     status_val = {}\n"
		    "  status_polish = {}\n"
		    "        obj_val = {}\n"
		    "   dual_obj_val = {}\n"
		    "       prim_res = {}\n"
		    "       dual_res = {}\n"
		    "    duality_gap = {}\n"
		    "           iter = {}\n"
		    "    rho_updates = {}\n"
		    "   rho_estimate = {}\n"
		    "     setup_time = {}\n"
		    "     solve_time = {}\n"
		    "    update_time = {}\n"
		    "    polish_time = {}\n"
		    "       run_time = {}\n"
		    "   primdual_int = {}\n"
		    "  rel_kkt_error = {}",
		    info.status, info.status_val, info.status_polish, info.obj_val,
		    info.dual_obj_val, info.prim_res, info.dual_res, info.duality_gap,
		    info.iter, info.rho_updates, info.rho_estimate, info.setup_time,
		    info.solve_time, info.update_time, info.polish_time, info.run_time,
		    info.primdual_int, info.rel_kkt_error);
	}
};
} // namespace std

namespace lego_assemble {

using Eigen::ColMajor;
using Eigen::Map;
using Eigen::SparseMatrix;
using Eigen::Triplet;
using Eigen::VectorXd;

void check_osqp_error(osqp::OSQPInt ret) {
	if (ret != 0) {
		throw std::runtime_error(osqp::osqp_error_message(ret));
	}
}

template <auto F> struct Deleter {
	template <class T> constexpr void operator()(T *arg) const {
		F(arg);
	}
};
using OsqpSolverPtr =
    std::unique_ptr<osqp::OSQPSolver, Deleter<&osqp::osqp_cleanup>>;

export struct OsqpInfo {
	bool converged{false};
	bool prj_converged{false};
	osqp::OSQPInfo prj_info{};
	bool rlx_converged{false};
	osqp::OSQPInfo rlx_info{};
	bool opt_converged{false};
	osqp::OSQPInfo opt_info{};

	std::string to_string() const {
		return std::format("Overall converged: {}\n"
		                   "Projection converged: {}\n"
		                   "{}\n"
		                   "Relaxation converged: {}\n"
		                   "{}\n"
		                   "Optimization converged: {}\n"
		                   "{}",
		                   converged, prj_converged, prj_info, rlx_converged,
		                   rlx_info, opt_converged, opt_info);
	}
};

void osqp_info_to_json(nlohmann::ordered_json &j, const osqp::OSQPInfo &info) {
	j = nlohmann::ordered_json{
	    {"status", info.status},
	    {"status_val", info.status_val},
	    {"status_polish", info.status_polish},
	    {"obj_val", info.obj_val},
	    {"dual_obj_val", info.dual_obj_val},
	    {"prim_res", info.prim_res},
	    {"dual_res", info.dual_res},
	    {"duality_gap", info.duality_gap},
	    {"iter", info.iter},
	    {"rho_updates", info.rho_updates},
	    {"rho_estimate", info.rho_estimate},
	    {"setup_time", info.setup_time},
	    {"solve_time", info.solve_time},
	    {"update_time", info.update_time},
	    {"polish_time", info.polish_time},
	    {"run_time", info.run_time},
	    {"primdual_int", info.primdual_int},
	    {"rel_kkt_error", info.rel_kkt_error},
	};
}
void osqp_info_from_json(const nlohmann::ordered_json &j,
                         osqp::OSQPInfo &info) {
	std::fill_n(info.status, sizeof(info.status), '\0');
	std::strncpy(info.status, j.at("status").get<std::string>().c_str(),
	             sizeof(info.status) - 1);
	j.at("status_val").get_to(info.status_val);
	j.at("status_polish").get_to(info.status_polish);
	j.at("obj_val").get_to(info.obj_val);
	j.at("dual_obj_val").get_to(info.dual_obj_val);
	j.at("prim_res").get_to(info.prim_res);
	j.at("dual_res").get_to(info.dual_res);
	j.at("duality_gap").get_to(info.duality_gap);
	j.at("iter").get_to(info.iter);
	j.at("rho_updates").get_to(info.rho_updates);
	j.at("rho_estimate").get_to(info.rho_estimate);
	j.at("setup_time").get_to(info.setup_time);
	j.at("solve_time").get_to(info.solve_time);
	j.at("update_time").get_to(info.update_time);
	j.at("polish_time").get_to(info.polish_time);
	j.at("run_time").get_to(info.run_time);
	j.at("primdual_int").get_to(info.primdual_int);
	j.at("rel_kkt_error").get_to(info.rel_kkt_error);
}

export void to_json(nlohmann::ordered_json &j, const OsqpInfo &info) {
	j = nlohmann::ordered_json{};
	j["converged"] = info.converged;
	j["prj_converged"] = info.prj_converged;
	osqp_info_to_json(j["prj_info"], info.prj_info);
	j["rlx_converged"] = info.rlx_converged;
	osqp_info_to_json(j["rlx_info"], info.rlx_info);
	j["opt_converged"] = info.opt_converged;
	osqp_info_to_json(j["opt_info"], info.opt_info);
}

export void from_json(const nlohmann::ordered_json &j, OsqpInfo &info) {
	j.at("converged").get_to(info.converged);
	j.at("prj_converged").get_to(info.prj_converged);
	osqp_info_from_json(j.at("prj_info"), info.prj_info);
	j.at("rlx_converged").get_to(info.rlx_converged);
	osqp_info_from_json(j.at("rlx_info"), info.rlx_info);
	j.at("opt_converged").get_to(info.opt_converged);
	osqp_info_from_json(j.at("opt_info"), info.opt_info);
}

osqp::OSQPCscMatrix osqp_csc_from_eigen(
    const SparseMatrix<osqp::OSQPFloat, ColMajor, osqp::OSQPInt> &mat) {
	return {
	    .m = static_cast<osqp::OSQPInt>(mat.rows()),
	    .n = static_cast<osqp::OSQPInt>(mat.cols()),
	    .p = const_cast<osqp::OSQPInt *>(mat.outerIndexPtr()),
	    .i = const_cast<osqp::OSQPInt *>(mat.innerIndexPtr()),
	    .x = const_cast<osqp::OSQPFloat *>(mat.valuePtr()),
	    .nzmax = static_cast<osqp::OSQPInt>(mat.nonZeros()),
	    .nz = -1,
	    .owned = 0,
	};
}

struct SolverData {
	OsqpSolverPtr prj_solver{};
	OsqpSolverPtr rlx_solver{};
	OsqpSolverPtr opt_solver{};

	SolverData() = default;
	~SolverData() = default;
	SolverData(const SolverData &) {
		// Solvers to be rebuilt
	}
	SolverData &operator=(const SolverData &o) {
		// Solvers to be rebuilt
		if (this != &o) {
			prj_solver.reset();
			rlx_solver.reset();
			opt_solver.reset();
		}
		return *this;
	}
	SolverData(SolverData &&) noexcept = default;
	SolverData &operator=(SolverData &&) noexcept = default;
};

export struct OsqpState {
	bool has_state{false};
	VectorXd prj_sol_{};
	VectorXd prj_dual_{};
	double prj_rho_{};
	VectorXd rlx_sol_{};
	VectorXd rlx_dual_{};
	double rlx_rho_{};
	VectorXd opt_sol_{};
	VectorXd opt_dual_{};
	double opt_rho_{};
	VectorXd slack_{};
	VectorXd v_{};
	SolverData data_{};

	const VectorXd &x() const {
		return opt_sol_;
	}
	const VectorXd &s() const {
		return slack_;
	}
	const VectorXd &v() const {
		return v_;
	}
	void reset() {
		*this = {};
	}
};

export void to_json(nlohmann::ordered_json &j, const OsqpState &s) {
	j = nlohmann::ordered_json{
	    {"has_state", s.has_state},
	    {"prj_sol", matrix_to_json(s.prj_sol_)},
	    {"prj_dual", matrix_to_json(s.prj_dual_)},
	    {"prj_rho", s.prj_rho_},
	    {"rlx_sol", matrix_to_json(s.rlx_sol_)},
	    {"rlx_dual", matrix_to_json(s.rlx_dual_)},
	    {"rlx_rho", s.rlx_rho_},
	    {"opt_sol", matrix_to_json(s.opt_sol_)},
	    {"opt_dual", matrix_to_json(s.opt_dual_)},
	    {"opt_rho", s.opt_rho_},
	    {"slack", matrix_to_json(s.slack_)},
	};
}

export void from_json(const nlohmann::ordered_json &j, OsqpState &s) {
	j.at("has_state").get_to(s.has_state);
	s.prj_sol_ = json_to_matrix<VectorXd>(j.at("prj_sol"));
	s.prj_dual_ = json_to_matrix<VectorXd>(j.at("prj_dual"));
	j.at("prj_rho").get_to(s.prj_rho_);
	s.rlx_sol_ = json_to_matrix<VectorXd>(j.at("rlx_sol"));
	s.rlx_dual_ = json_to_matrix<VectorXd>(j.at("rlx_dual"));
	j.at("rlx_rho").get_to(s.rlx_rho_);
	s.opt_sol_ = json_to_matrix<VectorXd>(j.at("opt_sol"));
	s.opt_dual_ = json_to_matrix<VectorXd>(j.at("opt_dual"));
	j.at("opt_rho").get_to(s.opt_rho_);
	s.slack_ = json_to_matrix<VectorXd>(j.at("slack"));
}

export enum class OsqpSolveType {
	ALWAYS_FULL,
	FULL_IF_VIOLATION,
	RELAX_ONLY,
};

export class OsqpSolver {
  public:
	explicit OsqpSolver(const SparseMatrix<double> &Q, SparseMatrix<double> A,
	                    const SparseMatrix<double> &G,
	                    const SparseMatrix<double> &H, SparseMatrix<double> V)
	    : nx_(static_cast<int>(Q.rows())), nv_(static_cast<int>(V.cols())),
	      me_(static_cast<int>(A.rows())), mi_(static_cast<int>(G.rows())),
	      mh_(static_cast<int>(H.rows())) {
		if (auto *env_eps = std::getenv("OSQP_EPSILON")) {
			epsilon = std::stod(env_eps);
		}
		if (!Q.isCompressed() || !A.isCompressed() || !G.isCompressed() ||
		    !H.isCompressed() || !V.isCompressed()) {
			throw std::invalid_argument("Input matrices must be compressed");
		}
		if (Q.cols() != nx_ || A.cols() != nx_ || G.cols() != nx_ ||
		    H.cols() != nx_ || V.rows() != mh_) {
			throw std::invalid_argument("Input matrix size mismatch");
		}
		P_prj_ = build_P_prj_();
		C_prj_ = build_C_prj_(A, G);
		l_prj_ = build_l_prj_();
		u_prj_ = build_u_prj_();
		P_rlx_ = build_P_rlx_();
		C_rlx_ = build_C_rlx_(A, G, H, V);
		q_rlx_ = build_q_rlx_();
		P_opt_ = build_P_opt_(Q);
		C_opt_ = build_C_opt_(A, G, H);
		q_opt_ = build_q_opt_();
		A_ = std::move(A);
		V_ = std::move(V);
	}

	OsqpInfo solve(const VectorXd &b, OsqpState &state) const {
		if (b.size() != me_) {
			throw std::invalid_argument("b dimension mismatch");
		}
		if (state.has_state) {
			if (state.prj_sol_.size() != C_prj_.cols() ||
			    state.prj_dual_.size() != C_prj_.rows() ||
			    state.rlx_sol_.size() != C_rlx_.cols() ||
			    state.rlx_dual_.size() != C_rlx_.rows() ||
			    state.opt_sol_.size() != C_opt_.cols() ||
			    state.opt_dual_.size() != C_opt_.rows()) {
				throw std::invalid_argument("state dimension mismatch");
			}
		}

		// 1. Solve projection
		OsqpSolverPtr &prj_solver = state.data_.prj_solver;
		if (state.has_state) {
			if (prj_solver) {
				update_solver_prj_(prj_solver, b);
			} else {
				prj_solver = build_solver_prj_(b);
				osqp::osqp_warm_start(prj_solver.get(), state.prj_sol_.data(),
				                      state.prj_dual_.data());
				osqp::osqp_update_rho(prj_solver.get(), state.prj_rho_);
			}
		} else {
			prj_solver = build_solver_prj_(b);
		}
		check_osqp_error(osqp::osqp_solve(prj_solver.get()));
		osqp::OSQPSolution *prj_sol = prj_solver->solution;
		osqp::OSQPInfo *prj_info = prj_solver->info;
		state.prj_sol_ = Map<const VectorXd>(prj_sol->x, C_prj_.cols());
		state.prj_dual_ = Map<const VectorXd>(prj_sol->y, C_prj_.rows());
		state.prj_rho_ = prj_solver->settings->rho;
		auto b_prj = A_ * state.prj_sol_.head(nx_);
		state.slack_ = b - b_prj;
		OsqpInfo info;
		info.prj_converged = prj_info->status_val == osqp::OSQP_SOLVED;
		info.prj_info = *prj_info;

		// 2. Solve relaxation
		OsqpSolverPtr &rlx_solver = state.data_.rlx_solver;
		if (state.has_state) {
			if (rlx_solver) {
				update_solver_rlx_(rlx_solver, b_prj);
			} else {
				rlx_solver = build_solver_rlx_(b_prj);
				osqp::osqp_warm_start(rlx_solver.get(), state.rlx_sol_.data(),
				                      state.rlx_dual_.data());
				osqp::osqp_update_rho(rlx_solver.get(), state.rlx_rho_);
			}
		} else {
			rlx_solver = build_solver_rlx_(b_prj);
		}
		check_osqp_error(osqp::osqp_solve(rlx_solver.get()));
		osqp::OSQPSolution *rlx_sol = rlx_solver->solution;
		osqp::OSQPInfo *rlx_info = rlx_solver->info;
		state.rlx_sol_ = Map<const VectorXd>(rlx_sol->x, C_rlx_.cols());
		state.rlx_dual_ = Map<const VectorXd>(rlx_sol->y, C_rlx_.rows());
		state.rlx_rho_ = rlx_solver->settings->rho;
		state.v_ = state.rlx_sol_.tail(nv_).cwiseMax(0.0);
		info.rlx_converged = rlx_info->status_val == osqp::OSQP_SOLVED;
		info.rlx_info = *rlx_info;

		// 3. Solve optimization
		OsqpSolverPtr &opt_solver = state.data_.opt_solver;
		if (state.has_state) {
			if (opt_solver) {
				update_solver_opt_(opt_solver, b_prj, state.v_);
			} else {
				opt_solver = build_solver_opt_(b_prj, state.v_);
				osqp::osqp_warm_start(opt_solver.get(), state.opt_sol_.data(),
				                      state.opt_dual_.data());
				osqp::osqp_update_rho(opt_solver.get(), state.opt_rho_);
			}
		} else {
			opt_solver = build_solver_opt_(b_prj, state.v_);
		}
		check_osqp_error(osqp::osqp_solve(opt_solver.get()));
		osqp::OSQPSolution *opt_sol = opt_solver->solution;
		osqp::OSQPInfo *opt_info = opt_solver->info;
		state.opt_sol_ = Map<const VectorXd>(opt_sol->x, C_opt_.cols());
		state.opt_dual_ = Map<const VectorXd>(opt_sol->y, C_opt_.rows());
		state.opt_rho_ = opt_solver->settings->rho;
		info.opt_converged = opt_info->status_val == osqp::OSQP_SOLVED;
		info.opt_info = *opt_info;
		info.converged =
		    info.prj_converged && info.rlx_converged && info.opt_converged;
		state.has_state = true;
		return info;
	}

  private:
	int nx_;
	int nv_;
	int me_;
	int mi_;
	int mh_;

	double epsilon = 1e-4;
	double rel_x_reg = 1e-4;

	SparseMatrix<double> A_;
	SparseMatrix<double> V_;
	SparseMatrix<double> P_prj_;
	SparseMatrix<double> C_prj_;
	VectorXd l_prj_;
	VectorXd u_prj_;
	SparseMatrix<double> P_rlx_;
	SparseMatrix<double> C_rlx_;
	VectorXd q_rlx_;
	SparseMatrix<double> P_opt_;
	SparseMatrix<double> C_opt_;
	VectorXd q_opt_;

	SparseMatrix<double> build_P_prj_() const {
		std::vector<Triplet<double>> triplets;
		triplets.reserve(me_);
		for (int i = 0; i < me_; ++i) {
			triplets.emplace_back(nx_ + i, nx_ + i, 1.0);
		}
		SparseMatrix<double> P(nx_ + me_, nx_ + me_);
		P.setFromTriplets(triplets.begin(), triplets.end());
		P.makeCompressed();
		return P;
	}
	SparseMatrix<double> build_C_prj_(const SparseMatrix<double> &A,
	                                  const SparseMatrix<double> &G) const {
		std::vector<Triplet<double>> triplets;
		triplets.reserve(A.nonZeros() + G.nonZeros() + me_);
		for (int k = 0; k < A.outerSize(); ++k) {
			for (SparseMatrix<double>::InnerIterator it(A, k); it; ++it) {
				triplets.emplace_back(it.row(), it.col(), it.value());
			}
		}
		for (int k = 0; k < G.outerSize(); ++k) {
			for (SparseMatrix<double>::InnerIterator it(G, k); it; ++it) {
				triplets.emplace_back(me_ + it.row(), it.col(), it.value());
			}
		}
		for (int i = 0; i < me_; ++i) {
			triplets.emplace_back(i, nx_ + i, -1.0);
		}
		SparseMatrix<double> C(me_ + mi_, nx_ + me_);
		C.setFromTriplets(triplets.begin(), triplets.end());
		C.makeCompressed();
		return C;
	}
	VectorXd build_l_prj_() const {
		return VectorXd::Zero(me_ + mi_);
	}
	VectorXd build_u_prj_() const {
		VectorXd u(me_ + mi_);
		u.head(me_).setZero();
		u.tail(mi_).setConstant(osqp::infinity);
		return u;
	}
	VectorXd build_q_prj_(const VectorXd &b) const {
		VectorXd q(nx_ + me_);
		q.head(nx_).setZero();
		q.tail(me_) = -b;
		return q;
	}
	SparseMatrix<double> build_P_rlx_() const {
		std::vector<Triplet<double>> triplets;
		triplets.reserve(nx_ + nv_);
		for (int i = 0; i < nx_; ++i) {
			triplets.emplace_back(i, i, rel_x_reg);
		}
		for (int i = 0; i < nv_; ++i) {
			triplets.emplace_back(nx_ + i, nx_ + i, 1.0);
		}
		SparseMatrix<double> P(nx_ + nv_, nx_ + nv_);
		P.setFromTriplets(triplets.begin(), triplets.end());
		P.makeCompressed();
		return P;
	}
	SparseMatrix<double> build_C_rlx_(const SparseMatrix<double> &A,
	                                  const SparseMatrix<double> &G,
	                                  const SparseMatrix<double> &H,
	                                  const SparseMatrix<double> &V) const {
		std::vector<Triplet<double>> triplets;
		triplets.reserve(A.nonZeros() + G.nonZeros() + H.nonZeros() +
		                 V.nonZeros() + nv_);
		for (int k = 0; k < A.outerSize(); ++k) {
			for (SparseMatrix<double>::InnerIterator it(A, k); it; ++it) {
				triplets.emplace_back(it.row(), it.col(), it.value());
			}
		}
		for (int k = 0; k < G.outerSize(); ++k) {
			for (SparseMatrix<double>::InnerIterator it(G, k); it; ++it) {
				triplets.emplace_back(me_ + it.row(), it.col(), it.value());
			}
		}
		for (int k = 0; k < H.outerSize(); ++k) {
			for (SparseMatrix<double>::InnerIterator it(H, k); it; ++it) {
				triplets.emplace_back(me_ + mi_ + it.row(), it.col(),
				                      it.value());
			}
		}
		for (int k = 0; k < V.outerSize(); ++k) {
			for (SparseMatrix<double>::InnerIterator it(V, k); it; ++it) {
				triplets.emplace_back(me_ + mi_ + it.row(), nx_ + it.col(),
				                      -it.value());
			}
		}
		for (int i = 0; i < nv_; ++i) {
			triplets.emplace_back(me_ + mi_ + mh_ + i, nx_ + i, 1.0);
		}
		SparseMatrix<double> C(me_ + mi_ + mh_ + nv_, nx_ + nv_);
		C.setFromTriplets(triplets.begin(), triplets.end());
		C.makeCompressed();
		return C;
	}
	VectorXd build_q_rlx_() const {
		return VectorXd::Zero(nx_ + nv_);
	}
	VectorXd build_l_rlx_(const VectorXd &b) const {
		VectorXd l(me_ + mi_ + mh_ + nv_);
		l.head(me_) = b;
		l.segment(me_, mi_).setConstant(-epsilon);
		l.segment(me_ + mi_, mh_).setConstant(-osqp::infinity);
		l.tail(nv_).setZero();
		return l;
	}
	VectorXd build_u_rlx_(const VectorXd &b) const {
		VectorXd u(me_ + mi_ + mh_ + nv_);
		u.head(me_) = b;
		u.segment(me_, mi_).setConstant(osqp::infinity);
		u.segment(me_ + mi_, mh_).setConstant(1.0 - epsilon);
		u.tail(nv_).setConstant(osqp::infinity);
		return u;
	}
	SparseMatrix<double> build_P_opt_(const SparseMatrix<double> &Q) const {
		std::vector<Triplet<double>> triplets;
		triplets.reserve(Q.nonZeros());
		for (int k = 0; k < Q.outerSize(); ++k) {
			for (SparseMatrix<double>::InnerIterator it(Q, k); it; ++it) {
				if (it.row() <= it.col()) { // upper triangular
					triplets.emplace_back(it.row(), it.col(), it.value());
				}
			}
		}
		SparseMatrix<double> P(nx_, nx_);
		P.setFromTriplets(triplets.begin(), triplets.end());
		P.makeCompressed();
		return P;
	}
	SparseMatrix<double> build_C_opt_(const SparseMatrix<double> &A,
	                                  const SparseMatrix<double> &G,
	                                  const SparseMatrix<double> &H) const {
		std::vector<Triplet<double>> triplets;
		triplets.reserve(A.nonZeros() + G.nonZeros() + H.nonZeros());
		for (int k = 0; k < A.outerSize(); ++k) {
			for (SparseMatrix<double>::InnerIterator it(A, k); it; ++it) {
				triplets.emplace_back(it.row(), it.col(), it.value());
			}
		}
		for (int k = 0; k < G.outerSize(); ++k) {
			for (SparseMatrix<double>::InnerIterator it(G, k); it; ++it) {
				triplets.emplace_back(me_ + it.row(), it.col(), it.value());
			}
		}
		for (int k = 0; k < H.outerSize(); ++k) {
			for (SparseMatrix<double>::InnerIterator it(H, k); it; ++it) {
				triplets.emplace_back(me_ + mi_ + it.row(), it.col(),
				                      it.value());
			}
		}
		SparseMatrix<double> C(me_ + mi_ + mh_, nx_);
		C.setFromTriplets(triplets.begin(), triplets.end());
		C.makeCompressed();
		return C;
	}
	VectorXd build_q_opt_() const {
		return VectorXd::Zero(nx_);
	}
	VectorXd build_l_opt_(const VectorXd &b) const {
		VectorXd l(me_ + mi_ + mh_);
		l.head(me_) = b;
		l.segment(me_, mi_).setConstant(-epsilon);
		l.tail(mh_).setConstant(-osqp::infinity);
		return l;
	}
	VectorXd build_u_opt_(const VectorXd &b, const VectorXd &v) const {
		VectorXd u(me_ + mi_ + mh_);
		u.head(me_) = b;
		u.segment(me_, mi_).setConstant(osqp::infinity);
		u.tail(mh_).setConstant(1.0);
		u.tail(mh_) += V_ * v;
		return u;
	}
	osqp::OSQPSettings build_settings_() const {
		osqp::OSQPSettings settings;
		osqp::osqp_set_default_settings(&settings);
		settings.verbose = 0;
		settings.max_iter = 100000;
		settings.eps_abs = epsilon;
		settings.eps_rel = epsilon;
		return settings;
	}
	OsqpSolverPtr build_solver_prj_(const VectorXd &b) const {
		osqp::OSQPSettings settings = build_settings_();
		osqp::OSQPCscMatrix P_csc = osqp_csc_from_eigen(P_prj_);
		osqp::OSQPCscMatrix C_csc = osqp_csc_from_eigen(C_prj_);
		VectorXd l = build_l_prj_();
		VectorXd u = build_u_prj_();
		VectorXd q = build_q_prj_(b);
		osqp::OSQPSolver *solver = nullptr;
		check_osqp_error(osqp::osqp_setup(&solver, &P_csc, q.data(), &C_csc,
		                                  l.data(), u.data(), C_prj_.rows(),
		                                  C_prj_.cols(), &settings));
		return OsqpSolverPtr(solver);
	}
	OsqpSolverPtr build_solver_rlx_(const VectorXd &b) const {
		osqp::OSQPSettings settings = build_settings_();
		osqp::OSQPCscMatrix P_csc = osqp_csc_from_eigen(P_rlx_);
		osqp::OSQPCscMatrix C_csc = osqp_csc_from_eigen(C_rlx_);
		VectorXd l = build_l_rlx_(b);
		VectorXd u = build_u_rlx_(b);
		VectorXd q = build_q_rlx_();
		osqp::OSQPSolver *solver = nullptr;
		check_osqp_error(osqp::osqp_setup(&solver, &P_csc, q.data(), &C_csc,
		                                  l.data(), u.data(), C_rlx_.rows(),
		                                  C_rlx_.cols(), &settings));
		return OsqpSolverPtr(solver);
	}
	OsqpSolverPtr build_solver_opt_(const VectorXd &b,
	                                const VectorXd &v) const {
		osqp::OSQPSettings settings = build_settings_();
		osqp::OSQPCscMatrix P_csc = osqp_csc_from_eigen(P_opt_);
		osqp::OSQPCscMatrix C_csc = osqp_csc_from_eigen(C_opt_);
		VectorXd l = build_l_opt_(b);
		VectorXd u = build_u_opt_(b, v);
		osqp::OSQPSolver *solver = nullptr;
		check_osqp_error(osqp::osqp_setup(
		    &solver, &P_csc, q_opt_.data(), &C_csc, l.data(), u.data(),
		    C_opt_.rows(), C_opt_.cols(), &settings));
		return OsqpSolverPtr(solver);
	}
	void update_solver_prj_(OsqpSolverPtr &solver, const VectorXd &b) const {
		VectorXd q = build_q_prj_(b);
		osqp::OSQPInt ret = osqp::osqp_update_data_vec(solver.get(), q.data(),
		                                               nullptr, nullptr);
		check_osqp_error(ret);
	}
	void update_solver_rlx_(OsqpSolverPtr &solver, const VectorXd &b) const {
		VectorXd l = build_l_rlx_(b);
		VectorXd u = build_u_rlx_(b);
		check_osqp_error(osqp::osqp_update_data_vec(solver.get(), nullptr,
		                                            l.data(), u.data()));
	}
	void update_solver_opt_(OsqpSolverPtr &solver, const VectorXd &b,
	                        const VectorXd &v) const {
		VectorXd l = build_l_opt_(b);
		VectorXd u = build_u_opt_(b, v);
		check_osqp_error(osqp::osqp_update_data_vec(solver.get(), nullptr,
		                                            l.data(), u.data()));
	}
};

} // namespace lego_assemble
