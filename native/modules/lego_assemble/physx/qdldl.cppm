export module lego_assemble.physx.qdldl;

import std;
import lego_assemble.vendor;

namespace lego_assemble {

using Eigen::SparseMatrix;
using Eigen::VectorXd;

export class QdldlSolver {
  public:
	QdldlSolver() = default;

	bool factorize(const SparseMatrix<double> &A_upper) {
		if (A_upper.rows() != A_upper.cols()) {
			return false;
		}

		if (!A_upper.isCompressed()) {
			SparseMatrix<double> tmp = A_upper;
			tmp.makeCompressed();
			return factorize(tmp);
		}

		n_ = static_cast<int>(A_upper.rows());

		Ap_.assign(A_upper.outerIndexPtr(), A_upper.outerIndexPtr() + (n_ + 1));
		Ai_.assign(A_upper.innerIndexPtr(),
		           A_upper.innerIndexPtr() + A_upper.nonZeros());
		Ax_.assign(A_upper.valuePtr(), A_upper.valuePtr() + A_upper.nonZeros());

		etree_.assign(n_, 0);
		Lnz_.assign(n_, 0);

		Lp_.assign(n_ + 1, 0);

		std::vector<int> work(n_, 0);
		int sumLnz = qdldl_etree_(n_, Ap_.data(), Ai_.data(), work.data(),
		                          etree_.data(), Lnz_.data());
		if (sumLnz < 0) {
			factorized_ = false;
			return false;
		}

		// Cumsum to form Lp
		Lp_[0] = 0;
		for (int i = 0; i < n_; ++i) {
			Lp_[i + 1] = Lp_[i] + Lnz_[i];
		}

		Li_.assign(sumLnz, 0);
		Lx_.assign(sumLnz, 0.0);
		D_.assign(n_, 0.0);
		Dinv_.assign(n_, 0.0);

		iwork_.assign(3 * n_, 0);
		bwork_.assign(n_, 0);
		fwork_.assign(n_, 0.0);

		int posD = qdldl_factor_(n_, Ap_.data(), Ai_.data(), Ax_.data(),
		                         Lp_.data(), Li_.data(), Lx_.data(), D_.data(),
		                         Dinv_.data(), etree_.data(), Lnz_.data(),
		                         iwork_.data(), bwork_.data(), fwork_.data());

		if (posD < 0) {
			factorized_ = false;
			return false;
		}

		factorized_ = true;
		return true;
	}

	bool solve_in_place(VectorXd &x) const {
		if (!factorized_) {
			return false;
		}
		if (x.size() != n_) {
			return false;
		}

		double *xd = x.data();
		qdldl_Lsolve_(n_, Lp_.data(), Li_.data(), Lx_.data(), xd);
		for (int i = 0; i < n_; ++i) {
			xd[i] *= Dinv_[i];
		}
		qdldl_Ltsolve_(n_, Lp_.data(), Li_.data(), Lx_.data(), xd);
		return true;
	}

  private:
	int n_ = 0;
	bool factorized_ = false;

	std::vector<int> Ap_;
	std::vector<int> Ai_;
	std::vector<double> Ax_;

	std::vector<int> etree_;
	std::vector<int> Lnz_;

	std::vector<int> Lp_;
	std::vector<int> Li_;
	std::vector<double> Lx_;

	std::vector<double> D_;
	std::vector<double> Dinv_;

	// Factorization workspaces
	mutable std::vector<int> iwork_;           // size 3n
	mutable std::vector<unsigned char> bwork_; // size n
	mutable std::vector<double> fwork_;        // size n

	int qdldl_etree_(int n, const int *Ap, const int *Ai, int *work, int *etree,
	                 int *Lnz) const {
		int unknown = -1;

		for (int i = 0; i < n; ++i) {
			work[i] = 0;
			Lnz[i] = 0;
			etree[i] = unknown;
			if (Ap[i] == Ap[i + 1]) {
				return -1;
			}
		}

		for (int j = 0; j < n; ++j) {
			work[j] = j;
			for (int p = Ap[j]; p < Ap[j + 1]; ++p) {
				int i = Ai[p];
				if (i > j) {
					return -1;
				}
				while (work[i] != j) {
					if (etree[i] == unknown) {
						etree[i] = j;
					}
					Lnz[i]++;
					work[i] = j;
					i = etree[i];
				}
			}
		}

		int sumLnz = 0;
		for (int i = 0; i < n; ++i) {
			if (sumLnz > std::numeric_limits<int>::max() - Lnz[i]) {
				return -2;
			}
			sumLnz += Lnz[i];
		}
		return sumLnz;
	}

	int qdldl_factor_(int n, const int *Ap, const int *Ai, const double *Ax,
	                  int *Lp, int *Li, double *Lx, double *D, double *Dinv,
	                  const int *etree, const int *Lnz, int *iwork,
	                  unsigned char *bwork, double *fwork) const {
		int unknown = -1;
		unsigned char used = 1;
		unsigned char unused = 0;
		int positiveValuesInD = 0;

		unsigned char *yMarkers = bwork;
		int *yIdx = iwork;
		int *elimBuffer = iwork + n;
		int *LNextSpaceInCol = iwork + n * 2;
		double *yVals = fwork;

		Lp[0] = 0;
		for (int i = 0; i < n; ++i) {
			Lp[i + 1] = Lp[i] + Lnz[i];
			yMarkers[i] = unused;
			yVals[i] = 0.0;
			D[i] = 0.0;
			LNextSpaceInCol[i] = Lp[i];
		}

		D[0] = Ax[0];
		if (D[0] == 0.0) {
			return -1;
		}
		if (D[0] > 0.0) {
			positiveValuesInD++;
		}
		Dinv[0] = 1.0 / D[0];

		for (int k = 1; k < n; ++k) {
			int nnzY = 0;
			int tmpIdx = Ap[k + 1];

			for (int i = Ap[k]; i < tmpIdx; ++i) {
				int bidx = Ai[i];
				if (bidx == k) {
					D[k] = Ax[i];
					continue;
				}
				yVals[bidx] = Ax[i];

				int nextIdx = bidx;
				if (yMarkers[nextIdx] == unused) {
					yMarkers[nextIdx] = used;
					elimBuffer[0] = nextIdx;
					int nnzE = 1;
					nextIdx = etree[bidx];

					while (nextIdx != unknown && nextIdx < k) {
						if (yMarkers[nextIdx] == used) {
							break;
						}
						yMarkers[nextIdx] = used;
						elimBuffer[nnzE] = nextIdx;
						nnzE++;
						nextIdx = etree[nextIdx];
					}

					while (nnzE) {
						nnzE--;
						yIdx[nnzY] = elimBuffer[nnzE];
						nnzY++;
					}
				}
			}

			for (int i = nnzY - 1; i >= 0; --i) {
				int cidx = yIdx[i];
				int nextSpace = LNextSpaceInCol[cidx];
				double yVals_cidx = yVals[cidx];

				for (int j = Lp[cidx]; j < nextSpace; ++j) {
					yVals[Li[j]] -= Lx[j] * yVals_cidx;
				}

				Li[nextSpace] = k;
				Lx[nextSpace] = yVals_cidx * Dinv[cidx];

				D[k] -= yVals_cidx * Lx[nextSpace];
				LNextSpaceInCol[cidx]++;

				yVals[cidx] = 0.0;
				yMarkers[cidx] = unused;
			}

			if (D[k] == 0.0) {
				return -1;
			}
			if (D[k] > 0.0) {
				positiveValuesInD++;
			}

			Dinv[k] = 1.0 / D[k];
		}

		return positiveValuesInD;
	}

	void qdldl_Lsolve_(int n, const int *Lp, const int *Li, const double *Lx,
	                   double *x) const {
		for (int i = 0; i < n; ++i) {
			double val = x[i];
			for (int j = Lp[i]; j < Lp[i + 1]; ++j) {
				x[Li[j]] -= Lx[j] * val;
			}
		}
	}

	void qdldl_Ltsolve_(int n, const int *Lp, const int *Li, const double *Lx,
	                    double *x) const {
		for (int i = n - 1; i >= 0; --i) {
			double val = x[i];
			for (int j = Lp[i]; j < Lp[i + 1]; ++j) {
				val -= Lx[j] * x[Li[j]];
			}
			x[i] = val;
		}
	}
};

} // namespace lego_assemble
