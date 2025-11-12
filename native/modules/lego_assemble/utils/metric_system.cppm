export module lego_assemble.utils.metric_system;

import std;
import lego_assemble.physx.scene_patcher;
import lego_assemble.utils.transforms;
import lego_assemble.vendor.pxr;
import lego_assemble.vendor.physx;

namespace lego_assemble {

export struct MetricSystem {
  private:
	double mpu_;
	double kpu_;
	double spu_;

  public:
	explicit MetricSystem(double meters_per_unit = 1.0,
	                      double kilograms_per_unit = 1.0,
	                      double seconds_per_unit = 1.0)
	    : mpu_{meters_per_unit}, kpu_{kilograms_per_unit},
	      spu_{seconds_per_unit} {}

	explicit MetricSystem(const pxr::UsdStageRefPtr &stage)
	    : mpu_{pxr::UsdGeomGetStageMetersPerUnit(stage)},
	      kpu_{pxr::UsdPhysicsGetStageKilogramsPerUnit(stage)}, spu_{1.0} {}

	double mpu() const {
		return mpu_;
	}
	double kpu() const {
		return kpu_;
	}

	auto from_m(const auto &m) const {
		return m * (1.0 / mpu_);
	}
	auto to_m(const auto &u) const {
		return u * mpu_;
	}

	auto from_kg(const auto &kg) const {
		return kg * (1.0 / kpu_);
	}
	auto to_kg(const auto &u) const {
		return u * kpu_;
	}

	auto from_s(const auto &s) const {
		return s * (1.0 / spu_);
	}
	auto to_s(const auto &u) const {
		return u * spu_;
	}

	// m*s^-1 (velocity)
	auto from_mps(const auto &v) const {
		const double f = spu_ / mpu_;
		return v * f;
	}
	auto to_mps(const auto &u) const {
		const double f = mpu_ / spu_;
		return u * f;
	}

	// s^-1 (angular velocity, or frequency)
	auto from_rps(const auto &w) const {
		return w * spu_;
	}
	auto to_rps(const auto &u) const {
		return u * (1.0 / spu_);
	}

	// m*s^-2 (acceleration)
	auto from_mps2(const auto &a) const {
		const double f = (spu_ * spu_) / mpu_;
		return a * f;
	}
	auto to_mps2(const auto &u) const {
		const double f = mpu_ / (spu_ * spu_);
		return u * f;
	}

	// s^-2 (angular acceleration)
	auto from_rps2(const auto &a) const {
		return a * (spu_ * spu_);
	}
	auto to_rps2(const auto &u) const {
		return u * (1.0 / (spu_ * spu_));
	}

	// kg*m*s^-2 (force)
	auto from_N(const auto &F) const {
		const double f = (spu_ * spu_) / (kpu_ * mpu_);
		return F * f;
	}
	auto to_N(const auto &u) const {
		const double f = (kpu_ * mpu_) / (spu_ * spu_);
		return u * f;
	}

	// kg*m^2*s^-2 (torque)
	auto from_Nm(const auto &tau) const {
		const double f = (spu_ * spu_) / (kpu_ * mpu_ * mpu_);
		return tau * f;
	}
	auto to_Nm(const auto &u) const {
		const double f = (kpu_ * mpu_ * mpu_) / (spu_ * spu_);
		return u * f;
	}

	// kg*m*s^-1 (impulse, linear momentum)
	auto from_Ns(const auto &J) const {
		const double f = spu_ / (kpu_ * mpu_);
		return J * f;
	}
	auto to_Ns(const auto &u) const {
		const double f = (kpu_ * mpu_) / spu_;
		return u * f;
	}

	// kg*m^2*s^-1 (angular impulse, angular momentum)
	auto from_Nms(const auto &L) const {
		const double f = spu_ / (kpu_ * mpu_ * mpu_);
		return L * f;
	}
	auto to_Nms(const auto &u) const {
		const double f = (kpu_ * mpu_ * mpu_) / spu_;
		return u * f;
	}

	// SE(3) transforms
	Transformd from_m(const Transformd &T) const {
		return {
		    T.first,
		    from_m(T.second),
		};
	}
	Transformd to_m(const Transformd &T) const {
		return {
		    T.first,
		    to_m(T.second),
		};
	}
};

} // namespace lego_assemble
