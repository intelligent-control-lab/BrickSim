module;
#include <Eigen/Eigen>

export module lego_assemble.vendor.eigen;

// Re-export a curated Eigen API
export namespace Eigen {
using Eigen::Affine;
using Eigen::Affine2d;
using Eigen::Affine2f;
using Eigen::Affine3d;
using Eigen::Affine3f;
using Eigen::Matrix;
using Eigen::Matrix2d;
using Eigen::Matrix2f;
using Eigen::Matrix3d;
using Eigen::Matrix3f;
using Eigen::Matrix4d;
using Eigen::Matrix4f;
using Eigen::Quaternion;
using Eigen::Quaterniond;
using Eigen::Quaternionf;
using Eigen::Rotation2D;
using Eigen::Rotation2Dd;
using Eigen::Rotation2Df;
using Eigen::SimplicialLDLT;
using Eigen::SparseMatrix;
using Eigen::Success;
using Eigen::Transform;
using Eigen::Triplet;
using Eigen::Vector2d;
using Eigen::Vector2f;
using Eigen::Vector2i;
using Eigen::Vector3d;
using Eigen::Vector3f;
using Eigen::Vector3i;
using Eigen::Vector4d;
using Eigen::Vector4f;
using Eigen::Vector4i;
using Eigen::VectorXd;
using Eigen::VectorXf;
using Eigen::VectorXi;
} // namespace Eigen
