#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <eigen3/Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

struct LidarEdgeFactor
{
	LidarEdgeFactor(Eigen::Vector2d curr_point, Eigen::Vector2d point_a, Eigen::Vector2d point_b, double s)
		: curr_point_(curr_point), point_a_(point_a), point_b_(point_b), s_(s) {}

	template <typename T>
	bool operator()(const T *const x, const T *const y, const T *const yaw, T *residual) const
	{
		Eigen::Matrix<T, 2, 1> cp{T(curr_point_.x()), T(curr_point_.y())};
		const T cos_yaw = ceres::cos(*yaw);
		const T sin_yaw = ceres::sin(*yaw);

		Eigen::Matrix<T, 2, 2> rotation;
		rotation << cos_yaw, -sin_yaw, sin_yaw, cos_yaw;
		Eigen::Matrix<T, 2, 1> translation{*x, *y};

		Eigen::Matrix<T, 2, 1> curr = rotation * cp + translation;

		// calculate the edge parameters, a, b, c
		double a, b, c;
		if (std::abs(point_a_.x() - point_b_.x()) < 1e-3)
		{
			a = 1;
			b = 0;
			c = -point_a_.x() - 0.5 * (point_b_.y() - point_a_.y());
		}
		else
		{
			a = (point_b_.y() - point_a_.y()) / (point_b_.x() - point_a_.x());
			b = -1;
			c = point_a_.y() - a * point_a_.x();
		}
		// the intersection point;
		residual[0] = (b * b * curr.x() - a * b * curr.y() - a * c) / (a * a + b * b) - curr.x();
		residual[1] = (b * b * curr.y() - a * b * curr.x() - b * c) / (a * a + b * b) - curr.y();

		return true;
	}

	static ceres::CostFunction *Create(const Eigen::Vector2d curr_point, const Eigen::Vector2d point_a,
									   const Eigen::Vector2d point_b, const double s)
	{
		return (new ceres::AutoDiffCostFunction<LidarEdgeFactor, 2, 1, 1, 1>(
			new LidarEdgeFactor(curr_point, point_a, point_b, s)));
	}

	Eigen::Vector2d curr_point_;
	Eigen::Vector2d point_a_;
	Eigen::Vector2d point_b_;
	double s_;
};

struct LidarPointFactor
{
	LidarPointFactor(Eigen::Vector2d curr_point, Eigen::Vector2d dest_point, double s)
		: curr_point_(curr_point), dest_point_(dest_point), s_(s) {}

	template <typename T>
	bool operator()(const T *const x, const T *const y, const T *const yaw, T *residual) const
	{
		Eigen::Matrix<T, 2, 1> cp{T(curr_point_.x()), T(curr_point_.y())};
		Eigen::Matrix<T, 2, 1> dp{T(dest_point_.x()), T(dest_point_.y())};
		const T cos_yaw = ceres::cos(*yaw);
		const T sin_yaw = ceres::sin(*yaw);

		Eigen::Matrix<T, 2, 2> rotation;
		rotation << cos_yaw, -sin_yaw, sin_yaw, cos_yaw;
		Eigen::Matrix<T, 2, 1> translation{*x, *y};

		Eigen::Map<Eigen::Matrix<T, 2, 1>> residuals_map(residual);
		Eigen::Matrix<T, 2, 1> curr = rotation * cp + translation;

		residuals_map = curr - dp;
		return true;
	}

	static ceres::CostFunction *Create(const Eigen::Vector2d curr_point, const Eigen::Vector2d dest_point,
									   const double s)
	{
		return (new ceres::AutoDiffCostFunction<
				LidarPointFactor, 2, 1, 1, 1>(new LidarPointFactor(curr_point, dest_point, s)));
	}

	Eigen::Vector2d curr_point_;
	Eigen::Vector2d dest_point_;
	double s_; // weight
};

// Normalizes the angle in radians between [-pi and pi).
template <typename T>
inline T NormalizeAngle(const T &angle_radians)
{
	// Use ceres::floor because it is specialized for double and Jet types.
	T two_pi(2.0 * M_PI);
	return angle_radians - two_pi * ceres::floor((angle_radians + T(M_PI)) / two_pi);
}

// Defines a local parameterization for updating the angle to be constrained in
// [-pi to pi).
class AngleLocalParameterization
{
public:
	template <typename T>
	bool operator()(const T *theta_radians, const T *delta_theta_radians, T *theta_radians_plus_delta) const
	{
		*theta_radians_plus_delta = NormalizeAngle(*theta_radians + *delta_theta_radians);

		return true;
	}

	static ceres::LocalParameterization *Create()
	{
		return (new ceres::AutoDiffLocalParameterization<AngleLocalParameterization, 1, 1>);
	}
};