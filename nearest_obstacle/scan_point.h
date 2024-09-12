//
// Created by ghm on 2021/10/25.
//

#ifndef CVR_LSE_SCAN_POINT_H
#define CVR_LSE_SCAN_POINT_H

#include <Eigen/Core>
#include <utility>
#include <vector>

class ScanPoint
{
public:
	ScanPoint() = default;

	ScanPoint(Eigen::Vector3d pointIn) : point_info_lidar_(std::move(pointIn)) {};

	ScanPoint(const double xIn, const double yIn, const double zIn, const bool groundValidIn, const double groundHeightIn,
			  const double timestampIn = 0.0)
	{
		point_info_lidar_.x() = xIn;
		point_info_lidar_.y() = yIn;
		point_info_lidar_.z() = zIn;
		ground_vaild_ = groundValidIn;
		ground_height_ = groundHeightIn;
		timestamp_ = timestampIn;
	}

	~ScanPoint() = default;

	inline void SetOdomPointInfo(const Eigen::Vector3d &pointIn)
	{
		point_info_odom_ = pointIn;
	}

	inline void SetVehiclePointInfo(const Eigen::Vector3d &pointIn)
	{
		piont_info_veh_ = pointIn;
	}

	inline void SetLidarPointInfo(const Eigen::Vector3d &pointIn)
	{
		point_info_lidar_ = pointIn;
	}

	inline const Eigen::Vector3d &GetOdomPointInfo() const
	{
		return point_info_odom_;
	}

	inline const Eigen::Vector3d &GetVehiclePointInfo() const
	{
		return piont_info_veh_;
	}

	inline const Eigen::Vector3d &GetLidarPointInfo() const
	{
		return point_info_lidar_;
	}

	inline void set_x(const double xIn)
	{
		point_info_lidar_.x() = xIn;
	}

	inline void set_y(const double yIn)
	{
		point_info_lidar_.y() = yIn;
	}

	inline void set_z(const double zIn)
	{
		point_info_lidar_.z() = zIn;
	}

	inline double x() const
	{
		return point_info_lidar_.x();
	}

	inline double y() const
	{
		return point_info_lidar_.y();
	}

	inline double z() const
	{
		return point_info_lidar_.z();
	}

	inline double time() const
	{
		return timestamp_;
	}

	inline bool IsGroundValid() const
	{
		return ground_vaild_;
	}

	inline double GetHeightDistance() const
	{
		return ground_height_;
	}

	inline double GetDistance() const
	{
		return std::sqrt(point_info_lidar_.x() * point_info_lidar_.x() +
						 point_info_lidar_.y() * point_info_lidar_.y() +
						 point_info_lidar_.z() * point_info_lidar_.z());
	}

	ScanPoint(const ScanPoint &) = default;

	ScanPoint &operator=(const ScanPoint &) = default;

	ScanPoint(ScanPoint &&) = default;

	ScanPoint &operator=(ScanPoint &&) = default;

private:
	Eigen::Vector3d point_info_lidar_;

	Eigen::Vector3d piont_info_veh_;

	Eigen::Vector3d point_info_odom_;

	bool ground_vaild_ = false;

	double ground_height_ = std::numeric_limits<double>::max();

	double timestamp_;
};

using ScanRay = std::vector<ScanPoint>;
using ScanAll = std::vector<ScanRay>;
#endif // SCAN_POINT_H
