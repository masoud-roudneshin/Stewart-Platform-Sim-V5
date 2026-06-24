#pragma once
#include "../geometry/Geometry.h"
#include "../math/Math.h"

class TaskSpaceController
{
	Eigen::DiagonalMatrix<real_t, 6> Kp_;
	Eigen::DiagonalMatrix<real_t, 6> Kd_;

public:

	TaskSpaceController(const Vec6& Kp_gains, const Vec6& Kd_gains);

	Vec6 compute(const Pose6DoF& actual_pos,
				const Pose6DoF& desired_pos,
				const Vec6& actual_velocity,
				const Mat6& Jacobian);
};
