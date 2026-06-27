#pragma once
#include "../geometry/Geometry.h"
#include "../math/Math.h"
#include "../control/IController.h"

class TaskSpaceController: public IController
{
	Eigen::DiagonalMatrix<real_t, 6> Kp_;
	Eigen::DiagonalMatrix<real_t, 6> Kd_;

public:

	TaskSpaceController(const Vec6& Kp_gains, const Vec6& Kd_gains);

	Vec6 compute(const ControlContext& control_cxt) override;
};
