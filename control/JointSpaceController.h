#pragma once
#include "../control/IController.h"
#include "../math/Math.h"
#include <array>
#include "../control/PDController.h"

class JointSpaceController: public IController
{
	std::array<PDController,6> pd_controller_;
public:
	JointSpaceController(real_t M, real_t b, real_t wn, real_t zeta, real_t dt);

	Vec6 compute(const Vec6& desired, const Vec6& actual, const Vec6& velocity, const Mat6& J) override;
};
