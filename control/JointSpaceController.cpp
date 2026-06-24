#include "../control/JointSpaceController.h"


JointSpaceController::JointSpaceController(real_t M, real_t b, real_t wn, real_t zeta, real_t dt): 
	pd_controller_{ PDController(M, b, wn, zeta, dt),
		PDController(M, b, wn, zeta, dt),
		PDController(M, b, wn, zeta, dt),
		PDController(M, b, wn, zeta, dt),
		PDController(M, b, wn, zeta, dt),
		PDController(M, b, wn, zeta, dt) } {};

Vec6 JointSpaceController::compute(const Vec6& desired, const Vec6& actual, const Vec6& velocity, const Mat6& J)
{
	(void)J; //suppress J, not used in joint space
	Vec6 leg_forces;

	for (int i = 0; i < 6; i++)
	{
		leg_forces(i) = pd_controller_[i].compute(desired(i), actual(i), velocity(i));
	}

	return leg_forces;
}