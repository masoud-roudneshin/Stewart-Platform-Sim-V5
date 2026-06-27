#include "../control/JointSpaceController.h"


JointSpaceController::JointSpaceController(real_t M, real_t b, real_t wn, real_t zeta, real_t dt): 
	pd_controller_{ PDController(M, b, wn, zeta, dt),
		PDController(M, b, wn, zeta, dt),
		PDController(M, b, wn, zeta, dt),
		PDController(M, b, wn, zeta, dt),
		PDController(M, b, wn, zeta, dt),
		PDController(M, b, wn, zeta, dt) } {};

Vec6 JointSpaceController::compute(const ControlContext& control_cxt)
{
	
	Vec6 leg_forces;

	for (int i = 0; i < 6; i++)
	{
		leg_forces(i) = pd_controller_[i].compute(control_cxt.desired_strokes(i), control_cxt.actual_strokes(i), control_cxt.actual_joints_velocity(i));
	}

	return leg_forces;
}