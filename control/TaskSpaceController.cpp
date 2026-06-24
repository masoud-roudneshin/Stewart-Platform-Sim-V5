#include "../control/TaskSpaceController.h"

TaskSpaceController::TaskSpaceController(const Vec6& Kp_gains, const Vec6& Kd_gains): 
					Kp_(Kp_gains.asDiagonal()), Kd_(Kd_gains.asDiagonal()){}

Vec6 TaskSpaceController::compute(const Vec6& desired_pos, 
	const Vec6& actual_pos,
	const Vec6& actual_velocity,
	const Mat6& Jacobian)// desired_pos = [x, y, z, roll, pitch, yaw]
// actual_pos  = [x, y, z, roll, pitch, yaw]
// velocity    = [vx, vy, vz, wx, wy, wz]
// J           = 6x6 Jacobian matrix
{
	// Position Error in Workspace
	Vec3 e_pos;
	e_pos << (desired_pos(0) - actual_pos(0)), (desired_pos(1) - actual_pos(1)), (desired_pos(2) - actual_pos(2));

	// Orientation Error in Workspace

	Mat3 R_des;
	Mat3 R_act;
	Mat3 R_err;

	R_des = compute_rotation_matrix(desired_pos(3), desired_pos(4), desired_pos(5));
	R_act = compute_rotation_matrix(actual_pos(3), actual_pos(4), actual_pos(5));

	R_err = R_des * R_act.transpose();

	Vec3 e_rot;

	e_rot(0) = R_err(2, 1) - R_err(1, 2);
	e_rot(1) = R_err(0, 2) - R_err(2, 0);
	e_rot(2) = R_err(1, 0) - R_err(0, 1);
	e_rot *= 0.5;

	// Stack Errors Together

	Vec6 error_total;

	error_total.head<3>() = e_pos;
	error_total.tail<3>() = e_rot;

	// Find Control Wrench

	Vec6 W_control;
	W_control = Kp_ * error_total - Kd_ * actual_velocity;

	// Control Force on each Actuator

	Vec6 f_actuators;
	Mat6 JT = Jacobian.transpose();
	f_actuators = JT.colPivHouseholderQr().solve(W_control);

	return f_actuators;
}