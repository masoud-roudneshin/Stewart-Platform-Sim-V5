#include "../control/TaskSpaceController.h"

TaskSpaceController::TaskSpaceController(const Vec6& Kp_gains, const Vec6& Kd_gains): 
					Kp_(Kp_gains.asDiagonal()), Kd_(Kd_gains.asDiagonal()){}

Vec6 TaskSpaceController::compute(const ControlContext& control_cxt)// desired_pos = [x, y, z, roll, pitch, yaw]
// actual_pos  = [x, y, z, roll, pitch, yaw]
// velocity    = [vx, vy, vz, wx, wy, wz]
// J           = 6x6 Jacobian matrix
{
	// Position Error in Workspace
	Vec3 e_pos;
	e_pos << (control_cxt.desired_pose.x - control_cxt.actual_pose.x),
		     (control_cxt.desired_pose.y - control_cxt.actual_pose.y),
		     (control_cxt.desired_pose.z - control_cxt.actual_pose.z);

	// Orientation Error in Workspace

	Mat3 R_des;
	Mat3 R_act;
	Mat3 R_err;

	R_des = compute_rotation_matrix(control_cxt.desired_pose.roll, control_cxt.desired_pose.pitch, control_cxt.desired_pose.yaw);
	R_act = compute_rotation_matrix(control_cxt.actual_pose.roll, control_cxt.actual_pose.pitch, control_cxt.actual_pose.yaw);

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
	W_control = Kp_ * error_total - Kd_ * control_cxt.actual_taskspace_velocity;

	// Control Force on each Actuator

	Vec6 f_actuators;
	Mat6 JT = control_cxt.Jacobian.transpose();
	f_actuators = JT.colPivHouseholderQr().solve(W_control);

	return f_actuators;
}