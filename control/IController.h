#pragma once
#include "../math/Math.h"
#include "../geometry/Kinematics.h"

struct ControlContext
{
	// JointSpace Params
	Vec6 actual_joints_length;
	Vec6 actual_strokes;
	Vec6 actual_joints_velocity;
	Vec6 desired_joints_length;
	Vec6 desired_strokes;
	Vec6 desired_joints_velocity;

	// TaskSpace Params
	Pose6DoF actual_pose;
	Vec6 actual_taskspace_velocity;
	Pose6DoF desired_pose;
	Vec6 desired_taskspace_velocity;

	Mat6 Jacobian;

};

class IController
{
public:
	virtual Vec6 compute(const ControlContext& control_cxt) = 0;
	virtual ~IController() = default;
};
