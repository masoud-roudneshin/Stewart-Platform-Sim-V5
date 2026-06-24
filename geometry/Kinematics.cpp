#include "../geometry/Kinematics.h"
#include <iostream>

void Kinematics::compute_InverseKinematics(
    const PlatformGeometry& geom, // input
    const Pose6DoF& pose,         // input
    Vec6& leg_lengths,//output
    Mat3x6& actuators_unit_vector,//output
    Mat3x6& jointCoordinates_Platform_World) //output

{

    // Compute once — reused for all 6 legs
    Mat3 rotation_matrix;
    rotation_matrix = compute_rotation_matrix(pose.roll, pose.pitch, pose.yaw);

    Vec3 p(pose.x, pose.y, pose.z);
    Mat3x6 leg_vectors;
    jointCoordinates_Platform_World = rotation_matrix * geom.jointCoordinates_Platform;
    leg_vectors = jointCoordinates_Platform_World.colwise() + p - geom.jointCoordinates_Base;

    leg_lengths = leg_vectors.colwise().norm();
    
    actuators_unit_vector = leg_vectors.colwise().normalized();

}

///
Pose6 Kinematics::pose_to_vec(const Pose6DoF& p)
{
    Pose6 x;
    x << p.x, p.y, p.z, p.roll, p.pitch, p.yaw;

    return x;
}

Pose6DoF Kinematics::vec_to_pose(const Pose6& x)
{
    return { x(0), x(1),  x(2), x(3) ,  x(4), x(5) };

}


void Kinematics::compute_Jacobian(const Pose6DoF& pose,
    Mat3x6& actuators_unit_vector,
    Mat3x6& jointCoordinates_Platform_World,
    Mat6& Jacobian)
{

    for (size_t i = 0; i < 6; i++)
    {
        Vec3 temp = jointCoordinates_Platform_World.col(i).cross(actuators_unit_vector.col(i));
            
        Jacobian.row(i).head<3>() = actuators_unit_vector.col(i).transpose();
        Jacobian.row(i).tail<3>() = temp.transpose();

    }
}

bool Kinematics::compute_forward_kinematics(const PlatformGeometry& geom,
                                const Vec6& leg_lengths_measured,
                                Pose6DoF& pose_estimate,
                                int max_iterations,
                                real_t tolerance)
{
    Vec6 leg_lengths_estimate_FK = Vec6::Zero();
    Mat3x6 actuators_unit_vector_estimate_FK = Mat3x6::Zero();
    Mat3x6 jointCoordinates_Platform_World_estimate_FK = Mat3x6::Zero();
    Mat6 Jacobian = Mat6::Zero();
    Pose6 pose_Pose6 = Kinematics::pose_to_vec(pose_estimate);
    Pose6 delta_pose;
    Vec6 delta_L;

    for (int i = 0; i < max_iterations; i++)
    {
        Kinematics::compute_InverseKinematics(geom,
            pose_estimate,
            leg_lengths_estimate_FK,
            actuators_unit_vector_estimate_FK,
            jointCoordinates_Platform_World_estimate_FK);

        Kinematics::compute_Jacobian(pose_estimate,
                                    actuators_unit_vector_estimate_FK,
                                    jointCoordinates_Platform_World_estimate_FK,
                                    Jacobian);
        delta_L = leg_lengths_measured - leg_lengths_estimate_FK;


        if (delta_L.norm() < tolerance)
        {
            return true;

        }

        delta_pose = Jacobian.colPivHouseholderQr().solve(delta_L);

        pose_Pose6 += delta_pose;

        pose_estimate = Kinematics::vec_to_pose(pose_Pose6);

    }

    return false;
}