#include "../threading/ThreadedController.h"

ThreadedController::ThreadedController(
	std::array<ActuatorSharedData*, 6> shared,
	ThreadedSafetyMonitor& safety,
	std::unique_ptr<IController> strategy,
	PlatformGeometry geom,
	real_t mid_heave,
	real_t force_to_iq_gain,
	real_t body_length,
	real_t dt)
	: shared_(shared),
	safety_(safety)
	, timer_(std::chrono::duration_cast<std::chrono::microseconds>(
		std::chrono::duration<real_t>(dt)))
	, geom_(geom)
	, mid_heave_(mid_heave)
	, force_to_iq_gain_(force_to_iq_gain)
	, body_length_(body_length)
	, running_(false)
	
{
	controller_.set_strategy(std::move(strategy));
	control_cxt_.actual_pose.z = mid_heave_;
	control_cxt_.actual_joints_length = Vec6::Constant(body_length_);  // all legs at body_length
}


void ThreadedController::start()
{
	running_.store(true);
	thread_ = std::thread([this]() {run(); });
}

void ThreadedController::stop()
{
	running_.store(false);
	thread_.join();
}

void ThreadedController::compute_kinematics(ControlContext& control_cxt_)
{
	// This is acting as an estimator/processing node for a real system
	Kinematics::compute_forward_kinematics(geom_, control_cxt_.actual_joints_length, control_cxt_.actual_pose);

	Mat3x6 unit_vectors;
	Mat3x6 platform_joints_world;
	Vec6 ik_lengths;
	Kinematics::compute_InverseKinematics(geom_, control_cxt_.actual_pose,
		ik_lengths, unit_vectors, platform_joints_world);

	
	Kinematics::compute_Jacobian(control_cxt_.actual_pose,
		unit_vectors, platform_joints_world, control_cxt_.Jacobian);

	control_cxt_.actual_taskspace_velocity =
		control_cxt_.Jacobian.colPivHouseholderQr().solve(control_cxt_.actual_joints_velocity);

}

void ThreadedController::set_target(const Vec6& desired_strokes, const Pose6DoF& desired_pose)
{
	std::lock_guard<std::mutex> lock(targets_mtx_); // To prevent data racing when target is called from outside
	
	for (int i = 0; i < 6; i++)
	{
		control_cxt_.desired_strokes(i) = desired_strokes(i);
	}

	control_cxt_.desired_pose.x = desired_pose.x;
	control_cxt_.desired_pose.y = desired_pose.y;
	control_cxt_.desired_pose.z = desired_pose.z;
	control_cxt_.desired_pose.roll = desired_pose.roll;
	control_cxt_.desired_pose.pitch = desired_pose.pitch;
	control_cxt_.desired_pose.yaw = desired_pose.yaw;

}

void ThreadedController::run()
{
	timer_.start();
	Vec6 leg_forces;

	while (running_.load())
	{
		if (safety_.is_estop())
		{

			timer_.wait_until();
			if (!running_.load()) break;
			continue;
		}


		for (size_t i = 0; i < 6; i++)
		{
			{
				std::lock_guard<std::mutex> lock(shared_[i]->mtx_foc);
				control_cxt_.actual_strokes(i) = shared_[i]->latest_state.stroke;
				control_cxt_.actual_joints_velocity(i) = shared_[i]->latest_state.velocity;
				control_cxt_.actual_joints_length(i) = shared_[i]->latest_state.stroke + body_length_;
			}
			
		}

		ThreadedController::compute_kinematics(control_cxt_);
		
		leg_forces = controller_.compute(control_cxt_);

		// Multiply by force_to_iq_gain_
		leg_forces *= force_to_iq_gain_;
		// real_t iq_ref_i = F_i * force_to_iq_gain_;
		for (int i = 0; i < 6; i++)
		{
			shared_[i]->iq_ref.store(leg_forces(i), std::memory_order_release);
		}

		iteration_++;
		if (iteration_ % 100 == 0)  // every 1 second (1kHz * 1000)
		{
			std::cout << "t=" << iteration_ << "ms"
				<< "  z=" << std::fixed << std::setprecision(4)
				<< control_cxt_.actual_pose.z - mid_heave_
				<< "  roll=" << control_cxt_.actual_pose.roll
				<< "  forces: " << leg_forces.transpose() / force_to_iq_gain_
				<< "\n" << std::flush;
		}

		timer_.wait_until();
		
	}
}