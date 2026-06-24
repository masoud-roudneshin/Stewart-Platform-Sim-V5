#include "../threading/ThreadedController.h"

ThreadedController::ThreadedController(
	std::array<ActuatorSharedData*, 6> shared,
	ThreadedSafetyMonitor&			safety,
	real_t wn,
	real_t zeta,
	real_t dt,
	real_t force_to_iq_gain,
	real_t moving_mass,
	real_t viscous_friction)
	: shared_(shared),
	safety_(safety)
	, timer_(std::chrono::duration_cast<std::chrono::microseconds>(
		std::chrono::duration<real_t>(dt)))
	, running_(false)
	, force_to_iq_gain_(force_to_iq_gain)
{

	for (int i = 0; i < 6; i++)
		pd_controller_[i] = PDController(moving_mass, viscous_friction, wn, zeta, dt);
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



void ThreadedController::set_target(const Vec6& target_strokes)
{
	std::lock_guard<std::mutex> lock(targets_mtx_); // To prevent data racing when target is called from outside
	target_strokes_ = target_strokes;
}

void ThreadedController::run()
{
	timer_.start();

	while (running_.load())
	{
		if (safety_.is_estop())
		{

			timer_.wait_until();
			if (!running_.load()) break;
			continue;
		}

		Vec6 current_targets;

		{
			std::lock_guard<std::mutex> lock(targets_mtx_);
			current_targets = target_strokes_;
		}

		for (size_t i = 0; i < 6; i++)
		{
			real_t stroke_i, velocity_i;

			{
				std::lock_guard<std::mutex> lock(shared_[i]->mtx_foc);
				stroke_i = shared_[i]->latest_state.stroke;
				velocity_i = shared_[i]->latest_state.velocity;
			}

			real_t F_i = pd_controller_[i].compute(current_targets[i], stroke_i, velocity_i);

			real_t iq_ref_i = F_i * force_to_iq_gain_;
			shared_[i]->iq_ref.store(iq_ref_i, std::memory_order_release);
		}

		timer_.wait_until();
		
	}
}