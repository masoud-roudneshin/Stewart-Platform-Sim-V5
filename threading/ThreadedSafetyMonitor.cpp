#include "../threading/ThreadedSafetyMonitor.h"

ThreadedSafetyMonitor::ThreadedSafetyMonitor(std::array <ActuatorSharedData*, 6>		shared,
	real_t max_velocity,
	real_t max_force,
	real_t max_stroke):
	shared_(shared),
	max_velocity_(max_velocity),
	max_force_(max_force),
	max_stroke_(max_stroke){}

void ThreadedSafetyMonitor::start()
{
	running_.store(true);
	thread_ = std::thread([this]() { run(); });
}

void ThreadedSafetyMonitor::stop()
{
	running_.store(false);
	thread_.join();
}

void ThreadedSafetyMonitor::estop()
{
	state_.store(PlatformState::ESTOP, std::memory_order_release);
	estop_.store(true);

	for (int i = 0; i < 6; i++)
	{
		shared_[i]->iq_ref.store(0.0, std::memory_order_release);
	}
}


void ThreadedSafetyMonitor::reset()
{
	if (state_.load() == PlatformState::READY)
	{
		return;
	}

	FOCState current_state;
	for (int i = 0; i < 6; i++)
	{
		std::lock_guard<std::mutex> lock(shared_[i]->mtx_foc);
		current_state = shared_[i]->latest_state;

		if (std::abs(current_state.force) > max_force_ ||
			std::abs(current_state.velocity) > max_velocity_ ||
			current_state.stroke < 0 || current_state.stroke > max_stroke_)
		{
			return;
		}
	}


	estop_.store(false);
	state_.store(PlatformState::READY, std::memory_order_release);
	fault_message_ = "";

}


bool ThreadedSafetyMonitor::is_operational()			const noexcept { return !estop_.load(); }
PlatformState ThreadedSafetyMonitor::get_state()		const noexcept { return state_.load(); }
std::string ThreadedSafetyMonitor::get_fault_message()	const noexcept { return fault_message_; };
bool ThreadedSafetyMonitor::is_estop() const noexcept { return  estop_.load(); }


void ThreadedSafetyMonitor::check_limits()
{
	FOCState current_state;
	for (int i = 0; i < 6; i++)
	{
		{
		std::lock_guard<std::mutex> lock(shared_[i]->mtx_foc);
		current_state = shared_[i]->latest_state;
		}

		if (std::abs(current_state.force) > max_force_ ||
			std::abs(current_state.velocity) > max_velocity_ ||
			current_state.stroke < 0 || current_state.stroke > max_stroke_)
		{
			{
				std::lock_guard<std::mutex> lock(fault_mtx_);
				fault_message_ = "[FAULT]";
			}
			
			state_.store(PlatformState::FAULT, std::memory_order_release);
			estop_.store(true);

			for (int j = 0; j < 6; j++)
			{
				shared_[j]->iq_ref.store(0.0, std::memory_order_release);
			}

			std::cerr << "[FAULT] " << fault_message_ << "\n";
			
			return;

		}
	}
}

void ThreadedSafetyMonitor::run()
{
	timer_.start();

	while (running_.load())
	{
		if (estop_.load())
		{
			for (int i = 0; i < 6; i++)
			{
				shared_[i]->iq_ref.store(0.0, std::memory_order_release);
			}
			timer_.wait_until();
			continue;
		}

		ThreadedSafetyMonitor::check_limits();
		timer_.wait_until();
	}
}

