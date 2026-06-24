// placeholder
#include "../threading/ThreadedFOCDriver.h"


ThreadedFOCDriver::ThreadedFOCDriver(	ActuatorSharedData& shared_data,
										ThreadedSafetyMonitor& safety,
										MotorParameters motor,
										LeadScrewParameters screw,
										real_t max_stroke,
										real_t dt,
										real_t V_supply):	driver_(motor, screw, max_stroke, dt, V_supply),
										timer_(std::chrono::duration_cast<std::chrono::microseconds>(
										std::chrono::duration<real_t>(dt))), 
										shared_(shared_data), 
										safety_(safety),
										running_(false) {}

void ThreadedFOCDriver::start()
{
	running_ = true;
	thread_ = std::thread([this]() { run(); });
}



void ThreadedFOCDriver::stop()
{
	running_ = false;
	thread_.join();
}



void ThreadedFOCDriver::run()
{
	timer_.start();
	FOCState snap_shot;
	while (running_.load())
	{
		if (safety_.is_estop())
		{
			shared_.iq_ref.store(0.0, std::memory_order_release);
			break;
		}
		real_t iq_ref = shared_.iq_ref.load();
		driver_.update(iq_ref, 0.0);
		{
			std::lock_guard<std::mutex> lock(shared_.mtx_foc);
			shared_.latest_state.force		= driver_.get_force();
			shared_.latest_state.iq			= driver_.get_iq();
			shared_.latest_state.id			= driver_.get_id();
			shared_.latest_state.stroke		= driver_.get_stroke();
			shared_.latest_state.velocity	= driver_.get_velocity();
			shared_.latest_state.timestamp  = std::chrono::steady_clock::now();
			shared_.latest_state.iteration++;
			snap_shot = shared_.latest_state;
		}
		
		shared_.log_buffer.push_overwrite(snap_shot);

		timer_.wait_until();



	}
}
