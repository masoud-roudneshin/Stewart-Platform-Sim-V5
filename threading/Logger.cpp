// placeholder
#include "../threading/Logger.h"
#include <iostream>

Logger::Logger(std::array<ActuatorSharedData*, 6> shared, ThreadedSafetyMonitor& safety): 
	shared_(shared), safety_(safety), running_(false){}

void Logger::start()
{
	running_.store(true);
	thread_ = std::thread([this]() { run(); });
}

void Logger::stop()
{
	running_.store(false);
	thread_.join();
}

void Logger::run()
{
	while (running_.load())
	{
		bool all_empty = true;
		for (size_t i = 0; i < 6; i++)
		{
			FOCState state;
			if (shared_[i]->log_buffer.pop(state) && !safety_.is_estop())
			{
				all_empty = false;
				std::lock_guard<std::mutex> lock(print_mtx_);
				std::cout << "Actuator" << i << "\n";
				std::cout << "iteration = " << state.iteration << "\n";
				std::cout << "Force = " << state.force << "\n";
				std::cout << "stroke = " << state.stroke << "\n";
				std::cout << "velocity = " << state.velocity << "\n";
				std::cout << "iq = " << state.iq << "\n";
				std::cout << "id = " << state.id << "\n";
				
			}
		}

		if (all_empty)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
	}
}