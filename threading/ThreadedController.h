#pragma once
#include <thread>
#include <chrono>
#include <atomic>
#include <mutex>
#include <array>
#include "../control/PDController.h"
#include "../utils/LoopTimer.h"
#include "../threading/SharedData.h"
#include "../platform/SafetyMonitor.h"
#include "../threading/ThreadedSafetyMonitor.h"
#include "../math/Math.h"
#include "../control/IController.h"
#include "../geometry/Kinematics.h"
#include "../geometry/Geometry.h"
#include "../control/StewartController.h"

class ThreadedController
{
	std::mutex								targets_mtx_;
	std::array<PDController, 6>				pd_controller_;
	ControlContext							control_cxt_;

	std::mutex								strategy_mtx_;
	StewartController						controller_;

	PlatformGeometry						geom_;
	real_t									mid_heave_;
	real_t									force_to_iq_gain_;

	LoopTimer								timer_{ std::chrono::microseconds(1000) };
	std::array<ActuatorSharedData*, 6>		shared_;
	ThreadedSafetyMonitor&					safety_;
	std::thread								thread_;
	std::atomic<bool>						running_{ false };
	

public:

	ThreadedController(std::array<ActuatorSharedData*, 6>		shared,
		ThreadedSafetyMonitor& safety,
		std::unique_ptr<IController> strategy,
		PlatformGeometry geom,
		real_t mid_heave,
		real_t force_to_iq_gain,
		real_t dt = 0.001);
	
	ThreadedController(const ThreadedController&)				= delete;
	ThreadedController& operator = (const ThreadedController&)	= delete;
	ThreadedController(ThreadedController&&)					= delete;
	ThreadedController& operator = (ThreadedController&&)		= delete;

	void start();
	void stop();

	void compute_kinematics(ControlContext& control_cxt_);

	//void set_target(const std::array<real_t,6>& target_strokes);

	void set_target(const Vec6& desired_strokes, const Pose6DoF& desired_pose);


private:
	void run();
};
