#include "../control/StewartController.h"
#include <stdexcept>

void StewartController::set_strategy(std::unique_ptr<IController> s)
{
	strategy_ = std::move(s);
}

Vec6 StewartController::compute(const Vec6& desired, const Vec6& actual, const Vec6& velocity, const Mat6& J)
{
	if (!strategy_)
	{
		throw std::runtime_error("Stewart Controller: No Control Strategy Set Yet!");
	}
	return strategy_->compute( desired, actual, velocity, J);
}