#include "../control/StewartController.h"
#include <stdexcept>

void StewartController::set_strategy(std::unique_ptr<IController> s)
{
	strategy_ = std::move(s);
}

Vec6 StewartController::compute(const ControlContext& control_cxt)
{
	if (!strategy_)
	{
		throw std::runtime_error("Stewart Controller: No Control Strategy Set Yet!");
	}
	return strategy_->compute(control_cxt);
}