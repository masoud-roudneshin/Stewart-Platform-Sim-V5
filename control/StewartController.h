#pragma once
#include <memory>
#include "../math/Math.h"
#include "../control/IController.h"
class StewartController
{
	std::unique_ptr<IController> strategy_;
public:

	StewartController() = default;
	void set_strategy(std::unique_ptr<IController> s);

	Vec6 compute(const Vec6& desired, const Vec6& actual, const Vec6& velocity, const Mat6& J);
};