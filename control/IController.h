#pragma once
#include "../math/Math.h"
class IController
{
public:
	virtual Vec6 compute(const Vec6& desired, const Vec6& actual, const Vec6& velocity, const Mat6& J) = 0;
	virtual ~IController() = default;
};
