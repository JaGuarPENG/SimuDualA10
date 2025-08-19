#include "t_curve.hpp"
#include <cmath>

using namespace std;
const double PI = 3.141592653589793;

auto TCurve::getCurveParam()->void
{
    // 计算加速/减速时间 (速度/加速度)
	this->ta_ = v_ / a_; 
    // 计算加速段移动的距离 (s = 0.5*a*t²)
	this->pa_ = 0.5 * a_ * ta_ * ta_;
    // 计算匀速阶段时间 (总距离 - 加减速段距离) / 速度
	this->tm_ = (p_ - 2 * pa_) / v_;
    // 计算总时间 (匀速时间 + 加速时间 + 减速时间)
	this->tc_ = tm_ + 2 * ta_;
}

auto TCurve::getTCurve(int count)->double
{
	int t = count + 1;
	double s = 0;
    // 检查是否在总运动时间内 
	if (t < tc_ * 2000 + 1)
	{
        // 情况1: 存在匀速段
		if (tc_ - 2 * ta_ > 0)
		{   
            // 阶段1: 加速段
			if (t < ta_ * 2000 + 1)
			{
				s = 0.5 * a_ * (t / 2000.0) * (t / 2000.0);
			}
            // 阶段2: 匀速段
			else if (ta_ * 2000 < t && t < tc_ * 2000 - ta_ * 2000 + 1)
			{
				s = 0.5 * a_ * ta_ * ta_ + a_ * ta_ * (t / 2000.0 - ta_);
			}
            // 阶段3: 减速段
			else
			{
				s = p_ - 0.5 * a_ * (tc_ - t / 2000.0) * (tc_ - t / 2000.0);
			}
		}
        // 情况2: 无匀速段 (三角形曲线, 加速完直接减速)
		else
		{
            // 重新计算加速/减速时间 (根据总距离和加速度)
			ta_ = sqrt(p_ / a_);
			tc_ = 2 * ta_;
			if (t < ta_ * 2000 + 1)
			{
				s = 0.5 * a_ * (t / 2000.0) * (t / 2000.0);
			}
			else
			{
				s = p_ - 0.5 * a_ * (tc_ - t / 2000.0) * (tc_ - t / 2000.0);
			}
		}
        // 返回当前count位置增量
		return s;
	}
	else
	{   // 返回总位置增量
		return p_;
	}

}