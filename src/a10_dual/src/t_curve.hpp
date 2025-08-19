#ifndef TCURVE_H
#define TCURVE_H

class TCurve
{
private:
    double tc_; // 总运动时间
    double tm_; // 匀速运动时间
    double v_; // 最大速度
    double a_; // 加速度
    double ta_; // 加速时间
    double p_; // 总位移
    double pa_; // 加速位移

public:
    auto getTCurve(int count) -> double;
    auto getCurveParam() -> void;
    auto getTc() -> double { return tc_; };
    TCurve(double a, double v, double p) { a_ = a; v_ = v; p_ = p; }
    ~TCurve() {}
};

#endif // TCURVE_H