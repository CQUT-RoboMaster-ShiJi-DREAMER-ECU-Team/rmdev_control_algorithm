/**
 * @file Pid.cppm
 * @module rmdev.control_algorithm.Pid
 * @author 杜以成
 * @date 2025-06-15
 * @brief PID
 */

module;

#include <cstdint>
#include <cmath>

#include "emdevif/concepts.hpp"

export module rmdev.controlAlgorithm.pid;

import rmdev.util.math;

export namespace rmdev {

using emdevif::ArithmeticType;

/**
 * Pid 增益
 */
enum PidImprove : std::uint8_t {
    PID_NONE = (0x00U),                             ///< 无增益
    PID_DERIVATIVE_ON_MEASUREMENT = (0X01U << 0U),  ///< 微分先行
    PID_TRAPEZOID_INTEGRAL = (0X01U << 1U),         ///< 梯形积分
    PID_CHANGING_INTEGRAL = (0X01U << 2U),          ///< 变速积分
    PID_PROPORTION_LIMIT = (0x01U << 3U)            ///< 比例限幅
};

/**
 * PID 控制类
 * @tparam Type 数据类型
 */
template<ArithmeticType Type>
class Pid
{
public:
    using ScaleType = Type;

    struct Kpid {
        ScaleType kp;
        ScaleType ki;
        ScaleType kd;
    };

    Pid() = delete;

    /**
     * 构造函数
     * @param kpid PID 参数
     * @param output_limit 输出限幅
     * @param integral_limit 积分限幅
     * @param improve 增益
     * @param deadband 死区（默认为零）
     * @param scalar_a 变速积分参数 a（默认为零）
     * @param scalar_b 变速积分参数 b（默认为零）
     */
    constexpr Pid(Kpid kpid,
                  ScaleType output_limit,
                  ScaleType integral_limit,
                  const PidImprove improve = PID_NONE,
                  ScaleType deadband = 0,
                  ScaleType scalar_a = 0,
                  ScaleType scalar_b = 0) noexcept
        : improve_(improve),
          kpid_(kpid),
          output_limit_(output_limit),
          integral_limit_(integral_limit),
          deadband_(deadband),
          ScalarA(scalar_a),
          ScalarB(scalar_b)
    {
    }

    /**
     * PID 计算
     * @param target 目标值
     * @param measure 测量值（实际值）
     * @param delta_time 间隔时间（默认为 1）
     * @return 计算结果
     */
    [[nodiscard]] ScaleType calc(ScaleType target, ScaleType measure, ScaleType delta_time = 1);

    /**
     * PID 计算
     * @param target 目标值
     * @param measure 测量值（实际值）
     * @param delta_time 间隔时间（默认为 1）
     * @return 计算结果
     */
    [[nodiscard]] ScaleType operator()(ScaleType target, ScaleType measure, ScaleType delta_time = 1)
    {
        return calc(target, measure, delta_time);
    }

    /**
     * 获得计算结果输出值
     * @return 计算结果输出值
     */
    [[nodiscard]] ScaleType getOutput() const
    {
        return output;
    }

    /**
     * 获得计算结果输出值
     * @return 计算结果输出值
     */
    [[nodiscard]] ScaleType operator()() const
    {
        return getOutput();
    }

    void setKp(ScaleType kp)
    {
        kpid_.kp = kp;
    }
    [[nodiscard]] ScaleType getKp() const
    {
        return kpid_.kp;
    }

    void setKi(ScaleType ki)
    {
        kpid_.ki = ki;
    }
    [[nodiscard]] ScaleType getKi() const
    {
        return kpid_.ki;
    }

    void setKd(ScaleType kd)
    {
        kpid_.kd = kd;
    }
    [[nodiscard]] ScaleType getKd() const
    {
        return kpid_.kd;
    }

private:
    /**
     * 梯形积分
     * @param dt 时间间隔
     */
    void f_Trapezoid_Integral(const ScaleType dt)
    {
        iterm = kpid_.ki * ((err + last_err) * dt / ScaleType(2));
    }

    /**
     * 变速积分
     */
    void f_Changing_Integral_Rate()
    {
        if (err * iout > 0) {
            // Integral still increasing
            if (std::abs(err) <= ScalarB) {
                return;  // Full integral
            }

            if (std::abs(err) <= ScalarA + ScalarB) {
                iterm *= (ScalarA - std::abs(err) + ScalarB) / ScalarA;
            }
            else {
                iterm = 0;
            }
        }
    }

    /**
     * 积分限幅
     */
    void f_Integral_Limit()
    {
        ScaleType temp_Output, temp_Iout;
        temp_Iout = iout + iterm;
        temp_Output = pout + iout + dout;
        if (std::abs(temp_Output) > output_limit_) {
            if (err * iout > 0) {
                // Integral still increasing
                iterm = 0;
            }
        }

        if (temp_Iout > integral_limit_) {
            iterm = 0;
            iout = integral_limit_;
        }
        if (temp_Iout < -integral_limit_) {
            iterm = 0;
            iout = -integral_limit_;
        }
    }

    /**
     * “微分先行”
     * @param dt 时间间隔
     */
    void f_Derivative_On_Measurement(const ScaleType dt)
    {
        dout = kpid_.kd * (last_measure - measure_) / dt;
    }

    /**
     * 输出限幅
     */
    void f_Output_Limit()
    {
        limitInRange(output, output_limit_);
    }

    /**
     * 比例限幅
     */
    void f_Proportion_Limit()
    {
        // Proportion limit is insignificant in control process,
        // but it makes variable chart look better
        if (pout > output_limit_) {
            pout = output_limit_;
        }
        if (pout < -(output_limit_)) {
            pout = -(output_limit_);
        }
    }

private:
    PidImprove improve_{};

    Kpid kpid_{};

    ScaleType output_limit_{};
    ScaleType integral_limit_{};

    ScaleType deadband_{};     ///< 死区值

    ScaleType target_{};       ///< 目标值

    ScaleType measure_{};      ///< 测量值
    ScaleType last_measure{};  ///< 上一次的测量值

    ScaleType err{};           ///< 误差值
    ScaleType last_err{};      ///< 上一次的误差

    ScaleType pout{};
    ScaleType iout{};
    ScaleType dout{};
    ScaleType iterm{};

    ScaleType output{};   ///< 输出值

    ScaleType ScalarA{};  // For Changing Integral
    ScaleType ScalarB{};  // iterm = err*((A-abs(err)+B)/A)  when B<|err|<A+B
};

template<ArithmeticType Type>
typename Pid<Type>::ScaleType Pid<Type>::calc(ScaleType target, ScaleType measure, ScaleType delta_time)
{
    measure_ = measure;
    target_ = target;
    err = target_ - measure_;

    // PID 计算设有死区限制，即当误差小于死区值时，认为已到达目标，将误差清零
    if (std::abs(err) < deadband_) {
        err = 0.0f;
    }

    // 开始 PID 计算

    pout = kpid_.kp * err;

    // 比例限幅
    if (improve_ & PID_PROPORTION_LIMIT) {
        f_Proportion_Limit();
    }

    // 梯形积分
    if (improve_ & PID_TRAPEZOID_INTEGRAL) {
        f_Trapezoid_Integral(delta_time);
    }
    else {
        // 常规积分
        iterm = kpid_.ki * err * delta_time;
    }

    // 变速积分
    if (improve_ & PID_CHANGING_INTEGRAL) {
        f_Changing_Integral_Rate();
    }
    // 积分限幅
    f_Integral_Limit();

    // “微分先行”
    if (improve_ & PID_DERIVATIVE_ON_MEASUREMENT) {
        f_Derivative_On_Measurement(delta_time);
    }
    else {
        // 常规微分（求导）
        dout = kpid_.kd * (err - last_err) / delta_time;
    }

    iout += iterm;

    output = pout + iout + dout;

    // 输出限幅
    f_Output_Limit();

    last_measure = measure_;
    last_err = err;

    return output;
}

}  // namespace rmdev
