/**
 * @file iir_filter.h
 * @brief declaration of iir filter
 * @author Haoyu Wang (qrpucp@qq.com)
 * @date 2023-02-26
 *
 * @copyright Copyright (C) 2023.
 *
 */
#pragma once

/* related header files */

/* c system header files */

/* c++ standard library header files */
#include <vector>

/* external project header files */

/* internal project header files */

/**
 * @brief discrete IIR filter, example input:
 *        order = 2; coe_a = {0.0675, 0.1349, 0.0675}; coe_b = {1, -1.1430, 0.4128}
 * @note 1) The coefficients of the discrete iir filter need to be obtained first
 *       2) If control cycle is changed, the coefficients of the filter also need to change
 * @details G(z) = (a[2]z^2 + a[1]z + a[0]) / (b[2]z^2 + b[1]z + b[0]) = Y(z) / R(z)
 *          a[2]R(n) + a[1]R(n-1) + a[0]R(n-2) = b[2]Y(n) + b[1]Y(n-1) + b[0]Y(n-2)
 *          Y(n) = (a[2]R(n) + a[1]R(n-1) + a[0]R(n-2) - b[1]Y(n-1)) - b[0]Y(n-2) / b[2]
 *          input_state/output_state: (old) [0] [1] [2] (new)
 */
class IirFilter
{
  public:
    IirFilter(int order, const std::vector<double> &coe_a, const std::vector<double> &coe_b);
    ~IirFilter() = default;
    void in(double input_now);
    [[nodiscard]] double out(void) const;
    void reset(void);
    void setInitialValue(const std::vector<double> &initial_value);

  private:
    unsigned int order_{};
    std::vector<double> coe_a_, coe_b_, input_state_, output_state_;
};

/**
 * @brief get data output of the filter
 * @return output data
 */
inline double IirFilter::out(void) const
{
    return output_state_.at(order_);
}
