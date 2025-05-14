/**
 * @file iir_filter.cpp
 * @brief implement of iir filter
 * @author Haoyu Wang (qrpucp@qq.com)
 * @date 2023-02-26
 *
 * @copyright Copyright (C) 2023.
 *
 */
/* related header files */
#include "iir_filter.h"

/* c system header files */

/* c++ standard library header files */

/* external project header files */
#include <ros/ros.h>

/* internal project header files */

/**
 * @brief Construct a new Iir Filter:: Iir Filter object
 * @param[in] order order of the iir filter
 * @param[in] coe_a numerator coefficients of the filter
 * @param[in] coe_b denominator coefficients of the filter
 */
IirFilter::IirFilter(int order, const std::vector<double> &coe_a, const std::vector<double> &coe_b)
    : order_(order), coe_a_(coe_a), coe_b_(coe_b), input_state_(order + 1, 0),
      output_state_(order + 1, 0)
{
    ROS_ASSERT(order > 0);
    ROS_ASSERT(coe_a.size() == order_ + 1);
    ROS_ASSERT(coe_b.size() == order_ + 1);

    // To be consistent with convention, the input coefficients are reversed
    std::reverse(coe_a_.begin(), coe_a_.end());
    std::reverse(coe_b_.begin(), coe_b_.end());
}

/**
 * @brief input data into the filter
 * @param[in] input_now input data
 */
void IirFilter::in(double input_now)
{
    input_state_.erase(input_state_.begin());
    input_state_.emplace_back(input_now);
    output_state_.erase(output_state_.begin());
    double output_now = coe_a_.at(order_) * input_state_.at(order_);
    for (unsigned int i = 0; i < order_; ++i)
    {
        output_now += coe_a_.at(i) * input_state_.at(i);
        output_now -= coe_b_.at(i) * output_state_.at(i);
    }
    output_now /= coe_b_.at(order_);
    output_state_.emplace_back(output_now);
}

/**
 * @brief reset the filter, clear the history states of the filter
 */
void IirFilter::reset(void)
{
    for (auto &state : input_state_)
        state = 0;
    for (auto &state : output_state_)
        state = 0;
}

/**
 * @brief set initial value of the filter
 * @param[in] initial_value initial value
 */
void IirFilter::setInitialValue(const std::vector<double> &initial_value)
{
    ROS_ASSERT(initial_value.size() == order_ + 1);
    for (auto &value : initial_value)
    {
        in(value);
    }
}
