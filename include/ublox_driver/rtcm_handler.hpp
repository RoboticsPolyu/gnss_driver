/**
* This file is part of ublox-driver.
*
* Copyright (C) 2021 Aerial Robotics Group, Hong Kong University of Science and Technology
* Author: CAO Shaozu (shaozu.cao@gmail.com)
*
* ublox-driver is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ublox-driver is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ublox-driver. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef RTCM_HANDLER_HPP_
#define RTCM_HANDLER_HPP_

#include <iostream>
#include <vector>

#include "RTKLIB/src/rtklib.h"
#ifdef R2D
#undef R2D
#endif
#ifdef D2R
#undef D2R
#endif

#include <ros/ros.h>
#include <gnss_comm/gnss_constant.hpp>
#include <gnss_comm/gnss_utility.hpp>
#include <gnss_comm/gnss_ros.hpp>

#include "parameter_manager.hpp"

class RtcmHandler
{
    public:
        RtcmHandler(ros::NodeHandle& nh);
        RtcmHandler(const RtcmHandler&) = delete;
        RtcmHandler& operator=(const RtcmHandler&) = delete;
        ~RtcmHandler();
        void processRtcm(const uint8_t *data, size_t len, uint32_t timeout_ms);

    private:
        gnss_comm::GnssMeasMsg meas2msg(const obs_t *meas);

        template <typename T>
        std::vector<T> array_to_vector(const T* arr, size_t size) 
        {
            return std::vector<T>(arr, arr + size);
        }

        double code_to_freq_hz(unsigned char code, unsigned char sat);

        template <typename T_out, typename T_in>
        std::vector<T_out> array_to_vector_with_conversion(const T_in* arr, size_t size, T_out (*convert)(T_in)) {
            std::vector<T_out> result(size);
            std::transform(arr, arr + size, result.begin(), convert);
            return result;
        }

        ros::NodeHandle nh_;
        ros::Publisher pub_range_meas_, pub_ephem_, pub_glo_ephem_, pub_iono_;

        rtcm_t rtcm{0};
};

#endif