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

#include "parameter_manager.hpp"

class RtcmHandler
{
    public:
        RtcmHandler();
        RtcmHandler(const RtcmHandler&) = delete;
        RtcmHandler& operator=(const RtcmHandler&) = delete;
        ~RtcmHandler();
        void processRtcm(const uint8_t *data, size_t len, uint32_t timeout_ms);

    private:
        rtcm_t rtcm{0};
};

#endif