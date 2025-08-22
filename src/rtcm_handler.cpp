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

#include "rtcm_handler.hpp"


RtcmHandler::RtcmHandler()
{
}

RtcmHandler::~RtcmHandler()
{
}

// extern int input_rtcm3(rtcm_t *rtcm, unsigned char data)
void RtcmHandler::processRtcm(const uint8_t *data, size_t len, uint32_t timeout_ms)
{
    int ret = 0;
    for(size_t i = 0; i < len; i++)
    {
        unsigned char byte = data[i];
        ret = input_rtcm3(&rtcm, byte);

        if (ret==1) { /* observation data */

        }
        else if (ret==2) { /* ephemeris */

        }
        else if (ret==3) { /* sbas message */

        }
        else if (ret==9) { /* ion/utc parameters */

        }
        else if (ret==5) { /* antenna postion parameters */

        }
        else if (ret==7) { /* dgps correction */

        }
        else if (ret==10) { /* ssr message */

        }
        else if (ret==31) { /* lex message */

        }
        else if (ret==-1) { /* error */

        }
    }
}