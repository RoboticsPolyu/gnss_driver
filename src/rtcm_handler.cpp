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


RtcmHandler::RtcmHandler(ros::NodeHandle& nh) : 
    nh_(nh)
{
    pub_range_meas_ = nh_.advertise<gnss_comm::GnssMeasMsg>("base_meas", 100);
    pub_ephem_ = nh_.advertise<gnss_comm::GnssEphemMsg>("base_ephem", 100);
    pub_glo_ephem_ = nh_.advertise<gnss_comm::GnssGloEphemMsg>("base_glo_ephem", 100);
    pub_iono_ = nh_.advertise<gnss_comm::StampedFloat64Array>("base_iono_params", 100);

    init_rtcm(&rtcm);
}


RtcmHandler::~RtcmHandler()
{
    free_rtcm(&rtcm);
}


gnss_comm::GnssMeasMsg RtcmHandler::meas2msg(const obs_t *meas)
{
    gnss_comm::GnssMeasMsg gnss_meas_msg;
    for(int i = 0; i < meas->n; i++)
    {
        gnss_comm::GnssObsMsg obs_msg;
        int week = 0;
        double tow = time2gpst(meas->data->time, &week);
        obs_msg.time.week = week;
        obs_msg.time.tow  = tow;
        obs_msg.sat       = static_cast<uint32_t>(meas->data[i].sat); 
        size_t num        = NFREQ+NEXOBS;
       
        for(int j = 0; j < num; j++)
        {
            // Check if this frequency has valid data
            if (meas->data[i].code[j] != CODE_NONE) {
                obs_msg.freqs.push_back(code_to_freq_hz(meas->data[i].code[j], meas->data[i].sat));
                obs_msg.CN0.push_back(meas->data[i].SNR[j] * 0.25f);
                obs_msg.LLI.push_back(meas->data[i].LLI[j]);
                obs_msg.code.push_back(meas->data[i].code[j]);
                obs_msg.psr.push_back(meas->data[i].P[j]);
                obs_msg.cp.push_back(meas->data[i].L[j]);
                obs_msg.dopp.push_back(static_cast<double>(meas->data[i].D[j]));
            }
        }

        gnss_meas_msg.meas.push_back(obs_msg);
    }
    return gnss_meas_msg;
}

// extern int input_rtcm3(rtcm_t *rtcm, unsigned char data)
void RtcmHandler::processRtcm(const uint8_t *data, size_t len, uint32_t timeout_ms)
{
    obs_t *obs;
    nav_t *nav;
    int ret = 0;
    std::cout << "processRtcm - " << len << std::endl;

    for(size_t idx = 0; idx < len; idx++)
    {
        unsigned char byte = data[idx];
        // std::cout << byte << std::endl;

        ret = input_rtcm3(&rtcm, byte);
        obs = &rtcm.obs; /* observation data */
        nav = &rtcm.nav; /* satellite ephemerides */

        if (ret == 1) 
        { /* Observation data - New epoch */
            std::cout << " Observation data - New epoch " << std::endl;
            gnss_comm::GnssMeasMsg gnss_meas_msg;
            gnss_meas_msg = meas2msg(obs);
            pub_range_meas_.publish(gnss_meas_msg);
        }
        else if (ret == 0) 
        {
            continue;
        }
        else if (ret > 1) 
        { /* ephemeris */
        }
        else if (ret == -1) 
        { /* error */
            init_rtcm(&rtcm);
        }
        
    }
}

double RtcmHandler::code_to_freq_hz(unsigned char code, unsigned char sat) {
    int sys = satsys(sat, nullptr);
    int prn;
    
    switch (sys) {
        case SYS_GPS:
            switch (code) {
                case CODE_L1C: case CODE_L1P: case CODE_L1W: case CODE_L1Y:
                case CODE_L1S: case CODE_L1L: case CODE_L1X: case CODE_L1N:
                    return FREQ1; // 1575.42e6
                case CODE_L2C: case CODE_L2D: case CODE_L2S: case CODE_L2L:
                case CODE_L2X: case CODE_L2P: case CODE_L2W: case CODE_L2Y:
                case CODE_L2N: case CODE_L2M:
                    return FREQ2; // 1227.60e6
                case CODE_L5I: case CODE_L5Q: case CODE_L5X:
                    return FREQ5; // 1176.45e6
                default:
                    return 0.0;
            }

        case SYS_GLO:
            satsys(sat, &prn);
            switch (code) {
                case CODE_L1C: case CODE_L1P:
                    return FREQ1_GLO + (prn-1)*DFRQ1_GLO;
                case CODE_L2C: case CODE_L2P:
                    return FREQ2_GLO + (prn-1)*DFRQ2_GLO;
                case CODE_L3I: case CODE_L3Q: case CODE_L3X:
                    return FREQ3_GLO;
                default:
                    return 0.0;
            }

        case SYS_GAL:
            switch (code) {
                case CODE_L1A: case CODE_L1B: case CODE_L1C: case CODE_L1X:
                case CODE_L1Z:
                    return FREQ1; // 1575.42e6
                case CODE_L5I: case CODE_L5Q: case CODE_L5X:
                    return FREQ5; // 1176.45e6
                case CODE_L7I: case CODE_L7Q: case CODE_L7X:
                    return FREQ7; // 1207.14e6
                case CODE_L8I: case CODE_L8Q: case CODE_L8X:
                    return FREQ8; // 1191.795e6
                default:
                    return 0.0;
            }

        case SYS_BDS:
            switch (code) {
                case CODE_L1I: case CODE_L2I: case CODE_L2Q: case CODE_L2X: // B1
                    return FREQ1_BDS; // 1561.098e6
                case CODE_L7I: case CODE_L7Q: case CODE_L7X: // B2
                    return FREQ2_BDS; // 1207.14e6
                case CODE_L6I: case CODE_L6Q: case CODE_L6X: // B3
                    return FREQ3_BDS; // 1268.52e6
                default:
                    return 0.0;
            }

        default:
            return 0.0;
    }
}