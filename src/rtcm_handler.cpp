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
}


RtcmHandler::~RtcmHandler()
{
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
        obs_msg.sat       = meas->data->sat;
        size_t num        = NFREQ+NEXOBS;
        // freqs: uchar[] (code) -> vector<double> (Hz)
        obs_msg.freqs.resize(num);
        for(int j = 0; j < num; j ++)
        {
            obs_msg.freqs[j] = code_to_freq_hz(meas->data->code[j], meas->data->sat);
        }
        std::vector<unsigned char> temp_snr = array_to_vector(meas->data->SNR, num);
        obs_msg.CN0.resize(num);
        for (size_t i = 0; i < num; ++i) {
            obs_msg.CN0[i] = temp_snr[i] * 0.25f;
        }
        obs_msg.LLI     = array_to_vector(meas->data->LLI, num);
        obs_msg.code    = array_to_vector(meas->data->code, num);
        obs_msg.psr     = array_to_vector(meas->data->P, num);
        // obs_msg.psr_std = meas->data->psr_std;
        obs_msg.cp      = array_to_vector(meas->data->L, num);
        // obs_msg.cp_std  = meas->data->cp_std;
        std::vector<float> temp_dopp = array_to_vector(meas->data->D, num);
        obs_msg.dopp.resize(num);
        for (size_t i = 0; i < num; ++i) {
            obs_msg.dopp[i] = static_cast<double>(temp_dopp[i]); // float -> double
        }
        // obs_msg.dopp_std = meas->data->dopp_std;
        // obs_msg.status  = meas->data->status;

        // typedef struct {        /* observation data record */
        //     gtime_t time;       /* receiver sampling time (GPST) */
        //     unsigned char sat,rcv; /* satellite/receiver number */
        //     unsigned char SNR [NFREQ+NEXOBS]; /* signal strength (0.25 dBHz) */
        //     unsigned char LLI [NFREQ+NEXOBS]; /* loss of lock indicator */
        //     unsigned char code[NFREQ+NEXOBS]; /* code indicator (CODE_???) */
        //     double L[NFREQ+NEXOBS]; /* observation data carrier-phase (cycle) */
        //     double P[NFREQ+NEXOBS]; /* observation data pseudorange (m) */
        //     float  D[NFREQ+NEXOBS]; /* observation data doppler frequency (Hz) */
        // } obsd_t;

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

    for(size_t i = 0; i < len; i++)
    {
        unsigned char byte = data[i];
        ret = input_rtcm3(&rtcm, byte);
        obs = &rtcm.obs; /* observation data */
        nav = &rtcm.nav; /* satellite ephemerides */

        if (ret == 1) 
        { /* Observation data - New epoch */
            gnss_comm::GnssMeasMsg gnss_meas_msg;
            gnss_meas_msg = meas2msg(obs);
            pub_range_meas_.publish(gnss_meas_msg);
        }
        if (ret == 0) 
        {
            continue;
        }
        else if (ret > 1) 
        { /* ephemeris */
        }
        else if (ret == -1) 
        { /* error */

        }
    }
}

double RtcmHandler::code_to_freq_hz(unsigned char code, unsigned char sat) {
    int sys = satsys(sat, nullptr);
    int prn;
    switch (sys) {
        case SYS_GPS:
        case SYS_QZS: // QZSS shares GPS frequencies
            switch (code) {
                case CODE_L1C: case CODE_L1S: case CODE_L1L: case CODE_L1X:
                    return 1575.42e6; // L1
                case CODE_L2C: case CODE_L2D: case CODE_L2W: case CODE_L2X:
                    return 1227.60e6; // L2
                case CODE_L5Q: case CODE_L5X:
                    return 1176.45e6; // L5
                default:
                    return 0.0; // Invalid code
            }
        case SYS_GLO:
            satsys(sat, &prn); // Get frequency channel (PRN)
            switch (code) {
                case CODE_L1C: case CODE_L1P:
                    return 1602.0e6 + (prn - 1) * 0.5625e6; // G1
                case CODE_L2C: case CODE_L2P:
                    return 1246.0e6 + (prn - 1) * 0.4375e6; // G2
                default:
                    return 0.0;
            }
        case SYS_GAL:
            switch (code) {
                case CODE_L1B: case CODE_L1C:
                    return 1575.42e6; // E1
                case CODE_L5Q: case CODE_L5X:
                    return 1176.45e6; // E5a
                case CODE_L7Q: case CODE_L7X:
                    return 1207.14e6; // E5b
                default:
                    return 0.0;
            }
        case SYS_BDS:
            switch (code) {
                case CODE_L2I: case CODE_L2Q:
                    return 1561.098e6; // B1
                case CODE_L7I: case CODE_L7Q:
                    return 1207.14e6; // B2
                case CODE_L6I: case CODE_L6Q:
                    return 1268.52e6; // B3
                default:
                    return 0.0;
            }
        default:
            return 0.0; // Unknown system
    }
}