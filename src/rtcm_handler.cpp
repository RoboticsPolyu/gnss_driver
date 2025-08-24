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
    pub_station_ = nh_.advertise<gnss_comm::GnssAntennaPosition>("base_station", 100);
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
    sta_t *sta;
    int ret = 0;
    std::cout << "processRtcm - " << len << std::endl;

    for(size_t idx = 0; idx < len; idx++)
    {
        unsigned char byte = data[idx];
        // std::cout << byte << std::endl;

        ret = input_rtcm3(&rtcm, byte);
        obs = &rtcm.obs; /* observation data */
        nav = &rtcm.nav; /* satellite ephemerides */
        sta = &rtcm.sta;

        if (ret == 1) 
        { /* Observation data - New epoch */
            std::cout << " rtcm - Observation data - New epoch " << std::endl;
            gnss_comm::GnssMeasMsg gnss_meas_msg;
            gnss_meas_msg = meas2msg(obs);
            pub_range_meas_.publish(gnss_meas_msg);
        }
        else if (ret == 0) 
        {
            continue;
        }
        else if (ret == 2) 
        { /* ephemeris */
            std::cout << " rtcm - Ephemeris " << std::endl;
            int sys = satsys(rtcm.ephsat, NULL);
            
            if (sys == SYS_GLO) 
            {
                // GLONASS 星历
                int prn;
                satsys(rtcm.ephsat, &prn);
                if (prn >= 1 && prn <= MAXPRNGLO) 
                {
                    // 发布GLONASS星历
                    geph_t *geph = &nav->geph[prn-1];
                    if (geph->tof.time != 0) 
                    {
                        gnss_comm::GnssGloEphemMsg glo_ephem_msg = glo_eph2msg(geph);
                        pub_glo_ephem_.publish(glo_ephem_msg);
                        std::cout << "Published GLONASS ephemeris for PRN: " << prn << std::endl;
                    }
                }
            }
            else 
            {
                // GPS, Galileo, BDS 等系统的广播星历
                eph_t *eph = &nav->eph[rtcm.ephsat-1];
                if (eph->ttr.time != 0) 
                {
                    gnss_comm::GnssEphemMsg ephem_msg = eph2msg(eph);
                    pub_ephem_.publish(ephem_msg);
                    std::cout << "Published ephemeris for SAT: " << rtcm.ephsat << std::endl;
                }
            }
            
            // 检查并发布电离层参数
            if (nav->ion_gps[0] != 0.0 || nav->ion_gal[0] != 0.0 || 
                nav->ion_qzs[0] != 0.0 || nav->ion_cmp[0] != 0.0) 
            {
                gnss_comm::StampedFloat64Array iono_msg;
                iono_msg.header.stamp = ros::Time::now();
                
                // 添加GPS电离层参数（8参数）
                for (int i = 0; i < 8; i++) {
                    iono_msg.data.push_back(nav->ion_gps[i]);
                }
                
                // 可以添加其他系统的电离层参数
                pub_iono_.publish(iono_msg);
                std::cout << "Published ionosphere parameters" << std::endl;
            }
        }
        else if (ret==3) { /* sbas message */
            std::cout << " rtcm - sbas message " << std::endl;
        }
        else if (ret==9) { /* ion/utc parameters */
            std::cout << " rtcm - ion/utc parameters " << std::endl;
        }
        else if (ret==5) { /* antenna postion parameters */
            std::cout << " rtcm - Antenna postion parameters " << std::endl;
            gnss_comm::GnssAntennaPosition ant_msg;
            ant_msg.header.stamp = ros::Time::now();
            ant_msg.pos.push_back(sta->pos[0]);
            ant_msg.pos.push_back(sta->pos[1]);
            ant_msg.pos.push_back(sta->pos[2]);
            pub_station_.publish(ant_msg);
        }
        else if (ret==7) { /* dgps correction */
            std::cout << " rtcm - dgps correction " << std::endl;
        }
        else if (ret==10) { /* ssr message */
            std::cout << " rtcm - ssr message " << std::endl;
        }
        else if (ret==31) { /* lex message */
            std::cout << " rtcm - lex message " << std::endl;
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

gnss_comm::GnssEphemMsg RtcmHandler::eph2msg(const eph_t *eph)
{
    gnss_comm::GnssEphemMsg ephem_msg;
    
    int32_t week = 0;
    double tow = 0.0;
    ephem_msg.sat = eph->sat;
    tow = time2gpst(eph->ttr, &week);
    ephem_msg.ttr.week = week;
    ephem_msg.ttr.tow = tow;
    tow = time2gpst(eph->toe, &week);
    ephem_msg.toe.week = week;
    ephem_msg.toe.tow = tow;
    tow = time2gpst(eph->toc, &week);
    ephem_msg.toc.week = week;
    ephem_msg.toc.tow = tow;
    ephem_msg.toe_tow = eph->toes;
    ephem_msg.week = eph->week;
    ephem_msg.iode = eph->iode;
    ephem_msg.iodc = eph->iodc;
    ephem_msg.health = eph->svh;
    ephem_msg.code = eph->code;
    ephem_msg.ura = eph->sva;
    ephem_msg.A = eph->A;
    ephem_msg.e = eph->e;
    ephem_msg.i0 = eph->i0;
    ephem_msg.omg = eph->omg;
    ephem_msg.OMG0 = eph->OMG0;
    ephem_msg.M0 = eph->M0;
    ephem_msg.delta_n = eph->deln;
    ephem_msg.OMG_dot = eph->OMGd;
    ephem_msg.i_dot = eph->idot;
    ephem_msg.cuc = eph->cuc;
    ephem_msg.cus = eph->cus;
    ephem_msg.crc = eph->crc;
    ephem_msg.crs = eph->crs;
    ephem_msg.cic = eph->cic;
    ephem_msg.cis = eph->cis;
    ephem_msg.af0 = eph->f0;
    ephem_msg.af1 = eph->f1;
    ephem_msg.af2 = eph->f2;
    ephem_msg.tgd0 = eph->tgd[0];
    ephem_msg.tgd1 = eph->tgd[1];
    ephem_msg.A_dot = eph->Adot;
    ephem_msg.n_dot = eph->ndot;
        
    return ephem_msg;
}

gnss_comm::GnssGloEphemMsg RtcmHandler::glo_eph2msg(const geph_t *geph)
{
    gnss_comm::GnssGloEphemMsg glo_ephem_msg;
    int32_t week = 0;
    double tow = 0.0;
    glo_ephem_msg.sat = geph->sat;
    tow = time2gpst(geph->tof, &week); /* message frame time (gpst) */
    glo_ephem_msg.ttr.week = week;
    glo_ephem_msg.ttr.tow = tow;
    tow = time2gpst(geph->toe, &week); /* epoch of epherides (gpst) */
    glo_ephem_msg.toe.week = week;
    glo_ephem_msg.toe.tow = tow;
    glo_ephem_msg.freqo = geph->frq;
    glo_ephem_msg.iode = geph->iode;
    glo_ephem_msg.health = geph->svh;
    glo_ephem_msg.age = geph->age;
    glo_ephem_msg.ura = geph->sva;
    glo_ephem_msg.pos_x = geph->pos[0];
    glo_ephem_msg.pos_y = geph->pos[1];
    glo_ephem_msg.pos_z = geph->pos[2];
    glo_ephem_msg.vel_x = geph->vel[0];
    glo_ephem_msg.vel_y = geph->vel[1];
    glo_ephem_msg.vel_z = geph->vel[2];
    glo_ephem_msg.acc_x = geph->acc[0];
    glo_ephem_msg.acc_y = geph->acc[1];
    glo_ephem_msg.acc_z = geph->acc[2];
    glo_ephem_msg.tau_n = geph->taun;
    glo_ephem_msg.gamma = geph->gamn;
    glo_ephem_msg.delta_tau_n = geph->dtaun;
    return glo_ephem_msg;
}