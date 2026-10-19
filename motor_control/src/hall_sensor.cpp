#include "hall_sensor.hpp"
#include "motor_parameters.hpp"
#include <algorithm>

namespace motor_control
{
    HallSensor::HallSensor(const HallConfig& cfg) 
    :config(cfg)
    {
        uint32_t t     = static_cast<uint32_t>(cfg.timeout_s * cfg.f_cnt_hz);
        uint32_t limit = cfg.arr / 5 * 4;
        timeout_cnt_   = std::min(t, limit);

        // s numaralı dilim edge[s] ile edge[s+1] arasında
        for (int s = 0; s < 6; ++s)
        {
            float w = cfg.edge_angle_rad[(s + 1) % 6] - cfg.edge_angle_rad[s];
            if (w < 0.0f) w += TWO_PI;
            width_[s] = w;
        }
    }

    bool HallSensor::HallSensorInit(uint8_t hall_state)
    {
    SectorInfo.sector = hall_idx[hall_state & 0x7U];
    if (SectorInfo.sector < 0) return false;                  // 000 / 111: sensor error

    sector_      = SectorInfo.sector;                         // decodeHall ilk kenarda doğru delta bulsun
    width_cur_   = width_[SectorInfo.sector];
    theta_edge_  = config.edge_angle_rad[SectorInfo.sector] + width_cur_ / 2.0f;    // sektör ortası
    theta_       = theta_edge_;
    k_           = 0.0f;                      // kenar gelene kadar açı sabit
    omega_e_     = 0.0f;
    SectorInfo.dir = 0;
    stoppedFlag  = true;                      // ilk kenarda CCR1 anlamsız
    return true;
    }

    HallSensor::SectorInfo_ HallSensor::decodeHall(uint8_t hall_state)
    {   
        SectorInfo.sector = hall_idx[hall_state & 0x7U];
        
        if (SectorInfo.sector < 0)
        {
            SectorInfo.valid = false;
            return SectorInfo;
        }

        int delta = ((SectorInfo.sector - sector_) + 6) % 6;    // (diff + 6) % 6

        switch (delta)
        {
        case 0:
            SectorInfo.valid = false;
            break;
        case 1:
            SectorInfo.dir = 1;     // CW direction 
            sector_ = SectorInfo.sector;
            SectorInfo.valid = true;
            break;
        case 5:
            SectorInfo.dir = -1;    // CCW direction
            sector_ = SectorInfo.sector;
            SectorInfo.valid = true;
            break;
        default:
            sector_ = SectorInfo.sector;
            SectorInfo.valid = false;
            break;
        }
        return SectorInfo;
    }

    void HallSensor::onEdge(const SectorInfo_& SectorInfo, uint32_t period_cnt)
    {
        if (SectorInfo.valid == false || period_cnt == 0) return;

        if (SectorInfo.dir != last_dir_) { resetAverage(); last_dir_ = SectorInfo.dir; }

        const int s = SectorInfo.sector;
        width_cur_ = width_[s];

        if (SectorInfo.dir > 0)
            theta_edge_ = config.edge_angle_rad[s];
        else
            theta_edge_ = config.edge_angle_rad[(s + 1) % 6];

        if (stoppedFlag)
        {
            stoppedFlag = false;
            return;
        }

        period_sum_ -= period_buf_[buf_idx_];
        period_buf_[buf_idx_] = period_cnt;
        period_sum_ += period_cnt;
        buf_idx_ = (buf_idx_ + 1) % 24;
        if (buf_n_ < 24) buf_n_++;

        const float angle = buf_n_ * PI_DIV_3;

        omega_e_ = SectorInfo.dir * angle * config.f_cnt_hz / period_sum_;
        k_ = omega_e_ / config.f_cnt_hz;
    }

    void HallSensor::updateAngle(uint32_t cnt)
    {
        float dtheta = k_ * cnt;
        dtheta = std::clamp(dtheta, -width_cur_, width_cur_);
        theta_ = theta_edge_ + dtheta;

        if (theta_ >= TWO_PI)   theta_ -= TWO_PI;
        else if (theta_ < 0.0f) theta_ += TWO_PI;
    }

    void HallSensor::onTimeout()
    {
        omega_e_ = 0.0f;
        k_ = 0.0f;
        theta_edge_ = config.edge_angle_rad[sector_] + width_[sector_] / 2.0f;
        stoppedFlag = true;
        resetAverage();
    }

    void HallSensor::update(uint8_t hall_state, uint32_t period_cnt, uint32_t cnt)
    {
        onEdge(decodeHall(hall_state), period_cnt);

        if (cnt > timeout_cnt_)  onTimeout();

        updateAngle(cnt);
    }

    void HallSensor::resetAverage()
    {
        period_buf_.fill(0);
        period_sum_ = 0;
        buf_idx_    = 0;
        buf_n_      = 0;
    }
}
