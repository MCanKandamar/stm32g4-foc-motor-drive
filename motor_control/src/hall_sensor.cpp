#include "hall_sensor.hpp"
#include <algorithm>
#include <cmath>

namespace motor_control
{
    HallSensor::HallSensor(const HallConfig& cfg)
    :config_(cfg)
    {
        uint32_t t     = static_cast<uint32_t>(cfg.timeout_s * cfg.f_cnt_hz);
        uint32_t limit = cfg.arr / 5 * 4;
        timeout_cnt_   = std::min(t, limit);

        width_ = calcWidths(cfg.edge_angle_rad);
    }

    // sector s is between edge[s] and edge[s+1]
    std::array<float, 6> HallSensor::calcWidths(const std::array<float, 6>& edges)
    {
        std::array<float, 6> w;
        for (int s = 0; s < 6; ++s)
        {
            w[s] = edges[(s + 1) % 6] - edges[s];
            if (w[s] < 0.0f) w[s] += TWO_PI;
        }
        return w;
    }

    bool HallSensor::init(uint8_t hall_state)
    {
        const int sector = hall_idx[hall_state & 0x7U];
        if (sector < 0) return false;                             // 000 / 111

        last_sector_ = sector;
        width_cur_   = width_[sector];
        theta_edge_  = config_.edge_angle_rad[sector] + width_cur_ / 2.0f;    // middle of the sector
        theta_       = theta_edge_;
        k_           = 0.0f;                      // angle stays fixed until an edge comes
        omega_e_     = 0.0f;
        dir_         = 0;
        stopped_     = true;
        return true;
    }

    void HallSensor::update(uint8_t hall_state, uint32_t period_cnt, uint32_t cnt)
    {
        const int sector = hall_idx[hall_state & 0x7U];
        cal_edge_ = -1;

        if (sector < 0)
        {
            if (cal_active_) cal_errors_++;                 // 000 / 111
        }
        else if (sector != last_sector_)
        {
            const int delta = (sector - last_sector_ + 6) % 6;
            last_sector_ = sector;

            if (delta == 1)
            {
                onEdge(sector, +1, period_cnt);
                cal_edge_ = sector;                         // forward: edge into s = edge[s]
            }
            else if (delta == 5)
            {
                onEdge(sector, -1, period_cnt);
                cal_edge_ = (sector + 1) % 6;               // backward: edge into s = edge[s+1]
            }
            else if (cal_active_) cal_errors_++;            // sector skipped (noise)
        }

        if (cnt > timeout_cnt_)  onTimeout();

        updateAngle(cnt);
    }

    void HallSensor::onEdge(int sector, int dir, uint32_t period_cnt)
    {
        if (period_cnt == 0) return;

        if (dir != dir_) { resetAverage(); dir_ = dir; }

        width_cur_ = width_[sector];

        if (dir > 0)
            theta_edge_ = config_.edge_angle_rad[sector];
        else
            theta_edge_ = config_.edge_angle_rad[(sector + 1) % 6];

        if (stopped_)
        {
            stopped_ = false;
            return;
        }

        period_sum_ -= period_buf_[buf_idx_];
        period_buf_[buf_idx_] = period_cnt;
        period_sum_ += period_cnt;
        buf_idx_ = (buf_idx_ + 1) % AVG_EDGES;
        if (buf_n_ < AVG_EDGES) buf_n_++;

        const float angle = buf_n_ * PI_DIV_3;   // total angle in the buffer

        omega_e_ = dir * angle * config_.f_cnt_hz / period_sum_;
        k_ = omega_e_ / config_.f_cnt_hz;
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
        theta_edge_ = config_.edge_angle_rad[last_sector_] + width_[last_sector_] / 2.0f;
        stopped_ = true;
        resetAverage();
    }

    void HallSensor::resetAverage()
    {
        period_buf_.fill(0);
        period_sum_ = 0;
        buf_idx_    = 0;
        buf_n_      = 0;
    }

    void HallSensor::startCalibration()
    {
        cal_sum_cos_.fill(0.0f);
        cal_sum_sin_.fill(0.0f);
        cal_count_.fill(0);
        cal_errors_ = 0;
        cal_active_ = true;
    }

    // Called after update(). If an edge came in this step, adds the reference angle to that edge.
    void HallSensor::recordCalibration(float theta_ref)
    {
        if (!cal_active_ || cal_edge_ < 0) return;

        cal_sum_cos_[cal_edge_] += cosf(theta_ref);
        cal_sum_sin_[cal_edge_] += sinf(theta_ref);
        cal_count_[cal_edge_]++;
    }

    bool HallSensor::finishCalibration()
    {
        cal_active_ = false;
        if (cal_errors_ > 0) return false;

        std::array<float, 6> edges;
        for (int i = 0; i < 6; ++i)
        {
            if (cal_count_[i] == 0) return false;            // this edge was never seen

            edges[i] = atan2f(cal_sum_sin_[i], cal_sum_cos_[i]);   // vector mean: 350° and 10° → 0°
            if (edges[i] < 0.0f) edges[i] += TWO_PI;
        }

        // Edges must increase in order; in reverse order (phase/Hall order swapped) the sectors come out ~300°
        const std::array<float, 6> widths = calcWidths(edges);
        for (float w : widths)
            if (w > PI) return false;

        config_.edge_angle_rad = edges;
        width_ = widths;
        return true;
    }
}
