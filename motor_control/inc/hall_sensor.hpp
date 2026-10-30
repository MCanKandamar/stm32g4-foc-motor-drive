#pragma once
#include "math_utils.hpp"
#include <array>
#include <cstdint>

namespace motor_control
{
    // sequence motor: 1 -> 5 -> 4 -> 6 -> 2 -> 3. 000 / 111 invalid (-1)
    static constexpr int hall_idx[8] = {-1, 0, 4, 5, 2, 1, 3, -1};

    struct HallConfig
    {
        float f_cnt_hz;
        int pole_pairs;
        std::array<float, 6> edge_angle_rad = { 0.0f, PI_DIV_3, 2.0f * PI_DIV_3,
                                                PI, 4.0f * PI_DIV_3, 5.0f * PI_DIV_3 };
        float timeout_s = 0.1f;
        uint32_t arr = 65535;
    };


    class HallSensor
    {
        public:
            HallSensor(const HallConfig& cfg);

            bool init(uint8_t hall_state);

            void update(uint8_t hall_state, uint32_t period_cnt, uint32_t cnt);

            float getMechanicalSpeedRpm() const    { return omega_e_ / config_.pole_pairs * RAD_TO_RPM; }
            float getElectricalAngleRad() const    { return theta_; }
            float getElectricalSpeedRads() const   { return omega_e_; }
            int   getDirection() const             { return dir_; }

            // Edge calibration: while the rotor is turned to a known angle (theta_ref),
            // the angle of each edge is recorded; at the end the mean goes into the table.
            void startCalibration();
            void recordCalibration(float theta_ref);    // after every update()
            bool finishCalibration();                   // true: table updated

        private:
            static constexpr int AVG_EDGES = 24;  // 4 pole pairs × 6 edges = 1 mechanical turn

            HallConfig config_;
            uint32_t   timeout_cnt_;
            std::array<float, 6> width_{};        // real sector widths [rad]

            int     last_sector_{0};              // last accepted sector
            int     dir_{0};                      // +1 forward, -1 backward, 0 unknown
            float   width_cur_{PI_DIV_3};         // width of the current sector (for the limit)
            float   theta_edge_{0.0f};            // exact angle at the last edge [rad]
            float   theta_{0.0f};                 // result of update() [rad]
            float   omega_e_{0.0f};               // [rad/s]
            float   k_{0.0f};                     // angle step after the edge [rad/count]
            bool    stopped_{true};               // period is invalid on the first edge after a stop

            std::array<uint32_t, AVG_EDGES> period_buf_{};  // last edge periods [count]
            uint32_t period_sum_{0};                        // sum of the buffer
            int      buf_idx_{0};                           // next slot to write
            int      buf_n_{0};                             // number of filled slots

            bool     cal_active_{false};
            int      cal_edge_{-1};                         // edge seen in this update(), else -1
            int      cal_errors_{0};                        // count of 000/111 or skipped sectors
            std::array<float, 6> cal_sum_cos_{};
            std::array<float, 6> cal_sum_sin_{};
            std::array<int, 6>   cal_count_{};

            static std::array<float, 6> calcWidths(const std::array<float, 6>& edges);
            void onEdge(int sector, int dir, uint32_t period_cnt);
            void onTimeout();
            void updateAngle(uint32_t cnt);
            void resetAverage();
    };
}
