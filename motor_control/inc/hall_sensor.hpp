#pragma once
#include "types.hpp"
#include <array>
#include <cstdint>

namespace motor_control 
{
    // sequence motor: 1 -> 5 -> 4 -> 6 -> 2 -> 3.
    static constexpr int hall_idx[8] = {-1, 0, 4, 5, 2, 1, 3, -1};

    constexpr float PI = 3.14159265359f;
    constexpr float PI_DIV_3 = PI / 3.0f;
    constexpr float TWO_PI = (2.0f * PI);
    constexpr float RAD_TO_RPM = (60.0f / TWO_PI);
    constexpr float RAD_TO_DEG = (180.0f / PI);

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

            bool HallSensorInit(uint8_t hall_state);

            void update(uint8_t hall_state, uint32_t period_cnt, uint32_t cnt);

            float getMechanicalVelocity_rpm() const    { return omega_e_ / config.pole_pairs * RAD_TO_RPM; }
            float getElectricalAngle_rad() const       { return theta_; }
            float getElectricalVelocity_rads() const   { return omega_e_; }
            int getDirection() const                   { return SectorInfo.dir; }

        private:
            HallConfig config;
            struct SectorInfo_ { int sector; int dir{0}; bool valid; }; 
            SectorInfo_ SectorInfo;
            uint32_t   timeout_cnt_;
            std::array<float, 6> width_{};        // gerçek dilim genişlikleri [rad]
            float   width_cur_{PI_DIV_3};         // şu anki dilimin genişliği (sınır için)

            int     sector_{0};
            float   theta_edge_{0.0f};            // son kenardaki kesin açı
            float   theta_{0.0f};                 // update() sonucu
            float   omega_e_{0.0f};               // [rad/s]
            float   k_{0.0f};                     // dir·w_prev/N [rad/count]
            bool stoppedFlag{true};


            std::array<uint32_t, 24> period_buf_{};  // period cnt 1 cycle
            uint32_t period_sum_{0};                        // sum of buffer
            int      buf_idx_{0};                           // next sector
            int      buf_n_{0};                             // number of valid sector
            int      last_dir_{0};

            SectorInfo_ decodeHall(uint8_t hall_state);
            void onTimeout();
            void onEdge(const SectorInfo_& SectorInfo, uint32_t period_cnt);
            void updateAngle(uint32_t cnt);
            void resetAverage();
    };
}