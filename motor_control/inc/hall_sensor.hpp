#pragma once
#include "math_utils.hpp"
#include <array>
#include <cstdint>

namespace motor_control
{
    // sequence motor: 1 -> 5 -> 4 -> 6 -> 2 -> 3. 000 / 111 geçersiz (-1)
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

            // Kenar kalibrasyonu: rotor dışarıdan bilinen açıyla (theta_ref) çevrilirken
            // her kenarın geldiği açı kaydedilir, sonunda ortalaması tabloya yazılır.
            void startCalibration();
            void recordCalibration(float theta_ref);    // her update()'ten sonra
            bool finishCalibration();                   // true: tablo güncellendi

        private:
            static constexpr int AVG_EDGES = 24;  // 4 çift kutup × 6 kenar = 1 mekanik tur

            HallConfig config_;
            uint32_t   timeout_cnt_;
            std::array<float, 6> width_{};        // gerçek dilim genişlikleri [rad]

            int     last_sector_{0};              // son kabul edilen sektör
            int     dir_{0};                      // +1 ileri, -1 geri, 0 bilinmiyor
            float   width_cur_{PI_DIV_3};         // şu anki dilimin genişliği (sınır için)
            float   theta_edge_{0.0f};            // son kenardaki kesin açı [rad]
            float   theta_{0.0f};                 // update() sonucu [rad]
            float   omega_e_{0.0f};               // [rad/s]
            float   k_{0.0f};                     // kenardan sonraki açı artışı [rad/count]
            bool    stopped_{true};               // durduktan sonraki ilk kenarda periyot geçersiz

            std::array<uint32_t, AVG_EDGES> period_buf_{};  // son kenar periyotları [count]
            uint32_t period_sum_{0};                        // buffer toplamı
            int      buf_idx_{0};                           // sıradaki yazılacak yuva
            int      buf_n_{0};                             // dolu yuva sayısı

            bool     cal_active_{false};
            int      cal_edge_{-1};                         // bu update()'te gelen kenar, yoksa -1
            int      cal_errors_{0};                        // 000/111 veya sektör atlama sayısı
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
