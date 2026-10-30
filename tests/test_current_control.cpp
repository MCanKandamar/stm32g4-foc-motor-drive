// current_control tests: the clarke → park → PI → inversePark → inverseClarke → svpwm chain,
// voltage vector limit (Vbus/√3), d-axis priority and reset.
//
// The tests work back from the duties to the Vd/Vq the motor really gets, and check those.

#include "current_control.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

using namespace motor_control;

namespace {

constexpr float kPi = std::numbers::pi_v<float>;
constexpr float kTol = 1e-3f;

float deg_to_rad(int deg)
{
    return static_cast<float>(deg) * kPi / 180.0f;
}

SinCos_t make_sin_cos(float theta)
{
    return {std::sin(theta), std::cos(theta)};
}

// P only: with Ki = 0, output = Kp * error, so the tests are easy to calculate
PiConfig_ p_only(float kp)
{
    return {.Kp = kp, .Ki = 0.0f, .step_time = 50e-6f, .out_max = 100.0f, .out_min = -100.0f};
}

// Balanced 3-phase current with peak 'amplitude' and electrical angle 'theta'
Iabc_t balanced_current(float amplitude, float theta)
{
    return {
        amplitude * std::cos(theta),
        amplitude * std::cos(theta - 2.0f * kPi / 3.0f),
        amplitude * std::cos(theta + 2.0f * kPi / 3.0f),
    };
}

// Works back from the duties to the voltage on the motor, in rotor axes (dq)
Vdq_t applied_vdq(const duties_t& d, float vdc, const SinCos_t& sc)
{
    const float va = d.duty_a * vdc;
    const float vb = d.duty_b * vdc;
    const float vc = d.duty_c * vdc;
    const float v_alpha = (2.0f / 3.0f) * (va - 0.5f * (vb + vc));
    const float v_beta = (vb - vc) / std::numbers::sqrt3_v<float>;
    const Idq_fdb_t dq = park({v_alpha, v_beta}, sc);
    return {dq.id_fdb, dq.iq_fdb};
}

constexpr Iabc_t kZeroCurrent{0.0f, 0.0f, 0.0f};

}  // namespace

TEST(CurrentControl, SifirHatadaSifirGerilim)
{
    current_control cc(p_only(1.0f), p_only(1.0f));
    const duties_t d = cc.update({0.0f, 0.0f}, kZeroCurrent, make_sin_cos(0.3f), 24.0f);
    EXPECT_NEAR(d.duty_a, 0.5f, kTol);
    EXPECT_NEAR(d.duty_b, 0.5f, kTol);
    EXPECT_NEAR(d.duty_c, 0.5f, kTol);
}

TEST(CurrentControl, QHatasiHerAcidaQEksenineGerilimUygular)
{
    // iq_ref = 2 A, measured current 0, Kp = 1  ->  Vq = 2 V, Vd = 0, for any rotor angle
    for (int deg = 0; deg < 360; deg += 10) {
        current_control cc(p_only(1.0f), p_only(1.0f));
        const SinCos_t sc = make_sin_cos(deg_to_rad(deg));
        const Vdq_t v = applied_vdq(cc.update({0.0f, 2.0f}, kZeroCurrent, sc, 24.0f), 24.0f, sc);
        EXPECT_NEAR(v.Vd, 0.0f, kTol) << "aci = " << deg;
        EXPECT_NEAR(v.Vq, 2.0f, kTol) << "aci = " << deg;
    }
}

TEST(CurrentControl, AkimReferansaEsitseGerilimSifir)
{
    // Measured current is exactly iq_ref = 2 A (90° ahead of the rotor)  ->  no error  ->  no voltage
    for (int deg = 0; deg < 360; deg += 30) {
        const float theta = deg_to_rad(deg);
        current_control cc(p_only(1.0f), p_only(1.0f));
        const SinCos_t sc = make_sin_cos(theta);
        const Vdq_t v = applied_vdq(
            cc.update({0.0f, 2.0f}, balanced_current(2.0f, theta + kPi / 2.0f), sc, 24.0f), 24.0f, sc);
        EXPECT_NEAR(v.Vd, 0.0f, kTol) << "aci = " << deg;
        EXPECT_NEAR(v.Vq, 0.0f, kTol) << "aci = " << deg;
    }
}

TEST(CurrentControl, GerilimVektoruVbusBolKok3SiniriniAsmaz)
{
    // Very large references: both PIs want to saturate. The vector length must equal Vbus/√3 at every angle.
    const float vdc = 24.0f;
    const float v_max = vdc / std::numbers::sqrt3_v<float>;
    for (int deg = 0; deg < 360; deg += 5) {
        current_control cc(p_only(1.0f), p_only(1.0f));
        const SinCos_t sc = make_sin_cos(deg_to_rad(deg));
        const Vdq_t v = applied_vdq(cc.update({100.0f, 100.0f}, kZeroCurrent, sc, vdc), vdc, sc);
        EXPECT_NEAR(std::hypot(v.Vd, v.Vq), v_max, 1e-2f) << "aci = " << deg;
    }
}

TEST(CurrentControl, DEkseniOnceliklidir)
{
    // Both id_ref and iq_ref are very large: the d axis takes all of V_max, nothing is left for q
    const float vdc = 24.0f;
    const float v_max = vdc / std::numbers::sqrt3_v<float>;
    current_control cc(p_only(1.0f), p_only(1.0f));
    const SinCos_t sc = make_sin_cos(deg_to_rad(40));
    const Vdq_t v = applied_vdq(cc.update({100.0f, 100.0f}, kZeroCurrent, sc, vdc), vdc, sc);
    EXPECT_NEAR(v.Vd, v_max, 1e-2f);
    EXPECT_NEAR(v.Vq, 0.0f, 1e-2f);
}

TEST(CurrentControl, QEksenineKalanPayVerilir)
{
    // Vd = 8 V (Kp = 1, id error 8). V_max = 13.856  ->  Vq at most √(13.856² − 8²) = 11.31 V
    const float vdc = 24.0f;
    const float v_max = vdc / std::numbers::sqrt3_v<float>;
    current_control cc(p_only(1.0f), p_only(1.0f));
    const SinCos_t sc = make_sin_cos(deg_to_rad(75));
    const Vdq_t v = applied_vdq(cc.update({8.0f, 100.0f}, kZeroCurrent, sc, vdc), vdc, sc);
    EXPECT_NEAR(v.Vd, 8.0f, 1e-2f);
    EXPECT_NEAR(v.Vq, std::sqrt(v_max * v_max - 64.0f), 1e-2f);
}

TEST(CurrentControl, VbusDusunceLimitDeDuser)
{
    current_control cc(p_only(1.0f), p_only(1.0f));
    const SinCos_t sc = make_sin_cos(deg_to_rad(20));

    for (const float vdc : {24.0f, 12.0f, 6.0f}) {
        const Vdq_t v = applied_vdq(cc.update({0.0f, 100.0f}, kZeroCurrent, sc, vdc), vdc, sc);
        EXPECT_NEAR(v.Vq, vdc / std::numbers::sqrt3_v<float>, 1e-2f) << "vdc = " << vdc;
    }
}

TEST(CurrentControl, VbusSifirkenCokmez)
{
    current_control cc(p_only(1.0f), p_only(1.0f));
    const duties_t d = cc.update({1.0f, 1.0f}, kZeroCurrent, make_sin_cos(0.0f), 0.0f);
    EXPECT_FALSE(std::isnan(d.duty_a));
    EXPECT_FALSE(std::isnan(d.duty_b));
    EXPECT_FALSE(std::isnan(d.duty_c));
}

TEST(CurrentControl, ResetSonrasiYeniKurulmusGibiDavranir)
{
    const PiConfig_ cfg{.Kp = 0.5f, .Ki = 200.0f, .step_time = 50e-6f, .out_max = 100.0f, .out_min = -100.0f};
    const SinCos_t sc = make_sin_cos(deg_to_rad(60));

    current_control used(cfg, cfg);
    for (int i = 0; i < 500; ++i) {
        used.update({1.0f, 3.0f}, kZeroCurrent, sc, 24.0f);  // integrals build up
    }
    used.reset();

    current_control fresh(cfg, cfg);
    const duties_t a = used.update({0.5f, 1.0f}, kZeroCurrent, sc, 24.0f);
    const duties_t b = fresh.update({0.5f, 1.0f}, kZeroCurrent, sc, 24.0f);
    EXPECT_FLOAT_EQ(a.duty_a, b.duty_a);
    EXPECT_FLOAT_EQ(a.duty_b, b.duty_b);
    EXPECT_FLOAT_EQ(a.duty_c, b.duty_c);
}
