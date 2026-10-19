// SVPWM testleri. Girdiler, gerçek kullanımdaki gibi inverseClarke'tan geçirilerek verilir.

#include "svpwm.hpp"
#include "transforms.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <numbers>

using namespace motor_control;

namespace {

constexpr float kPi = std::numbers::pi_v<float>;
constexpr float kVbus = 24.0f;
constexpr float kTol = 1e-4f;

float deg_to_rad(int deg)
{
    return static_cast<float>(deg) * kPi / 180.0f;
}

// Genliği 'magnitude', açısı 'theta' olan gerilim vektörünü duty'lere çevirir
duties_t svpwm_from_vector(float magnitude, float theta)
{
    return svpwm(inverseClarke({magnitude * std::cos(theta), magnitude * std::sin(theta)}), kVbus);
}

// Duty'lerden motora gerçekte uygulanan αβ vektörünü geri hesaplar.
// (Ortak mod gerilimi motoru etkilemediği için düşer.)
V_AlphaBeta_t applied_vector(const duties_t& d)
{
    const float va = d.duty_a * kVbus;
    const float vb = d.duty_b * kVbus;
    const float vc = d.duty_c * kVbus;
    return {
        (2.0f / 3.0f) * (va - 0.5f * (vb + vc)),
        (vb - vc) / std::numbers::sqrt3_v<float>,
    };
}

}  // namespace

// ---------------------------------------------------------------- Bilinen örnekler

TEST(Svpwm, AlphaEksenindeOnVolt)
{
    const duties_t d = svpwm(inverseClarke({10.0f, 0.0f}), kVbus);
    EXPECT_NEAR(d.duty_a, 0.8125f, kTol);
    EXPECT_NEAR(d.duty_b, 0.1875f, kTol);
    EXPECT_NEAR(d.duty_c, 0.1875f, kTol);
}

TEST(Svpwm, DogrusalBolgeSinirindaBirFaz100BirFaz0)
{
    // |V| = Vbus/sqrt(3), açı 30°  ->  Vα = 12, Vβ = 6.928
    const duties_t d = svpwm_from_vector(kVbus / std::numbers::sqrt3_v<float>, deg_to_rad(30));
    EXPECT_NEAR(d.duty_a, 1.0f, kTol);
    EXPECT_NEAR(d.duty_b, 0.5f, kTol);
    EXPECT_NEAR(d.duty_c, 0.0f, kTol);
}

TEST(Svpwm, SifirGerilimdeTumFazlarYuzde50)
{
    const duties_t d = svpwm(inverseClarke({0.0f, 0.0f}), kVbus);
    EXPECT_NEAR(d.duty_a, 0.5f, kTol);
    EXPECT_NEAR(d.duty_b, 0.5f, kTol);
    EXPECT_NEAR(d.duty_c, 0.5f, kTol);
}

TEST(Svpwm, VbusSifirVeyaNegatifseTumDutylerSifir)
{
    for (const float vbus : {0.0f, -5.0f}) {
        const duties_t d = svpwm(inverseClarke({10.0f, 3.0f}), vbus);
        EXPECT_FLOAT_EQ(d.duty_a, 0.0f) << "vbus = " << vbus;
        EXPECT_FLOAT_EQ(d.duty_b, 0.0f) << "vbus = " << vbus;
        EXPECT_FLOAT_EQ(d.duty_c, 0.0f) << "vbus = " << vbus;
    }
}

// ---------------------------------------------------------------- Tam tur

TEST(Svpwm, HerAcidaKomutEdilenVektorAynenUygulanir)
{
    // Doğrusal bölgede motora giden vektör, komutun genlik ve açısıyla birebir aynı olmalı
    for (int deg = 0; deg < 360; deg += 5) {
        const float theta = deg_to_rad(deg);
        const V_AlphaBeta_t v = applied_vector(svpwm_from_vector(10.0f, theta));
        EXPECT_NEAR(v.Valpha, 10.0f * std::cos(theta), 1e-3f) << "aci = " << deg;
        EXPECT_NEAR(v.Vbeta, 10.0f * std::sin(theta), 1e-3f) << "aci = " << deg;
    }
}

TEST(Svpwm, DutylerOrtalanmistir)
{
    // Min-max yönteminin özelliği: en büyük ve en küçük duty 0.5 etrafında simetriktir
    for (int deg = 0; deg < 360; deg += 5) {
        const duties_t d = svpwm_from_vector(10.0f, deg_to_rad(deg));
        const float d_max = std::max({d.duty_a, d.duty_b, d.duty_c});
        const float d_min = std::min({d.duty_a, d.duty_b, d.duty_c});
        EXPECT_NEAR(d_max + d_min, 1.0f, kTol) << "aci = " << deg;
    }
}

TEST(Svpwm, DogrusalBolgeSinirindaDutylerTasmaz)
{
    const float v_limit = kVbus / std::numbers::sqrt3_v<float>;
    for (int deg = 0; deg < 360; deg += 1) {
        const duties_t d = svpwm_from_vector(v_limit, deg_to_rad(deg));
        for (const float duty : {d.duty_a, d.duty_b, d.duty_c}) {
            EXPECT_GE(duty, -kTol) << "aci = " << deg;
            EXPECT_LE(duty, 1.0f + kTol) << "aci = " << deg;
        }
    }
}

TEST(Svpwm, AsiriModulasyondaDutyler0ile1ArasindaKalir)
{
    // |V| sınırın çok üstünde: duty'ler clamp ile [0, 1] içinde tutulmalı
    for (int deg = 0; deg < 360; deg += 5) {
        const duties_t d = svpwm_from_vector(30.0f, deg_to_rad(deg));
        for (const float duty : {d.duty_a, d.duty_b, d.duty_c}) {
            EXPECT_GE(duty, 0.0f) << "aci = " << deg;
            EXPECT_LE(duty, 1.0f) << "aci = " << deg;
        }
    }
}
