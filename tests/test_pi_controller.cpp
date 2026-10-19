// PiController testleri.
// Kontrol edilenler: P ve I terimleri, çıkış limitleri, anti-windup (koşullu integrasyon),
// gürültü dayanıklılığı, reset ve geçersiz ayarların yakalanması.

#include "pi_controller.hpp"

#include <gtest/gtest.h>

using namespace motor_control;

namespace {

constexpr float kTol = 1e-4f;

// Testlerde ortak kullanılan ayar: dt = 1 ms, çıkış ±10 (ör. volt)
PiConfig_ make_config(float kp, float ki)
{
    return {.Kp = kp, .Ki = ki, .step_time = 1e-3f, .out_max = 10.0f, .out_min = -10.0f};
}

}  // namespace

TEST(PiController, SadecePTerimi)
{
    PiController pi(make_config(2.0f, 0.0f));

    // hata = 1.0 - 0.25 = 0.75  ->  çıkış = 2 * 0.75
    EXPECT_NEAR(pi.update(1.0f, 0.25f), 1.5f, kTol);
    EXPECT_NEAR(pi.getIntegral(), 0.0f, kTol);
}

TEST(PiController, IntegralZamanlaBirikir)
{
    PiController pi(make_config(0.0f, 100.0f));

    // Her adımda integral += Ki * hata * dt = 100 * 1 * 0.001 = 0.1
    float output = 0.0f;
    for (int i = 0; i < 10; ++i) {
        output = pi.update(1.0f, 0.0f);
    }
    EXPECT_NEAR(pi.getIntegral(), 1.0f, kTol);
    EXPECT_NEAR(output, 1.0f, kTol);
}

TEST(PiController, CikisLimitleriAsilmaz)
{
    PiController pi(make_config(5.0f, 1000.0f));

    for (int i = 0; i < 1000; ++i) {
        const float output = pi.update(100.0f, 0.0f);
        EXPECT_LE(output, 10.0f);
    }
    EXPECT_FLOAT_EQ(pi.update(100.0f, 0.0f), 10.0f);

    for (int i = 0; i < 1000; ++i) {
        const float output = pi.update(-100.0f, 0.0f);
        EXPECT_GE(output, -10.0f);
    }
    EXPECT_FLOAT_EQ(pi.update(-100.0f, 0.0f), -10.0f);
}

TEST(PiController, PTerimiDoyurunca_IntegralBirikmez)
{
    // P terimi tek başına çıkışı doyuruyor (1 * 100 > 10).
    // Koşullu integrasyon sayesinde integral 0'da kalmalı (windup yok).
    PiController pi(make_config(1.0f, 100.0f));

    for (int i = 0; i < 1000; ++i) {
        pi.update(100.0f, 0.0f);
    }
    EXPECT_NEAR(pi.getIntegral(), 0.0f, kTol);

    // Hata ters dönünce (hata = -1) çıkış hemen tepki vermeli:
    //   P = -1, integral bu adımda 0 + 100 * (-1) * 0.001 = -0.1  ->  çıkış = -1.1
    // Windup olsaydı integral 10'a dayanmış olurdu ve çıkış -1 + 9.9 = 8.9 olurdu.
    EXPECT_NEAR(pi.update(0.0f, 1.0f), -1.1f, kTol);
}

TEST(PiController, GurultuOrnegiIntegraliBozmaz)
{
    // Senaryo: Integral sabit yükte ~8'de duruyor. Tek bir gürültülü örnek
    // P terimini anlık olarak büyütüyor ve çıkışı doyuruyor.
    PiController pi(make_config(1.0f, 100.0f));

    for (int i = 0; i < 80; ++i) {
        pi.update(1.0f, 0.0f);  // her adım +0.1
    }
    const float integral_before = pi.getIntegral();
    ASSERT_NEAR(integral_before, 8.0f, 1e-3f);

    // Gürültü: hata = 5  ->  P + I = 5 + 8 = 13 >= 10, hata pozitif  ->  integral eklenmez
    EXPECT_FLOAT_EQ(pi.update(5.0f, 0.0f), 10.0f);
    EXPECT_FLOAT_EQ(pi.getIntegral(), integral_before);

    // Gürültü geçti (hata = 0): çıkış hemen eski değerine döner
    EXPECT_NEAR(pi.update(0.0f, 0.0f), integral_before, kTol);
}

TEST(PiController, AltLimitteDeAntiWindupCalisir)
{
    PiController pi(make_config(1.0f, 100.0f));

    for (int i = 0; i < 1000; ++i) {
        pi.update(-100.0f, 0.0f);
    }
    EXPECT_NEAR(pi.getIntegral(), 0.0f, kTol);
    // Hata ters dönünce: P = 1, integral bu adımda +0.1  ->  çıkış = 1.1
    EXPECT_NEAR(pi.update(0.0f, -1.0f), 1.1f, kTol);
}

TEST(PiController, ResetIntegraliSifirlar)
{
    PiController pi(make_config(0.0f, 100.0f));

    for (int i = 0; i < 20; ++i) {
        pi.update(1.0f, 0.0f);
    }
    ASSERT_GT(pi.getIntegral(), 1.0f);

    pi.reset();
    EXPECT_FLOAT_EQ(pi.getIntegral(), 0.0f);
    EXPECT_FLOAT_EQ(pi.update(0.0f, 0.0f), 0.0f);
}

TEST(PiController, SetLimitsYeniLimitleriUygular)
{
    PiController pi(make_config(5.0f, 0.0f));
    pi.setLimits(-3.0f, 3.0f);

    EXPECT_FLOAT_EQ(pi.update(100.0f, 0.0f), 3.0f);
    EXPECT_FLOAT_EQ(pi.update(-100.0f, 0.0f), -3.0f);
}

TEST(PiController, LimitDaralincaIntegralDeDaralir)
{
    // Senaryo: Integral 8 V'ta, limitler ±10 V. Vbus düşüyor ve limit ±5 V'a iniyor
    // (current_control her döngüde setLimits çağırıyor).
    PiController pi(make_config(1.0f, 100.0f));
    for (int i = 0; i < 80; ++i) {
        pi.update(1.0f, 0.0f);  // her adım +0.1  ->  integral ~8
    }
    ASSERT_NEAR(pi.getIntegral(), 8.0f, 1e-3f);

    pi.setLimits(-5.0f, 5.0f);
    EXPECT_FLOAT_EQ(pi.update(0.0f, 0.0f), 5.0f);

    // Integral de yeni limite inmiş olmalı. İnmezse 8'de kalır ve hata ters döndüğünde
    // çıkış, integral 5'in altına inene kadar (30 adım) 5 V'ta takılı kalır (windup).
    EXPECT_LE(pi.getIntegral(), 5.0f);

    // Hata ters döndü (hata = -1): P = -1, integral 5 - 0.1 = 4.9  ->  çıkış hemen 3.9
    EXPECT_NEAR(pi.update(0.0f, 1.0f), 3.9f, kTol);
}

// Death test: geçersiz ayarla oluşturulunca constructor'daki assert programı durdurmalı.
// (assert sadece Debug derlemede aktif; Release'de bu testler atlanır.)
#ifndef NDEBUG
TEST(PiControllerDeathTest, AdimSuresiSifirsaDurur)
{
    PiConfig_ config = make_config(1.0f, 1.0f);
    config.step_time = 0.0f;
    EXPECT_DEATH(PiController{config}, "");
}

TEST(PiControllerDeathTest, LimitlerTersseDurur)
{
    PiConfig_ config = make_config(1.0f, 1.0f);
    config.out_max = -10.0f;
    config.out_min = 10.0f;
    EXPECT_DEATH(PiController{config}, "");
}
#endif
