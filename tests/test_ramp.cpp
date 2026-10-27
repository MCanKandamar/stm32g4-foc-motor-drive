// Ramp testleri.
// Kontrol edilenler: hızlanma/yavaşlama eğimi, hedefte tam durma, sıfırdan geçişte
// önce rate_down sonra rate_up, negatif yön ve reset.

#include "ramp.hpp"

#include <gtest/gtest.h>

using namespace motor_control;

namespace {

constexpr float kTol = 1e-3f;
constexpr float kDt  = 1e-3f;      // 1 ms

// n adım çalıştır, son çıkışı döndür
float run(Ramp& r, float target, float up, float down, int n)
{
    float out = r.get_output();
    for (int i = 0; i < n; ++i) {
        out = r.update(target, up, down, kDt);
    }
    return out;
}

}  // namespace

TEST(Ramp, HizlanmaRateUpIle)
{
    Ramp r;

    // 1000 rpm/s, 100 ms -> 100 rpm
    EXPECT_NEAR(run(r, 3000.0f, 1000.0f, 500.0f, 100), 100.0f, kTol);
}

TEST(Ramp, HedefteTamDurur)
{
    Ramp r;

    // 1000 rpm/s ile 300 rpm'e 300 ms'de varılır; fazladan adımlar hedefi aşmaz
    run(r, 300.0f, 1000.0f, 1000.0f, 400);
    EXPECT_EQ(r.get_output(), 300.0f);
}

TEST(Ramp, YavaslamaRateDownIle)
{
    Ramp r;
    r.reset(1000.0f);

    // |çıkış| küçülüyor: 500 rpm/s, 100 ms -> 50 rpm düşer
    EXPECT_NEAR(run(r, 0.0f, 2000.0f, 500.0f, 100), 950.0f, kTol);
}

TEST(Ramp, SifirdanGecis)
{
    Ramp r;
    r.reset(100.0f);

    // +100 -> -100: önce 100 rpm 1000 rpm/s ile 100 ms'de 0'a,
    // sonra 2000 rpm/s ile 50 ms'de -100'e
    EXPECT_NEAR(run(r, -100.0f, 2000.0f, 1000.0f, 100), 0.0f, kTol);
    EXPECT_NEAR(run(r, -100.0f, 2000.0f, 1000.0f, 25), -50.0f, kTol);
    EXPECT_EQ(run(r, -100.0f, 2000.0f, 1000.0f, 30), -100.0f);
}

TEST(Ramp, NegatifYondeHizlanma)
{
    Ramp r;

    // 0'dan negatife: |çıkış| büyüyor -> rate_up
    EXPECT_NEAR(run(r, -3000.0f, 1000.0f, 100.0f, 100), -100.0f, kTol);
}

TEST(Ramp, ResetCikisiAyarlar)
{
    Ramp r;
    run(r, 500.0f, 1000.0f, 1000.0f, 100);

    r.reset(42.0f);
    EXPECT_EQ(r.get_output(), 42.0f);
    r.reset();
    EXPECT_EQ(r.get_output(), 0.0f);
}
