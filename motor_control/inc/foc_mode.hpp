#pragma once
#include <cstdint>

namespace motor_control
{
    // ISR açıyı nereden alır
    enum class AngleSource : uint8_t
    {
        FIXED,          // sabit açı (hizalama)
        OPEN_LOOP,      // açı rampası
        HALL,           // Hall + PLL
        SMO             // SMO + PLL
    };

    // ISR hangi döngüyü kapatır
    enum class LoopMode : uint8_t
    {
        OFF,            // FOC çalışmaz
        CURRENT,        // idq_ref doğrudan
        SPEED           // idq_ref hız PI'dan
    };
}
