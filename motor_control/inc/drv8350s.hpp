#pragma once
#include <cstdint>

namespace motor_control
{
    // DRV8350S gate driver over SPI (datasheet SLVSDY6A, registers p.57–62).
    // The chip resets its registers while ENABLE is low: call configure() after every power-on.
    class Drv8350s
    {
        public:
            // Sends one 16-bit SPI frame and returns the 16 bits received.
            using Transfer = uint16_t (*)(uint16_t tx);

            // Raw register codes. Defaults = chip reset values.
            struct Config
            {
                bool    ocp_act{false};     // 1 = over-current turns off all three half-bridges
                uint8_t idrivep_hs{0xF};    // high-side source current, 0xF = 1000 mA
                uint8_t idriven_hs{0xF};    // high-side sink current, 0xF = 2000 mA
                bool    cbc{true};          // over-current retry clears on the next PWM edge
                uint8_t tdrive{3};          // 0..3 = 500/1000/2000/4000 ns
                uint8_t idrivep_ls{0xF};    // low-side source current, 0xF = 1000 mA
                uint8_t idriven_ls{0xF};    // low-side sink current, 0xF = 2000 mA
                bool    tretry{false};      // over-current retry time, 0 = 8 ms, 1 = 50 us
                uint8_t dead_time{1};       // 0..3 = 50/100/200/400 ns
                uint8_t ocp_mode{1};        // 0 = latch, 1 = retry, 2 = report only, 3 = off
                uint8_t ocp_deg{2};         // 0..3 = 1/2/4/8 us
                uint8_t vds_lvl{9};         // VDS trip level, 9 = 0.6 V
            };

            // Fault status registers 0x00 and 0x01.
            struct Status
            {
                uint16_t fault{0};
                uint16_t vgs{0};
            };

            // Status::fault bits
            static constexpr uint16_t FAULT   = 1u << 10;  // any fault (nFAULT pin)
            static constexpr uint16_t VDS_OCP = 1u << 9;   // MOSFET over-current
            static constexpr uint16_t GDF     = 1u << 8;   // gate drive fault
            static constexpr uint16_t UVLO    = 1u << 7;   // supply under-voltage
            static constexpr uint16_t OTSD    = 1u << 6;   // over-temperature shutdown
            // Status::vgs bits
            static constexpr uint16_t OTW     = 1u << 7;   // over-temperature warning
            static constexpr uint16_t GDUV    = 1u << 6;   // gate supply under-voltage

            explicit Drv8350s(Transfer transfer) : transfer{transfer} {}

            // Writes the config and reads it back; false if the read-back differs.
            bool configure(const Config& cfg);

            Status read_status();

            uint16_t read_register(uint8_t address);

        private:
            void write_register(uint8_t address, uint16_t data);

            Transfer transfer;
    };
}
