#include "drv8350s.hpp"

namespace motor_control
{
    namespace
    {
        constexpr uint8_t REG_FAULT_STATUS  = 0x00;
        constexpr uint8_t REG_VGS_STATUS    = 0x01;
        constexpr uint8_t REG_DRIVER_CTRL   = 0x02;
        constexpr uint8_t REG_GATE_DRIVE_HS = 0x03;
        constexpr uint8_t REG_GATE_DRIVE_LS = 0x04;
        constexpr uint8_t REG_OCP_CTRL      = 0x05;

        // Frame: bit 15 = read, bits 14-11 = address, bits 10-0 = data.
        constexpr uint16_t READ      = 1u << 15;
        constexpr uint16_t DATA_MASK = 0x7FF;

        constexpr uint16_t UNLOCKED = 0b011 << 8;  // LOCK bits read back as 011

        constexpr uint16_t frame(uint8_t address, uint16_t data)
        {
            return static_cast<uint16_t>(((address & 0xFu) << 11) | (data & DATA_MASK));
        }

        // value placed at bit pos, cut to width bits
        constexpr uint16_t field(unsigned value, unsigned pos, unsigned width)
        {
            return static_cast<uint16_t>((value & ((1u << width) - 1u)) << pos);
        }
    }

    bool Drv8350s::configure(const Config& cfg)
    {
        const uint16_t driver  = field(cfg.ocp_act, 10, 1);
        const uint16_t gate_hs = UNLOCKED | field(cfg.idrivep_hs, 4, 4) | field(cfg.idriven_hs, 0, 4);
        const uint16_t gate_ls = field(cfg.cbc, 10, 1) | field(cfg.tdrive, 8, 2)
                               | field(cfg.idrivep_ls, 4, 4) | field(cfg.idriven_ls, 0, 4);
        const uint16_t ocp     = field(cfg.tretry, 10, 1) | field(cfg.dead_time, 8, 2)
                               | field(cfg.ocp_mode, 6, 2) | field(cfg.ocp_deg, 4, 2) | field(cfg.vds_lvl, 0, 4);

        const uint8_t  address[] = {REG_DRIVER_CTRL, REG_GATE_DRIVE_HS, REG_GATE_DRIVE_LS, REG_OCP_CTRL};
        const uint16_t value[]   = {driver, gate_hs, gate_ls, ocp};

        bool ok = true;
        for (unsigned i = 0; i < 4; ++i)
        {
            write_register(address[i], value[i]);
            if (read_register(address[i]) != value[i])
            {
                ok = false;
            }
        }
        return ok;
    }

    Drv8350s::Status Drv8350s::read_status()
    {
        return Status{read_register(REG_FAULT_STATUS), read_register(REG_VGS_STATUS)};
    }

    uint16_t Drv8350s::read_register(uint8_t address)
    {
        return transfer(READ | frame(address, 0)) & DATA_MASK;
    }

    void Drv8350s::write_register(uint8_t address, uint16_t data)
    {
        transfer(frame(address, data));
    }
}
