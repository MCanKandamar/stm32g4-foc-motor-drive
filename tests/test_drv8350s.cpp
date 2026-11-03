// Drv8350s tests: SPI frames, register packing and read-back against a fake chip.

#include "drv8350s.hpp"

#include <gtest/gtest.h>

using namespace motor_control;

namespace {

// Fake DRV8350S: a register file behind the SPI frame format.
uint16_t regs[8];
uint16_t last_tx;
bool     chip_answers;
uint16_t idle_line;  // what SDO reads when the chip does not answer

uint16_t fake_transfer(uint16_t tx)
{
    last_tx = tx;
    if (!chip_answers)
    {
        return idle_line;
    }
    const unsigned address = (tx >> 11) & 0xF;
    const uint16_t data    = tx & 0x7FF;
    const uint16_t reply   = regs[address];
    if ((tx & 0x8000) == 0)
    {
        regs[address] = data;
    }
    return reply;
}

class Drv8350sTest : public ::testing::Test
{
    protected:
        void SetUp() override
        {
            for (auto& r : regs) r = 0;
            last_tx      = 0;
            chip_answers = true;
            idle_line    = 0x0000;
        }

        Drv8350s drv{fake_transfer};
};

}  // namespace

TEST_F(Drv8350sTest, ReadFrameHasReadBitAndAddress)
{
    regs[0x05] = 0x169;
    EXPECT_EQ(drv.read_register(0x05), 0x169);
    EXPECT_EQ(last_tx, 0xA800);  // 1 | 0101 | 000 0000 0000
}

TEST_F(Drv8350sTest, DefaultConfigWritesResetValues)
{
    EXPECT_TRUE(drv.configure(Drv8350s::Config{}));
    EXPECT_EQ(regs[0x02], 0x000);
    EXPECT_EQ(regs[0x03], 0x3FF);  // LOCK 011, IDRIVEP_HS 1111, IDRIVEN_HS 1111
    EXPECT_EQ(regs[0x04], 0x7FF);  // CBC 1, TDRIVE 11, IDRIVEP_LS 1111, IDRIVEN_LS 1111
    EXPECT_EQ(regs[0x05], 0x169);  // TRETRY 0, DEAD_TIME 01, OCP_MODE 01, OCP_DEG 10, VDS_LVL 1001
}

TEST_F(Drv8350sTest, FieldsGoToTheirBits)
{
    Drv8350s::Config cfg;
    cfg.ocp_act    = true;
    cfg.idrivep_hs = 0x5;
    cfg.idriven_hs = 0xA;
    cfg.cbc        = false;
    cfg.tdrive     = 1;
    cfg.idrivep_ls = 0x3;
    cfg.idriven_ls = 0xC;
    cfg.tretry     = true;
    cfg.dead_time  = 2;
    cfg.ocp_mode   = 0;
    cfg.ocp_deg    = 3;
    cfg.vds_lvl    = 4;

    EXPECT_TRUE(drv.configure(cfg));
    EXPECT_EQ(regs[0x02], 0x400);  // OCP_ACT
    EXPECT_EQ(regs[0x03], 0x35A);  // 011 0101 1010
    EXPECT_EQ(regs[0x04], 0x13C);  // 0 01 0011 1100
    EXPECT_EQ(regs[0x05], 0x634);  // 1 10 00 11 0100
}

TEST_F(Drv8350sTest, TooLargeValueDoesNotSpillIntoNextField)
{
    Drv8350s::Config cfg;
    cfg.dead_time = 7;  // 2-bit field: becomes 3, TRETRY stays 0

    EXPECT_TRUE(drv.configure(cfg));
    EXPECT_EQ(regs[0x05] & 0x700, 0x300);
}

TEST_F(Drv8350sTest, ConfigureFailsWhenChipDoesNotAnswer)
{
    chip_answers = false;  // e.g. ENABLE low or SPI not working
    idle_line    = 0x0000; // SDO low (no pull-up): matches 0x02 = 0, not the others
    EXPECT_FALSE(drv.configure(Drv8350s::Config{}));
    idle_line    = 0xFFFF; // SDO pulled up: matches 0x04 = 0x7FF, not the others
    EXPECT_FALSE(drv.configure(Drv8350s::Config{}));
}

TEST_F(Drv8350sTest, ReadStatusReturnsBothWords)
{
    regs[0x00] = Drv8350s::FAULT | Drv8350s::VDS_OCP | 0x20;  // VDS_HA
    regs[0x01] = Drv8350s::OTW;

    const Drv8350s::Status s = drv.read_status();
    EXPECT_EQ(s.fault, 0x620);
    EXPECT_EQ(s.vgs, Drv8350s::OTW);
}
