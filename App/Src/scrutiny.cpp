#include "scrutiny.h"
#include "scrutiny.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "tusb.h"
#include <algorithm>

static scrutiny::MainHandler scrutiny_main;
static uint8_t  scr_rx[128];
static uint8_t  scr_tx[256];
static uint8_t  scr_dlog[8192];
static scrutiny::FixedFrequencyLoopHandler foc_loop(500, "FOC 20kHz");
static scrutiny::LoopHandler* loops[] = { &foc_loop };
static TickType_t t_last;

void ScrutinyInit(void)
{
    scrutiny::Config cfg;                   
    cfg.set_buffers(scr_rx,
                    sizeof(scr_rx),
                    scr_tx,
                    sizeof(scr_tx));

    cfg.set_loops(loops, 1);
    cfg.set_datalogging_buffers(scr_dlog,
                                sizeof(scr_dlog));

    cfg.display_name = "BLDC FOC";
    scrutiny_main.init(&cfg);
    t_last = xTaskGetTickCount();
}

void ScrutinyRun(void)
{
    uint8_t buf[64];

    uint32_t n = tud_cdc_read(buf, sizeof(buf));
    if (n > 0) scrutiny_main.receive_data(buf, n);

    TickType_t now = xTaskGetTickCount();
    scrutiny_main.process((now - t_last) * 10000);
    t_last = now;

    while (tud_mounted() && scrutiny_main.data_to_send() > 0)
    {
        uint32_t room = tud_cdc_write_available();
        if (room == 0) break;       // the rest goes out on the next run
        uint32_t len = scrutiny_main.pop_data(buf, std::min<uint32_t>(room, sizeof(buf)));
        tud_cdc_write(buf, len);
    }
    tud_cdc_write_flush();
}

void ScrutinyFocTick(void) { foc_loop.process(); }