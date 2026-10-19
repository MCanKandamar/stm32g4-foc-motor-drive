#include "scrutiny.h"
#include "scrutiny.hpp"
#include "ux_device_cdc_acm.h"
#include <atomic>

static scrutiny::MainHandler scrutiny_main;
static uint8_t  scr_rx[128];
static uint8_t  scr_tx[256];
static uint8_t  scr_dlog[8192];
static scrutiny::FixedFrequencyLoopHandler foc_loop(500, "FOC 20kHz");
static scrutiny::LoopHandler* loops[] = { &foc_loop };
static ULONG t_last;


static uint8_t rx_ring[512];
static std::atomic<uint16_t> rx_head{0};
static std::atomic<uint16_t> rx_tail{0};
extern UX_SLAVE_CLASS_CDC_ACM *cdc_acm;

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
    t_last = tx_time_get();
}

void ScrutinyRxRun(void)
{
    UCHAR buf[64];
    ULONG n = 0;

    if (cdc_acm == UX_NULL)
    {
        tx_thread_sleep(10);
        return;
    }

    if (ux_device_class_cdc_acm_read(cdc_acm,
                                        buf,
                                        sizeof(buf),
                                        &n) != UX_SUCCESS)
    {
        tx_thread_sleep(1);
        return;
    }

    for (ULONG i = 0; i < n; i++)
    {
        uint16_t next = (rx_head + 1) % sizeof(rx_ring);
        if (next == rx_tail) break;
        rx_ring[rx_head] = buf[i];
        rx_head = next;
    }
}

void ScrutinyRun(void)
{
    uint8_t buf[64];
    uint16_t n = 0;

    while (rx_tail != rx_head && n < sizeof(buf))
    {
        buf[n++] = rx_ring[rx_tail];
        rx_tail = (rx_tail + 1) % sizeof(rx_ring);
    }
    if (n > 0) scrutiny_main.receive_data(buf, n);

    ULONG now = tx_time_get();
    scrutiny_main.process((now - t_last) * 10000);
    t_last = now;

    while (cdc_acm != UX_NULL && scrutiny_main.data_to_send() > 0)
    {
        ULONG len = scrutiny_main.pop_data(buf, sizeof(buf));
        ULONG sent;
        if (ux_device_class_cdc_acm_write(cdc_acm,
                                            buf,
                                            len,
                                            &sent) != UX_SUCCESS) break;
    }
    tx_thread_sleep(1);           
}

void ScrutinyFocTick(void) { foc_loop.process(); }