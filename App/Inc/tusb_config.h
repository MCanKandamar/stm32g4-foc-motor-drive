#pragma once

#define CFG_TUSB_MCU            OPT_MCU_STM32G4
#define CFG_TUSB_OS             OPT_OS_FREERTOS
#define CFG_TUD_ENABLED         1

#define CFG_TUD_ENDPOINT0_SIZE  64

// One CDC port for Scrutiny
#define CFG_TUD_CDC             1
#define CFG_TUD_CDC_RX_BUFSIZE  512
#define CFG_TUD_CDC_TX_BUFSIZE  512
#define CFG_TUD_CDC_EP_BUFSIZE  64
