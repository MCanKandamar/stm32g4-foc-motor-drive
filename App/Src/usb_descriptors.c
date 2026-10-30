#include "tusb.h"
#include <string.h>

#define USB_VID             0x0483
#define USB_PID             0x5710

#define EP_CDC_OUT          0x01
#define EP_CDC_IN           0x81
#define EP_CDC_NOTIFY       0x82
#define CONFIG_TOTAL_LEN    (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN)

enum { ITF_CDC_CONTROL, ITF_CDC_DATA, ITF_COUNT };
enum { STR_LANGID, STR_MANUFACTURER, STR_PRODUCT, STR_SERIAL, STR_COUNT };

// Same IDs, class and strings as the USBX firmware, so Windows keeps the same COM port
static const tusb_desc_device_t device_desc =
{
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = TUSB_CLASS_CDC,
    .bDeviceSubClass    = CDC_COMM_SUBCLASS_ABSTRACT_CONTROL_MODEL,
    .bDeviceProtocol    = 0x00,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = USB_VID,
    .idProduct          = USB_PID,
    .bcdDevice          = 0x0200,
    .iManufacturer      = STR_MANUFACTURER,
    .iProduct           = STR_PRODUCT,
    .iSerialNumber      = STR_SERIAL,
    .bNumConfigurations = 1,
};

static const uint8_t config_desc[] =
{
    TUD_CONFIG_DESCRIPTOR(1, ITF_COUNT, 0, CONFIG_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_SELF_POWERED, 50),
    TUD_CDC_DESCRIPTOR(ITF_CDC_CONTROL, 0, EP_CDC_NOTIFY, 8, EP_CDC_OUT, EP_CDC_IN, CFG_TUD_CDC_EP_BUFSIZE),
};

static const char *const strings[STR_COUNT] =
{
    [STR_MANUFACTURER] = "STMicroelectronics",
    [STR_PRODUCT]      = "STM32 USB Device",
    [STR_SERIAL]       = "000000000001",
};

const uint8_t *tud_descriptor_device_cb(void)
{
    return (const uint8_t *)&device_desc;
}

const uint8_t *tud_descriptor_configuration_cb(uint8_t index)
{
    (void)index;
    return config_desc;
}

const uint16_t *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    static uint16_t desc[1 + 32];   // header word + up to 32 UTF-16 characters
    uint8_t len;
    (void)langid;

    if (index == STR_LANGID)
    {
        desc[1] = 0x0409;           // English (US)
        len = 1;
    }
    else if (index < STR_COUNT)
    {
        len = (uint8_t)strlen(strings[index]);
        for (uint8_t i = 0; i < len; i++) desc[1 + i] = (uint16_t)strings[index][i];
    }
    else
    {
        return NULL;
    }

    desc[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * len + 2));
    return desc;
}
