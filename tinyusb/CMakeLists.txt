# TinyUSB device stack, CDC class only. See VERSIONS.md for the version.
add_library(tinyusb STATIC
    src/tusb.c
    src/common/tusb_fifo.c
    src/device/usbd.c
    src/class/cdc/cdc_device.c
    src/portable/st/stm32_fsdev/dcd_stm32_fsdev.c
    src/portable/st/stm32_fsdev/fsdev_common.c
)

# tusb_config.h is in App/Inc
target_include_directories(tinyusb PUBLIC src ../App/Inc)

# STM32 and FreeRTOS headers (include + define only, no sources added)
target_link_libraries(tinyusb PUBLIC stm32cubemx)
