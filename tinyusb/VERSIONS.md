# Third-party sources in tinyusb/

| Folder | Source | Version | Commit | License |
|---|---|---|---|---|
| `src/`, `LICENSE` | https://github.com/hathach/tinyusb | 0.21.0 (2026-06-30) | `dae3f9a366bfcddbf9dcf1b48d7500286a849539` | MIT (`LICENSE`) |

- Archive: `https://codeload.github.com/hathach/tinyusb/tar.gz/refs/tags/0.21.0`, SHA-256 `b7cdf35c5ccefb0f61640aff94a732ab4ebcb21498201e4a161545891b23ba01`.
- Only the files this firmware needs are copied, unmodified: `tusb.*`, `tusb_option.h`, `common/`, `device/`, `osal/osal.h`, `osal/osal_freertos.h`, `class/cdc/cdc.h`, `class/cdc/cdc_device.*`, `portable/st/stm32_fsdev/` (`dcd_stm32_fsdev.c`, `fsdev_common.*`, `fsdev_stm32.h`).
- `CMakeLists.txt` and this file are ours. Keep `src/` unmodified; the configuration is in `App/Inc/tusb_config.h`, the descriptors in `App/Src/usb_descriptors.c`.
