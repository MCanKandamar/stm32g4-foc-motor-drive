# Third-party sources in canopen/

| Folder | Source | Commit | Date | License |
|---|---|---|---|---|
| `CANopenNode/` | https://github.com/CANopenNode/CANopenNode | `9b8beed8367241e96ac03f916cd5a500bcb2cf23` | 2026-07-10 | Apache-2.0 |
| `CANopenNode_STM32/` | https://github.com/CANopenNode/CanOpenSTM32 (folder `CANopenNode_STM32`) | `2f1d9ea7060b03ffde8974f6428f6357a2ff2b1d` | 2026-10-12 | MIT (`CANopenNode_STM32/LICENSE`) |
| `od/OD.c`, `od/OD.h` | copied from `CANopenNode/example/` (DS301 profile) | — | — | Apache-2.0 |

- `CANopenNode/` is the commit CanOpenSTM32 pins as its submodule; keep it unmodified.
- `CANopenNode_STM32/` is adapted to this project (ThreadX, BASEPRI locking); changes are ours.
- `od/` will be regenerated with CANopenEditor (exporter "CANopenNode_V4") in phase 3.
