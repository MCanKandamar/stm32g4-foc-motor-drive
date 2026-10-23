/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CANOPEN_BRIDGE_H
#define CANOPEN_BRIDGE_H

#ifdef __cplusplus
extern "C" {
#endif

void CanopenBridgeUpdate(void);     // CANopen thread'inden her 1 ms: OD <-> motor verisi

#ifdef __cplusplus
}
#endif

#endif
