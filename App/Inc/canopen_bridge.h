/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CANOPEN_BRIDGE_H
#define CANOPEN_BRIDGE_H

#ifdef __cplusplus
extern "C" {
#endif

void CanopenBridgeInit(void);       // once after canopen_app_init: OD write checks
void CanopenBridgeUpdate(void);     // every 1 ms from the CANopen thread: OD <-> motor data

#ifdef __cplusplus
}
#endif

#endif
