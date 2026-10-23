/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CANOPEN_APP_H
#define CANOPEN_APP_H

#ifdef __cplusplus
extern "C" {
#endif

void CanopenInit(void);     // CANopen thread'inin başında bir kez
void CanopenRun(void);      // CANopen thread'inden her 1 ms

#ifdef __cplusplus
}
#endif

#endif
