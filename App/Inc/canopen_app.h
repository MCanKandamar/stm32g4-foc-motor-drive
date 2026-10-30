/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CANOPEN_APP_H
#define CANOPEN_APP_H

#ifdef __cplusplus
extern "C" {
#endif

void CanopenInit(void);     // once, at the start of the CANopen thread
void CanopenRun(void);      // every 1 ms from the CANopen thread

#ifdef __cplusplus
}
#endif

#endif
