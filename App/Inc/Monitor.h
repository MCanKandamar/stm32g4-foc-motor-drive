
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MONITOR_H
#define MONITOR_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Set by each watched task once per loop, cleared by Monitor after an IWDG kick */
extern volatile bool alive_supervisor;
extern volatile bool alive_canopen;
extern volatile bool alive_monitor;

void MonitorRun(void);


#ifdef __cplusplus
}
#endif

#endif /* APP_H */