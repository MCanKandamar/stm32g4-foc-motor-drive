#include "canopen_app.h"
#include "CO_app_STM32.h"

#define CO_NODE_ID      1       // ağdaki adres (1..127)
#define CO_BAUDRATE     500     // kbit/s, CubeMX'teki FDCAN1 ayarıyla aynı olmalı

extern FDCAN_HandleTypeDef hfdcan1;     // MX_FDCAN1_Init main.h'de (CubeMX'te static kaldırıldı)

static CANopenNodeSTM32 co_node;

void CanopenInit(void)
{
    co_node.CANHandle      = &hfdcan1;
    co_node.HWInitFunction = MX_FDCAN1_Init;    // sürücü CAN'i sıfırlarken çağırır
    co_node.timerHandle    = NULL;              // timer yok: CanopenRun 1 ms thread'den
    co_node.desiredNodeID  = CO_NODE_ID;
    co_node.baudrate       = CO_BAUDRATE;

    canopen_app_init(&co_node);
}

void CanopenRun(void)
{
    canopen_app_process();      // NMT, Heartbeat, SDO, EMCY
    canopen_app_interrupt();    // SYNC, RPDO, TPDO
}
