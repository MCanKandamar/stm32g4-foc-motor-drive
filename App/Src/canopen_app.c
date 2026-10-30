#include "canopen_app.h"
#include "canopen_bridge.h"
#include "CO_app_STM32.h"

#define CO_NODE_ID      13       // node id (1..127)
#define CO_BAUDRATE     500     // kbit/s

extern FDCAN_HandleTypeDef hfdcan1;

static CANopenNodeSTM32 co_node;

void CanopenInit(void)
{
    co_node.CANHandle      = &hfdcan1;
    co_node.HWInitFunction = MX_FDCAN1_Init;
    co_node.timerHandle    = NULL;              
    co_node.desiredNodeID  = CO_NODE_ID;
    co_node.baudrate       = CO_BAUDRATE;

    canopen_app_init(&co_node);
    CanopenBridgeInit();
}

void CanopenRun(void)
{
    canopen_app_process();      // NMT, Heartbeat, SDO, EMCY
    CanopenBridgeUpdate();      // OD <-> motor data
    canopen_app_interrupt();    // SYNC, RPDO, TPDO

    // handles standard LED indicators
    HAL_GPIO_WritePin(CAN_LED_GPIO_Port, CAN_LED_Pin,
                      co_node.outStatusLEDRed ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
