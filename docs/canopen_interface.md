# CANopen interface

How to command the drive over CANopen (CiA 301 and the CiA 402 drive profile). Back to the [README](../README.md). For an introduction to CANopen itself, read the [CANopen guide](canopen.md).

The full object list is in [canopen/od/bldc_drive.eds](../canopen/od/bldc_drive.eds). [canopen/od/bldc_drive.dbc](../canopen/od/bldc_drive.dbc) decodes the PDOs in CAN tools. The bus runs at 500 kbit/s and the node-ID is 13.

## Run the motor

To run the motor at 1000 rpm over SDO:

| Step | Object | Value | Meaning |
|---|---|---|---|
| 0 | NMT | start | Only if the node is not operational: it starts operational after boot, but a CAN error drops it to pre-operational. With the default `0x6007` = `1`, Enable Operation in pre-operational ends in `COMMUNICATION_LOST` |
| 1 | `0x6060` | `3` | Select profile velocity mode (`4` = profile torque) |
| 2 | `0x2012` | `1` | Select the sensor: `1` = Hall, `0` = sensorless |
| 3 | `0x60FF` | `1000` | Set the target velocity in rpm |
| 4 | `0x6040` | `0x06` → `0x07` → `0x0F` | Shutdown → Switch On → Enable Operation |

Write `0x06` (or `0x07`) to `0x6040` to stop. In profile velocity the speed reference falls on the `0x6084` ramp to 0 rpm with Hall or to 300 rpm sensorless, then the PWM outputs turn off and the motor coasts. The statusword leaves Operation Enabled at once, and `0x0F` is accepted again when the motor is idle. `0x00` or `0x02` during the ramp turns off the PWM outputs at once. In profile torque mode, a stop turns off the PWM outputs at once. To clear a fault, write `0x00`, then `0x80`, then `0x00`.

A state command such as `0x06` written during start-up is not lost. `0x6040` keeps the value, and the drive acts on it when `INIT` ends.

To measure the Hall table, write `4` to `0x2011` while the motor is idle. The motor turns for about 10 s. Wait until `0x2002` shows `IDLE` again before you start it.

## Main objects

| Object | Name | Unit or values |
|---|---|---|
| `0x6040` / `0x6041` | Controlword / Statusword | CiA 402 |
| `0x6060` / `0x6061` | Modes of operation / display | `3` = profile velocity, `4` = profile torque |
| `0x60FF` / `0x606C` | Target velocity / actual velocity | rpm |
| `0x6071` / `0x6077` | Target torque / actual torque | ‰ of `0x6076` (400 mN·m) |
| `0x6083` / `0x6084` | Profile acceleration / deceleration | rpm/s. `0x6084` also sets the stop ramp. Default 1000 |
| `0x6087` | Torque slope | ‰/s |
| `0x6007` | Abort connection option code | `0` none, `1` fault (default), `2` disable voltage, `3` quick stop |
| `0x603F` | Error code | CiA 402, see [Fault codes](#fault-codes) |
| `0x1016:01..08` | Consumer heartbeat time | node-ID in bits 16..23, time in ms in bits 0..15. `0x000101F4` = node 1, 500 ms |
| `0x1400:05` / `0x1401:05` | RPDO1 / RPDO2 event timer | timeout in ms, `0` = off. Set it longer than the master's period |
| `0x2000` / `0x2001` | DC-bus voltage / speed | mV / rpm |
| `0x2002` / `0x2003` | Motor state / fault code | numbers from [state_machine.hpp](../motor_control/inc/state_machine.hpp) |
| `0x2010` | Speed reference without a ramp | rpm |
| `0x2011` | Command | `2` = stop, `4` = calibrate Hall |
| `0x2012` | Sensor mode (profile velocity only) | `0` = sensorless, `1` = Hall |

## PDOs

PDOs for node-ID 13:

| PDO | COB-ID | Content |
|---|---|---|
| RPDO1 | `0x20D` | Controlword, target velocity |
| RPDO2 | `0x30D` | Controlword, target torque |
| TPDO1 | `0x18D` | Statusword, actual velocity |
| TPDO2 | `0x28D` | Statusword, actual torque, mode display |

The drive applies RPDOs on receipt and sends TPDOs every 100 ms. For synchronous TPDOs, set `0x1800:02` (TPDO1) or `0x1801:02` (TPDO2) to a value from 1 to 240.

## Fault codes

A motor fault sets `0x2003`, `0x603F` and the statusword fault bit, and sends an EMCY.

| `0x2003` | Fault | `0x603F` |
|---|---|---|
| 1 | `DRIVER_FAULT`: gate driver nFAULT | `0x5000` |
| 2 | `MOTOR_OVER_CURRENT`: above 20 A in a phase | `0x2310` |
| 3 | `DRIVER_OVER_CURRENT` (never raised, see the README) | `0x2300` |
| 4 | `OVER_VOLTAGE`: DC bus above 30 V | `0x3210` |
| 5 | `UNDER_VOLTAGE`: DC bus below 12 V | `0x3220` |
| 6 | `MOTOR_OVER_TEMP` (never raised: the motor has no sensor) | `0x4310` |
| 7 | `DRIVER_OVER_TEMP`: board NTC above 85 °C | `0x4210` |
| 8 | `SENSOR_ERROR` | `0x7300` |
| 9 | `SENSORLESS_LOST`: SMO speed below 200 rpm for 50 ms | `0xFF01` |
| 10 | `COMMUNICATION_LOST`: see below | `0x8100` |
| 11 | `FOC_RATE_ERROR`: the FOC interrupt does not run at 20 kHz | `0x6100` |

Communication errors come from CANopenNode as EMCY messages. The second column is the error status bit in the EMCY data:

| EMCY code | Bit | Cause |
|---|---|---|
| `0x8130` | `0x1B` | Heartbeat consumer timeout |
| `0x8130` | `0x1C` | The master sent a boot-up (it was reset) |
| `0x8250` | `0x17` | RPDO timeout |
| `0x8140` | `0x12` | CAN bus-off |
| `0x8110` | `0x14` | CAN transmit buffer overflow |
| `0x1000` | `0x20` | EMCY queue full |

## Communication loss

When the node leaves operational during Operation Enabled, `0x6007` sets the reaction. Values `2` and `3` both let the motor coast. The drive sees a silent master only if the master writes `0x1016` (heartbeat consumer) or `0x1400:05` and `0x1401:05` (RPDO timeout) after each boot. These are 0 by default, and watching starts at the first heartbeat or RPDO.

- A start by SDO in pre-operational also ends in this fault, so send NMT start first.
- After a communication loss, the order is: NMT start, then controlword `0x00` → `0x80` → `0x00`.
- A CAN communication error, such as the master's adapter closing, moves the node to pre-operational, and PDOs stop. Closing the CAN adapter while the motor runs stops the motor: no node ACKs the frames, the transmit buffer overflows, and the node moves to pre-operational.
- After such an error, the master waits until bit 4 of `0x1001` is clear, then sends NMT start. An NMT start sent while the error is still active has no lasting effect.

## Control owner: CAN or Scrutiny

`cmd.control_source` in Scrutiny selects who may start the motor: `CANOPEN` (0, default) or `SCRUTINY` (1). A change stops the motor. In `SCRUTINY` mode statusword bit 9 (remote) is 0, `0x2011` calibrate is ignored (stop still works), and a lost master does not affect the motor.
