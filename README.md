# REV3-TMS

## Introduction

This is the main repository for the REV3 TMS (TMS V4) to read the cooling loop temperatures, control the pumps, and read
the water flow speed. The board uses the STM32F302R8 as the main MCU and communicates over CAN with node ID 2.

## Implemented Features
The TMS currently implements the full CAN interface with 3 TPDOs (Temps and Flows) and SDO pump control. There is a 
temporary RPDO (PDO0) for the VCU to control the pump speed, but this should be removed in favor of the SDO once that 
functionality is in VCU. 

| TPDO  | DATA                                    | DataTypes |
|-------|-----------------------------------------|-----------|
| TPDO0 | BoardTemp, ExtTemp0, ExtTemp1, ExtTemp2 | INT16     |
| TPDO1 | ExtTemp3, ExtTemp4, ExtTemp5, ExtTemp6  | INT16     |
| TPDO2 | Flow1, Flow2                            | UINT16    |

The board is configured with 5 temperature sensors at the moment with 2 on bus0 and bus1 set with an address of 0x48 and 
0x4A. 

Pump PWM is functioning in that it PWMs. Has not been tested on an actual pump.

PWM input for flow is currently non-functional and temporally echos the pump speed until support is added to EVT-core.

## Debugging
In debugging, a number of CANOpen messages were constructed by hand for testing in order to control the TMS. These are 
placed here for future testing and maybe be helpful in debugging other boards.

| COB-ID (CAN-ID) | Len | Data                     | Description                                                     |
|-----------------|-----|--------------------------|-----------------------------------------------------------------|
| 0x00            | 2   | 0x01 0x00                | (NMT) Broadcast operational mode command. Starts TMS TPDO msgs. |
| 0x00            | 2   | 0x02 0x00                | (NMT) Broadcast stopped mode command. Stops TMS TPDO msgs.      |
| 0x602           | 5   | 0x2F 0x00 0x22 0x01 0x32 | (SDO) Pump 1 speed command (0-100). Replace byte 5 with speed.  |
| 0x602           | 5   | 0x2F 0x00 0x22 0x02 0x32 | (SDO) Pump 2 speed command (0-100). Replace byte 5 with speed.  |
| 0x280           | 2   | 0x32 0x32                | (RPDO) VCU TPDO to set pump speeds. Replace data with speed.    |
