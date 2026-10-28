#ifndef __CONFIG_H__
#define __CONFIG_H__

// --- ESP32 pin mapping (adapted from your ESP32 example) ---
// Adjust these if your wiring differs.

#define CHARGER1_ENABLE     25   // Enable output for charger module 1
#define CHARGER2_ENABLE     33   // Enable output for charger module 2
#define CHARGER3_ENABLE     32   // Enable output for charger module 3

#define CHARGER1_ACTIVATE   4    // Activate output for charger module 1
#define CHARGER2_ACTIVATE   0    // Activate output for charger module 2
#define CHARGER3_ACTIVATE   2    // Activate output for charger module 3

#define DIG_OUT_1           26   // General digital output 1 (HV enable)
#define DIG_OUT_2           27   // General digital output 2 (AC contactor / indicator)

#define DIG_IN_1            23   // Manual start input (PP or manual enable)
#define DIG_IN_2            35   // Secondary digital input (pullup or other)

#define EVSE_ACTIVATE       22   // Output to pull pilot to 6V (EVSE activate)
#define EVSE_PILOT          21   // Pilot signal input (CP) - attachInterrupt capable
#define EVSE_PROX           36   // Proximity analog input (A0)

// EEPROM version
#define EEPROM_VERSION      11

typedef struct
{
    uint8_t  version; // eeprom version stored
    uint8_t  enabledChargers;
    uint8_t  mainsRelay; //which output is used to control the AC relay
    uint8_t  autoEnableCharger;
    uint8_t  canControl;
    uint8_t  type;
    uint8_t  phaseconfig;
    uint16_t voltSet;
    uint16_t tVolt;
    uint16_t currReq;
    uint32_t can0Speed;
    uint32_t can1Speed;
    uint16_t dcdcsetpoint;
}   ChargerParams;

#endif
