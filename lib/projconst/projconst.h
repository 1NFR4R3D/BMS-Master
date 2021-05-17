/*
 * This file contains global constants for the project
 */

// Internal communication Device IDs
#define MYID                1                                       // ID of this device. Ranges bet'n  1 to 254
#define VSENSE_NUM          3                                       // Number of slaves for voltage sensing. Must be betn 1 and 126
#define VSENSE_MIN          2                                       // ** DO NOT CHANGE THIS VALUE **
//#define FCTRL_NUM           2                                       // Number of slaves for fan control
//#define FCTRL_MIN           129                                     // ** DO NOT CHANGE THIS VALUE **
#define ISENSE_SLAVE        128                                     // Slave ID of current sensing slave
#define CELL_MAX            90                                      // Number of cells in the system
#define VSENSE_MAX_CELLS    15                                      // Maximum number of cells per slave
#define TSENSE_MAX          5                                       // Maximum number of temperature sensors per 
#define TEMP_TARGET         55                                      // Temperature target for the temperature sensors
#define TEMP_HYST_EN        5
#define TEMP_HYST_DIS       10

// Internal communication Message IDs
#define PING                1
#define VOLT_REQ            2
#define ISENSE_REQ          3
#define TEMP_REQ            4
#define FAN_CMD_EN          5
#define FAN_CMD_DIS         5