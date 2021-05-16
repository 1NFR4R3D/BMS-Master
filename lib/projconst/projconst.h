/*
 * This file contains global constants for the project
 */

// Internal communication Device IDs
#define MYID 1                                             // ID of this device. Ranges bet'n  1 to 254
#define VSENSE_NUM 3                                       // Number of slaves for voltage sensing
#define VSENSE_MIN 2                                       // ** DO NOT CHANGE THIS VALUE **
#define FCTRL_NUM 2                                        // Number of slaves for fan control
#define FCTRL_MIN 129                                      // ** DO NOT CHANGE THIS VALUE **
#define ISENSE_SLAVE 127                                   // Slave ID of current sensing slave
#define CELL_MAX 90                                        // Number of cells in the system
#define VSENSE_MAX_CELLS 15                                // Maximum number of cells per slave

// Internal communication Message IDs
#define PING 2
#define VAL_PREFIX 1