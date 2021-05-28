/* 
 * Notes - 
 *
 *   Debug
 *
 *   Internal Communication - 
 *     Data rate set to 2Mbaud (2000000)
 *     Frame - 
 *       Packet Start - $
 *       Packet End   - %
 *       CRC Begin    - &
 *       General packet structure - "$SourceID,DestinationID,MessageID,Response&CRC%"
 *       Response field MUST be there. If no response, fill with 0.
 *     DevID ranges - 
 *       Master         - 1
 *       Vsense Slaves  - 2...127
 *       Isense Slave   - 128
 *       Fan Controller - 129...254
 *     Message IDs - 
 *       0   - 
 *       2   - ping. If received, respond with SourceID duplicated in Response field.
 *       1xx - request data from device, field xx
 * 
 *   Error handling - 
 *     Positive values or 0 for no error.
 *     Negative values indicate errors.
 *     Errors with greater than ____ are fatal errors, OK signal will not be sent.                                       !!!!!!!!!!!!FILL THIS VALUE IN LATER!!!!!!!!!!!!
 *     All other errors will send broadcast on CAN bus. The driver interface/VCU should acknowledge this and alert user.
 *     Update the list here as new error codes are required.
 *       No error                           - >=(0)
 *       CAN Bus Initialisation Error       - (-9) 
 *       Fatal Internal Communication Error - (-10)
 *       Non-fatal Internal Comms Error     - (-11)
 */

/*
 * Includeed Libraries
 */
#include <Arduino.h>
#include <projconst.h>
#include <errno.h>
#include <SPI.h>
#include <mcp_can.h>
#include <CRC32.h>
#include <CSV_Parser.h>

MCP_CAN CAN(10);                                            // Set CS to pin 10

/*
 *Function Prototypes
 */
void MCP2515_ISR(void);
int slave_ping(int);
void slave_tx(int, int, int);
String slave_rx(void);
float read_current(void);
float read_voltage(uint8_t,uint8_t);
float read_temp(uint8_t,uint8_t);
void fan_ctrl(uint8_t,uint8_t,bool);

/* 
 * Global variables
 */
// Error handling
int8_t err_flag=0;
// CAN
unsigned char CAN_Recv_Flag = 0;
unsigned char len = 0;
unsigned char buf[8];
char str[20];

// 1 cycle old data
float sysCurrent_old = 0;
float cellVoltages_old[CELL_MAX];
float cellTemps_old[TSENSE_MAX*VSENSE_NUM];
// Current cycle data
float sysCurrent = 0;
float cellVoltages[CELL_MAX];
float cellTemps[TSENSE_MAX*VSENSE_NUM];

void setup() {
  // put your setup code here, to run once:
/*
 * Debug Bus Initialisation
 */
  Serial.begin(115200);

/*
 * CAN Bus Initialisation
 */
  if(CAN_OK == CAN.begin(CAN_1000KBPS)) {                   // init can bus : baudrate = 1000k
    Serial.println("CAN BUS init ok!");
  } else {
    Serial.println("CAN BUS init failed!");
    err_flag = ENOCAN;
  }
  attachInterrupt(0, MCP2515_ISR, FALLING); // start interrupt

/*
 * Internal Communication Bus Initialisation
 */
  Serial1.begin(2000000); //2Mbaud
  // Voltage Sense Slaves
  for(int i=VSENSE_MIN; i<=(VSENSE_NUM+VSENSE_MIN); i++) {
    Serial.print("Voltage Sensing Slave " + i);
    if(slave_ping(i)){
      Serial.println(" is alive.");
    } else {
      Serial.println("is not responding. Please check the system.");
      err_flag = ENOSLAVE_F;
    }
  }

  // Current Sensing Slave
  Serial.print("Current Sensing Slave");
  if(slave_ping(128)) {
    Serial.println(" is alive.");
  } else {
    Serial.println(" is not responding. Please check the system.");
    err_flag = ENOSLAVE_F;
  }

  // Fan Control Slaves
  /*for(int i=FCTRL_MIN; i<=(FCTRL_MIN+FCTRL_NUM); i++) {
    Serial.print("Fan Control Slave " + i);
    if(slave_ping(i)){
      Serial.println(" is alive.");
    } else {
      Serial.println("is not responding. Please check the system.");
      err_flag = ENOSLAVE_NF;
    }
  }*/ // TO be done through vsense and temp sense slaves

}

void loop() {
  // put your main code here, to run repeatedly:
  if(CAN_Recv_Flag)                   // check if get data
  /*{
    CAN_Recv_Flag = 0;                // clear flag
    CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf

    for(int i = 0; i<len; i++) { // print the data
      Serial.print(buf[i]);Serial.print("\t");
      }
    Serial.println();
  }*/
  Serial.println("Received CAN data. Parsing not implemented yet."); 

  // Acquire current flow
  sysCurrent = read_current();

  // Acquire cell voltages
  for(int iSlaveNumber=0; iSlaveNumber < (CELL_MAX/VSENSE_MAX_CELLS); iSlaveNumber++){
    for(int jCellNumber=0; jCellNumber < VSENSE_MAX_CELLS; jCellNumber++){
      // Get cell stuff
      //cellVoltages[i] = read_voltage(i);
      cellVoltages[iSlaveNumber*jCellNumber] = read_voltage(iSlaveNumber,jCellNumber);
    }
  }

  // Acquire cell temperatures
  for(int iSlaveNumber=0; iSlaveNumber < (CELL_MAX/VSENSE_MAX_CELLS); iSlaveNumber++){
    for(int jCellNumber=0; jCellNumber < TSENSE_MAX; jCellNumber++){
      // Get cell stuff
      cellTemps[iSlaveNumber*jCellNumber] = read_temp(iSlaveNumber,jCellNumber);
    }
  }

  // Adjust fan states if needed
  for (int fanNumber = 0; fanNumber < VSENSE_NUM; fanNumber++) {
    for (int sensorNumber; sensorNumber < TSENSE_MAX; sensorNumber++) {
      if (cellTemps[fanNumber*sensorNumber] < TEMP_TARGET-TEMP_HYST_DIS) {
        fan_ctrl(fanNumber, sensorNumber,0);
      } else if (cellTemps[fanNumber*sensorNumber] > TEMP_TARGET-TEMP_HYST_EN) {
        fan_ctrl(fanNumber, sensorNumber,1);
      }
    }
  }

  sysCurrent_old = sysCurrent;
  for (int i = 0; i < (sizeof(cellVoltages)/sizeof(float)); i++) {
    cellVoltages_old[i] = cellVoltages[i];
  }
  for (int i = 0; i < (sizeof(cellTemps)/sizeof(float)); i++) {
    cellTemps_old[i] = cellTemps[i];
  }

}

void MCP2515_ISR(void)
{
  CAN_Recv_Flag = 1;
}

int slave_ping(int destID)
{
  int ret = -1;
  //Ping the slave here
  slave_tx(destID, 0, 0);
  // Implement Rx function, check response, pass if ping passed.
  String pingResponse = slave_rx();
  CSV_Parser cp(pingResponse.c_str(),"i--i",false);
  cp.parseLeftover();
  int16_t *tempID1 = (int16_t*)cp[0];
  int16_t *tempID2 = (int16_t*)cp[0]; 
  if (tempID1[0] == destID && tempID2[0] == destID) {
    ret = 0;
  }
  return ret;
}

void slave_tx(int destID, int msgID, int resp)
{
  // Transmit message to a certain slave
  String packet = String(MYID + ',' + destID + ',' + msgID+',' + resp);
  uint32_t crcsum = CRC32::calculate(packet.c_str(),packet.length());
  packet.concat('&' + crcsum + '%');
  Serial1.print('$' + packet);
}

String slave_rx(void)
{
  // Receive message from any slave
  // Wait until message is here
  while(Serial1.available() < 10);
  String rxString = Serial1.readStringUntil('%');
  
  // Extract CRC
  String crc = rxString.substring('&','%');
  crc.remove(0);
  
  // Extract actual payload
  uint8_t endIndex = rxString.lastIndexOf('&');
  uint8_t lenIndex = rxString.length();
  rxString.remove(endIndex, (lenIndex - endIndex));
  
  //Verify checksum match
  uint32_t crcsum = CRC32::calculate(rxString.c_str(),rxString.length());
  if(!crc.equals(String(crcsum))){
    // If CRC does not match, return empty string, set error flag
    err_flag = ECRCINVAL;
    return String();
  }

  // Check if data is for us
  CSV_Parser cp(rxString.c_str(),"-ud--",false);
  cp.parseLeftover();
  if ((int16_t)cp[1] != MYID) {
    return String();
  }

  // Return payload
  return rxString;
}

float read_current(void)
{
  float resp = 5000;
  slave_tx(ISENSE_SLAVE,10,0);
  // Implement Rx function, return response
  String isenseResponse = slave_rx();
  CSV_Parser cp(isenseResponse.c_str(),"i--f",false);
  cp.parseLeftover();
  int8_t *tempIsenseID = (int8_t*)cp[0];
  if (tempIsenseID[0] == ISENSE_SLAVE) {
    float *tempIsenseBuffer = (float*)cp[3];
    resp = tempIsenseBuffer[0];
  }
  return resp;
}

float read_voltage(uint8_t slaveID,uint8_t cellID)
{
  float ret=5.5;
  slave_tx(slaveID,VOLT_REQ,cellID);
  String vSenseResponse = slave_rx();
  CSV_Parser cp(vSenseResponse.c_str(),"i-if",false);
  cp.parseLeftover();
  int8_t *vSenseSlaveID = (int8_t*)cp[0];
  int8_t *vSenseMessageID = (int8_t*)cp[2];
  if (vSenseSlaveID[0] == slaveID) {
    if (vSenseMessageID[0] == cellID) {
      float *tempvSenseResponse = (float*)cp[3];
      ret = tempvSenseResponse[0];
    }
  }
  return ret;
}

float read_temp(uint8_t slaveID,uint8_t tempID)
{
  float ret=99.99;
  slave_tx(slaveID,TEMP_REQ,tempID);
  String vSenseResponse = slave_rx();
  CSV_Parser cp(vSenseResponse.c_str(),"i-if",false);
  cp.parseLeftover();
  int8_t *vSenseSlaveID = (int8_t*)cp[0];
  int8_t *vSenseMessageID = (int8_t*)cp[2];
  if (vSenseSlaveID[0] == slaveID) {
    if (vSenseMessageID[0] == tempID) {
      float *tempvSenseResponse = (float*)cp[3];
      ret = tempvSenseResponse[0];
    }
  }
  return ret;
}

void fan_ctrl(uint8_t slaveID,uint8_t fanNum,bool status) 
{
  // Enable or disable fan based on the value of status
  slave_tx(slaveID,(status?FAN_CMD_EN:FAN_CMD_DIS),fanNum);
}