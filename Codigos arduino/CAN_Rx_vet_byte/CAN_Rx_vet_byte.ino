// CAN Receive Example
///

#include <mcp_can.h>
#include <SPI.h>
#include <string.h> 


long unsigned int rxId;
unsigned char len = 8, flag_frame;
unsigned char rxBuf[8];
char msgString[128];                        // Array to store serial string

#define CAN0_INT 2                              // Set INT to pin 2
MCP_CAN CAN0(10);                               // Set CS to pin 10


void setup() 
{
  Serial.begin(115200);

  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if(CAN0.begin(MCP_ANY, CAN_20KBPS, MCP_8MHZ) == CAN_OK)
    Serial.println("MCP2515 Initialized Successfully!");
  else
    Serial.println("Error Initializing MCP2515...");

  CAN0.setMode(MCP_NORMAL);                     // Set operation mode to normal so the MCP2515 sends acks to received data.

  pinMode(CAN0_INT, INPUT);                            // Configuring pin for /INT input

  Serial.println("MCP2515 Library Receive Example...");
}
int i=0;
char msg[17];
int dado;
void loop()
{
  if(!digitalRead(CAN0_INT))                         // If CAN0_INT pin is low, read receive buffer
  {
    dado=0;
    CAN0.readMsgBuf(&rxId, &flag_frame, &len, rxBuf);      // Read data: len = data length, buf = data byte(s)
    sprintf(msgString, "ID: 0x%X  DLC: %1d  Data: \n", rxId, len);
    Serial.print(msgString);
    sprintf(msgString, " data[0]: %u\n data[1]: %u\n data[2]: %u\n data[3]: %u\n data[4]: %u\n data[5]: %u\n data[6]: %u\n data[7]: %u\n",
                       rxBuf[0], rxBuf[1], rxBuf[2], rxBuf[3], rxBuf[4], rxBuf[5], rxBuf[6], rxBuf[7] );
    Serial.print(msgString);
    Serial.println();
  }
}






/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
