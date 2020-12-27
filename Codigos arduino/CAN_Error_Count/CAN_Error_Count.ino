// CAN Receive Example
//

#include <mcp_can.h>
#include <SPI.h>

long unsigned int rxId;
unsigned char len = 8;
byte rxBuf[8];
char msgString[256];                        // Array to store serial string
int data;
#define CAN0_INT 2                              // Set INT to pin 2
MCP_CAN CAN0(10);                               // Set CS to pin 10


void setup()
{
  Serial.begin(115200);

  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if(CAN0.begin(MCP_ANY, CAN_10KBPS, MCP_8MHZ) == CAN_OK)
    Serial.println("MCP2515 Initialized Successfully!");
  else
    Serial.println("Error Initializing MCP2515...");

  CAN0.setMode(MCP_NORMAL);                     // Set operation mode to normal so the MCP2515 sends acks to received data.

  pinMode(CAN0_INT, INPUT);                            // Configuring pin for /INT input

  Serial.println("MCP2515 Library Receive Example...");
}
int errorCount=0, newData=0, previousData=0;
void loop()
{
  if(!digitalRead(CAN0_INT))                         // If CAN0_INT pin is low, read receive buffer
  {
    CAN0.readMsgBuf(&rxId, &len, rxBuf);      // Read data: len = data length, buf = data byte(s)
    data = *((int*) rxBuf);
    if(rxId == 3)     // Determine if ID is standard (11 bits) or extended (29 bits)
    {
      newData = data;
      sprintf(msgString, "ID: 0x%lX  Data: %d\n", rxId, data);
      Serial.print(msgString);
//      sprintf(msgString, " %d\n", data);
//      Serial.print(msgString);

      previousData = newData;

      if((newData - previousData) > 1 ){
        errorCount++;
        sprintf(msgString, " ErrorCount: %d \n", errorCount);
        Serial.print(msgString);
      }
      Serial.println();
    }
    if(rxId == 1)     // Determine if ID is standard (11 bits) or extended (29 bits)
    {
      sprintf(msgString, "ID: 0x%lX  Velocidade: %d\n", rxId, data);
      Serial.print(msgString);
    }
    if(rxId == 0)     // Determine if ID is standard (11 bits) or extended (29 bits)
    {
      sprintf(msgString, "ID: 0x%lX DLC: %d Comando: %d\n", rxId, (int)len, data);
      Serial.print(msgString);
    }
    if((rxId & 0x40000000) == 0x40000000){    // Determine if message is a remote request frame.
      sprintf(msgString, " REMOTE REQUEST FRAME: ID: 0x%lX  DLC: %1d  Data:%d \n", rxId, len, newData);
      Serial.print(msgString);
    }

  }
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
