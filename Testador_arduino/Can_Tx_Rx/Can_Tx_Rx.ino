// CAN Send Example
//

#include "mcp_can.h"
#include <SPI.h>
#include <string.h> 

//BIBLIOTECA ANTIGA
unsigned char flagRecv = 0;

unsigned char buf[8];
char str[20];
/////////////////////////////////////////////////
//BIBLIOTECA NOVA
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
  while(CAN0.begin(MCP_ANY, CAN_125KBPS, MCP_8MHZ) != CAN_OK)  Serial.println("Error Initializing MCP2515...");
  Serial.println("MCP2515 Initialized Successfully!");
  CAN0.setMode(MCP_NORMAL);   // Change to normal mode to allow messages to be transmitted
  attachInterrupt(digitalPinToInterrupt(CAN0_INT), MCP2515_ISR, FALLING);
  Serial.println("MCP2515 Library RECEIVE AND TRANSMIT Example...");
}

int data[4] = {1,2,3,4};
int i=0;
char msg[17];
int dado;

void MCP2515_ISR()
{
 
    dado=0;
    CAN0.readMsgBuf(&rxId, &flag_frame, &len, rxBuf);      // Read data: len = data length, buf = data byte(s)
    sprintf(msgString, "ID: 0x%lu  \t  \t Data: ", rxId);
    Serial.print(msgString);
    sprintf(msgString, "\tWord1: %u \tWord2: %u \tWord3: %u \tWord4: %u", *((int*)(rxBuf)), *((int*)(rxBuf+2)), *((int*)(rxBuf+4)), *((int*)(rxBuf+6)));
    Serial.print(msgString);
    Serial.println();
}


void loop()
{
  
  // send data:  ID = 0x100, Standard CAN Frame, Data length = 8 bytes, 'data' = array of data bytes to send

  byte sndStat = CAN0.sendMsgBuf(0x1, 0, 8, (byte*)data);
  if(sndStat == CAN_OK){
    Serial.println("Message Sent Successfully!");
  } else {
    Serial.println("Error Sending Message...");
  }
  delay(1000);   // send data per 100ms

////////////////////////////////////////////////////////////// 
}

/*
  //Serial.print("Rx:"); 
  //Serial.println(CAN0.errorCountRX());
  //Serial.print("Tx:");
  //Serial.println(CAN0.errorCountTX());
  Serial.print("CheckError:");
  Serial.println(CAN0.checkError());
  Serial.print("GetError:");
  Serial.println(CAN0.getError());

//////////////////////////////////////TESTAR ESSA PARTE///////////////////////////////////////////
  if(CAN0.checkError() == CAN_CTRLERROR){
    Serial.print("Error register value: ");
    byte tempErr = CAN0.getError() & MCP_EFLG_ERRORMASK; // We are only interested in errors, not warnings.
    Serial.println(tempErr, BIN);
    
    Serial.print("Transmit error counter register value: ");
    tempErr = CAN0.errorCountTX();
    Serial.println(tempErr, DEC);
    
    Serial.print("Receive error counter register value: ");
    tempErr = CAN0.errorCountRX();
    Serial.println(tempErr, DEC);
    
    //I do not have a function that clears errors and that has been added to my todo list. 04/26/17 CJF
    */
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
