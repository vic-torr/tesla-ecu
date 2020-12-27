#include <mcp_can.h>
#include <SPI.h>

const int SPI_CS_PIN = 9;
const int INT_PIN = 2;

MCP_CAN CAN(SPI_CS_PIN);                                    // Set CS pin

long unsigned int rxId;
unsigned char lenRxBuf = 2;
unsigned char lenTxBuf = 2;
unsigned char rxBuf[100];
word txBuf[8] = {0x07, 0x06, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
byte sndStat;
unsigned char flagRecv = 0;
char msgString[100];

void setup()
{
  Serial.begin(115200);

  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  while (CAN_OK != CAN.begin(CAN_10KBPS))              // init can bus : baudrate = 500k
  {
      Serial.println("CAN BUS Shield init fail");
      Serial.println(" Init CAN BUS Shield again");
      delay(100);
  }
  Serial.println("CAN BUS Shield init ok!");

  attachInterrupt(digitalPinToInterrupt(INT_PIN), MCP2515_ISR, FALLING); // start interrupt

}

void MCP2515_ISR()
{
    flagRecv = 1;
}

void loop()
{
  // send data:  ID = 0x2, non-Extended CAN Frame, Data length = 2 bytes, 'data' = dado
  byte sndStat = CAN.sendMsgBuf(0x10, 0,2,(byte*)txBuf);

  if(sndStat == CAN_OK){
    Serial.println("Mensagem enviada:");
    for(int i = 0; i<lenTxBuf; i++)
    {
      Serial.print(txBuf[i]);Serial.print("\t");
    }
    Serial.println();
    if(CAN.checkReceive())
    {
      Serial.println(" mensagem entregue\n");
    }
  }
  else
  {
    Serial.println("Error Sending Message... \n");
  }
  if(CAN.checkError()){
    sprintf(msgString, " flag de erro: %d\n\n", CAN.checkError());
    Serial.print(msgString);
  }
  delay(1000);   // send data per 100ms


  /*
  if(flagRecv)
  {                                   // check if get data

      flagRecv = 0;                   // clear flag

      // iterate over all pending messages
      // If either the bus is saturated or the MCU is busy,
      // both RX buffers may be in use and reading a single
      // message does not clear the IRQ conditon.
      while (CAN_MSGAVAIL == CAN.checkReceive())
      {
          // read data,  len: data length, buf: data buf
          CAN.readMsgBuf(&lenRxBuf, rxBuf);

          // print the data
          Serial.print("Mensagem recebida:");
          for(int i = 0; i<lenRxBuf; i++)
          {
              Serial.print(rxBuf[i]);Serial.print("\t");
              txBuf[i] = rxBuf[i]+1;
          }
          Serial.println();
      }

  }
  */
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
