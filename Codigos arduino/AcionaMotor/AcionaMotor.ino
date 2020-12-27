// CAN Send Example
//

#include <mcp_can.h>
#include <SPI.h>




MCP_CAN CAN0(10);     // Set CS to pin 10


char msgString[128];                        // Array to store serial string


void setup()
{
  Serial.begin(115200);

  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if(CAN0.begin(MCP_ANY, CAN_10KBPS, MCP_8MHZ) == CAN_OK) Serial.println("MCP2515 Initialized Successfully!");
  else Serial.println("Error Initializing MCP2515...");

  CAN0.setMode(MCP_NORMAL);   // Change to normal mode to allow messages to be transmitted
}


byte LEN_INT=2;
word parametroControle = 0x0;

word referenciaVelocidade = 0x0;
const int analogInPin = A3, ACIONA = 0x3 , DESABILITA = 0;


int sensorValue = 0, outputValue = 0, velocidadeMax = 4000;
unsigned int testeComunicacao = 0;

void loop()
{
sensorValue = 0;
  for(int i = 0; i<10; i++)
    sensorValue += analogRead(analogInPin)/10.;

  outputValue = map(sensorValue, 0, 1023, 0, velocidadeMax);
  referenciaVelocidade=(word)outputValue;
  parametroControle = ((sensorValue > 100 && referenciaVelocidade < velocidadeMax )? ACIONA : DESABILITA) ;
  byte sndStat = CAN0.sendMsgBuf(0x0, LEN_INT, (byte*)&(parametroControle));
  if(sndStat == CAN_OK){
    sprintf(msgString, "Message Sent Successfully: ID 0x00");
    Serial.print(msgString);
    sprintf(msgString, " 0x%X\n", parametroControle);
    Serial.print(msgString);
  } else {
    Serial.println("Error Sending Message...");
  }

  byte sndStat2 = CAN0.sendMsgBuf(0x01, LEN_INT, (byte*)&(referenciaVelocidade));
  if(sndStat2 == CAN_OK){
    sprintf(msgString, "Message Sent Successfully: ID 0x01");
    Serial.print(msgString);
    sprintf(msgString, " referencia: %d\n", referenciaVelocidade);
    Serial.print(msgString);
  } else {
    Serial.println("Error Sending Message...");
  }

  //envia um uma contagem de teste de comunicacao
  byte sndStat3 = CAN0.sendMsgBuf(0x02, LEN_INT, (byte*)&(testeComunicacao));
  if(sndStat3 == CAN_OK){
    sprintf(msgString, "Message Sent Successfully: ID 0x02");
    Serial.print(msgString);
    sprintf(msgString, " contagem teste: %d\n", testeComunicacao);
    Serial.print(msgString);
  } else {
    Serial.println("Error Sending Message...");
  }
  testeComunicacao++;

  Serial.println();

  delay(300);   // send data per 300ms
}
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
