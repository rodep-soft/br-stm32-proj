// CAN Send Example
//

#include <Arduino.h>
#include <mcp_can.h>
#include <SPI.h>

MCP_CAN CAN0(10);     // Set CS to pin 10
                      //

// 4 bytes
uint32_t counter = 0;

void setup()
{
  Serial.begin(115200);
  delay(1500);

  // Initialize MCP2515 running at 8MHz with a baudrate of 1000kb/s and the masks and filters disabled.
  byte stat = CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ);
  if(stat == CAN_OK) {
    Serial.println("MCP2515 Initialized Successfully!");
  } else {
    Serial.print("Error Initializing MCP2515, code = ");
    Serial.println(stat);
  }

  CAN0.setMode(MCP_NORMAL);   // Change to normal mode to allow messages to be transmitted
}


void loop()
{
  byte data[4];
  data[0] = (counter & 0xFF);
  data[1] = (counter >> 8) & 0xFF;
  data[2] = (counter >> 16) & 0xFF;
  data[3] = (counter >> 24) & 0xFF;

  // send data:  ID = 0x100, Standard CAN Frame, Data length = 8 bytes, 'data' = array of data bytes to send
  byte sndStat = CAN0.sendMsgBuf(0x400, 0, 4, data);

  if(sndStat == CAN_OK){
    Serial.println("Message Sent Successfully!");
  } else {
    Serial.println("Error Sending Message...");
  }
  counter++;
  delay(100);   // send data per 100ms
}

/*
 * Based on the MCP_CAN_lib example by Cory J. Fowler.
 *
 * Original library:
 * https://github.com/coryjfowler/MCP_CAN_lib
 *
 * Licensed under the LGPL-3.0.
 */


/*********************************************************************************************************
  END FILE
*********************************************************************************************************/

