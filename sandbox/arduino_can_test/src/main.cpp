// CAN Send Example
//

#include <Arduino.h>
#include <mcp_can.h>
#include <SPI.h>

MCP_CAN CAN0(10);     // Set CS to pin 10


  uint32_t counter = 0;
  float value = 12.34f;


void setup()
{
  Serial.begin(115200);

  if(CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_16MHZ) == CAN_OK) {
    Serial.println("MCP2515 Initialized Successfully!");
  } 
  else {
    Serial.println("Error Initializing MCP2515...");
  }

  CAN0.setMode(MCP_NORMAL);   // Change to normal mode to allow messages to be transmitted

}


void loop()
{


  byte data[8];

    // uint32_t → 4バイト
    memcpy(&data[0], &counter, sizeof(counter));

    // float → 4バイト
    memcpy(&data[4], &value, sizeof(value));

    if (CAN0.sendMsgBuf(0x400, 0, 8, data) == CAN_OK)
    {
        Serial.print("counter = ");
        Serial.print(counter);

        Serial.print(", value = ");
        Serial.println(value);
    } 

    counter++;
    value += 0.1f;

    delay(1000);
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

