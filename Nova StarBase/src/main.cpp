#include <Arduino.h>
#include "configuration.h"
#include <Adafruit_MCP23X17.h>

Adafruit_MCP23X17 mcp_a;

void setup()
{
  Serial.begin(921600);
  delay(1000); // Give time for the serial monitor to connect
  Serial.println("NOVA: STARBASE");

  // pinMode(FOG_STATUS, INPUT);
  // pinMode(FOG_POWER, OUTPUT);
  // pinMode(FOG_ACTIVATE, OUTPUT);

  pinMode(BLOWER_DUTY_PIN, OUTPUT);

  Serial.println("MCP23X17 interfaces setup.");
  if (!mcp_a.begin_I2C(0x20))
  {
    Serial.println("Error - mcp_a");
    while (1)
      ;
  }

  mcp_a.pinMode(FOG_STATUS, INPUT);
  mcp_a.pinMode(FOG_POWER, OUTPUT);
  mcp_a.pinMode(FOG_ACTIVATE, OUTPUT);

  mcp_a.digitalWrite(FOG_POWER, LOW);
  mcp_a.digitalWrite(FOG_ACTIVATE, LOW);
}

void loop()
{
  // put your main code here, to run repeatedly:
  mcp_a.digitalWrite(FOG_POWER, HIGH);
  // mcp_a.digitalWrite(FOG_ACTIVATE, HIGH);
  // delay(1000);
  // mcp_a.digitalWrite(FOG_POWER, LOW);
  //  mcp_a.digitalWrite(FOG_ACTIVATE, LOW);
  if (mcp_a.digitalRead(FOG_STATUS))
  // if (1)
  {
    Serial.println("Fog machine ready!");
    mcp_a.digitalWrite(FOG_ACTIVATE, HIGH);
    delay(3000);
    mcp_a.digitalWrite(FOG_ACTIVATE, LOW);
    delay(5000);
  }
}