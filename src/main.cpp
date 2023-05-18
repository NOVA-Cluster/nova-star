#include <Arduino.h>
#include "configuration.h"
#include <Adafruit_MCP23X17.h>

Adafruit_MCP23X17 mcp_a;

void setup()
{
  Serial.begin(921600);
  delay(500); // Give time for the serial monitor to connect
  Serial.println("NOVA: STARBASE");

  uint8_t blowerDuty = 250;
  float blowerVolt = ((blowerDuty / 255.0) * 24.0);

  // pinMode(FOG_STATUS, INPUT);
  // pinMode(FOG_POWER, OUTPUT);
  // pinMode(FOG_ACTIVATE, OUTPUT);

  pinMode(BLOWER_DUTY_PIN, OUTPUT);

  Serial.print("Blower: Setting up Blower Duty Pin (Max is 255) @ ");
  Serial.print(blowerDuty);
  Serial.print(" or  ");
  Serial.print(blowerVolt);
  Serial.println(" volts DC");

  // digitalWrite(BLOWER_DUTY_PIN, HIGH);
  analogWrite(BLOWER_DUTY_PIN, blowerDuty); // Note: Best not to use blower below ~130 (~12.2v)
  // analogWrite(BLOWER_DUTY_PIN, 255); // Note: Best not to use blower below ~130 (~12.2v)

  Serial.println("MCP23X17: interfaces setup.");
  if (!mcp_a.begin_I2C(0x20))
  {
    Serial.println("MCP23X17: interfaces setup error. May be a problem with the I2C bus.");
    while (1)
    {
      // Do nothing
    };
  }

  Serial.println("Fog Machine: Setting Pin States.");

  mcp_a.pinMode(FOG_STATUS, INPUT);
  mcp_a.pinMode(FOG_POWER, OUTPUT);
  mcp_a.pinMode(FOG_ACTIVATE, OUTPUT);

  Serial.println("Fog Machine: Turning on power to the Fog Machine.");
  mcp_a.digitalWrite(FOG_POWER, HIGH);

  Serial.println("Fog Machine: Ensuring the machine is not currently active.");
  mcp_a.digitalWrite(FOG_ACTIVATE, LOW);
}

void loop()
{
  // put your main code here, to run repeatedly:
  mcp_a.digitalWrite(FOG_POWER, HIGH);

  // if (1)
  if (mcp_a.digitalRead(FOG_STATUS))
  {
    uint16_t fogRandomDelay = 0;
    uint16_t fogRandomOutputDelay = 0;
    fogRandomDelay = random(5, 30);
    fogRandomOutputDelay = random(200, 1000);

    Serial.print("Fog Machine: Will activate for ");
    Serial.print(fogRandomOutputDelay);
    Serial.println(" ms.");
    mcp_a.digitalWrite(FOG_ACTIVATE, HIGH);
    delay(fogRandomOutputDelay);

    Serial.print("Fog Machine: Will delay for ");
    Serial.print(fogRandomDelay);
    Serial.println(" seconds.");
    mcp_a.digitalWrite(FOG_ACTIVATE, LOW);
    delay(fogRandomDelay * 1000);
  }
}