#include <Arduino.h>
#include "configuration.h"
#include <Adafruit_MCP23X17.h>

Adafruit_MCP23X17 mcp_a;

void setup()
{
  Serial.begin(921600);
  delay(500); // Give time for the serial monitor to connect
  Serial.println("NOVA: STARBASE");

  // pinMode(FOG_STATUS, INPUT);
  // pinMode(FOG_POWER, OUTPUT);
  // pinMode(FOG_ACTIVATE, OUTPUT);

  pinMode(BLOWER_DUTY_PIN, OUTPUT);

  Serial.println("Setting up Blower Duty Pin to 100%");
  //digitalWrite(BLOWER_DUTY_PIN, HIGH);
  analogWrite(BLOWER_DUTY_PIN, 150); // Note: Best not to use blower below ~130 (~12.2v)

  Serial.println("MCP23X17 interfaces setup.");
  if (!mcp_a.begin_I2C(0x20))
  {
    Serial.println("Error - mcp_a");
    while (1)
      ;
  }

  Serial.println("Setting Fog Machine Pin States.");

  mcp_a.pinMode(FOG_STATUS, INPUT);
  mcp_a.pinMode(FOG_POWER, OUTPUT);
  mcp_a.pinMode(FOG_ACTIVATE, OUTPUT);

  Serial.println("Turning off fog states for now.");
  mcp_a.digitalWrite(FOG_POWER, LOW);
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
    fogRandomDelay = random(10, 120);

    Serial.println("Fog machine activate!");
    mcp_a.digitalWrite(FOG_ACTIVATE, HIGH);
    delay(1500);

    Serial.print("Fog machine - Will delay for ");
    Serial.print(fogRandomDelay);
    Serial.println(" seconds.");
    mcp_a.digitalWrite(FOG_ACTIVATE, LOW);
    delay(fogRandomDelay * 1000);
  }
}