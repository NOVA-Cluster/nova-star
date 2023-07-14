#include <Arduino.h>
#include "configuration.h"
#include "NovaNet.h"
#include "NovaIO.h"
#include "DmxNet.h"




void TaskNovaNet(void *pvParameters);

void setup()
{
  Serial.begin(921600);
  delay(500); // Give time for the serial monitor to connect
  Serial.println("NOVA: STARBASE");

  pinMode(BLOWER_DUTY_PIN, OUTPUT);

  Serial.println("Setting up Serial2");
  Serial2.begin(921600, SERIAL_8N1, UART2_RX, UART2_TX);

  Serial.println("Set clock of I2C interface to 400khz");
  Wire.begin();
  Wire.setClock(400000UL);

  Serial.println("new NovaIO");
  novaIO = new NovaIO();



  uint8_t blowerDuty = 250;
  float blowerVolt = ((blowerDuty / 255.0) * 24.0);

  Serial.print("Blower: Setting up Blower Duty Pin (Max is 255) @ ");
  Serial.print(blowerDuty);
  Serial.print(" or  ");
  Serial.print(blowerVolt);
  Serial.println(" volts DC");

  Serial.println("new NovaNet");
  novaNet = new NovaNet();

  // digitalWrite(BLOWER_DUTY_PIN, HIGH);
  analogWrite(BLOWER_DUTY_PIN, blowerDuty); // Note: Best not to use blower below ~130 (~12.2v)
  // analogWrite(BLOWER_DUTY_PIN, 255); // Note: Best not to use blower below ~130 (~12.2v)

  /*
    Serial.println("MCP23X17: interfaces setup.");
    if (!mcp_a.begin_I2C(0x20))
    {
      Serial.println("MCP23X17: interfaces setup error. May be a problem with the I2C bus.");
      while (1)
      {
        // Do nothing
      };
    }
  */

  Serial.println("DMX: Setting Pin States.");
  novaIO->mcp_a.pinMode(DMX_DE, OUTPUT);
  // mcp_a.pinMode(DMX_DE, OUTPUT);
  novaIO->mcp_a.pinMode(DMX_RE, OUTPUT);
  // mcp_a.pinMode(DMX_RE, OUTPUT);

  novaIO->mcpA_digitalWrite(DMX_DE, HIGH);
  // mcp_a.digitalWrite(DMX_DE, HIGH);
  novaIO->mcpA_digitalWrite(DMX_RE, HIGH);
  // mcp_a.digitalWrite(DMX_RE, HIGH);

  Serial.println("Fog Machine: Setting Pin States.");

  novaIO->mcp_a.pinMode(FOG_STATUS, INPUT);
  novaIO->mcp_a.pinMode(FOG_POWER, OUTPUT);
  novaIO->mcp_a.pinMode(FOG_ACTIVATE, OUTPUT);

  Serial.println("Fog Machine: Turning on power to the Fog Machine.");
  novaIO->mcp_a.digitalWrite(FOG_POWER, HIGH);

  Serial.println("Fog Machine: Ensuring the machine is not currently active.");
  novaIO->mcp_a.digitalWrite(FOG_ACTIVATE, LOW);

  Serial.println("Create TaskNovaNet");
  xTaskCreate(&TaskNovaNet, "TaskNovaNet", 6 * 1024, NULL, 5, NULL);
  Serial.println("Create TaskNovaNet - Done");
}

void loop()
{
  /* Best not to have anything in this loop.
    Everything should be in freeRTOS tasks
  */
  

  // put your main code here, to run repeatedly:

  novaIO->mcpA_digitalWrite(FOG_POWER, HIGH);
  // mcp_a.digitalWrite(FOG_POWER, HIGH);

  // Todo -- Need to update this to a threadsafe wrapper
  if (novaIO->mcp_a.digitalRead(FOG_STATUS))
  {
    uint16_t fogRandomDelay = 0;
    uint16_t fogRandomOutputDelay = 0;
    fogRandomDelay = random(5, 30);
    fogRandomOutputDelay = random(200, 1000);

    Serial.print("Fog Machine: Will activate for ");
    Serial.print(fogRandomOutputDelay);
    Serial.println(" ms.");
    novaIO->mcpA_digitalWrite(FOG_ACTIVATE, HIGH);
    delay(fogRandomOutputDelay);

    Serial.print("Fog Machine: Will delay for ");
    Serial.print(fogRandomDelay);
    Serial.println(" seconds.");
    novaIO->mcpA_digitalWrite(FOG_ACTIVATE, LOW);
    delay(fogRandomDelay * 1000);
  }

}

void TaskNovaNet(void *pvParameters) // This is a task.
{
  (void)pvParameters;

  Serial.println("TaskNovaNet is running");
  while (1) // A Task shall never return or exit.
  {
    novaNet->loop();
    yield(); // Should't do anything but it's here incase the watchdog needs it.
    delay(1);
  }
}
