#include <Arduino.h>
#include "configuration.h"
#include <Adafruit_MCP23X17.h>
// #include <DmxSimple.h>
#include <FastLED.h>
#include "esp_dmx.h"

Adafruit_MCP23X17 mcp_a;

dmx_port_t dmxPort = 1;
// byte data[DMX_PACKET_SIZE];
byte data[DMX_PACKET_SIZE];
unsigned long lastUpdate = millis();
CRGB leds[2];

void setup()
{
  Serial.begin(921600);
  delay(500); // Give time for the serial monitor to connect
  Serial.println("NOVA: STARBASE");

  pinMode(BLOWER_DUTY_PIN, OUTPUT);

  /* Set the DMX hardware pins to the pins that we want to use. */
  dmx_set_pin(dmxPort, DMX_DI, DMX_RO, 0);

  /* Now we can install the DMX driver! We'll tell it which DMX port to use and
    which interrupt priority it should have. If you aren't sure which interrupt
    priority to use, you can use the macro `DMX_DEFAULT_INTR_FLAG` to set the
    interrupt to its default settings.*/
  dmx_driver_install(dmxPort, DMX_DEFAULT_INTR_FLAGS);

  uint8_t blowerDuty = 250;
  float blowerVolt = ((blowerDuty / 255.0) * 24.0);

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

  Serial.println("DMX: Setting Pin States.");
  if (1)
  {
    mcp_a.pinMode(DMX_DE, OUTPUT);
    mcp_a.pinMode(DMX_RE, OUTPUT);

    mcp_a.digitalWrite(DMX_DE, HIGH);
    mcp_a.digitalWrite(DMX_RE, HIGH);
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

  /* Get the current time since boot in milliseconds so that we can find out how
   long it has been since we last updated data and printed to the Serial
   Monitor. */
  unsigned long now = millis();
  // leds[0].

  if (now - lastUpdate >= 1000)
  {
    /* Increment every byte in our packet. Notice we don't increment the zeroeth
      byte, since that is our DMX start code. Then we must write our changes to
      the DMX packet. Maximum is kept in DMX_PACKET_SIZE */
    data[0] = 0x00; // Null

    // Channel 1
    data[1] = 0xff; // Brightness
    data[2] = 0x00; // Red
    data[3] = 0xff; // Green
    data[4] = 0x00; // Blue
    data[5] = 0x00;
    data[6] = 0x00;
    data[7] = 0x00; // Null

    // Channel 2
    data[8] = 0xff;  // Brightness
    data[9] = 0xff;  // Red
    data[10] = 0x00; // Green
    data[11] = 0xff; // Blue
    data[12] = 0x00;
    data[13] = 0x00;
    data[14] = 0x00; // Null

    dmx_write(dmxPort, data, DMX_PACKET_SIZE);

    /* Log our changes to the Serial Monitor. */
    Serial.printf("Sending DMX 0x%02X\n", data[1]);
    lastUpdate = now;
  }

  /* Now we can transmit the DMX packet! */
  dmx_send(dmxPort, DMX_PACKET_SIZE);

  /* We can do some other work here if we want. */

  /* If we have no more work to do, we will wait until we are done sending our
    DMX packet. */
  if (!dmx_wait_sent(dmxPort, DMX_TIMEOUT_TICK))
  {
    /* If we get here, it means that we timed out waiting for the DMX packet to
      send. */
    Serial.println("DMX: Timed out waiting for DMX packet to send.");
  }
}