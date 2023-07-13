#include <Arduino.h>
#include "DmxNet.h"
#include "configuration.h"
#include "esp_dmx.h"
#include <FastLED.h>

/*

This class is responsible for sending and receiving messages over the NovaNet protocol.

*/

DmxNet *dmxNet = NULL;

dmx_port_t dmxPort = 1;
byte data[DMX_PACKET_SIZE];

unsigned long lastUpdate = millis();

#define NUM_LEDS 4
CRGBArray<NUM_LEDS> leds;
uint8_t hue = 0;

DmxNet::DmxNet()
{

    Serial.println("DmxNet setup started");

    /* Set the DMX hardware pins to the pins that we want to use. */
    dmx_set_pin(dmxPort, DMX_DI, DMX_RO, 0);

    /* Now we can install the DMX driver! We'll tell it which DMX port to use and
      which interrupt priority it should have. If you aren't sure which interrupt
      priority to use, you can use the macro `DMX_DEFAULT_INTR_FLAG` to set the
      interrupt to its default settings.*/
    dmx_driver_install(dmxPort, DMX_DEFAULT_INTR_FLAGS);
    // Setup goes in here
}

void DmxNet::loop()
{
    // Serial.println("NovaNet loop");
    // delay(1000);

    static uint8_t hue;

    /* Get the current time since boot in milliseconds so that we can find out how
     long it has been since we last updated data and printed to the Serial
     Monitor. */
    unsigned long now = millis();
    // leds[0].

    if (now - lastUpdate >= 70)
    {
        /* Increment every byte in our packet. Notice we don't increment the zeroeth
          byte, since that is our DMX start code. Then we must write our changes to
          the DMX packet. Maximum is kept in DMX_PACKET_SIZE */
        data[0] = 0x00; // Null

        /*
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
        */

        for (int i = 0; i < NUM_LEDS; i++)
        {
            // let's set an led value
            leds[i] = CHSV(hue + (i * 127), 255, 255);
        }

        data[1] = 0xff; // Brightness
        data[2] = leds[0].r;
        data[3] = leds[0].g;
        data[4] = leds[0].b;

        data[8] = 0xff; // Brightness
        data[9] = leds[1].r;
        data[10] = leds[1].g;
        data[11] = leds[1].b;

        hue++;

        dmx_write(dmxPort, data, DMX_PACKET_SIZE);

        /* Log our changes to the Serial Monitor. */
        lastUpdate = now;
        // Serial.printf("Sending DMX 0x%02X 0x%02X 0x%02X\n", data[2], data[3], data[4]);
        // Serial.printf("            0x%02X 0x%02X 0x%02X\n", data[9], data[10], data[11]);
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
