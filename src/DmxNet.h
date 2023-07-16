#ifndef DMXNET_H
#define DMXNET_H

#pragma once

#include <Arduino.h>
#include "esp_dmx.h"


class DmxNet
{
private:
    byte data[DMX_PACKET_SIZE] = {0};
    uint32_t lastReceivededPacketTime = 0;

public:
    DmxNet();

    void loop();

    uint16_t crc16_ccitt(const uint8_t *data, uint16_t length);

    void receiveDMX512(uint8_t *data);
};

extern DmxNet *dmxNet;

#endif