#ifndef DMXNET_H
#define DMXNET_H

#pragma once

#include <Arduino.h>


class DmxNet
{
private:
public:
    DmxNet();

    void loop();

    void receiveProtobuf();

    uint16_t crc16_ccitt(const uint8_t* data, uint16_t length);
};

extern DmxNet *dmxNet;

#endif