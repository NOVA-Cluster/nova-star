#include <Arduino.h>
#include "NovaNet.h"
#include "configuration.h"
#include "pb_arduino.h"

#include "messaging.pb.h"

/*

This class is responsible for sending and receiving messages over the NovaNet protocol.

*/

NovaNet *novaNet = NULL;

NovaNet::NovaNet()
{

    messaging_Request request = messaging_Request_init_zero;
    Serial.println("NovaNet setup started");

    Serial2.begin(NOVANET_BAUD);

    // Setup goes in here
}

void NovaNet::loop()
{
    Serial.println("NovaNet loop");
    //sendProtobuf();
    delay(1000);
}

void NovaNet::sendProtobuf()
{

    uint8_t dmxValues[DMX512_MAX] = {};
    dmxValues[0] = 0x00;
    dmxValues[1] = 0xff; // Brightness
    dmxValues[2] = 0xff; // red
    dmxValues[3] = 0x00; // green
    dmxValues[4] = 0xff; // blue

    dmxValues[5] = 0x01; // blue
    dmxValues[6] = 0x02; // blue
    dmxValues[7] = 0x03; // blue

    // Create a DmxRequest object
    messaging_DmxRequest dmxRequest = messaging_DmxRequest_init_zero;

    memcpy(dmxRequest.values.bytes, dmxValues, sizeof(dmxValues));

    // Find the last index with data
    int lastIndexWithData = 0;
    for (int i = DMX512_MAX; i >= 0; i--)
    {
        if (dmxValues[i] != 0)
        {
            lastIndexWithData = i + 1;
            break;
        }
    }

    Serial.print("lastIndexWithData ");
    Serial.println(lastIndexWithData);

    dmxRequest.values.size = lastIndexWithData;

    dmxRequest.ack = false; // Request acknoledgement

    // Create a Request object and set its type to DMX_REQUEST
    messaging_Request request = messaging_Request_init_zero;
    // request.type = messaging_RequestType_REQUEST_DMX;
    request.request_payload.dmx_request = dmxRequest;
    request.which_request_payload = messaging_Request_dmx_request_tag;

    // Initialize a buffer stream for the encoded message
    uint8_t buffer[NOVABUF_MAX];
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

    // Encode the protobuf
    if (!pb_encode(&stream, messaging_Request_fields, &request))
    {
        // Encode error. Maybe the buffer isn't big enough?
        Serial.println("PB_Encode Error!!!");
    }

    if (1)
    {
        // Print the size of the encoded message.
        Serial.println(stream.bytes_written);

        // Print the encoded message in hexadecimal format.
        for (size_t i = 0; i < stream.bytes_written; i++)
        {
            if (buffer[i] < 16)
            {
                Serial.print('0'); // print leading zero for single-digit hex values
            }
            Serial.print(buffer[i], HEX);
        }
        Serial.println();
    }

    // Calculate the CRC of the protobuf
    uint16_t protobuf_crc = crc16_ccitt(buffer, stream.bytes_written);

    // Prepare the header: F0 9F 92 A5 followed by the CRC and the size of the protobuf
    uint8_t header[4] = {0xF0, 0x9F, 0x92, 0xA5};
    uint16_t msg_size = stream.bytes_written;

    // Send the header
    Serial2.write(header, sizeof(header));

    // Send the CRC of the protobuf
    Serial2.write((uint8_t *)&protobuf_crc, sizeof(protobuf_crc));

    // Send the size of the protobuf
    Serial2.write((uint8_t *)&msg_size, sizeof(msg_size));

    // Then send the protobuf
    Serial2.write(buffer, msg_size);

    Serial.println("Written");

    delay(100); // Wait a bit before reading
    return;

    // Read and check the header
    uint8_t received_header[4];
    while (Serial2.available() < sizeof(received_header))
    {
        // Wait until the header has been received
        yield();
    }
    Serial2.readBytes((char *)received_header, sizeof(received_header));
    if (memcmp(received_header, header, sizeof(header)) != 0)
    {
        // Handle the error: invalid header
        Serial.println("NovaNet: Invalid Header");
        return;
    }

    // Read the CRC of the protobuf
    uint16_t received_protobuf_crc;
    while (Serial2.available() < sizeof(received_protobuf_crc))
    {
        // Wait until the CRC has been received
        yield();
    }
    Serial2.readBytes((char *)&received_protobuf_crc, sizeof(received_protobuf_crc));

    // Read the size of the received protobuf
    while (Serial2.available() < sizeof(msg_size))
    {
        // Wait until the size has been received
        yield();
    }
    uint16_t received_size;
    Serial2.readBytes((char *)&received_size, sizeof(received_size));

    // Wait until the entire protobuf has been received
    while (Serial2.available() < received_size)
    {
        // Wait
        yield();
    }

    // Now read the protobuf
    uint8_t received_buffer[NOVABUF_MAX];
    Serial2.readBytes((char *)received_buffer, received_size);

    // Calculate the CRC of the received protobuf
    uint16_t calculated_protobuf_crc = crc16_ccitt(received_buffer, received_size);
    if (received_protobuf_crc != calculated_protobuf_crc)
    {
        // Handle the error: invalid CRC
        Serial.println("NovaNet: Invalid receive CRC");
        return;
    }

    // Initialize a protobuf input stream
    pb_istream_t pb_istream = pb_istream_from_buffer(received_buffer, received_size);

    // Decode the received protobuf
    messaging_Request received_msg = messaging_Request_init_zero;
    if (!pb_decode(&pb_istream, messaging_Request_fields, &received_msg))
    {
        // Handle the decoding error
    }
}

// CRC-16-CCITT function
uint16_t NovaNet::crc16_ccitt(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < length; i++)
    {
        crc ^= data[i] << 8;
        for (uint16_t j = 0; j < 8; j++)
        {
            if (crc & 0x8000)
            {
                crc = (crc << 1) ^ 0x1021; // Polynomial: x^16 + x^12 + x^5 + 1
            }
            else
            {
                crc <<= 1;
            }
        }
    }

    return crc;
}