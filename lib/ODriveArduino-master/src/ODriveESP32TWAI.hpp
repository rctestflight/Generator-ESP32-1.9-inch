// CAN glue layer for ESP32 platforms using the native TWAI driver.
// See ODriveHardwareCAN.hpp for documentation.

#pragma once

#include "ODriveCAN.h"
#include "driver/twai.h"

// Simple struct to hold TWAI interface state. Unlike other platforms, ESP32's
// TWAI driver uses global functions rather than a class instance.
struct ESP32TWAIIntf {};

struct CanMsg {
    uint32_t id;
    uint8_t len;
    uint8_t buffer[8];
};

// Must be defined by the application
void onCanMessage(const CanMsg& msg);

static bool sendMsg(ESP32TWAIIntf& intf, uint32_t id, uint8_t length, const uint8_t* data) {
    (void)intf;
    twai_message_t tx_msg = {};
    tx_msg.identifier = id;
    tx_msg.data_length_code = length;
    tx_msg.extd = (id & 0x80000000) ? 1 : 0;
    tx_msg.rtr = (data == nullptr) ? 1 : 0;

    if (data) {
        for (int i = 0; i < length; ++i) {
            tx_msg.data[i] = data[i];
        }
    }

    return twai_transmit(&tx_msg, 0) == ESP_OK;
}

static void onReceive(const CanMsg& msg, ODriveCAN& odrive) {
    odrive.onReceive(msg.id, msg.len, msg.buffer);
}

static void pumpEvents(ESP32TWAIIntf& intf, int max_events = 100) {
    (void)intf;
    // max_events prevents an infinite loop if messages come at a high rate
    twai_message_t rx_msg;
    while (twai_receive(&rx_msg, 0) == ESP_OK && max_events--) {
        CanMsg msg;
        msg.id = rx_msg.identifier;
        msg.len = rx_msg.data_length_code;
        for (int i = 0; i < rx_msg.data_length_code && i < 8; ++i) {
            msg.buffer[i] = rx_msg.data[i];
        }
        onCanMessage(msg);
    }
}

CREATE_CAN_INTF_WRAPPER(ESP32TWAIIntf)
