#include <Arduino.h>
#include "ODriveCAN.h"
#include "driver/twai.h"
#include "ODriveESP32TWAI.hpp"

#ifndef CAN_TX_PIN
#define CAN_TX_PIN 17
#endif
#ifndef CAN_RX_PIN
#define CAN_RX_PIN 16
#endif
#ifndef ODRIVE_NODE_ID
#define ODRIVE_NODE_ID 0
#endif
#ifndef CAN_BAUD_RATE
#define CAN_BAUD_RATE 1000000
#endif

ESP32TWAIIntf can_intf;

bool setupCan() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)CAN_TX_PIN,
        (gpio_num_t)CAN_RX_PIN,
        TWAI_MODE_NORMAL
    );
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) return false;
    if (twai_start() != ESP_OK) { twai_driver_uninstall(); return false; }
    return true;
}

ODriveCAN odrv0(wrap_can_intf(can_intf), ODRIVE_NODE_ID);
ODriveCAN* odrives[] = {&odrv0};

struct ODriveUserData {
    Heartbeat_msg_t last_heartbeat;
    bool received_heartbeat = false;
    Get_Encoder_Estimates_msg_t last_feedback;
    bool received_feedback = false;
    Get_Bus_Voltage_Current_msg_t last_busVI;
    bool received_busVI = false;
};

ODriveUserData odrv0_user_data;

void onHeartbeat(Heartbeat_msg_t& msg, void* user_data) {
    ODriveUserData* d = static_cast<ODriveUserData*>(user_data);
    d->last_heartbeat = msg;
    d->received_heartbeat = true;
}

void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data) {
    ODriveUserData* d = static_cast<ODriveUserData*>(user_data);
    d->last_feedback = msg;
    d->received_feedback = true;
}

void onBusVI(Get_Bus_Voltage_Current_msg_t& msg, void* user_data) {
    ODriveUserData* d = static_cast<ODriveUserData*>(user_data);
    d->last_busVI = msg;
    d->received_busVI = true;
}

void onCanMessage(const CanMsg& msg) {
    for (auto odrive : odrives) onReceive(msg, *odrive);
}

// Recover TWAI from bus-off or stopped state. Returns true if action was taken.
bool recoverCan() {
    twai_status_info_t status;
    if (twai_get_status_info(&status) != ESP_OK) return false;

    if (status.state == TWAI_STATE_BUS_OFF) {
        Serial.print("TWAI bus-off! TX err:");
        Serial.print(status.tx_error_counter);
        Serial.print(" RX err:");
        Serial.println(status.rx_error_counter);
        twai_initiate_recovery();  // starts 128 recessive-bit recovery sequence
        // Wait for recovery to complete (up to 200ms)
        for (int i = 0; i < 20; ++i) {
            delay(10);
            twai_get_status_info(&status);
            if (status.state == TWAI_STATE_STOPPED) break;
        }
        twai_start();
        Serial.println("TWAI recovered");
        return true;
    }

    if (status.state == TWAI_STATE_STOPPED) {
        twai_start();
        Serial.println("TWAI restarted");
        return true;
    }

    return false;
}

void setup() {
    Serial.begin(115200);
    for (int i = 0; i < 30 && !Serial; ++i) delay(100);
    delay(200);

    Serial.println("Starting ODriveCAN...");

    odrv0.onFeedback(onFeedback, &odrv0_user_data);
    odrv0.onStatus(onHeartbeat, &odrv0_user_data);
    odrv0.onBusVI(onBusVI, &odrv0_user_data);

    if (!setupCan()) {
        Serial.println("CAN init failed — reset required");
        while (true);
    }

    Serial.println("Waiting for ODrive heartbeat...");
    while (!odrv0_user_data.received_heartbeat) pumpEvents(can_intf);
    Serial.println("ODrive found");

    // Single immediate VBus read (awaitMsg pumps events internally)
    Get_Bus_Voltage_Current_msg_t vbus;
    if (odrv0.getBusVI(vbus, 100)) {
        Serial.print("VBus [V]: "); Serial.println(vbus.Bus_Voltage);
        Serial.print("IBus [A]: "); Serial.println(vbus.Bus_Current);
    } else {
        Serial.println("VBus initial read timed out (set dc_bus_voltage_rate_ms in odrivetool)");
    }
}

void loop() {
    recoverCan();  // no-op if healthy; recovers from bus-off automatically
    pumpEvents(can_intf);

    if (odrv0_user_data.received_feedback) {
        Get_Encoder_Estimates_msg_t fb = odrv0_user_data.last_feedback;
        odrv0_user_data.received_feedback = false;
        Serial.print("pos:"); Serial.print(fb.Pos_Estimate);
        Serial.print(",vel:"); Serial.println(fb.Vel_Estimate);
    }

    if (odrv0_user_data.received_busVI) {
        Get_Bus_Voltage_Current_msg_t vi = odrv0_user_data.last_busVI;
        odrv0_user_data.received_busVI = false;
        Serial.print("VBus [V]: "); Serial.print(vi.Bus_Voltage);
        Serial.print(", IBus [A]: "); Serial.println(vi.Bus_Current);
    }
}
