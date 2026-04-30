#include <Arduino.h>
#include "ODriveCAN.h"
#include "ODriveESP32TWAI.hpp"
#include "driver/twai.h"

// SN65HVD230 on UART2 pins
#define CAN_TX_PIN 17
#define CAN_RX_PIN 16
#define CAN_BAUDRATE 500000
#define ODRV0_NODE_ID 0
#define ACTIVE_TX_TEST false
#define HEARTBEAT_STALE_MS 7000

void onCanMessage(const CanMsg& msg);

ESP32TWAIIntf can_intf;
static ODriveCAN* odrv0_ptr = nullptr;
volatile uint32_t heartbeat_rx_count = 0;
volatile uint32_t feedback_rx_count = 0;
volatile uint32_t last_heartbeat_interval_ms = 0;

struct ODriveUserData {
  Heartbeat_msg_t last_heartbeat;
  bool received_heartbeat = false;
  unsigned long last_heartbeat_ms = 0;
  Get_Encoder_Estimates_msg_t last_feedback;
  bool received_feedback = false;
  Get_Bus_Voltage_Current_msg_t last_vbus;
  bool received_vbus = false;
} odrv0_user_data;

void onHeartbeat(Heartbeat_msg_t& msg, void* user_data) {
  ODriveUserData* d = static_cast<ODriveUserData*>(user_data);
  unsigned long now = millis();
  if (d->last_heartbeat_ms != 0) {
    last_heartbeat_interval_ms = (uint32_t)(now - d->last_heartbeat_ms);
  }
  d->last_heartbeat = msg;
  d->received_heartbeat = true;
  d->last_heartbeat_ms = now;
  heartbeat_rx_count++;
}

void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data) {
  ODriveUserData* d = static_cast<ODriveUserData*>(user_data);
  d->last_feedback = msg;
  d->received_feedback = true;
  feedback_rx_count++;
}

void onCanMessage(const CanMsg& msg) {
  if (odrv0_ptr) onReceive(msg, *odrv0_ptr);
}

// Recover TWAI from bus-off or stopped state. No-op if healthy.
void recoverCan() {
  twai_status_info_t status;
  if (twai_get_status_info(&status) != ESP_OK) return;
  if (status.state == TWAI_STATE_BUS_OFF) {
    Serial.printf("TWAI bus-off! txErr=%lu rxErr=%lu\n",
                  status.tx_error_counter, status.rx_error_counter);
    twai_initiate_recovery();
    for (int i = 0; i < 20; ++i) {
      delay(10);
      twai_get_status_info(&status);
      if (status.state == TWAI_STATE_STOPPED) break;
    }
    twai_start();
    Serial.println("TWAI recovered from bus-off");
  } else if (status.state == TWAI_STATE_STOPPED) {
    twai_start();
    Serial.println("TWAI restarted from stopped");
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n\n=== ODrive CAN Test ===");
  Serial.printf("TX=GPIO%d  RX=GPIO%d  Baud=%d  NodeID=%d\n",
                CAN_TX_PIN, CAN_RX_PIN, CAN_BAUDRATE, ODRV0_NODE_ID);

  // Create ODriveCAN instance and register callbacks BEFORE starting TWAI
  static ODriveCAN odrv0(wrap_can_intf(can_intf), ODRV0_NODE_ID);
  odrv0_ptr = &odrv0;
  odrv0.onStatus(onHeartbeat, &odrv0_user_data);
  odrv0.onFeedback(onFeedback, &odrv0_user_data);

  // Start TWAI driver
  twai_general_config_t g_cfg = TWAI_GENERAL_CONFIG_DEFAULT(
      (gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
  g_cfg.rx_queue_len = 64;
  g_cfg.alerts_enabled = TWAI_ALERT_ALL;
  twai_timing_config_t t_cfg = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_cfg = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  esp_err_t err = twai_driver_install(&g_cfg, &t_cfg, &f_cfg);
  if (err != ESP_OK) {
    Serial.printf("TWAI install failed: %d\n", (int)err);
    return;
  }
  err = twai_start();
  if (err != ESP_OK) {
    Serial.printf("TWAI start failed: %d\n", (int)err);
    return;
  }
  Serial.println("TWAI started OK");

  // Wait up to 10s for heartbeat
  Serial.println("Waiting for ODrive heartbeat (10s)...");
  unsigned long deadline = millis() + 10000;
  while (!odrv0_user_data.received_heartbeat && millis() < deadline) {
    pumpEvents(can_intf);
    delay(10);
  }

  if (!odrv0_user_data.received_heartbeat) {
    Serial.println("TIMEOUT — no heartbeat received");

    twai_status_info_t s;
    twai_get_status_info(&s);
    Serial.printf("  TWAI state     : %d\n", s.state);
    Serial.printf("  TX error count : %lu\n", s.tx_error_counter);
    Serial.printf("  RX error count : %lu\n", s.rx_error_counter);
    Serial.printf("  Bus errors     : %lu\n", s.bus_error_count);
    Serial.printf("  TX failed      : %lu\n", s.tx_failed_count);
    Serial.printf("  RX missed      : %lu\n", s.rx_missed_count);

    uint32_t alerts = 0;
    twai_read_alerts(&alerts, 0);
    Serial.printf("  Alerts         : 0x%08X\n", alerts);
    if (alerts == 0)
      Serial.println("  -> RX line completely silent (wiring/RS pin/ODrive CAN not enabled)");
    if (alerts & TWAI_ALERT_BUS_ERROR)
      Serial.println("  -> BUS_ERROR: frames arriving but unreadable (baud rate mismatch?)");
    if (alerts & TWAI_ALERT_BUS_OFF)
      Serial.println("  -> BUS_OFF: no termination or no ACK");
    if (alerts & TWAI_ALERT_ERR_PASS)
      Serial.println("  -> ERR_PASSIVE");
    if (alerts & TWAI_ALERT_TX_FAILED)
      Serial.println("  -> TX_FAILED: sent frames not acknowledged");
    return;
  }

  Serial.println("Heartbeat received!");
  Serial.printf("  Axis_State : %lu\n", odrv0_user_data.last_heartbeat.Axis_State);
  Serial.printf("  Axis_Error : 0x%08lX\n", odrv0_user_data.last_heartbeat.Axis_Error);

  if (ACTIVE_TX_TEST) {
    // Optional transmit test: request Vbus to verify bidirectional communication.
    Serial.println("Requesting Vbus...");
    if (odrv0.request(odrv0_user_data.last_vbus, 1000)) {
      Serial.printf("  Bus Voltage : %.2f V\n", odrv0_user_data.last_vbus.Bus_Voltage);
      Serial.printf("  Bus Current : %.2f A\n", odrv0_user_data.last_vbus.Bus_Current);
    } else {
      Serial.println("  Vbus request timed out");
    }
  } else {
    Serial.println("RX-only mode: transmit requests disabled");
  }

  Serial.println("\nSetup complete — loop will print heartbeat every second");
}

void loop() {
  recoverCan();  // no-op if healthy; recovers automatically from bus-off
  pumpEvents(can_intf);

  static unsigned long last_print = 0;
  unsigned long now = millis();
  if (now - last_print >= 1000) {
    last_print = now;
    if (odrv0_user_data.last_heartbeat_ms != 0 && (now - odrv0_user_data.last_heartbeat_ms) <= HEARTBEAT_STALE_MS) {
      Serial.printf("[HB] state=%lu  error=0x%08lX\n",
                    odrv0_user_data.last_heartbeat.Axis_State,
                    odrv0_user_data.last_heartbeat.Axis_Error);
    } else {
      if (odrv0_user_data.last_heartbeat_ms == 0) {
        Serial.println("[HB] No heartbeat yet");
      } else {
        Serial.printf("[HB] Heartbeat stale (%lums old)\n", now - odrv0_user_data.last_heartbeat_ms);
      }
    }
    if (odrv0_user_data.received_feedback) {
      odrv0_user_data.received_feedback = false;
      Serial.printf("[FB] pos=%.3f  vel=%.3f rev/s\n",
                    odrv0_user_data.last_feedback.Pos_Estimate,
                    odrv0_user_data.last_feedback.Vel_Estimate);
    }

    uint32_t alerts = 0;
    twai_read_alerts(&alerts, 0);
    if (alerts) {
      Serial.printf("[ALERT] 0x%08lX", alerts);
      if (alerts & TWAI_ALERT_BUS_ERROR) Serial.print(" BUS_ERROR");
      if (alerts & TWAI_ALERT_BUS_OFF) Serial.print(" BUS_OFF");
      if (alerts & TWAI_ALERT_ERR_PASS) Serial.print(" ERR_PASSIVE");
      if (alerts & TWAI_ALERT_TX_FAILED) Serial.print(" TX_FAILED");
      if (alerts & TWAI_ALERT_RX_QUEUE_FULL) Serial.print(" RX_QUEUE_FULL");
      Serial.println();
    }

    // Print TWAI stats every 5s
    static int tick = 0;
    if (++tick % 5 == 0) {
      twai_status_info_t s;
      twai_get_status_info(&s);
      Serial.printf("[TWAI] state=%d txErr=%lu rxErr=%lu busErr=%lu rxMissed=%lu hbRx=%lu fbRx=%lu hbInt=%lums\n",
                    s.state,
                    s.tx_error_counter,
                    s.rx_error_counter,
                    s.bus_error_count,
                    s.rx_missed_count,
                    (unsigned long)heartbeat_rx_count,
                    (unsigned long)feedback_rx_count,
                    (unsigned long)last_heartbeat_interval_ms);
    }
  }
}
