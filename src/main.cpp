#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include "ODriveCAN.h"
#include "ODriveESP32TWAI.hpp"
#include <SPI.h>
#include "driver/twai.h"

#define LCD_MOSI 23
#define LCD_SCK 18
#define LCD_CS 15
#define LCD_DC 2
#define LCD_RST 4
#define LCD_BLK 32

const int screenWidth = 320;
const int screenHeight = 170;
const int canTxPin = 17;
const int canRxPin = 16;
const int potPinA3 = A3;
const int potPinA0 = A0;
const int servoPin = 26;
const int resetButtonPin = 33;
const long canBaudrate = 1000000;
const int odriveNodeId = 0;
const unsigned long pollIntervalMs = 100;
const bool canMonitorOnlyMode = false;
const float maxTargetRpm = 8000.0f;
const float powerFilterAlpha = 0.02f; // bigger number = less filtering
const float rpmFilterAlpha = 0.05f;   // Smaller number = smoother RPM
const float torqueFilterAlpha = 0.05f; // bigger number = less filtering
float currentSoftMaxMax = 70.0f;
float wattHours = 0.00f;
float filteredPower = 0.0f;
bool powerFilterInitialized = false;
float filteredRpm = 0.0f;
bool rpmFilterInitialized = false;
float filteredTorque = 0.0f;
bool torqueFilterInitialized = false;

//#define SERVO_CALIBRATION_MODE  // Uncomment to enable: pot A3 sweeps servo from min to max pulse
const int servoPulseMinUs = 1000; //should correspond to idle on carburetor
const int servoPulseMaxUs = 2000; //should correspond to wide open on carburetor
const int idleRPM = 3000;         // RPM at minimum power setpoint (startup target)
const int powerSetpointMinW = 100;
const int powerSetpointMaxW = 700;
const float vFoldbackOnsetV  = 50.0f;  // voltage at which foldback begins
const float vFoldbackFullV   = 50.4f;  // voltage at which power is clamped to minimum
const float vOvervoltageTripV = 50.5f; // voltage at which motor ramps down; restarts below vFoldbackOnsetV
const int servoPwmChannel = 0;
const int servoPwmFrequencyHz = 50;
const int servoPwmResolutionBits = 16;

struct PowerPoint {
  int powerW;
  int rpmSetpoint;
  int servoUs;
};

int clampInt(int value, int minValue, int maxValue);

const PowerPoint powerCurve[] = {
  {100, 3000, 1000},
  {125, 3032, 1150},
  {150, 3040, 1225},
  {175, 3047, 1300},
  {200, 3037, 1475},
  {225, 3026, 1650},
  {250, 3271, 1800},
  {275, 3516, 1950},
  {300, 3510, 1925},
  {325, 3504, 1900},
  {350, 3764, 1900},
  {375, 4023, 1900},
  {400, 4257, 1925},
  {425, 4491, 1950},
  {450, 4496, 1900},
  {475, 4502, 1850},
  {500, 4748, 1900},
  {525, 4995, 1950},
  {550, 5254, 1850},
  {575, 6020, 1800},
  {600, 6270, 1850},
  {625, 6519, 1900},
  {650, 6755, 1875},
  {675, 6991, 1850},
  {700, 6991, 1850},
};
const int powerCurveCount = sizeof(powerCurve) / sizeof(powerCurve[0]);

const int graphY = 74;
const int graphHeight = screenHeight - graphY;
float graphRpm[screenWidth];
float graphPower[screenWidth];
int graphCount = 0;

Adafruit_ST7789 lcd = Adafruit_ST7789(LCD_CS, LCD_DC, LCD_RST);
ESP32TWAIIntf can_intf;
ODriveCAN odrive(wrap_can_intf(can_intf), odriveNodeId);

struct ODriveUserData {
  Heartbeat_msg_t last_heartbeat;
  bool received_heartbeat = false;
  unsigned long last_heartbeat_ms = 0;
  Get_Encoder_Estimates_msg_t last_feedback;
  bool received_feedback = false;
  unsigned long last_feedback_ms = 0;
  Get_Bus_Voltage_Current_msg_t last_vbus;
  bool received_vbus = false;
  unsigned long last_vbus_ms = 0;
  Get_Torques_msg_t last_torques;
  bool received_torques = false;
  Get_Iq_msg_t last_iq;
  bool received_iq = false;
  unsigned long last_iq_ms = 0;
  Get_Error_msg_t last_error;
  bool received_error = false;
} odriveUserData;

unsigned long lastPollMs = 0;
bool odriveConnected = false;
int lastSwitchRaw = HIGH;
int stableSwitchState = HIGH;         // debounced switch level: LOW=on, HIGH=off
unsigned long switchDebounceMs = 0;
const unsigned long switchDebounceDelayMs = 50;
unsigned long nextClosedLoopRequestMs = 0;
volatile uint32_t rawCanRxCount = 0;
volatile uint32_t lastRawCanId = 0;

enum EngineState { ENGINE_OFF, ENGINE_STARTING, ENGINE_RUNNING, ENGINE_STOPPING };
EngineState engineState = ENGINE_OFF;
float velocityCmd = 0.0f;
unsigned long rampStartMs = 0;
float rampStartVel = 0.0f;

void onHeartbeat(Heartbeat_msg_t& msg, void* user_data) {
  ODriveUserData* d = static_cast<ODriveUserData*>(user_data);
  static uint8_t prev_state = 0xff;
  if (msg.Axis_State != prev_state) {
    Serial.print("HB state ");
    Serial.print(prev_state);
    Serial.print("->");
    Serial.print(msg.Axis_State);
    Serial.print(" err=0x");
    Serial.print(msg.Axis_Error, HEX);
    Serial.print(" pr=");
    Serial.println(msg.Procedure_Result);
    prev_state = msg.Axis_State;
  }
  d->last_heartbeat = msg;
  d->received_heartbeat = true;
  d->last_heartbeat_ms = millis();
}

void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data) {
  ODriveUserData* d = static_cast<ODriveUserData*>(user_data);
  d->last_feedback = msg;
  d->received_feedback = true;
  d->last_feedback_ms = millis();
}

void onVbus(Get_Bus_Voltage_Current_msg_t& msg, void* user_data) {
  ODriveUserData* d = static_cast<ODriveUserData*>(user_data);
  d->last_vbus = msg;
  d->received_vbus = true;
  d->last_vbus_ms = millis();
}

void onTorques(Get_Torques_msg_t& msg, void* user_data) {
  ODriveUserData* d = static_cast<ODriveUserData*>(user_data);
  d->last_torques = msg;
  d->received_torques = true;
}

void onCurrents(Get_Iq_msg_t& msg, void* user_data) {
  ODriveUserData* d = static_cast<ODriveUserData*>(user_data);
  d->last_iq = msg;
  d->received_iq = true;
  d->last_iq_ms = millis();
}

void onError(Get_Error_msg_t& msg, void* user_data) {
  ODriveUserData* d = static_cast<ODriveUserData*>(user_data);
  static uint32_t prev_active = 0;
  static uint32_t prev_disarm = 0;
  if (msg.Active_Errors != prev_active || msg.Disarm_Reason != prev_disarm) {
    Serial.print("ODrive Active_Errors=0x");
    Serial.print(msg.Active_Errors, HEX);
    Serial.print(" Disarm_Reason=0x");
    Serial.println(msg.Disarm_Reason, HEX);
    prev_active = msg.Active_Errors;
    prev_disarm = msg.Disarm_Reason;
  }
  d->last_error = msg;
  d->received_error = true;
}

void onCanMessage(const CanMsg& msg) {
  rawCanRxCount++;
  lastRawCanId = msg.id;
  onReceive(msg, odrive);
}

bool setupCan() {
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
    (gpio_num_t)canTxPin,
    (gpio_num_t)canRxPin,
    TWAI_MODE_NORMAL
  );
  // Default rx_queue_len is 5; LCD redraw can block pumpEvents long enough
  // to drop heartbeats with 6+ broadcast streams enabled on the ODrive.
  g_config.rx_queue_len = 64;
  g_config.tx_queue_len = 16;
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    return false;
  }
  if (twai_start() != ESP_OK) {
    twai_driver_uninstall();
    return false;
  }
  return true;
}

void recoverCan() {
  twai_status_info_t status;
  if (twai_get_status_info(&status) != ESP_OK) {
    return;
  }
  if (status.state == TWAI_STATE_BUS_OFF) {
    twai_initiate_recovery();
    for (int i = 0; i < 20; ++i) {
      delay(10);
      twai_get_status_info(&status);
      if (status.state == TWAI_STATE_STOPPED) {
        break;
      }
    }
    twai_start();
  } else if (status.state == TWAI_STATE_STOPPED) {
    twai_start();
  }
}

bool requestClosedLoopIfSafe(unsigned long now, bool logState) {
  if (now < nextClosedLoopRequestMs) {
    return false;
  }

  twai_status_info_t status;
  if (twai_get_status_info(&status) != ESP_OK) {
    nextClosedLoopRequestMs = now + 500;
    return false;
  }

  if (status.state != TWAI_STATE_RUNNING) {
    nextClosedLoopRequestMs = now + 500;
    return false;
  }

  // Avoid stacking TX frames; wait until queue drains before sending more.
  if (status.msgs_to_tx > 0) {
    nextClosedLoopRequestMs = now + 100;
    return false;
  }

  if (odriveUserData.last_heartbeat.Axis_Error != 0) {
  Serial.println("ODRIVE ERROR DETECTED:");
  Serial.print("\tHeartbeat Axis_Error=0x");
  Serial.print(odriveUserData.last_heartbeat.Axis_Error, HEX);
  Serial.print("\tProcedure_Result=");
  Serial.println(odriveUserData.last_heartbeat.Procedure_Result);
  odrive.clearErrors();
}


  nextClosedLoopRequestMs = now + 500;
  const bool ok = odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
  if (logState) {
    Serial.print("Requesting CLOSED_LOOP, send=");
    Serial.print(ok ? "OK" : "FAIL");
    Serial.print(" hb_state=");
    Serial.print(odriveUserData.last_heartbeat.Axis_State);
    Serial.print(" hb_age_ms=");
    Serial.println(now - odriveUserData.last_heartbeat_ms);
  }
  return ok;
}

// ---------------- Potentiometer Median Filter + Hysteresis ----------------
const int potHysteresisThreshold = 60; // ADC counts (12-bit) to ignore — prevents servo jitter
int a0Buffer[3] = {0, 0, 0};
int a0BufferIndex = 0;
int stableA0Raw = -1;
int stableA3Raw = -1;

int applyPotHysteresis(int newVal, int& stableVal) {
  if (stableVal < 0 || abs(newVal - stableVal) > potHysteresisThreshold) {
    stableVal = newVal;
  }
  return stableVal;
}

int getMedian(int v1, int v2, int v3) {
  if (v1 > v2) {
    if (v2 > v3) return v2; // v1 > v2 > v3
    if (v1 > v3) return v3; // v1 > v3 >= v2
    return v1; // v3 >= v1 > v2
  } else {
    if (v1 > v3) return v1; // v2 >= v1 > v3
    if (v2 > v3) return v3; // v2 > v3 >= v1
    return v2; // v3 >= v2 >= v1
  }
}

int filterPotentiometer(int rawValue) {
  a0Buffer[a0BufferIndex] = rawValue;
  a0BufferIndex = (a0BufferIndex + 1) % 3;
  return getMedian(a0Buffer[0], a0Buffer[1], a0Buffer[2]);
}

int clampInt(int value, int minValue, int maxValue) {
  if (value < minValue) {
    return minValue;
  }
  if (value > maxValue) {
    return maxValue;
  }
  return value;
}

void writeServoPulseUs(int pulseUs) {
  const int clampedPulseUs = clampInt(pulseUs, servoPulseMinUs, servoPulseMaxUs);
  const uint32_t maxDuty = (1UL << servoPwmResolutionBits) - 1UL;
  const uint32_t duty = ((uint32_t)clampedPulseUs * maxDuty) / 20000UL;
  ledcWrite(servoPwmChannel, duty);
}

float filterPower(float rawPower) {
  if (!powerFilterInitialized) {
    filteredPower = rawPower;
    powerFilterInitialized = true;
    return filteredPower;
  }

  filteredPower += (rawPower - filteredPower) * powerFilterAlpha;
  return filteredPower;
}

float filterTorque(float rawTorque) {
  if (!torqueFilterInitialized) {
    filteredTorque = rawTorque;
    torqueFilterInitialized = true;
    return filteredTorque;
  }
  filteredTorque += (rawTorque - filteredTorque) * torqueFilterAlpha;
  return filteredTorque;
}

float filterRpm(float rawRpm) {
  if (!rpmFilterInitialized) {
    filteredRpm = rawRpm;
    rpmFilterInitialized = true;
    return filteredRpm;
  }

  filteredRpm += (rawRpm - filteredRpm) * rpmFilterAlpha;
  return filteredRpm;
}

void lookupPowerPoint(float powerW, float* outRpm, int* outServoUs) {
  if (powerW <= powerCurve[0].powerW) {
    *outRpm = powerCurve[0].rpmSetpoint;
    *outServoUs = powerCurve[0].servoUs;
    return;
  }
  if (powerW >= powerCurve[powerCurveCount - 1].powerW) {
    *outRpm = powerCurve[powerCurveCount - 1].rpmSetpoint;
    *outServoUs = powerCurve[powerCurveCount - 1].servoUs;
    return;
  }
  for (int i = 1; i < powerCurveCount; i++) {
    if (powerW <= powerCurve[i].powerW) {
      const float t = (powerW - powerCurve[i - 1].powerW) / (float)(powerCurve[i].powerW - powerCurve[i - 1].powerW);
      *outRpm = powerCurve[i - 1].rpmSetpoint + (powerCurve[i].rpmSetpoint - powerCurve[i - 1].rpmSetpoint) * t;
      *outServoUs = powerCurve[i - 1].servoUs + (int)((powerCurve[i].servoUs - powerCurve[i - 1].servoUs) * t + 0.5f);
      return;
    }
  }
  *outRpm = powerCurve[powerCurveCount - 1].rpmSetpoint;
  *outServoUs = powerCurve[powerCurveCount - 1].servoUs;
}

const char* engineStateToText(EngineState s) {
  switch (s) {
    case ENGINE_OFF:      return "OFF";
    case ENGINE_STARTING: return "STARTING";
    case ENGINE_RUNNING:  return "RUNNING";
    case ENGINE_STOPPING: return "STOPPING";
    default:              return "UNKNOWN";
  }
}

void appendGraphSample(float rpm, float power) {
  if (graphCount < screenWidth) {
    graphRpm[graphCount] = rpm;
    graphPower[graphCount] = power;
    graphCount++;
    return;
  }

  for (int i = 1; i < screenWidth; i++) {
    graphRpm[i - 1] = graphRpm[i];
    graphPower[i - 1] = graphPower[i];
  }

  graphRpm[screenWidth - 1] = rpm;
  graphPower[screenWidth - 1] = power;
}

int mapValueToGraphY(float value, float vMin, float vMax) {
  if (vMax <= vMin) {
    return graphY + graphHeight / 2;
  }
  float ratio = (value - vMin) / (vMax - vMin);
  if (ratio < 0.0f) ratio = 0.0f;
  if (ratio > 1.0f) ratio = 1.0f;
  return (graphY + graphHeight - 1) - (int)(ratio * (graphHeight - 1));
}

void computeRange(const float* data, int count, float* outMin, float* outMax) {
  if (count <= 0) {
    *outMin = 0.0f;
    *outMax = 1.0f;
    return;
  }

  float minVal = data[0];
  float maxVal = data[0];
  for (int i = 1; i < count; i++) {
    if (data[i] < minVal) minVal = data[i];
    if (data[i] > maxVal) maxVal = data[i];
  }

  float padding = (maxVal - minVal) * 0.1f;
  if (padding < 0.05f) padding = 0.05f;
  *outMin = minVal - padding;
  *outMax = maxVal + padding;
}

void drawGraph() {
  lcd.fillRect(0, graphY, screenWidth, graphHeight, ST77XX_BLACK);
  if (graphCount < 2) {
    return;
  }

  float rpmMin, rpmMax;
  float pwrMin, pwrMax;
  computeRange(graphRpm, graphCount, &rpmMin, &rpmMax);
  computeRange(graphPower, graphCount, &pwrMin, &pwrMax);

  for (int i = 1; i < graphCount; i++) {
    const int x1 = i - 1;
    const int x2 = i;

    const int y1R = mapValueToGraphY(graphRpm[i - 1], rpmMin, rpmMax);
    const int y2R = mapValueToGraphY(graphRpm[i], rpmMin, rpmMax);
    lcd.drawLine(x1, y1R, x2, y2R, ST77XX_CYAN);

    const int y1P = mapValueToGraphY(graphPower[i - 1], pwrMin, pwrMax);
    const int y2P = mapValueToGraphY(graphPower[i], pwrMin, pwrMax);
    lcd.drawLine(x1, y1P, x2, y2P, ST77XX_WHITE);
  }
}

void drawLine(const char* text, int y, uint16_t color, uint8_t textSize) {
  lcd.setCursor(0, y);
  lcd.setTextSize(textSize);
  lcd.setTextColor(color);
  lcd.print(text);
}

void redrawDisplay(const char* stateText, float vbusVoltage, float busCurrent, float motorPhaseCurrent, float power, float rpm, bool isClosedLoop, float wattHours, int servoPulseUs, int powerSetpointW, bool engineOn, float torqueNm) {
  char line[48];

  // Prevent long text from wrapping into the graph area.
  lcd.setTextWrap(false);

  // Clear only the text rows we redraw to avoid full-screen flicker.
  lcd.fillRect(0, 0, screenWidth, 26, ST77XX_BLACK);
  lcd.fillRect(0, 30, screenWidth, 18, ST77XX_BLACK);
  lcd.fillRect(0, 48, screenWidth, 26, ST77XX_BLACK);

  lcd.setCursor(0, 0);
  lcd.setTextSize(3);
  lcd.setTextColor(ST77XX_WHITE);
  snprintf(line, sizeof(line), "%.0f W ", power);
  lcd.print(line);
  lcd.setTextColor(ST77XX_CYAN);
  snprintf(line, sizeof(line), "RPM: %.0f", rpm);
  lcd.print(line);

  lcd.setCursor(0, 30);
  lcd.setTextSize(2);
  lcd.setTextColor(isClosedLoop ? ST77XX_GREEN : ST77XX_RED);
  lcd.print(stateText);
  lcd.setTextColor(ST77XX_YELLOW);
  lcd.print(" ");
  snprintf(line, sizeof(line), "%.1fV", vbusVoltage);
  lcd.print(line);
  lcd.setTextColor(ST77XX_MAGENTA);
  lcd.print(" ");
  snprintf(line, sizeof(line), "%.0fA", motorPhaseCurrent);
  lcd.print(line);
  lcd.setTextColor(ST77XX_GREEN);
  snprintf(line, sizeof(line), " %.2fWh", wattHours);
  lcd.print(line);

  lcd.setCursor(0, 48);
  lcd.setTextSize(2);
  lcd.setTextColor(engineOn ? ST77XX_GREEN : ST77XX_RED);
  lcd.print(engineOn ? "ON" : "OFF");
  lcd.setTextColor(ST77XX_YELLOW);
  snprintf(line, sizeof(line), " %dW", powerSetpointW);
  lcd.print(line);
  lcd.setTextColor(ST77XX_CYAN);
  snprintf(line, sizeof(line), " %dus", servoPulseUs);
  lcd.print(line);
  lcd.setTextColor(ST77XX_MAGENTA);
  snprintf(line, sizeof(line), " %.1fNm", torqueNm);
  lcd.print(line);
}

const char* axisStateToText(ODriveAxisState state) {
  switch (state) {
    case AXIS_STATE_IDLE:
      return "IDLE";
    case AXIS_STATE_STARTUP_SEQUENCE:
      return "STARTUP";
    case AXIS_STATE_FULL_CALIBRATION_SEQUENCE:
      return "CALIBRATING";
    case AXIS_STATE_MOTOR_CALIBRATION:
      return "MOTOR CAL";
    case AXIS_STATE_ENCODER_INDEX_SEARCH:
      return "INDEX SEARCH";
    case AXIS_STATE_ENCODER_OFFSET_CALIBRATION:
      return "ENC OFFSET";
    case AXIS_STATE_CLOSED_LOOP_CONTROL:
      return "ACTIVE";
    case AXIS_STATE_LOCKIN_SPIN:
      return "LOCKIN";
    case AXIS_STATE_ENCODER_DIR_FIND:
      return "DIR FIND";
    case AXIS_STATE_HOMING:
      return "HOMING";
    case AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION:
      return "HALL POL";
    case AXIS_STATE_ENCODER_HALL_PHASE_CALIBRATION:
      return "HALL PHASE";
    default:
      return "UNKNOWN";
  }
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  pinMode(potPinA0, INPUT);
  pinMode(resetButtonPin, INPUT_PULLUP);  // HIGH = 0ff, LOW = on
  ledcSetup(servoPwmChannel, servoPwmFrequencyHz, servoPwmResolutionBits);
  ledcAttachPin(servoPin, servoPwmChannel);
  writeServoPulseUs(servoPulseMinUs);

  odrive.onStatus(onHeartbeat, &odriveUserData);
  odrive.onFeedback(onFeedback, &odriveUserData);
  odrive.onBusVI(onVbus, &odriveUserData);
  odrive.onTorques(onTorques, &odriveUserData);
  odrive.onCurrents(onCurrents, &odriveUserData);
  odrive.onError(onError, &odriveUserData);

  if (!setupCan()) {
    Serial.println("CAN init failed");
    while (true);
  }
  delay(10);

  Serial.println("Waiting for ODrive...");

  const unsigned long heartbeatTimeoutMs = millis() + 8000;
  unsigned long nextHeartbeatStatusPrintMs = 0;
  while (!odriveUserData.received_heartbeat && millis() < heartbeatTimeoutMs) {
    pumpEvents(can_intf);

    const unsigned long nowMs = millis();
    if (nowMs >= nextHeartbeatStatusPrintMs) {
      nextHeartbeatStatusPrintMs = nowMs + 500;
      twai_status_info_t status;
      if (twai_get_status_info(&status) == ESP_OK) {
        Serial.print("No heartbeat yet | TWAI state=");
        Serial.print((int)status.state);
        Serial.print(" tx_err=");
        Serial.print(status.tx_error_counter);
        Serial.print(" rx_err=");
        Serial.print(status.rx_error_counter);
        Serial.print(" raw_rx=");
        Serial.print((uint32_t)rawCanRxCount);
        Serial.print(" last_id=0x");
        Serial.println((uint32_t)lastRawCanId, HEX);
      } else {
        Serial.println("No heartbeat yet | TWAI status read failed");
      }
    }

    delay(10);
  }

  if (!(odriveUserData.received_heartbeat || odriveUserData.received_feedback || odriveUserData.received_vbus)) {
    Serial.println("No ODrive telemetry detected. Check wiring, node id, baud rate, and ODrive CAN config.");
    while (true) {
      delay(1000);
    }
  }

  odriveConnected = true;
  if (!odriveUserData.received_heartbeat) {
    Serial.println("found ODrive (telemetry present, heartbeat missing)");
  } else {
    Serial.println("found ODrive");
  }

  const float vbusVoltage = odriveUserData.received_vbus ? odriveUserData.last_vbus.Bus_Voltage : 0.0f;
  //Serial.print("DC voltage: ");
  //Serial.println(vbusVoltage);

  if (!canMonitorOnlyMode) {
    odrive.setState(AXIS_STATE_IDLE);
    Serial.println("ODrive ready (engine OFF, axis IDLE).");
  } else {
    Serial.println("CAN monitor-only mode: skipping axis state commands.");
  }

  // Initialize display after CAN startup to match the known-good minimal CAN startup behavior.
  pinMode(LCD_BLK, OUTPUT);
  digitalWrite(LCD_BLK, HIGH);
  SPI.begin(LCD_SCK, -1, LCD_MOSI, LCD_CS);
  lcd.init(screenHeight, screenWidth);
  lcd.setRotation(1);
  lcd.fillScreen(ST77XX_BLACK);

  const float busCurrent = -(odriveUserData.received_vbus ? odriveUserData.last_vbus.Bus_Current : 0.0f);
  const float motorPhaseCurrent = 0.0f;
  const float power = filterPower(vbusVoltage * busCurrent);
  const float rpm = odriveUserData.received_feedback ? odriveUserData.last_feedback.Vel_Estimate * 60.0f : 0.0f;
  redrawDisplay(
    "OFF",
    vbusVoltage,
    busCurrent,
    motorPhaseCurrent,
    power,
    rpm,
    false,
    0.0f,
    servoPulseMinUs,
    0,
    false,
    0.0f
  );
  appendGraphSample(rpm, power);
  drawGraph();
}

#ifdef SERVO_CALIBRATION_MODE
void runServoCalibration() {
  const int raw = analogRead(potPinA3);
  const int pulseUs = servoPulseMinUs + (int)((raw / 4095.0f) * (servoPulseMaxUs - servoPulseMinUs));
  writeServoPulseUs(pulseUs);
  Serial.print("[CAL] pot=");
  Serial.print(raw);
  Serial.print(" servo=");
  Serial.print(pulseUs);
  Serial.println("us");
}
#endif

void loop() {
  recoverCan();
  pumpEvents(can_intf);

#ifdef SERVO_CALIBRATION_MODE
  runServoCalibration();
  delay(50);
  return;
#endif

  const unsigned long now = millis();
  const float vbusNow = odriveUserData.received_vbus ? odriveUserData.last_vbus.Bus_Voltage : 0.0f;

  // --- Motor switch: HIGH = on, LOW = off ---
  const int switchRaw = digitalRead(resetButtonPin);
  //Serial.print("[SW] pin33 raw=");
  //Serial.println(switchRaw);
  if (switchRaw != lastSwitchRaw) {
    lastSwitchRaw = switchRaw;
    switchDebounceMs = now;
  }
  if ((now - switchDebounceMs) >= switchDebounceDelayMs) {
    stableSwitchState = switchRaw;
  }

  if (stableSwitchState == LOW) {   // LOW = on (switch pulls to GND)
    if (engineState == ENGINE_OFF && vbusNow < vFoldbackOnsetV) {
      engineState = ENGINE_STARTING;
      rampStartMs = now + 1000;  // 1 s for closed-loop to engage before spinning
      rampStartVel = 0.0f;
      wattHours = 0.0f;
      odrive.setLimits(maxTargetRpm / 60.0f, currentSoftMaxMax);
      odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
      Serial.println("Engine starting: requesting closed-loop");
    }
  } else {                           // HIGH = off
    if (engineState == ENGINE_STARTING || engineState == ENGINE_RUNNING) {
      engineState = ENGINE_STOPPING;
      rampStartMs = now;
      rampStartVel = velocityCmd;
      Serial.println("Engine stopping");
    }
    // STOPPING runs to completion regardless of switch.
  }

  // Overvoltage trip: ramp down if bus voltage exceeds trip threshold
  if (vbusNow >= vOvervoltageTripV && (engineState == ENGINE_STARTING || engineState == ENGINE_RUNNING)) {
    engineState = ENGINE_STOPPING;
    rampStartMs = now;
    rampStartVel = velocityCmd;
    Serial.println("Overvoltage trip: ramping down");
  }

  // --- State machine: update velocity command and servo position ---
  const float idleVelRevPerSec = idleRPM / 60.0f;
  const int a0Filtered = applyPotHysteresis(filterPotentiometer(analogRead(potPinA0)), stableA0Raw);
  float powerSetpointW = powerSetpointMinW + (a0Filtered / 4095.0f) * (float)(powerSetpointMaxW - powerSetpointMinW);

  // Voltage foldback: proportionally cap power setpoint when bus voltage is high
  const float foldbackFrac = constrain((vbusNow - vFoldbackOnsetV) / (vFoldbackFullV - vFoldbackOnsetV), 0.0f, 1.0f);
  const float foldbackCeilingW = powerSetpointMaxW - foldbackFrac * (powerSetpointMaxW - powerSetpointMinW);
  if (powerSetpointW > foldbackCeilingW) powerSetpointW = foldbackCeilingW;

  int servoPulseUs = servoPulseMinUs;
  int powerSetpointDisplayW = 0;

  switch (engineState) {
    case ENGINE_OFF:
      velocityCmd = 0.0f;
      break;

    case ENGINE_STARTING: {
      const long elapsed = (long)(now - rampStartMs);
      if (elapsed < 0) {
        // Pre-spin delay: waiting for closed-loop to engage
        velocityCmd = 0.0f;
      } else {
        const float t = (float)elapsed / 1500.0f;
        if (t >= 1.0f) {
          velocityCmd = idleVelRevPerSec;
          engineState = ENGINE_RUNNING;
          Serial.println("Engine running");
        } else {
          velocityCmd = idleVelRevPerSec * t;
        }
      }
      powerSetpointDisplayW = powerSetpointMinW;
      break;
    }

    case ENGINE_RUNNING: {
      float lookupRpm;
      int lookupServo;
      lookupPowerPoint(powerSetpointW, &lookupRpm, &lookupServo);
      velocityCmd = lookupRpm / 60.0f;
      servoPulseUs = clampInt(lookupServo, servoPulseMinUs, servoPulseMaxUs);
      powerSetpointDisplayW = (int)(powerSetpointW + 0.5f);
      break;
    }

    case ENGINE_STOPPING: {
      const float elapsed = (float)(now - rampStartMs);
      const float t = elapsed / 1000.0f;
      if (t >= 1.0f) {
        velocityCmd = 0.0f;
        odrive.setState(AXIS_STATE_IDLE);
        engineState = ENGINE_OFF;
        Serial.println("Engine off, ODrive idle");
      } else {
        velocityCmd = rampStartVel * (1.0f - t);
      }
      break;
    }
  }

  // A3 servo trim: max A3 (4095) = no reduction, min A3 (0) = -100us
  if (engineState == ENGINE_RUNNING) {
    const int a3Raw = applyPotHysteresis(analogRead(potPinA3), stableA3Raw);
    const int reductionUs = (int)((1.0f - a3Raw / 4095.0f) * 100.0f + 0.5f);
    servoPulseUs = clampInt(servoPulseUs - reductionUs, servoPulseMinUs, servoPulseMaxUs);
  }

  // --- Poll interval gate for ODrive telemetry + display ---
  if (now - lastPollMs < pollIntervalMs) {
    return;
  }
  lastPollMs = now;

  const bool heartbeatFresh = odriveUserData.received_heartbeat && ((int32_t)(now - odriveUserData.last_heartbeat_ms) < 1500);
  const bool feedbackFresh = odriveUserData.received_feedback && ((int32_t)(now - odriveUserData.last_feedback_ms) < 1500);
  const bool vbusFresh = odriveUserData.received_vbus && ((int32_t)(now - odriveUserData.last_vbus_ms) < 1500);
  const bool telemetryFresh = heartbeatFresh || feedbackFresh || vbusFresh;
  const ODriveAxisState state = heartbeatFresh
    ? static_cast<ODriveAxisState>(odriveUserData.last_heartbeat.Axis_State)
    : AXIS_STATE_IDLE;
  odriveConnected = telemetryFresh;

  if (!odriveConnected) {
    powerFilterInitialized = false;
    filteredPower = 0.0f;
    rpmFilterInitialized = false;
    filteredRpm = 0.0f;
    redrawDisplay("WAITING", 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false, wattHours, servoPulseMinUs, 0, false, 0.0f);
    appendGraphSample(0.0f, 0.0f);
    drawGraph();
    return;
  }

  writeServoPulseUs(servoPulseUs);

  const float motorIqA = -(odriveUserData.received_iq ? odriveUserData.last_iq.Iq_Measured : 0.0f);

  if (canMonitorOnlyMode) {
    const float vbusVoltage = odriveUserData.received_vbus ? odriveUserData.last_vbus.Bus_Voltage : 0.0f;
    const float busCurrent = -(odriveUserData.received_vbus ? odriveUserData.last_vbus.Bus_Current : 0.0f);
    const float power = filterPower(vbusVoltage * busCurrent);
    const float rpm = odriveUserData.received_feedback
      ? filterRpm(odriveUserData.last_feedback.Vel_Estimate * 60.0f)
      : 0.0f;
    redrawDisplay(
      engineStateToText(engineState),
      vbusVoltage, busCurrent, motorIqA, power, rpm,
      false, wattHours, servoPulseUs, powerSetpointDisplayW,
      engineState == ENGINE_STARTING || engineState == ENGINE_RUNNING,
      filterTorque(-(odriveUserData.received_torques ? odriveUserData.last_torques.Torque_Estimate : 0.0f))
    );
    appendGraphSample(rpm, power);
    drawGraph();
    return;
  }

  // Only send motion commands in CLOSED_LOOP_CONTROL.
  if (state != AXIS_STATE_CLOSED_LOOP_CONTROL) {
    // In ENGINE_OFF we want IDLE — do not request closed-loop.
    if (engineState != ENGINE_OFF && requestClosedLoopIfSafe(now, true)) {
      Serial.print("Current axis state: ");
      Serial.println(axisStateToText(state));
    }
    const float vbusVoltage = odriveUserData.received_vbus ? odriveUserData.last_vbus.Bus_Voltage : 0.0f;
    const float busCurrent = -(odriveUserData.received_vbus ? odriveUserData.last_vbus.Bus_Current : 0.0f);
    const float power = filterPower(vbusVoltage * busCurrent);
    const float rpm = odriveUserData.received_feedback
      ? filterRpm(odriveUserData.last_feedback.Vel_Estimate * 60.0f)
      : 0.0f;
    redrawDisplay(
      axisStateToText(state),
      vbusVoltage, busCurrent, motorIqA, power, rpm,
      false, wattHours, servoPulseUs, powerSetpointDisplayW,
      engineState == ENGINE_STARTING || engineState == ENGINE_RUNNING,
      filterTorque(-(odriveUserData.received_torques ? odriveUserData.last_torques.Torque_Estimate : 0.0f))
    );
    appendGraphSample(rpm, power);
    drawGraph();
    return;
  }

  // Send ODrive commands based on engine state.
  if (engineState == ENGINE_STOPPING) {
    odrive.setVelocity(velocityCmd);
  } else if (engineState == ENGINE_STARTING || engineState == ENGINE_RUNNING) {
    odrive.setLimits(maxTargetRpm / 60.0f, currentSoftMaxMax);
    odrive.setVelocity(velocityCmd);
  }
  // ENGINE_OFF: ODrive is in IDLE, no commands sent.

  const float vbusVoltage = odriveUserData.received_vbus ? odriveUserData.last_vbus.Bus_Voltage : 0.0f;
  const float busCurrent = -(odriveUserData.received_vbus ? odriveUserData.last_vbus.Bus_Current : 0.0f);
  const float motorPhaseCurrent = motorIqA;
  const float torque = filterTorque(-(odriveUserData.received_torques ? odriveUserData.last_torques.Torque_Estimate : 0.0f));
  const float power = filterPower(vbusVoltage * busCurrent);
  const float rpm = odriveUserData.received_feedback
    ? filterRpm(odriveUserData.last_feedback.Vel_Estimate * 60.0f)
    : 0.0f;

  wattHours += power * (pollIntervalMs / 1000.0f / 3600.0f);

  redrawDisplay(
    engineStateToText(engineState),
    vbusVoltage, busCurrent, motorPhaseCurrent, power, rpm,
    state == AXIS_STATE_CLOSED_LOOP_CONTROL,
    wattHours, servoPulseUs, powerSetpointDisplayW,
    engineState == ENGINE_STARTING || engineState == ENGINE_RUNNING,
    torque
  );

  appendGraphSample(rpm, power);
  drawGraph();
}
