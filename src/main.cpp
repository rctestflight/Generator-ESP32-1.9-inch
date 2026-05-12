#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include "ODriveCAN.h"
#include "ODriveESP32TWAI.hpp"
#include <SPI.h>
#include <HX711.h>
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
const int resetButtonPin = 25;
const long canBaudrate = 1000000;
const int odriveNodeId = 0;
const unsigned long pollIntervalMs = 100;
const bool canMonitorOnlyMode = false;
const float maxTargetRpm = 8000.0f;
const float testSequenceSetpointThresholdRpm = 2500.0f;
const float powerFilterAlpha = 0.4f; //0.1 bigger number = less filtering
const float rpmFilterAlpha = 0.08f;   // Smaller number = smoother RPM
const double rateFilterAlpha = 0.5f; // Smaller number = smoother rate value
const float servoPotFilterAlpha = 0.35f; // Lower = smoother servo us, higher = faster response.
float currentSoftMaxMax = 70.0f;
float wattHours = 0.00f;
float filteredPower = 0.0f;
bool powerFilterInitialized = false;
float filteredRpm = 0.0f;
bool rpmFilterInitialized = false;


const bool useA3DirectServoControl = true; // Set false to switch back to LUT-based servo control.

const int servoPulseMinUs = 1000; //should correspond to idle on carburetor
const int servoPulseMaxUs = 2000; //should correspond to wide open on carburetor
const int idleRPM = 3200; //above this RPM the throttle will be mapped to RPM
const int servoOffsetRangeUs = 300;
const int servoPwmChannel = 0;
const int servoPwmFrequencyHz = 50;
const int servoPwmResolutionBits = 16;
const int testSequenceStepUs = 50;
const unsigned long testSequenceStepDurationMs = 20000;

struct CurvePoint {
  int rpm;
  int pulseUs;
};

int clampInt(int value, int minValue, int maxValue);

const CurvePoint servoCurve[] = {
  {3200, 1000},
  {3300, 1065},
  {3400, 1125},
  {3500, 1180},
  {3600, 1230},
  {3700, 1270},
  {3800, 1305},
  {3900, 1335},
  {4000, 1550},
  {4040, 1550},
  {4081, 1550},
  {4121, 1550},
  {4162, 1550},
  {4202, 1550},
  {4242, 1550},
  {4283, 1550},
  {4323, 1550},
  {4364, 1550},
  {4404, 1550},
  {4444, 1550},
  {4485, 1550},
  {4525, 1550},
  {4566, 1550},
  {4606, 1550},
  {4646, 1550},
  {4687, 1550},
  {4727, 1550},
  {4768, 1550},
  {4808, 1550},
  {4848, 1550},
  {4889, 1550},
  {4929, 1550},
  {4970, 1550},
  {5010, 1552},
  {5051, 1558},
  {5091, 1564},
  {5131, 1570},
  {5172, 1576},
  {5212, 1582},
  {5253, 1588},
  {5293, 1594},
  {5333, 1600},
  {5374, 1606},
  {5414, 1612},
  {5455, 1618},
  {5495, 1624},
  {5535, 1630},
  {5576, 1636},
  {5616, 1642},
  {5657, 1648},
  {5697, 1655},
  {5737, 1661},
  {5778, 1667},
  {5818, 1673},
  {5859, 1679},
  {5899, 1685},
  {5939, 1691},
  {5980, 1697},
  {6020, 1700},
  {6061, 1700},
  {6101, 1700},
  {6141, 1700},
  {6182, 1700},
  {6222, 1700},
  {6263, 1700},
  {6303, 1700},
  {6343, 1700},
  {6384, 1700},
  {6424, 1700},
  {6465, 1700},
  {6505, 1700},
  {6545, 1700},
  {6586, 1700},
  {6626, 1700},
  {6667, 1700},
  {6707, 1700},
  {6747, 1700},
  {6788, 1700},
  {6828, 1700},
  {6869, 1700},
  {6909, 1700},
  {6949, 1700},
  {6990, 1700},
  {7030, 1706},
  {7071, 1714},
  {7111, 1722},
  {7152, 1730},
  {7192, 1738},
  {7232, 1746},
  {7273, 1755},
  {7313, 1763},
  {7354, 1771},
  {7394, 1779},
  {7434, 1787},
  {7475, 1795},
  {7515, 1803},
  {7556, 1811},
  {7596, 1819},
  {7636, 1827},
  {7677, 1835},
  {7717, 1843},
  {7758, 1852},
  {7798, 1860},
  {7838, 1880},
  {7879, 1900},
  {7919, 1940},
  {7960, 1950},
  {8000, 1950}
};
const int servoCurveCount = sizeof(servoCurve) / sizeof(servoCurve[0]);

const int graphY = 74;
const int graphHeight = screenHeight - graphY;
float graphRpm[screenWidth];
float graphPower[screenWidth];
float graphEfficiency[screenWidth];
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
bool testSequenceActive = false;
int testSequencePulseUs = servoPulseMinUs;
unsigned long testSequenceStepStartMs = 0;
int lastResetButtonState = HIGH;
unsigned long nextClosedLoopRequestMs = 0;
volatile uint32_t rawCanRxCount = 0;
volatile uint32_t lastRawCanId = 0;

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

// ---------------- HX711 ----------------
HX711 scale;
const int LOADCELL_DOUT_PIN = 5;
const int LOADCELL_SCK_PIN = 21;
const int numReadings = 20; // Loadcell Filtering
const double maxRateStepGPerMin = 300.0; // Reject sudden single-sample rate jumps beyond this step.
double readings[numReadings]; // Array to store filtered sensor readings with decimal precision
int readIndex = 0; // The index of the current reading in the array
double total = 0.0; // Running sum for moving average
double weight = 0.0;
double weightRateGPerMin = 0.0;
double filteredWeightRateGPerMin = 0.0;
bool rateFilterInitialized = false;
double lastAcceptedRateGPerMin = 0.0;
bool lastAcceptedRateInitialized = false;
double lastWeightForRate = 0.0;
unsigned long lastRateCalcMs = 0;
int loadCellSampleCount = 0;

// ---------------- Potentiometer Median Filter ----------------
int a0Buffer[3] = {0, 0, 0};
int a0BufferIndex = 0;
int a3Buffer[3] = {0, 0, 0};
int a3BufferIndex = 0;
int lastA0Raw = 0;  // For hysteresis
const int a0Deadband = 30;  // ~40 RPM deadband 
const int a0ZeroSnapThreshold = 4;  // Force zero below this ADC reading

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

int filterServoPotentiometer(int rawValue) {
  a3Buffer[a3BufferIndex] = rawValue;
  a3BufferIndex = (a3BufferIndex + 1) % 3;

  const int medianValue = getMedian(a3Buffer[0], a3Buffer[1], a3Buffer[2]);
  static float filteredA3Value = 0.0f;
  static bool filteredA3Initialized = false;

  if (!filteredA3Initialized) {
    filteredA3Value = (float)medianValue;
    filteredA3Initialized = true;
  } else {
    filteredA3Value += ((float)medianValue - filteredA3Value) * servoPotFilterAlpha;
  }

  int filteredInt = (int)(filteredA3Value + 0.5f);
  if (filteredInt < 0) {
    filteredInt = 0;
  } else if (filteredInt > 4095) {
    filteredInt = 4095;
  }
  return filteredInt;
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

int mapPotToServoPulseUs(int rawValue) {
  const int clampedRawValue = clampInt(rawValue, 0, 4095);
  const int pulseRangeUs = servoPulseMaxUs - servoPulseMinUs;
  const int pulseUs = servoPulseMinUs + (int)(((long)clampedRawValue * pulseRangeUs) / 4095L);
  return clampInt(pulseUs, servoPulseMinUs, servoPulseMaxUs);
}

int mapTargetRpmToServoPulseUs(float targetRpm) {
  if (targetRpm <= servoCurve[0].rpm) {
    return clampInt(servoCurve[0].pulseUs, servoPulseMinUs, servoPulseMaxUs);
  }

  if (targetRpm >= servoCurve[servoCurveCount - 1].rpm) {
    return clampInt(servoCurve[servoCurveCount - 1].pulseUs, servoPulseMinUs, servoPulseMaxUs);
  }

  for (int i = 1; i < servoCurveCount; i++) {
    if (targetRpm <= servoCurve[i].rpm) {
      const CurvePoint& lower = servoCurve[i - 1];
      const CurvePoint& upper = servoCurve[i];
      const float ratio = (targetRpm - lower.rpm) / (float)(upper.rpm - lower.rpm);
      const float pulse = lower.pulseUs + (upper.pulseUs - lower.pulseUs) * ratio;
      return clampInt((int)(pulse + 0.5f), servoPulseMinUs, servoPulseMaxUs);
    }
  }

  return clampInt(servoCurve[servoCurveCount - 1].pulseUs, servoPulseMinUs, servoPulseMaxUs);
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

float filterRpm(float rawRpm) {
  if (!rpmFilterInitialized) {
    filteredRpm = rawRpm;
    rpmFilterInitialized = true;
    return filteredRpm;
  }

  filteredRpm += (rawRpm - filteredRpm) * rpmFilterAlpha;
  return filteredRpm;
}

double filterWeightRate(double rawRateGPerMin) {
  if (!rateFilterInitialized) {
    filteredWeightRateGPerMin = rawRateGPerMin;
    rateFilterInitialized = true;
    return filteredWeightRateGPerMin;
  }

  filteredWeightRateGPerMin += (rawRateGPerMin - filteredWeightRateGPerMin) * rateFilterAlpha;
  return filteredWeightRateGPerMin;
}

void updateLoadCell(unsigned long now) {
  if (!scale.is_ready()) {
    return;
  }

  total = total - readings[readIndex];
  readings[readIndex] = scale.get_units(1);
  total = total + readings[readIndex];
  readIndex++;

  if (readIndex >= numReadings) {
    readIndex = 0;
  }

  weight = total / numReadings;

  // Wait for the moving-average buffer to be fully populated before computing
  // rate. Until then, keep sliding lastWeightForRate forward so the first real
  // rate sample starts from a stable, fully-warmed baseline.
  if (loadCellSampleCount < numReadings) {
    loadCellSampleCount++;
    lastRateCalcMs = now;
    lastWeightForRate = weight;
    weightRateGPerMin = 0.0;
    filteredWeightRateGPerMin = 0.0;
    lastAcceptedRateGPerMin = 0.0;
    lastAcceptedRateInitialized = false;
    rateFilterInitialized = false;
    return;
  }

  const unsigned long elapsedMs = now - lastRateCalcMs;
  if (elapsedMs == 0) {
    return;
  }

  const double elapsedMinutes = (double)elapsedMs / 60000.0;
  const double rawWeightRateGPerMin = -((weight - lastWeightForRate) / elapsedMinutes);
  double gatedWeightRateGPerMin = rawWeightRateGPerMin;

  if (lastAcceptedRateInitialized) {
    const double rateDelta = rawWeightRateGPerMin - lastAcceptedRateGPerMin;
    if (rateDelta > maxRateStepGPerMin || rateDelta < -maxRateStepGPerMin) {
      gatedWeightRateGPerMin = lastAcceptedRateGPerMin;
    }
  } else {
    lastAcceptedRateInitialized = true;
  }

  lastAcceptedRateGPerMin = gatedWeightRateGPerMin;
  weightRateGPerMin = filterWeightRate(gatedWeightRateGPerMin);
  lastRateCalcMs = now;
  lastWeightForRate = weight;
}

void appendGraphSample(float rpm, float power, float efficiency) {
  if (graphCount < screenWidth) {
    graphRpm[graphCount] = rpm;
    graphPower[graphCount] = power;
    graphEfficiency[graphCount] = efficiency;
    graphCount++;
    return;
  }

  for (int i = 1; i < screenWidth; i++) {
    graphRpm[i - 1] = graphRpm[i];
    graphPower[i - 1] = graphPower[i];
    graphEfficiency[i - 1] = graphEfficiency[i];
  }

  graphRpm[screenWidth - 1] = rpm;
  graphPower[screenWidth - 1] = power;
  graphEfficiency[screenWidth - 1] = efficiency;
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
  float effMin, effMax;
  computeRange(graphRpm, graphCount, &rpmMin, &rpmMax);
  computeRange(graphPower, graphCount, &pwrMin, &pwrMax);
  computeRange(graphEfficiency, graphCount, &effMin, &effMax);

  for (int i = 1; i < graphCount; i++) {
    const int x1 = i - 1;
    const int x2 = i;

    const int y1R = mapValueToGraphY(graphRpm[i - 1], rpmMin, rpmMax);
    const int y2R = mapValueToGraphY(graphRpm[i], rpmMin, rpmMax);
    lcd.drawLine(x1, y1R, x2, y2R, ST77XX_CYAN);

    const int y1P = mapValueToGraphY(graphPower[i - 1], pwrMin, pwrMax);
    const int y2P = mapValueToGraphY(graphPower[i], pwrMin, pwrMax);
    lcd.drawLine(x1, y1P, x2, y2P, ST77XX_WHITE);

    const int y1E = mapValueToGraphY(graphEfficiency[i - 1], effMin, effMax);
    const int y2E = mapValueToGraphY(graphEfficiency[i], effMin, effMax);
    lcd.drawLine(x1, y1E, x2, y2E, ST77XX_RED);
  }
}

void drawLine(const char* text, int y, uint16_t color, uint8_t textSize) {
  lcd.setCursor(0, y);
  lcd.setTextSize(textSize);
  lcd.setTextColor(color);
  lcd.print(text);
}

void redrawDisplay(const char* stateText, float vbusVoltage, float busCurrent, float motorPhaseCurrent, float power, float rpm, bool isClosedLoop, float wattHours, int servoPulseUs, int servoOffsetUs, double weightRateDisplayGPerMin, double efficiency) {
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
  lcd.setTextColor(ST77XX_CYAN);
  snprintf(line, sizeof(line), "%dus", servoPulseUs);
  lcd.print(line);
  lcd.setTextColor(ST77XX_YELLOW);
  snprintf(line, sizeof(line), " %+dus", servoOffsetUs);
  lcd.print(line);
  lcd.setTextColor(ST77XX_WHITE);
  snprintf(line, sizeof(line), " Rt:%.1f", weightRateDisplayGPerMin);
  lcd.print(line);
  lcd.setTextColor(ST77XX_RED);
  snprintf(line, sizeof(line), " %.1f", efficiency);
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

void resetMeasurementsAfterTare(unsigned long now) {
  scale.tare(20);
  Serial.println("TARE");
  Serial.print("HX711 offset: ");
  Serial.println(scale.get_offset());
  wattHours = 0.0f;
  weightRateGPerMin = 0.0;
  filteredWeightRateGPerMin = 0.0;
  lastAcceptedRateGPerMin = 0.0;
  lastAcceptedRateInitialized = false;
  rateFilterInitialized = false;
  // Reset the moving-average buffer so the post-tare warmup is clean.
  for (int i = 0; i < numReadings; i++) {
    readings[i] = 0.0;
  }
  total = 0.0;
  readIndex = 0;
  loadCellSampleCount = 0;
  lastWeightForRate = 0.0;
  lastRateCalcMs = 0;
}

void startServoTestSequence(unsigned long now) {
  testSequenceActive = true;
  testSequencePulseUs = servoPulseMinUs;
  testSequenceStepStartMs = now;
  Serial.println("Starting servo test sequence");
}

void updateServoTestSequence(unsigned long now) {
  if (!testSequenceActive) {
    return;
  }

  if (now - testSequenceStepStartMs < testSequenceStepDurationMs) {
    return;
  }

  testSequenceStepStartMs = now;
  if (testSequencePulseUs < servoPulseMaxUs) {
    testSequencePulseUs += testSequenceStepUs;
    if (testSequencePulseUs > servoPulseMaxUs) {
      testSequencePulseUs = servoPulseMaxUs;
    }
  } else {
    testSequenceActive = false;
    Serial.println("Servo test sequence complete");
  }
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  pinMode(potPinA3, INPUT);
  pinMode(potPinA0, INPUT);
  pinMode(resetButtonPin, INPUT_PULLUP);
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
    Serial.println("Enabling closed loop control...");
    const unsigned long closedLoopTimeoutMs = millis() + 6000;
    unsigned long nextClosedLoopStatusPrintMs = 0;
    while (odriveUserData.last_heartbeat.Axis_State != AXIS_STATE_CLOSED_LOOP_CONTROL && millis() < closedLoopTimeoutMs) {
      requestClosedLoopIfSafe(millis(), false);

      const unsigned long nowMs = millis();
      if (nowMs >= nextClosedLoopStatusPrintMs) {
        nextClosedLoopStatusPrintMs = nowMs + 500;
        const ODriveAxisState state = static_cast<ODriveAxisState>(odriveUserData.last_heartbeat.Axis_State);
        Serial.print("Axis state: ");
        Serial.print(axisStateToText(state));
        Serial.print(" | Axis error: 0x");
        Serial.println(odriveUserData.last_heartbeat.Axis_Error, HEX);
      }

      for (int i = 0; i < 15; ++i) {
        delay(10);
        pumpEvents(can_intf);
      }
    }

    if (odriveUserData.last_heartbeat.Axis_State != AXIS_STATE_CLOSED_LOOP_CONTROL) {
      Serial.println("Closed-loop entry timed out.");
      Serial.print("Final axis state: ");
      Serial.println(axisStateToText(static_cast<ODriveAxisState>(odriveUserData.last_heartbeat.Axis_State)));
      Serial.print("Final axis error: 0x");
      Serial.println(odriveUserData.last_heartbeat.Axis_Error, HEX);
    }
  }

  const bool inClosedLoopAfterSetup =
    (odriveUserData.last_heartbeat.Axis_State == AXIS_STATE_CLOSED_LOOP_CONTROL);

  if (inClosedLoopAfterSetup) {
    Serial.println("ODrive running!");
    odrive.setLimits(maxTargetRpm / 60.0f, currentSoftMaxMax);
  } else if (canMonitorOnlyMode) {
    Serial.println("CAN monitor-only mode: skipping closed-loop and setpoint commands.");
  } else {
    Serial.println("ODrive not in closed loop yet; continuing in monitor mode.");
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
    inClosedLoopAfterSetup ? "CLOSED LOOP" : "IDLE",
    vbusVoltage,
    busCurrent,
    motorPhaseCurrent,
    power,
    rpm,
    inClosedLoopAfterSetup,
    0.0f,
    servoPulseMinUs,
    0,
    0.0,
    0.0
  );
  appendGraphSample(rpm, power, 0.0f);
  drawGraph();

    //HX711 Setup------------69.65
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  Serial.println("HX711 initialized");
  scale.set_scale(-18150); //-760.8
  scale.set_offset(-302952); //-25600
  // Initialize all the readings to 0:
  for (int i = 0; i < numReadings; i++) {
      readings[i] = 0.0;
      }  
}

void loop() {
  recoverCan();
  pumpEvents(can_intf);

  const unsigned long now = millis();

  updateLoadCell(now);

  if (now - lastPollMs < pollIntervalMs) {
    return;
  }

  lastPollMs = now;

  const int a3Filtered = filterServoPotentiometer(analogRead(potPinA3));
  const int a0Filtered = filterPotentiometer(analogRead(potPinA0));
  
  // Apply hysteresis to prevent drift
  int a0Raw = lastA0Raw;
  if (a0Filtered <= a0ZeroSnapThreshold) {
    a0Raw = 0;
    lastA0Raw = 0;
  } else if (abs(a0Filtered - lastA0Raw) > a0Deadband) {
    a0Raw = a0Filtered;
    lastA0Raw = a0Raw;
  }
  
  const float targetRpm = (a0Raw * maxTargetRpm) / 4095.0f;
  const float servoRpm = (a0Filtered * maxTargetRpm) / 4095.0f;

  const int resetButtonState = digitalRead(resetButtonPin);
  const bool resetButtonPressed = (lastResetButtonState == HIGH) && (resetButtonState == LOW);
  lastResetButtonState = resetButtonState;

  if (resetButtonPressed) {
    if (targetRpm > testSequenceSetpointThresholdRpm) {
      startServoTestSequence(now);
    } else {
      resetMeasurementsAfterTare(now);
    }
  }

  int servoOffsetUs = useA3DirectServoControl
    ? 0
    : ((targetRpm > idleRPM)
      ? (int)((a3Filtered * (2.0f * servoOffsetRangeUs)) / 4095.0f) - servoOffsetRangeUs
      : 0);
  const int baseServoPulseUs = useA3DirectServoControl
    ? mapPotToServoPulseUs(a3Filtered)
    : mapTargetRpmToServoPulseUs(servoRpm);
  int servoPulseUs = clampInt(baseServoPulseUs + servoOffsetUs, servoPulseMinUs, servoPulseMaxUs);

  updateServoTestSequence(now);
  if (testSequenceActive) {
    servoPulseUs = testSequencePulseUs;
    servoOffsetUs = 0;
  }

  const float targetVelRevPerSec = targetRpm / 60.0f;


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
    redrawDisplay("WAITING", 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false, wattHours, servoPulseMinUs, 0, weightRateGPerMin, 0.0);
    appendGraphSample(0.0f, 0.0f, 0.0f);
    drawGraph();
    return;
  }

  writeServoPulseUs(servoPulseUs);

  // Bus voltage/current, Iq, and error are received via cyclic broadcasts
  // configured on the ODrive (vbus/iq/error msg_rate_ms). No polling needed.
  const float motorIqA = -(odriveUserData.received_iq ? odriveUserData.last_iq.Iq_Measured : 0.0f);

  if (canMonitorOnlyMode) {
    const float vbusVoltage = odriveUserData.received_vbus ? odriveUserData.last_vbus.Bus_Voltage : 0.0f;
    const float busCurrent = -(odriveUserData.received_vbus ? odriveUserData.last_vbus.Bus_Current : 0.0f);
    const float power = filterPower(vbusVoltage * busCurrent);
    const float rpm = odriveUserData.received_feedback
      ? filterRpm(odriveUserData.last_feedback.Vel_Estimate * 60.0f)
      : 0.0f;

    redrawDisplay(
      axisStateToText(state),
      vbusVoltage,
      busCurrent,
      motorIqA,
      power,
      rpm,
      false,
      wattHours,
      servoPulseUs,
      servoOffsetUs,
      weightRateGPerMin,
      0.0
    );
    appendGraphSample(rpm, power, 0.0f);
    drawGraph();
    return;
  }

  // Only send motion/current commands in CLOSED_LOOP_CONTROL.
  if (state != AXIS_STATE_CLOSED_LOOP_CONTROL) {
    if (requestClosedLoopIfSafe(now, true)) {
      Serial.print("Current axis state: ");
      Serial.println(axisStateToText(state));
    }
    // Errors are logged from onError() callback whenever they change.

    const float vbusVoltage = odriveUserData.received_vbus ? odriveUserData.last_vbus.Bus_Voltage : 0.0f;
    const float busCurrent = -(odriveUserData.received_vbus ? odriveUserData.last_vbus.Bus_Current : 0.0f);
    const float power = filterPower(vbusVoltage * busCurrent);
    const float rpm = odriveUserData.received_feedback
      ? filterRpm(odriveUserData.last_feedback.Vel_Estimate * 60.0f)
      : 0.0f;

    redrawDisplay(
      axisStateToText(state),
      vbusVoltage,
      busCurrent,
      motorIqA,
      power,
      rpm,
      false,
      wattHours,
      servoPulseUs,
      servoOffsetUs,
      weightRateGPerMin,
      0.0
    );
    appendGraphSample(rpm, power, 0.0f);
    drawGraph();
    return;
  }

  if (targetVelRevPerSec < 1.0f) {
    odrive.setLimits((maxTargetRpm / 60.0f), 0.0f);
  } else {
    odrive.setLimits((maxTargetRpm / 60.0f), currentSoftMaxMax);
  }
  
  odrive.setVelocity(targetVelRevPerSec);
  //Serial.print("Target RPM Sent: ");
  //Serial.println(targetRpm, 1);

  const float vbusVoltage = odriveUserData.received_vbus ? odriveUserData.last_vbus.Bus_Voltage : 0.0f;
  const float busCurrent = -(odriveUserData.received_vbus ? odriveUserData.last_vbus.Bus_Current : 0.0f);
  const float motorPhaseCurrent = motorIqA;
  const float torque = -(odriveUserData.received_torques ? odriveUserData.last_torques.Torque_Estimate : 0.0f);
  const float power = filterPower(vbusVoltage * busCurrent);
  const float rpm = odriveUserData.received_feedback
    ? filterRpm(odriveUserData.last_feedback.Vel_Estimate * 60.0f)
    : 0.0f;
  const double efficiency = (weightRateGPerMin > 0.001 || weightRateGPerMin < -0.001)
    ? (double)power / weightRateGPerMin
    : 0.0;


  //Serial.print(" Weight: ");
  //Serial.println(scale.get_units(10)); 
  ///*
  Serial.print(servoPulseUs);
  Serial.print(",");
  Serial.print(torque);
  Serial.print(",");
  Serial.print(rpm, 1);
  Serial.print(",");
  Serial.print(power);
  Serial.print(",");
  Serial.print(weightRateGPerMin, 1);
  Serial.print(",");
  Serial.println(efficiency, 1);
  //*/


  wattHours += power * (pollIntervalMs / 1000.0f / 3600.0f);

  redrawDisplay(
    axisStateToText(state),
    vbusVoltage,
    busCurrent,
    motorPhaseCurrent,
    power,
    rpm,
    state == AXIS_STATE_CLOSED_LOOP_CONTROL,
    wattHours,
    servoPulseUs,
    servoOffsetUs,
    weightRateGPerMin,
    efficiency
  );

  appendGraphSample(rpm, power, (float)efficiency);
  drawGraph();
}
