#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <ODriveUART.h>
#include <SPI.h>

#define LCD_MOSI 23
#define LCD_SCK 18
#define LCD_CS 15
#define LCD_DC 2
#define LCD_RST 4
#define LCD_BLK 32

const int screenWidth = 320;
const int screenHeight = 170;
const int odriveRxPin = 16;
const int odriveTxPin = 17;
const int potPinA3 = A3;
const int potPinA0 = A0;
const int resetButtonPin = 25;
const unsigned long odriveBaudrate = 115200;
const unsigned long pollIntervalMs = 250;
float currentSoftMaxMax = 70.0f;
float lastSentCurrentSoftMax = -1.0f;
float wattHours = 0.00f;

const int graphY = 56;
const int graphHeight = screenHeight - graphY;
float graphRpm[screenWidth];
float graphPower[screenWidth];
int graphCount = 0;

Adafruit_ST7789 lcd = Adafruit_ST7789(LCD_CS, LCD_DC, LCD_RST);
HardwareSerial& odriveSerial = Serial2;
ODriveUART odrive(odriveSerial);

unsigned long lastPollMs = 0;
bool odriveConnected = false;

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

void redrawDisplay(const char* stateText, float vbusVoltage, float busCurrent, float motorPhaseCurrent, float power, float rpm, bool isClosedLoop, float wattHours) {
  char line[48];

  // Clear only the text rows we redraw to avoid full-screen flicker.
  lcd.fillRect(0, 0, screenWidth, 26, ST77XX_BLACK);
  lcd.fillRect(0, 34, screenWidth, 18, ST77XX_BLACK);

  lcd.setCursor(0, 0);
  lcd.setTextSize(3);
  lcd.setTextColor(ST77XX_WHITE);
  snprintf(line, sizeof(line), "%.0f W ", power);
  lcd.print(line);
  lcd.setTextColor(ST77XX_CYAN);
  snprintf(line, sizeof(line), "RPM: %.0f", rpm);
  lcd.print(line);

  lcd.setCursor(0, 34);
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
  snprintf(line, sizeof(line), " %.1fWh", wattHours);
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
  pinMode(potPinA3, INPUT);
  pinMode(potPinA0, INPUT);
  pinMode(resetButtonPin, INPUT_PULLUP);

  pinMode(LCD_BLK, OUTPUT);
  digitalWrite(LCD_BLK, HIGH);

  SPI.begin(LCD_SCK, -1, LCD_MOSI, LCD_CS);
  lcd.init(screenHeight, screenWidth);
  lcd.setRotation(1);
  lcd.fillScreen(ST77XX_BLACK);

  odriveSerial.begin(odriveBaudrate, SERIAL_8N1, odriveRxPin, odriveTxPin);
  delay(10);

  redrawDisplay("WAITING", 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false, 0.0f);
  Serial.println("Waiting for ODrive...");

  while (odrive.getState() == AXIS_STATE_UNDEFINED) {
    redrawDisplay("WAITING", 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false, 0.0f);
    delay(100);
  }

  odriveConnected = true;
  Serial.println("found ODrive");

  const float vbusVoltage = odrive.getParameterAsFloat("vbus_voltage");
  Serial.print("DC voltage: ");
  Serial.println(vbusVoltage);

  Serial.println("Enabling closed loop control...");
  while (odrive.getState() != AXIS_STATE_CLOSED_LOOP_CONTROL) {
    odrive.clearErrors();
    odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
    delay(10);
  }

  Serial.println("ODrive running!");
  const float hardCurrentMax = odrive.getParameterAsFloat("axis0.config.motor.current_hard_max");
  if (hardCurrentMax > 0.1f) {
    // Keep soft max below hard max margin if available.
    currentSoftMaxMax = hardCurrentMax * 0.9f;
  }
  const float busCurrent = odrive.getParameterAsFloat("ibus");
  const float motorPhaseCurrent = odrive.getParameterAsFloat("axis0.motor.foc.Iq_measured");
  const float power = busCurrent * motorPhaseCurrent;
  const float rpm = odrive.getFeedback().vel * 60.0f;
  redrawDisplay("CLOSED LOOP", vbusVoltage, busCurrent, motorPhaseCurrent, power, rpm, true, 0.0f);
  appendGraphSample(rpm, power);
  drawGraph();
}

void loop() {
  const unsigned long now = millis();
  if (now - lastPollMs < pollIntervalMs) {
    return;
  }

  lastPollMs = now;

  if (digitalRead(resetButtonPin) == LOW) {
    wattHours = 0.0f;
  }

  const int a3Raw = analogRead(potPinA3);
  const int a0Raw = analogRead(potPinA0);
  const float targetCurrentSoftMax = (a3Raw * currentSoftMaxMax) / 4095.0f;
  const float targetRpm = (a0Raw * 8000.0f) / 4095.0f;
  const float targetVelRevPerSec = targetRpm / 60.0f;

  Serial.print("A3: ");
  Serial.print(a3Raw);
  Serial.print(" A0: ");
  Serial.print(a0Raw);
  Serial.print(" I Limit: ");
  Serial.print(targetCurrentSoftMax, 2);
  Serial.print(" A");
  Serial.print(" Target RPM: ");
  Serial.println(targetRpm, 1);


  const ODriveAxisState state = odrive.getState();
  odriveConnected = state != AXIS_STATE_UNDEFINED;

  if (!odriveConnected) {
    redrawDisplay("WAITING", 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false, wattHours);
    appendGraphSample(0.0f, 0.0f);
    drawGraph();
    return;
  }

  if (lastSentCurrentSoftMax < 0.0f || targetCurrentSoftMax > lastSentCurrentSoftMax + 0.2f || targetCurrentSoftMax < lastSentCurrentSoftMax - 0.2f) {
    odrive.setParameter("axis0.config.motor.current_soft_max", String(targetCurrentSoftMax, 2));
    lastSentCurrentSoftMax = targetCurrentSoftMax;
  }

  odrive.setVelocity(targetVelRevPerSec);

  const float vbusVoltage = odrive.getParameterAsFloat("vbus_voltage");
  const float busCurrent = odrive.getParameterAsFloat("ibus");
  const float motorPhaseCurrent = odrive.getParameterAsFloat("axis0.motor.foc.Iq_measured");
  const float power = busCurrent * motorPhaseCurrent;
  const float rpm = odrive.getFeedback().vel * 60.0f;
  if (power > 0.0f) {
    wattHours += power * (pollIntervalMs / 1000.0f / 3600.0f);
  }

  redrawDisplay(
    axisStateToText(state),
    vbusVoltage,
    busCurrent,
    motorPhaseCurrent,
    power,
    rpm,
    state == AXIS_STATE_CLOSED_LOOP_CONTROL,
    wattHours
  );

  appendGraphSample(rpm, power);
  drawGraph();
}
