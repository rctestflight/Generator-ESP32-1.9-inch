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
const unsigned long odriveBaudrate = 115200;
const unsigned long pollIntervalMs = 250;

Adafruit_ST7789 lcd = Adafruit_ST7789(LCD_CS, LCD_DC, LCD_RST);
HardwareSerial& odriveSerial = Serial2;
ODriveUART odrive(odriveSerial);

unsigned long lastPollMs = 0;
bool odriveConnected = false;

void drawLine(const char* text, int y, uint16_t color, uint8_t textSize) {
  lcd.setCursor(0, y);
  lcd.setTextSize(textSize);
  lcd.setTextColor(color);
  lcd.print(text);
}

void redrawDisplay(const char* stateText, float vbusVoltage, float busCurrent, float motorPhaseCurrent, float power, float rpm, bool isClosedLoop) {
  char line[48];

  lcd.fillScreen(ST77XX_BLACK);

  snprintf(line, sizeof(line), "%.0f W RPM: %.0f", power, rpm);
  drawLine(line, 0, ST77XX_WHITE, 3);

  lcd.setCursor(0, 34);
  lcd.setTextSize(2);
  lcd.setTextColor(isClosedLoop ? ST77XX_GREEN : ST77XX_RED);
  lcd.print(stateText);
  lcd.setTextColor(ST77XX_YELLOW);
  lcd.print(" ");
  snprintf(line, sizeof(line), "%.1f V", vbusVoltage);
  lcd.print(line);
  lcd.setTextColor(ST77XX_CYAN);
  lcd.print(" ");
  snprintf(line, sizeof(line), "%.1f A", motorPhaseCurrent);
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

  pinMode(LCD_BLK, OUTPUT);
  digitalWrite(LCD_BLK, HIGH);

  SPI.begin(LCD_SCK, -1, LCD_MOSI, LCD_CS);
  lcd.init(screenHeight, screenWidth);
  lcd.setRotation(1);
  lcd.fillScreen(ST77XX_BLACK);

  odriveSerial.begin(odriveBaudrate, SERIAL_8N1, odriveRxPin, odriveTxPin);
  delay(10);

  redrawDisplay("WAITING", 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false);
  Serial.println("Waiting for ODrive...");

  while (odrive.getState() == AXIS_STATE_UNDEFINED) {
    redrawDisplay("WAITING", 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false);
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
  const float busCurrent = odrive.getParameterAsFloat("ibus");
  const float motorPhaseCurrent = odrive.getParameterAsFloat("axis0.motor.foc.Iq_measured");
  const float power = busCurrent * motorPhaseCurrent;
  const float rpm = odrive.getFeedback().vel * 60.0f;
  redrawDisplay("CLOSED LOOP", vbusVoltage, busCurrent, motorPhaseCurrent, power, rpm, true);
}

void loop() {
  const unsigned long now = millis();
  if (now - lastPollMs < pollIntervalMs) {
    return;
  }

  lastPollMs = now;

  const ODriveAxisState state = odrive.getState();
  odriveConnected = state != AXIS_STATE_UNDEFINED;

  if (!odriveConnected) {
    redrawDisplay("WAITING", 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false);
    return;
  }

  const float vbusVoltage = odrive.getParameterAsFloat("vbus_voltage");
  const float busCurrent = odrive.getParameterAsFloat("ibus");
  const float motorPhaseCurrent = odrive.getParameterAsFloat("axis0.motor.foc.Iq_measured");
  const float power = busCurrent * motorPhaseCurrent;
  const float rpm = odrive.getFeedback().vel * 60.0f;

  redrawDisplay(
    axisStateToText(state),
    vbusVoltage,
    busCurrent,
    motorPhaseCurrent,
    power,
    rpm,
    state == AXIS_STATE_CLOSED_LOOP_CONTROL
  );
}
