#include <Arduino.h>

// Minimal wiring test: prints whenever GPIO16 reads HIGH.
static const int TEST_PIN = 16;

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(TEST_PIN, INPUT);

  Serial.println("\n=== GPIO16 High Test ===");
  Serial.println("Watching GPIO16...");
  Serial.println("Prints on LOW->HIGH edges and once per second while HIGH.");
}

void loop() {
  static bool last_state = false;
  static unsigned long last_high_print_ms = 0;

  bool state = (digitalRead(TEST_PIN) == HIGH);
  unsigned long now = millis();

  if (!last_state && state) {
    Serial.printf("GPIO16 HIGH (rising edge) at %lu ms\n", now);
    last_high_print_ms = now;
  }

  if (state && (now - last_high_print_ms >= 1000)) {
    Serial.printf("GPIO16 still HIGH at %lu ms\n", now);
    last_high_print_ms = now;
  }

  if (last_state && !state) {
    Serial.printf("GPIO16 LOW (falling edge) at %lu ms\n", now);
  }

  last_state = state;
  delay(5);
}
