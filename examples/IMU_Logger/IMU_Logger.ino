// IMU_Logger.ino
// Streams BMI323 IMU data over USB and captures camera strobe interrupts.
// Use -help over Serial to view commands. Format: Euroc-style CSV, int, or binary.
//
// Pins:
//   IMU_CS  = PA4  (SPI chip select)
//   INT1    = PA1  (BMI323 data ready)
//   STROBE  = PB4  (Camera strobe GPIO input)

/// → Full implementation provided in accompanying documentation.

void setup() {
  Serial.begin(460800);
  while (!Serial);
  Serial.println("North Star STM32C071GBU6 IMU Logger Example");
}

void loop() {
  // Placeholder: refer to full sketch in repository.
}
