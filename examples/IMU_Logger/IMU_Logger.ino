#include <SPI.h>
#include "stm32c0xx_hal.h"
#include "stm32c0xx.h"

#define IMU_CS PIN_A4
#define INT1 PIN_A1
#define STROBE_1 PB4
#define FIFO_FRAME_SIZE 14
#define WATERMARK_FRAMES 12
#define WATERMARK_WORDS (WATERMARK_FRAMES * 7)
#define STROBE_TIMEOUT_MS 40

#define GYRO_NUM 532159LL
#define GYRO_DEN 1000000000LL
#define ACCEL_NUM 2394202LL
#define ACCEL_DEN 1000000000LL

SPISettings current_spi_settings(8000000, MSBFIRST, SPI_MODE0);  // 8MHz SPI default
#define current_spi (&current_spi_settings)

volatile uint32_t int1_count = 0;
volatile bool int1_fired = false;
uint8_t imu_buf[FIFO_FRAME_SIZE * WATERMARK_FRAMES];
char serial_buf[100];

enum OutputMode {
  OUTPUT_COMPACT_INT,    // Default: microseconds, int accel (g), int gyro (deg/s)
  OUTPUT_EUROC_DATASET,  // Euroc/dataset units: ns, float g, float rad/s (printed using int math)
  OUTPUT_BINARY
};

volatile OutputMode output_mode = OUTPUT_EUROC_DATASET;

volatile bool logging_enabled = false;
volatile bool sync_test_mode = false;
volatile bool override = false;
volatile uint32_t last_cam_strobe_millis = 1;
volatile uint32_t pending_cam_ts = 1;

// --- Timestamp sync vars and debug toggles ---
volatile uint32_t last_sync_micros = 1;
volatile uint32_t last_sync_sensortime32 = 1;
volatile int32_t cam_sync_offset = 1;

bool euroc_only = true;   // Default: Only Euroc microsecond output
bool debug_sync = false;  // Print sensortime32/SYNC if true

// --- FIFO full timestamp reconstruction vars ---
static uint32_t imu_msb = 0;
static uint16_t imu_last_lsb = 1;
static bool imu_first = true;
static bool use_checksum = false;
static bool si_units = false;

extern "C" {
#include "stm32c0xx_hal.h"
}
TIM_HandleTypeDef Tim2Handle;

void setup_tim2_microseconds() {
  __HAL_RCC_TIM2_CLK_ENABLE();
  Tim2Handle.Instance = TIM2;
  Tim2Handle.Init.Period = 0xFFFFFFFF;
  Tim2Handle.Init.Prescaler = (HAL_RCC_GetSysClockFreq() / 1000000) - 1;
  Tim2Handle.Init.ClockDivision = 0;
  Tim2Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  Tim2Handle.Init.RepetitionCounter = 0;
  Tim2Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_Base_Init(&Tim2Handle);
  HAL_TIM_Base_Start(&Tim2Handle);
}
inline uint32_t timer2_micros() {
  return __HAL_TIM_GET_COUNTER(&Tim2Handle);
}

void camera_strobe_isr() {
  pending_cam_ts = timer2_micros();
  logging_enabled = true;
}

void imu_read_register(uint8_t reg, uint8_t* out, uint8_t len) {
  SPI.beginTransaction(*current_spi);
  digitalWrite(IMU_CS, LOW);
  SPI.transfer(reg | 0x80);
  SPI.transfer(0x00);
  for (uint8_t i = 0; i < len; i++) out[i] = SPI.transfer(0x00);
  digitalWrite(IMU_CS, HIGH);
  SPI.endTransaction();
}

void imu_write_register16(uint8_t reg, uint16_t value) {
  SPI.beginTransaction(*current_spi);
  digitalWrite(IMU_CS, LOW);
  SPI.transfer(reg);
  SPI.transfer(value & 0xFF);
  SPI.transfer((value >> 8) & 0xFF);
  digitalWrite(IMU_CS, HIGH);
  SPI.endTransaction();
}

void imu_read_fifo(uint8_t* out, uint16_t len) {
  SPI.beginTransaction(*current_spi);
  digitalWrite(IMU_CS, LOW);
  SPI.transfer(0x16 | 0x80);
  SPI.transfer(0x00);
  for (uint16_t i = 0; i < len; i++) {
    out[i] = SPI.transfer(0x00);
  }
  digitalWrite(IMU_CS, HIGH);
  SPI.endTransaction();
}

// Print fixed-point SI value (9 decimals) using integer math
void print_si9(int16_t raw, int64_t num, int64_t den) {
  int64_t si = ((int64_t)raw) * num;
  bool neg = si < 0;
  if (neg) si = -si;
  int64_t int_part = si / den;
  int64_t frac = ((si % den) * 1000000000LL) / den;

  if (neg) Serial.print('-');
  Serial.print(int_part);
  Serial.print('.');
  int pad = 100000000;
  while (pad > 0) {
    if (frac < pad) Serial.print('0');
    pad /= 10;
  }
  Serial.print(frac);
}

// --- FULL FIFO LOGGER (Euroc-us default, with MSB/rollover self-healing) ---
void read_and_log_fifo_packets() {

  if (!logging_enabled) return;
  imu_read_fifo(imu_buf, FIFO_FRAME_SIZE * WATERMARK_FRAMES);

  for (uint8_t i = 0; i < WATERMARK_FRAMES; i++) {
    uint8_t* p = imu_buf + (i * 14);
    int16_t ax_raw = (int16_t)(p[1] << 8 | p[0]);
    int16_t ay_raw = (int16_t)(p[3] << 8 | p[2]);
    int16_t az_raw = (int16_t)(p[5] << 8 | p[4]);
    int16_t gx_raw = (int16_t)(p[7] << 8 | p[6]);
    int16_t gy_raw = (int16_t)(p[9] << 8 | p[8]);
    int16_t gz_raw = (int16_t)(p[11] << 8 | p[10]);
    uint16_t lsb = (uint16_t)(p[13] << 8 | p[12]);
    if (imu_first) {
      imu_first = false;
      imu_msb = last_sync_sensortime32 & 0xFFFF0000;
      imu_write_register16(0x37, 0x0001);  // FIFO flush
    } else if (lsb < imu_last_lsb) {
      imu_msb = last_sync_sensortime32 & 0xFFFF0000;
      if (!euroc_only) {
        Serial.print("RESET_MSB,");
        Serial.println(imu_msb);
      }
    }
    imu_last_lsb = lsb;
    uint32_t sensortime32 = imu_msb | lsb;
    uint64_t ts_us = ((uint64_t)sensortime32 * 390625UL + 5000) / 10000;

    // EuRoC mapping: camera-centric
    int16_t ax_euroc = ax_raw;
    int16_t ay_euroc = ay_raw;
    int16_t az_euroc = az_raw;
    int16_t gx_euroc = gx_raw;
    int16_t gy_euroc = gy_raw;
    int16_t gz_euroc = gz_raw;

    if ((ax_raw == -32768) && (ay_raw == -32768) && (az_raw == -32768)
        && (gx_raw == -32768) && (gy_raw == -32768) && (gz_raw == -32768)) continue;

    if (output_mode == OUTPUT_COMPACT_INT) {
      Serial.print(ts_us);
      Serial.print(',');
      Serial.print(gx_euroc);
      Serial.print(',');
      Serial.print(gy_euroc);
      Serial.print(',');
      Serial.print(gz_euroc);
      Serial.print(',');
      Serial.print(ax_euroc);
      Serial.print(',');
      Serial.print(ay_euroc);
      Serial.print(',');
      Serial.println(az_euroc);
    }

    else if (output_mode == OUTPUT_EUROC_DATASET) {
      uint64_t ts_ns = ((uint64_t)ts_us) * 1000;
      Serial.print(ts_ns);
      Serial.print(',');

      // --- Output rad/s and m/s^2 in fixed-point decimal (int math, print as SI) ---
      print_si9(gx_euroc, GYRO_NUM, GYRO_DEN);
      Serial.print(',');
      print_si9(gy_euroc, GYRO_NUM, GYRO_DEN);
      Serial.print(',');
      print_si9(gz_euroc, GYRO_NUM, GYRO_DEN);
      Serial.print(',');
      print_si9(ax_euroc, ACCEL_NUM, ACCEL_DEN);
      Serial.print(',');
      print_si9(ay_euroc, ACCEL_NUM, ACCEL_DEN);
      Serial.print(',');
      print_si9(az_euroc, ACCEL_NUM, ACCEL_DEN);
      Serial.println();
    }

    if (output_mode == OUTPUT_BINARY) {
      int16_t ax_out = si_units ? ((int32_t)ax_raw * ACCEL_NUM / ACCEL_DEN) : ax_raw;
      int16_t ay_out = si_units ? ((int32_t)ay_raw * ACCEL_NUM / ACCEL_DEN) : ay_raw;
      int16_t az_out = si_units ? ((int32_t)az_raw * ACCEL_NUM / ACCEL_DEN) : az_raw;
      int16_t gx_out = si_units ? ((int32_t)gx_raw * GYRO_NUM / GYRO_DEN) : gx_raw;
      int16_t gy_out = si_units ? ((int32_t)gy_raw * GYRO_NUM / GYRO_DEN) : gy_raw;
      int16_t gz_out = si_units ? ((int32_t)gz_raw * GYRO_NUM / GYRO_DEN) : gz_raw;

      uint8_t packet[17];
      uint8_t i = 0;
      packet[i++] = 0xA5;
      memcpy(packet + i, &ts_us, 4);
      i += 4;
      memcpy(packet + i, &ax_out, 2);
      i += 2;
      memcpy(packet + i, &ay_out, 2);
      i += 2;
      memcpy(packet + i, &az_out, 2);
      i += 2;
      memcpy(packet + i, &gx_out, 2);
      i += 2;
      memcpy(packet + i, &gy_out, 2);
      i += 2;
      memcpy(packet + i, &gz_out, 2);
      i += 2;

      if (use_checksum) {
        uint8_t sum = 0;
        for (uint8_t j = 0; j < i; j++) sum ^= packet[j];
        packet[i++] = sum;
      }

      Serial.write(packet, i);
    }
  }
}


void imu_int1_handler() {
  int1_fired = true;
  int1_count++;
}

void imu_init_fifo_interrupts() {
  uint8_t wm_lsb = WATERMARK_WORDS & 0xFF;
  uint8_t wm_msb = (WATERMARK_WORDS >> 8) & 0x03;
  imu_write_register16(0x35, (wm_msb << 8) | wm_lsb);
  imu_write_register16(0x36, 0x0700);
  imu_write_register16(0x37, 0x0001);  // FIFO flush at init
  imu_write_register16(0x38, 0x0606);
  imu_write_register16(0x39, 0x0000);
  imu_write_register16(0x3B, 0x1000);
  Serial.println("BMI323 INT1 configured for FIFO watermark.");
}

void imu_start_sensors() {
  //no accel filter
  //imu_write_register16(0x20, 0x702B);

  //accel filter
  imu_write_register16(0x20, 0x73AB);

  //no gyro filter
  //imu_write_register16(0x21, 0x703B);

  //gyro filter
  imu_write_register16(0x21, 0x72AB);
}

void JumpToBootloader(void) {
  HAL_RCC_DeInit();
  HAL_DeInit();
  __HAL_RCC_USB_CLK_DISABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitTypeDef gpio;
  gpio.Pin = GPIO_PIN_12;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_TIM_Base_DeInit(&Tim2Handle);
  HAL_GPIO_Init(GPIOA, &gpio);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
  HAL_Delay(10);
  SysTick->CTRL = 0;
  SysTick->LOAD = 0;
  SysTick->VAL = 0;
  __disable_irq();
  __set_MSP(*(__IO uint32_t*)0x1FFF0000);
  ((void (*)(void))(*(__IO uint32_t*)(0x1FFF0004)))();
}

void setup() {
  Serial.begin(460800);
  while (!Serial)
    ;
  delay(100);

  setup_tim2_microseconds();
  Serial.println("TIM2 microsecond timer started.");

  pinMode(IMU_CS, OUTPUT);
  digitalWrite(IMU_CS, HIGH);
  SPI.begin();

  pinMode(INT1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INT1), imu_int1_handler, FALLING);

  pinMode(STROBE_1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(STROBE_1), camera_strobe_isr, RISING);

  delay(10);
  uint8_t dummy[2];
  imu_read_register(0x00, dummy, 2);
  imu_read_register(0x00, dummy, 2);
  Serial.print("BMI323 chip ID: 0x");
  Serial.println(dummy[0], HEX);

  imu_init_fifo_interrupts();
  delay(2);
  imu_start_sensors();
  Serial.println("BMI323 FIFO + sensors configured. Waiting for interrupts...");
}

void loop() {
  static bool last_logging_enabled = false;

  // Flush FIFO on rising edge of logging_enabled!
  if (logging_enabled && !last_logging_enabled) {
    imu_write_register16(0x37, 0x0001);  // FIFO flush
    Serial.println("IMU FIFO flushed at log start.");
    imu_first = true;
    imu_msb = last_sync_sensortime32 & 0xFFFF0000;
  }
  last_logging_enabled = logging_enabled;

  // Poll and print pending strobe event from ISR
  if (pending_cam_ts) {
    uint8_t st_buf[4];
    imu_read_register(0x0A, st_buf, 4);
    uint32_t sensor_time32 = ((uint32_t)st_buf[3] << 24) | ((uint32_t)st_buf[2] << 16) | ((uint32_t)st_buf[1] << 8) | ((uint32_t)st_buf[0]);
    uint64_t sensor_time_us = ((uint64_t)sensor_time32 * 390625UL + 5000) / 10000;

    Serial.print("CAM,");
    Serial.print(sensor_time_us);
    Serial.println("000");

    pending_cam_ts = 0;
    last_cam_strobe_millis = millis();
  }

  // Timeout logic using millis()
  if (logging_enabled && (millis() - last_cam_strobe_millis > STROBE_TIMEOUT_MS) && !override) {
    logging_enabled = false;
    Serial.println("Logging disabled due to strobe timeout.");
  }

  // FIFO logging as before
  static uint32_t prev_int1_count = 0;
  if (logging_enabled && ((int1_count != prev_int1_count) || int1_fired) && !sync_test_mode) {
    prev_int1_count = int1_count;
    read_and_log_fifo_packets();

    // After each FIFO dump, update sync anchor and print SYNC only if debug_sync enabled
    uint8_t st_buf[4];
    imu_read_register(0x0A, st_buf, 4);
    uint32_t sensor_time32 = ((uint32_t)st_buf[3] << 24) | ((uint32_t)st_buf[2] << 16) | ((uint32_t)st_buf[1] << 8) | ((uint32_t)st_buf[0]);
    uint32_t now_us = timer2_micros();
    last_sync_micros = now_us;
    last_sync_sensortime32 = sensor_time32;
    if (debug_sync) {
      Serial.print("SYNC,");
      Serial.print(now_us);
      Serial.print(",");
      Serial.println(sensor_time32);
    }
    int1_fired = false;
  }

  if (Serial.available()) {
    memset(serial_buf, 0, sizeof(serial_buf));
    Serial.readBytesUntil('\n', serial_buf, sizeof(serial_buf));
    String s = String(serial_buf);

    if (s == "-csv" || s == "-int") {
      output_mode = OUTPUT_COMPACT_INT;
      Serial.println("Output mode: compact int, host does all conversion.");
    } else if (s == "-euroc") {
      output_mode = OUTPUT_EUROC_DATASET;
      Serial.println("Output mode: Euroc/dataset units, direct log.");
    } else if (s == "-bin") {
      output_mode = OUTPUT_BINARY;
      Serial.println("Output mode: binary.");
    } else if (s == "-logon") {
      override = true;
      logging_enabled = true;
      Serial.println("Logging enabled (manual override).");
    } else if (s == "-logoff") {
      override = false;
      logging_enabled = false;
      Serial.println("Logging disabled (manual override).");
    } else if (s == "-debug") {
      euroc_only = !euroc_only;
      Serial.print("Euroc-only output: ");
      Serial.println(euroc_only ? "ON" : "OFF");
    } else if (s == "-syncdebug") {
      debug_sync = !debug_sync;
      Serial.print("SYNC debug: ");
      Serial.println(debug_sync ? "ON" : "OFF");
    } else if (s == "-synctest") {
      Serial.println("Entering sync test mode. Logging strobes...");
      sync_test_mode = true;
    } else if (s == "-endtest") {
      sync_test_mode = false;
      Serial.println("Exiting sync test mode.");
    } else if (s == "-dfu") {
      Serial.println("🌀 DFU jump requested");
      delay(100);
      JumpToBootloader();
    } else if (s == "-sn") {
      uint32_t uid[3];
      uid[0] = HAL_GetUIDw0();
      uid[1] = HAL_GetUIDw1();
      uid[2] = HAL_GetUIDw2();
      Serial.printf("STM32_UID: %08lX%08lX%08lX\n", uid[2], uid[1], uid[0]);
    } else if (s == "-bin") {
      output_mode = OUTPUT_BINARY;
      Serial.println("Output mode: binary (g, deg/s, header, no checksum).");
    } else if (s == "-bin_si") {
      output_mode = OUTPUT_BINARY;
      si_units = true;
      Serial.println("Output mode: binary (m/s², rad/s, SI-scaled).");
    } else if (s == "-csum") {
      use_checksum = true;
      Serial.println("Binary checksum: ON");
    } else if (s == "-nocsum") {
      use_checksum = false;
      Serial.println("Binary checksum: OFF");
    } else if (s == "-help") {
      Serial.println("Available commands:");
      Serial.println(" -int       Output compact CSV (int, µs, g/deg/s)");
      Serial.println(" -euroc     Output Euroc format CSV (ns, float)");
      Serial.println(" -bin       Output binary packets (g, deg/s)");
      Serial.println(" -bin_si    Output binary (m/s², rad/s)");
      Serial.println(" -csum      Enable checksum in binary mode");
      Serial.println(" -nocsum    Disable checksum in binary mode");
      Serial.println(" -logon     Start logging");
      Serial.println(" -logoff    Stop logging");
      Serial.println(" -sn        Print STM32 unique ID");
      Serial.println(" -syncdebug Toggle debug SYNC printout");
      Serial.println(" -dfu       Jump to bootloader");
    } else {
      Serial.println("Unknown command.");
    }
  }
}