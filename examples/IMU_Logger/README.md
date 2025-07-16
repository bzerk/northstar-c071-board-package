# IMU Logger Example

This sketch is the canonical North Star stereocamera firmware for the STM32C071GBU6 board. It:

- Streams IMU data from a BMI323 over SPI
- Detects camera strobe pulses via GPIO interrupt
- Outputs data in multiple formats (Euroc CSV, compact int, or binary)
- Accepts runtime serial commands to control behavior
- Supports DFU bootloader jump via `-dfu` command

## Pin Assignments

| Signal        | STM32 Pin | Notes                      |
|---------------|-----------|----------------------------|
| IMU_CS        | PA4       | BMI323 chip select         |
| IMU_INT1      | PA1       | FIFO watermark interrupt   |
| CAM_STROBE    | PB4       | Rising-edge input (GPIO)   |
| SPI           | PA5/PA6/PA7 | SPI1 (SCK/MISO/MOSI)     |
| USB CDC       | PA11/PA12 | Native USB (CDC Serial)    |

## Serial Commands

Send via serial terminal (no prefix required):

- `-int` – Compact integer output format
- `-euroc` – Euroc dataset format (ns, SI units via int math)
- `-bin` – Raw binary packet mode
- `-bin_si` – Binary with SI unit scaling
- `-csum` / `-nocsum` – Toggle binary checksum
- `-logon` / `-logoff` – Manually enable or disable logging
- `-dfu` – Jump to USB DFU bootloader
- `-sn` – Print unique STM32 serial number
- `-syncdebug` – Print raw timestamp sync debug data
- `-help` – List all supported commands

---

## Notes

- TIM2 provides microsecond resolution timestamping.
- Logging is triggered by strobe events, with timeout fallback.
- DFU jump is always responsive.

