# PlatformIO Support for North Star STM32C071GBU6

To use this board with PlatformIO:

## 1. Clone This Repo

```bash
git clone https://github.com/bzerk/northstar-c071-board-package.git
```

## 2. Create `platformio.ini`

```ini
[env:northstar]
platform = ststm32
board = northstar_c071
framework = arduino
board_build.variants_dir = northstar-c071-board-package/STM32C0/1.0.0/variants
board_build.variant = NorthStar_C071GBU6
board_build.core = stm32
upload_protocol = dfu
```

## 3. Register the Board

Copy `platformio/board.json` to:

```bash
~/.platformio/boards/northstar_c071.json
```

Now you can build + flash:

```bash
pio run
pio run -t upload
```
