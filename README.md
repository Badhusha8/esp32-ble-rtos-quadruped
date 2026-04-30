# RoboDog — ESP-IDF Port

ESP-IDF (v5.x) port of the 12-servo quadruped robot originally written for Arduino + Adafruit PWMServoDriver.

## Hardware

| Signal | ESP32 GPIO |
|--------|-----------|
| I2C SDA | 21 |
| I2C SCL | 22 |

PCA9685 default address: **0x40** (all address pins low).

Servo layout (channel order):

```
Ch 0-2   Front-Left  (hip, knee, foot)
Ch 3-5   Front-Right
Ch 6-8   Back-Left
Ch 9-11  Back-Right
```

## Project structure

```
robodog/
├── CMakeLists.txt          # Top-level ESP-IDF project file
├── sdkconfig.defaults      # Non-interactive config overrides
├── .gitignore
├── components/
│   └── pca9685/            # Reusable PCA9685 driver component
│       ├── CMakeLists.txt
│       ├── include/
│       │   └── pca9685.h
│       └── pca9685.c
└── main/
    ├── CMakeLists.txt
    └── main.c              # Application entry point
```

## Build & flash

```bash
# 1. Source the ESP-IDF environment (adjust path to your installation)
. $HOME/esp/esp-idf/export.sh

# 2. Configure target (only needed once)
idf.py set-target esp32

# 3. Build
idf.py build

# 4. Flash + monitor  (replace /dev/ttyUSB0 with your port)
idf.py -p /dev/ttyUSB0 flash monitor
```

## Serial commands

Connect at **115200 baud** (any terminal; `idf.py monitor` works great).

| Command | Action |
|---------|--------|
| `stand` | Transition through mid-pose → stand |
| `sit`   | Transition through mid-pose → sit   |
| `wave`  | Raise front-right leg and wave twice, return to stand |

## Porting notes

| Arduino / Adafruit | ESP-IDF equivalent |
|---|---|
| `Wire.begin(SDA, SCL)` | `i2c_new_master_bus()` (new I2C master API) |
| `pca.begin()` + `pca.setPWMFreq()` | `pca9685_init()` |
| `pca.setPWM(ch, 0, val)` | `pca9685_set_pwm_value()` |
| `delay(ms)` | `vTaskDelay(pdMS_TO_TICKS(ms))` |
| `Serial.readStringUntil('\n')` | UART driver + ring-buffer task |

## Git setup (first time)

```bash
cd robodog
git init
git add .
git commit -m "Initial commit: ESP-IDF port of RoboDog"

# Add remote and push
git remote add origin <your-repo-url>
git push -u origin main
```
