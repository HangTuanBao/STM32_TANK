# 🚜 STM32 Dual-Track Robotic Tank Controller

<p align="center">
  <img src="https://img.shields.io/badge/MCU-STM32F103C8T6-blue.svg?style=for-the-badge&logo=stmicroelectronics" alt="STM32F103C8T6" />
  <img src="https://img.shields.io/badge/Language-C99-00599C.svg?style=for-the-badge&logo=c" alt="Language C" />
  <img src="https://img.shields.io/badge/Framework-STM32%20HAL-03234B.svg?style=for-the-badge" alt="STM32 HAL" />
  <img src="https://img.shields.io/badge/Controller-PS2%20Wireless-003791.svg?style=for-the-badge&logo=playstation" alt="PS2 Controller" />
  <img src="https://img.shields.io/badge/Sensor-MPU6050%20IMU-FF6F00.svg?style=for-the-badge" alt="MPU6050" />
  <img src="https://img.shields.io/badge/License-MIT-green.svg?style=for-the-badge" alt="License MIT" />
</p>

<p align="center">
  <b>🌐 Language Switcher / Chuyển đổi ngôn ngữ:</b><br>
  <a href="README.md"><b>🇬🇧 English</b></a> &nbsp;|&nbsp; <a href="README_VI.md"><b>🇻🇳 Tiếng Việt</b></a>
</p>

---

## 📖 Overview

**STM32 Dual-Track Robotic Tank** is a high-performance embedded robotics firmware designed for tracked/differential-drive vehicles. Built around the **STM32F103C8T6 (ARM Cortex-M3 @ 72MHz)** microcontroller, this firmware integrates closed-loop IMU feedback, dual quadrature encoder odometry, responsive PS2 wireless remote control, and real-time safety fail-safes.

The system features advanced algorithms including **PID Heading Hold** (auto-straight driving on slippery/uneven terrain), **Smart 90° Snap-Turning** with static friction compensation, and an **Anti-Flip Pitch Guard** for safe slope climbing.

---

## ✨ Key Features

- 🎮 **Wireless PS2 Remote Control (Hardware SPI1)**
  - Seamless DualShock wireless controller integration.
  - Dual Mode Support: **Analog Mode** (smooth 2-axis joystick driving) and **Digital Mode** (tactile D-Pad control).
  - Deadzone filtering ($\pm 15$) and automatic mode synchronization.

- 🧭 **Closed-Loop 6-DOF IMU Navigation (MPU6050 via I2C2)**
  - **PID Heading Hold:** Automatically corrects trajectory drift when driving straight over rough or uneven surfaces.
  - **Smart Turn (90° Snap Turn):** One-touch exact 90-degree pivot turns via D-Pad Left/Right with track friction boost ($PWM_{min} \ge 75$).
  - **Dynamic $\Delta t$ Angle Integration:** Real-time gyro integration with time-drift protection.

- 🛡️ **Autonomous Safety & Fail-Safe Guards**
  - **Anti-Flip / Pitch Guard:** Automatically brakes and backs up if vehicle pitch angle exceeds $\pm 35^\circ$ to prevent backwards flipping.
  - **Connection Lost Guard:** Immediate zero-power emergency stop ($PWM = 0$) upon PS2 receiver timeout or MPU6050 disconnect.
  - **Priority Manual Override:** Instant manual control takeover whenever joystick input is detected.

- 🔄 **Dual Quadrature Hardware Encoders**
  - High-precision 4x decoding on **TIM2** (Left Track) and **TIM4** (Right Track) for real-time odometry and velocity feedback.

- ⚡ **High-Frequency Motor PWM (TIM1)**
  - 1kHz smooth PWM switching (`ARR = 999`, `PSC = 7`) for silent, high-torque motor response.
  - Differential / Arcade Drive mixing algorithm for intuitive maneuvering.

---

## 📐 System Architecture & Hardware Pinout

### 🔌 Pinout Mapping Table

| Peripheral | Pin | Type | Function / Description |
| :--- | :--- | :--- | :--- |
| **Motor 1 PWM (Left)** | `PA8` | TIM1_CH1 | ENA - PWM Speed Control (0–999) |
| **Motor 2 PWM (Right)** | `PA11` | TIM1_CH4 | ENB - PWM Speed Control (0–999) |
| **Motor 1 Direction** | `PB12`, `PB13` | GPIO Output | IN1, IN2 - Left Direction Control (H-Bridge) |
| **Motor 2 Direction** | `PB14`, `PB15` | GPIO Output | IN3, IN4 - Right Direction Control (H-Bridge) |
| **Encoder 1 (Left)** | `PA0`, `PA1` | TIM2_CH1 / CH2 | Encoder Phase A / Phase B (4x mode) |
| **Encoder 2 (Right)** | `PB6`, `PB7` | TIM4_CH1 / CH2 | Encoder Phase A / Phase B (4x mode) |
| **PS2 Wireless - SS (CS)** | `PA4` | GPIO Output | Chip Select (Active LOW) |
| **PS2 Wireless - SCK** | `PA5` | SPI1_SCK | Hardware SPI Clock |
| **PS2 Wireless - MISO**| `PA6` | SPI1_MISO | Data In (Pull-up) |
| **PS2 Wireless - MOSI**| `PA7` | SPI1_MOSI | Command Out |
| **MPU6050 - SCL** | `PB10` | I2C2_SCL | I2C Clock (100kHz Standard Mode) |
| **MPU6050 - SDA** | `PB11` | I2C2_SDA | I2C Data Line (Open-Drain + Pull-up) |
| **Status LED** | `PC13` | GPIO Output | On-board Blue LED |

---

## 🕹️ Control Schema (PS2 Wireless Controller)

```
                       [L1] [L2]                 [R1] [R2]
                      +---------+---------------+---------+
                      |         |   [SELECT]    |         |
      [D-PAD UP]      |   (^)   |    [MODE]     |   (^)   |  [TRIANGLE]
  [LEFT]     [RIGHT]  | (<) (>) |    [START]    | ([]) (O)|  [SQUARE] [CIRCLE]
     [D-PAD DOWN]     |   (v)   |               |   (X)   |  [CROSS]
                      +---------+---------------+---------+
                             \     (LY/LX)     (RY/RX)     /
                              \   [JOY_L]     [JOY_R]    /
                               \                         /
```

### 1. Analog Mode (Red LED ON) — Recommended
- **Left Joystick (LY):** Throttle Forward / Backward (`speed = -ly_val`)
- **Right Joystick (RX):** Steering Left / Right (`steer = rx_val`)
- **D-Pad Left:** Trigger Auto-Turn **90° Left** (Yaw +90°)
- **D-Pad Right:** Trigger Auto-Turn **90° Right** (Yaw -90°)
- **Straight Drive (Speed $\ne$ 0, Steer = 0):** **PID Heading Hold Active** 🧭

### 2. Digital Mode (Red LED OFF)
- **D-Pad UP:** Full-speed Forward
- **D-Pad DOWN:** Full-speed Backward
- **D-Pad LEFT / RIGHT:** Spin Turn Left / Right

---

## 🧠 Control Algorithms

### 1. Differential / Arcade Drive Mixer
$$\begin{aligned}
\text{PWM}_{\text{Left}} &= \text{clamp}\left((\text{Speed} + \text{Steer}) \times \frac{999}{128}, -999, 999\right) \\
\text{PWM}_{\text{Right}} &= \text{clamp}\left((\text{Speed} - \text{Steer}) \times \frac{999}{128}, -999, 999\right)
\end{aligned}$$

### 2. Heading Hold Closed-Loop PID
When driving in a straight line, the vehicle locks its current yaw heading $Y_{\text{target}}$:
$$\text{Error} = \text{Wrap}_{[-180, 180]}(Y_{\text{target}} - Y_{\text{current}})$$
$$\text{Correction} = K_p \cdot \text{Error} + K_d \cdot \frac{d(\text{Error})}{dt}$$
$$\text{Steer}_{\text{PID}} = -\text{clamp}(\text{Correction}, -40, 40)$$

### 3. Anti-Flip Pitch Guard
$$\text{if } |\text{Pitch}| > 35^\circ \implies \text{Force Emergency Reverse } (\text{Speed} = -64, \text{Steer} = 0) \text{ for } \ge 500\text{ms}$$

---

## 📁 Project Structure

```
STM32_TANK/
├── Core/
│   ├── Inc/
│   │   ├── main.h               # Core definitions & pin mappings
│   │   ├── gpio.h               # GPIO initialization headers
│   │   ├── tim.h                # TIM1 (PWM), TIM2/TIM4 (Encoders)
│   │   ├── spi.h                # SPI1 for PS2 controller
│   │   ├── i2c.h                # I2C2 for MPU6050 IMU
│   │   └── mpu6050.h            # 6-DOF IMU driver header
│   └── Src/
│       ├── main.c               # Main state machine, PID & control logic
│       ├── gpio.c               # GPIO peripheral setup
│       ├── tim.c                # Timer configurations (PWM & Encoder Mode)
│       ├── spi.c                # Hardware SPI configuration
│       ├── i2c.c                # I2C hardware bus setup
│       ├── mpu6050.c            # MPU6050 math & calibration routines
│       └── stm32f1xx_it.c       # Interrupt Service Routines (ISRs)
├── Drivers/                     # STMicroelectronics CMSIS & HAL Drivers
├── STM32F103C8TX_FLASH.ld       # Linker script for STM32F103C8T6
├── STM32_DC_2MOTOR.ioc          # STM32CubeMX Project Configuration
├── .gitignore                   # Git ignore for embedded build artifacts
└── README.md                    # Project documentation
```

---

## 🛠️ Getting Started & Build Instructions

### Prerequisites
- **IDE:** [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) (v1.14.0 or newer) / VS Code with Cortex-Debug
- **Toolchain:** `arm-none-eabi-gcc`
- **Hardware Debugger:** ST-Link V2 / J-Link / DAPLink

### Build & Flash Steps

1. **Clone the repository:**
   ```bash
   git clone https://github.com/<your-username>/STM32_TANK.git
   cd STM32_TANK
   ```

2. **Open in STM32CubeIDE:**
   - Launch STM32CubeIDE.
   - Go to `File` $\rightarrow$ `Open Projects from File System...`.
   - Select the `STM32_TANK` project directory and click **Finish**.

3. **Build the Project:**
   - Press `Ctrl + B` (or click the Hammer icon 🔨) to compile.
   - Output files (`.elf`, `.hex`, `.bin`) will be generated inside the `Debug/` folder.

4. **Flash to Target Board:**
   - Connect your ST-Link V2 to the SWD pins (`SWDIO`, `SWCLK`, `GND`, `3V3`).
   - Click `Run` $\rightarrow$ `Debug` (or press `F11`) to download firmware.
   - Place the robot on a flat, level surface during startup for automatic MPU6050 gyro calibration.

---

## 🛡️ Embedded Safety Compliance

This codebase strictly adheres to deterministic embedded programming best practices:
- ✅ **Zero Dynamic Memory Allocation:** No `malloc()`, `free()`, or heap fragmentation at runtime.
- ✅ **Deterministic Loop Execution:** Non-blocking 40Hz (~25ms) cycle time.
- ✅ **Hardware Deadband & Bounding:** Strict clamp on all PWM channels ($0 \le |PWM| \le 999$).
- ✅ **Fail-Safe Watch:** Instant shutoff on signal lost.

---

## 👤 Author & Acknowledgments

- **Author:** Hang Tuan Bao
- **Email:** [770004htb@gmail.com](mailto:770004htb@gmail.com)
- **Organization:** PIF Embedded Robotics Club

---

## 📄 License

This project is licensed under the [MIT License](LICENSE) - feel free to use and adapt for academic and personal robotics projects.
