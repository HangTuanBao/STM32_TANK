# 🚜 Hệ Thống Điều Khiển Xe Tăng Bánh Xích STM32

<p align="center">
  <img src="https://img.shields.io/badge/MCU-STM32F103C8T6-blue.svg?style=for-the-badge&logo=stmicroelectronics" alt="STM32F103C8T6" />
  <img src="https://img.shields.io/badge/Ngôn_ngữ-C99-00599C.svg?style=for-the-badge&logo=c" alt="Language C" />
  <img src="https://img.shields.io/badge/Framework-STM32%20HAL-03234B.svg?style=for-the-badge" alt="STM32 HAL" />
  <img src="https://img.shields.io/badge/Tay_cầm-PS2%20Wireless-003791.svg?style=for-the-badge&logo=playstation" alt="PS2 Controller" />
  <img src="https://img.shields.io/badge/Cảm_biến-MPU6050%20IMU-FF6F00.svg?style=for-the-badge" alt="MPU6050" />
  <img src="https://img.shields.io/badge/Bản_quyền-MIT-green.svg?style=for-the-badge" alt="License MIT" />
</p>

<p align="center">
  <b>🌐 Chuyển đổi ngôn ngữ / Language Switcher:</b><br>
  <a href="README_VI.md"><b>🇻🇳 Tiếng Việt</b></a> &nbsp;|&nbsp; <a href="README.md"><b>🇬🇧 English</b></a>
</p>

---

## 📖 Tổng quan dự án

**STM32 Dual-Track Robotic Tank** là dự án firmware điều khiển robot xe tăng bánh xích (hoặc xe 2 bánh vi sai) thời gian thực, độ chính xác cao. Dự án xây dựng trên nền tảng vi điều khiển **STM32F103C8T6 (ARM Cortex-M3 xung nhịp 72MHz)** kết hợp giao tiếp không dây thời gian thực với tay cầm **PS2 DualShock**, cảm biến quán tính **MPU6050 (6-DOF IMU)** và hệ thống đọc **Encoder phần cứng** 2 kênh độc lập.

Mã nguồn được tối ưu theo chuẩn lập trình nhúng an toàn, tích hợp các thuật toán điều khiển tự động như **PID Heading Hold** (tự động giữ thẳng hướng lái khi xích bị trượt/địa hình gồ ghề), **Smart Turn 90°** (tự động xoay góc vuông chuẩn xác bù ma sát nghỉ) và cơ chế bảo vệ **Anti-Flip Guard** (tự động hãm lùi chống lật xe khi leo dốc đứng).

---

## ✨ Các tính năng nổi bật

- 🎮 **Điều khiển không dây qua tay cầm PS2 (Hardware SPI1)**
  - Giao tiếp ổn định qua chuẩn SPI phần cứng tần số cao.
  - Hỗ trợ 2 chế độ: **Analog Mode** (lái mượt mà bằng cần gạt Joystick) và **Digital Mode** (lái bằng cụm phím D-Pad).
  - Tích hợp bộ lọc vùng chết (Deadzone $\pm 15$) và thuật toán tự động nhận diện ngắt kết nối.

- 🧭 **Hệ thống điều hướng quán tính IMU 6-DOF (MPU6050 qua I2C2)**
  - **PID Heading Hold:** Tự động điều chỉnh vi sai công suất 2 động cơ khi đi thẳng, triệt tiêu hoàn toàn hiện tượng lệch hướng do xích trượt.
  - **Smart Turning (Rẽ góc 90° thông minh):** Nhấn phím D-Pad Trái/Phải để xe tự động quay chính xác góc 90° với thuật toán bù mô-men xoắn thắng ma sát nghỉ của xích xe ($PWM_{min} \ge 75$).
  - **Tính toán góc thời gian thực $\Delta t$:** Lấy tích phân Gyro theo chu kỳ quét động, chống trôi góc và bão hòa cảm biến.

- 🛡️ **Hệ thống an toàn chủ động & Fail-Safe**
  - **Anti-Flip / Pitch Guard:** Nếu góc dốc vượt ngưỡng an toàn $\pm 35^\circ$ (xe bị ngóc đầu nguy cơ lật ngửa), hệ thống lập tức ngắt lệnh người dùng và tự động lùi hãm khẩn cấp tối thiểu 500ms.
  - **Fail-Safe mất kết nối:** Lập tức dừng toàn bộ động cơ ($PWM = 0$) nếu tay cầm PS2 mất sóng hoặc cảm biến MPU6050 mất tín hiệu I2C.
  - **Manual Override:** Ưu tiên tuyệt đối quyền điều khiển của người lái khi can thiệp gạt cần Joystick.

- 🔄 **Đọc Encoder phần cứng 2 kênh (Quadrature 4x Decoding)**
  - Đếm xung vị trí và tốc độ chính xác từng mili-giây bằng bộ đếm Timer phần cứng trên **TIM2** (Xích Trái) và **TIM4** (Xích Phải).

- ⚡ **Điều chế xung PWM tần số cao (TIM1)**
  - Tần số 1kHz (`ARR = 999`, `PSC = 7`) giúp động cơ vận hành êm ái, lực kéo khỏe, không phát tiếng rít khó chịu.
  - Thuật toán pha trộn lái xe tăng (Arcade Drive Mixing) phản hồi mượt mà.

---

## 📐 Sơ đồ kết nối phần cứng (Pinout)

### 🔌 Bảng đấu nối chân vi điều khiển STM32F103C8T6

| Thiết bị / Module | Chân STM32 | Chế độ ngoại vi | Chức năng / Ý nghĩa |
| :--- | :--- | :--- | :--- |
| **Động cơ Trái - ENA** | `PA8` | TIM1_CH1 | Cấp xung tốc độ PWM (0 – 999) |
| **Động cơ Phải - ENB** | `PA11` | TIM1_CH4 | Cấp xung tốc độ PWM (0 – 999) |
| **Động cơ Trái - IN1, IN2** | `PB12`, `PB13` | GPIO Output | Điều khiển chiều quay Động cơ Trái (Cầu H) |
| **Động cơ Phải - IN3, IN4** | `PB14`, `PB15` | GPIO Output | Điều khiển chiều quay Động cơ Phải (Cầu H) |
| **Encoder Trái (Kênh A / B)**| `PA0`, `PA1` | TIM2_CH1 / CH2 | Đọc Encoder 4x Động cơ Trái |
| **Encoder Phải (Kênh A / B)**| `PB6`, `PB7` | TIM4_CH1 / CH2 | Đọc Encoder 4x Động cơ Phải |
| **Đầu thu PS2 - Chân CS (SS)**| `PA4` | GPIO Output | Chọn chip (Kéo xuống 0 khi giao tiếp) |
| **Đầu thu PS2 - Chân SCK** | `PA5` | SPI1_SCK | Xung nhịp đồng hồ SPI |
| **Đầu thu PS2 - Chân MISO (DAT)**| `PA6` | SPI1_MISO | Nhận dữ liệu từ tay cầm (Kéo trở treo) |
| **Đầu thu PS2 - Chân MOSI (CMD)**| `PA7` | SPI1_MOSI | Gửi lệnh sang tay cầm |
| **Cảm biến MPU6050 - SCL** | `PB10` | I2C2_SCL | Xung đồng hồ I2C (100kHz) |
| **Cảm biến MPU6050 - SDA** | `PB11` | I2C2_SDA | Dữ liệu nối tiếp I2C |
| **LED Trạng thái On-board** | `PC13` | GPIO Output | LED báo trạng thái bo mạch |

---

## 🕹️ Sơ đồ phím điều khiển (Tay cầm PS2 Wireless)

```
                       [L1] [L2]                 [R1] [R2]
                      +---------+---------------+---------+
                      |         |   [SELECT]    |         |
      [D-PAD UP]      |   (^)   |    [MODE]     |   (^)   |  [TAM GIÁC]
  [TRÁI]      [PHẢI]  | (<) (>) |    [START]    | ([]) (O)|  [VUÔNG] [TRÒN]
     [D-PAD DOWN]     |   (v)   |               |   (X)   |  [X]
                      +---------+---------------+---------+
                             \     (LY/LX)     (RY/RX)     /
                              \   [JOY_L]     [JOY_R]    /
                               \                         /
```

### 1. Chế độ Analog (Đèn Đỏ trên tay cầm SÁNG) — *Khuyên dùng*
- **Cần gạt Trái (Trục Y - LY):** Tiến / Lùi (`Tốc độ`)
- **Cần gạt Phải (Trục X - RX):** Bẻ lái Trái / Phải (`Độ rẽ`)
- **D-Pad TRÁI:** Tự động quay xe sang **Trái 90°** chuẩn xác
- **D-Pad PHẢI:** Tự động quay xe sang **Phải 90°** chuẩn xác
- **Khi đi thẳng (Speed $\ne$ 0, Steer = 0):** **Tự động kích hoạt giữ thẳng hướng (PID Heading Hold)** 🧭

### 2. Chế độ Digital (Đèn Đỏ trên tay cầm TẮT)
- **D-Pad LÊN / XUỐNG:** Tiến / Lùi tối đa công suất
- **D-Pad TRÁI / PHẢI:** Xoay tròn tại chỗ Trái / Phải

---

## 🧠 Thuật toán điều khiển cốt lõi

### 1. Thuật toán pha trộn lái vi sai (Arcade Drive)
$$\begin{aligned}
\text{PWM}_{\text{Trái}} &= \text{clamp}\left((\text{Speed} + \text{Steer}) \times \frac{999}{128}, -999, 999\right) \\
\text{PWM}_{\text{Phải}} &= \text{clamp}\left((\text{Speed} - \text{Steer}) \times \frac{999}{128}, -999, 999\right)
\end{aligned}$$

### 2. Bộ điều khiển PID giữ thẳng hướng (Heading Hold)
Khi xe đang di chuyển thẳng, hệ thống khóa góc Yaw chuẩn ban đầu $Y_{\text{target}}$:
$$\text{Sai số} = \text{Wrap}_{[-180, 180]}(Y_{\text{target}} - Y_{\text{hiện tại}})$$
$$\text{Lực vi chỉnh} = K_p \cdot \text{Sai số} + K_d \cdot \frac{d(\text{Sai số})}{dt}$$
$$\text{Steer}_{\text{PID}} = -\text{clamp}(\text{Lực vi chỉnh}, -40, 40)$$

### 3. Cơ chế bảo vệ chống lật xe (Anti-Flip Guard)
$$\text{Nếu } |\text{Góc Pitch}| > 35^\circ \implies \text{Hãm lùi khẩn cấp } (\text{Speed} = -64, \text{Steer} = 0) \text{ trong } \ge 500\text{ms}$$

---

## 📁 Cấu trúc thư mục mã nguồn

```
STM32_TANK/
├── Core/
│   ├── Inc/
│   │   ├── main.h               # Định nghĩa chung và định danh phần cứng
│   │   ├── gpio.h               # Cấu hình các chân GPIO
│   │   ├── tim.h                # Khởi tạo TIM1 (PWM), TIM2/TIM4 (Encoder)
│   │   ├── spi.h                # Khởi tạo SPI1 cho tay cầm PS2
│   │   ├── i2c.h                # Khởi tạo I2C2 cho MPU6050
│   │   └── mpu6050.h            # Thư viện đọc và tính toán góc MPU6050
│   └── Src/
│       ├── main.c               # Vòng lặp chính, máy trạng thái & thuật toán
│       ├── gpio.c               # Cấu hình GPIO chi tiết
│       ├── tim.c                # Cấu hình bộ đếm Timer & ngắt
│       ├── spi.c                # Giao thức SPI phần cứng
│       ├── i2c.c                # Giao thức I2C phần cứng
│       ├── mpu6050.c            # Hiệu chuẩn Gyro & tính góc nghiêng
│       └── stm32f1xx_it.c       # Trình phục vụ ngắt hệ thống (ISR)
├── Drivers/                     # Thư viện chuẩn STMicroelectronics HAL & CMSIS
├── STM32F103C8TX_FLASH.ld       # File kịch bản Linker bộ nhớ Flash STM32F103
├── STM32_DC_2MOTOR.ioc          # File cấu hình đồ họa STM32CubeMX
├── .gitignore                   # Danh sách loại trừ file tạm build
├── README.md                    # Bản tài liệu tiếng Anh (English)
└── README_VI.md                 # Bản tài liệu tiếng Việt (Tiếng Việt)
```

---

## 🛠️ Hướng dẫn cài đặt & Nạp chương trình

### Yêu cầu môi trường
- **Phần mềm:** [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) (phiên bản v1.14.0 trở lên)
- **Công cụ nạp:** Mạch nạp ST-Link V2 (kết nối qua 4 chân `SWDIO`, `SWCLK`, `GND`, `3V3`)
- **Trình biên dịch:** `arm-none-eabi-gcc`

### Các bước thực hiện

1. **Tải mã nguồn về máy:**
   ```bash
   git clone https://github.com/HangTuanBao/STM32_TANK.git
   cd STM32_TANK
   ```

2. **Mở dự án trong STM32CubeIDE:**
   - Khởi động phần mềm STM32CubeIDE.
   - Chọn `File` $\rightarrow$ `Open Projects from File System...`.
   - Bấm `Directory...` trỏ đến thư mục dự án `STM32_TANK` rồi bấm **Finish**.

3. **Biên dịch mã nguồn (Build):**
   - Nhấn phím tắt `Ctrl + B` (hoặc biểu tượng Chiếc búa 🔨).
   - Đảm bảo biên dịch thành công `0 errors, 0 warnings`.

4. **Nạp code vào xe tăng:**
   - Cắm mạch nạp ST-Link V2 vào máy tính và kết nối cổng SWD trên vi điều khiển.
   - Bấm `Run` $\rightarrow$ `Debug` (hoặc phím `F11`) để nạp chương trình vào chip.
   - ⚠️ **Lưu ý:** Khi vừa bật nguồn hoặc reset, hãy đặt xe trên mặt phẳng yên tĩnh trong 1-2 giây để hệ thống tự động cân chỉnh (Calibrate) MPU6050.

---

## 🛡️ Tiêu chuẩn lập trình nhúng an toàn

Dự án tuân thủ nghiêm ngặt các nguyên tắc phần mềm nhúng tin cậy cao:
- ✅ **Không cấp phát bộ nhớ động:** Tuyệt đối không sử dụng `malloc()` / `free()` trong suốt quá trình chạy để tránh phân mảnh RAM.
- ✅ **Vòng lặp không nghẽn (Non-blocking):** Quét dữ liệu định thời ~40Hz (25ms) mượt mà.
- ✅ **Kiểm tra giới hạn an toàn phần cứng (Bounding check):** Ép giới hạn PWM nghiêm ngặt $[-999, 999]$.
- ✅ **Fail-Safe chủ động:** Tự động cắt nguồn động cơ ngay khi mất kết nối.

---

## 👤 Tác giả & Liên hệ

- **Tác giả:** Hạng Tuấn Bảo (Hang Tuan Bao)
- **Email:** [770004htb@gmail.com](mailto:770004htb@gmail.com)
- **Câu lạc bộ:** PIF Embedded Robotics Club

---

## 📄 Giấy phép bản quyền (License)

Dự án được phân phối dưới giấy phép **[MIT License](LICENSE)** — cho phép tự do sử dụng, chỉnh sửa và phát triển cho các mục đích học tập và nghiên cứu robot.
