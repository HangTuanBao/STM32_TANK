/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include <math.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile int encoderCount1 =
    0; // Biến lưu số xung đếm được từ Encoder 1 (Động cơ trái)
volatile int encoderCount2 =
    0; // Biến lưu số xung đếm được từ Encoder 2 (Động cơ phải)

volatile int32_t total_encoder1 =
    0; // Biến tích lũy tổng vị trí xung từ Encoder 1
volatile int32_t total_encoder2 =
    0; // Biến tích lũy tổng vị trí xung từ Encoder 2

/* Các biến lưu trạng thái tay cầm PS2 */
uint8_t ps2_lx =
    128; // Giá trị cần Analog trái trục X (Trái: 0, Giữa: 128, Phải: 255)
uint8_t ps2_ly =
    128; // Giá trị cần Analog trái trục Y (Lên: 0, Giữa: 128, Xuống: 255)
uint8_t ps2_rx =
    128; // Giá trị cần Analog phải trục X (Trái: 0, Giữa: 128, Phải: 255)
uint8_t ps2_ry =
    128; // Giá trị cần Analog phải trục Y (Dự phòng / Reserved - Chưa dùng)
uint16_t ps2_buttons = 0xFFFF; // Trạng thái 16 nút bấm (Mỗi bit đại diện 1 nút,
                               // mặc định 1 là nhả, 0 là nhấn)
uint8_t ps2_connected = 0; // Trạng thái kết nối tay cầm (1: Đang kết nối thành
                           // công, 0: Mất kết nối)
uint8_t ps2_rx_raw[9] = {
    0}; // Mảng lưu 9 byte phản hồi thô từ tay cầm để phục vụ debug

/* Biến và tham số PID Heading Hold (Tự động giữ thẳng hướng) */
float Kp_heading = 3.5f;  // Hệ số P vi chỉnh hướng (giảm xuống 3.5 để vi chỉnh
                          // mượt, không dừng bánh)
float Kd_heading = 0.02f; // Hệ số D giảm dao động (đã chia dt)
float target_heading = 0.0f;     // Góc hướng mục tiêu khi đi thẳng
uint8_t heading_hold_active = 0; // Trạng thái kích hoạt Heading Hold

/* Biến Anti-Flip Guard (Chống lật xe khi leo dốc) */
uint8_t is_anti_flip = 0;     // Cờ an toàn chống lật khẩn cấp
uint32_t anti_flip_timer = 0; // Thời gian hãm lùi khi ngóc đầu

/* Biến Smart Turning (Rẽ góc 90 độ chuẩn xác bằng D-Pad Trái/Phải) */
uint8_t smart_turn_active = 0;  // Cờ xoay góc tự động
float smart_turn_target = 0.0f; // Góc xoay mục tiêu
float smart_turn_angle = 108.0f; // Góc xoay cảm biến (tinh chỉnh bù sai số Gyro/xích xe để quay thực tế đúng 90 độ)

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Update_Encoder_Position(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Update_Encoder_Position(void) {
  static uint8_t initialized = 0;
  static uint16_t prev_cnt1 = 0;
  static uint16_t prev_cnt2 = 0;

  uint16_t current_cnt1 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
  uint16_t current_cnt2 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);

  // Lần đọc đầu tiên: Khởi tạo giá trị cũ bằng giá trị hiện tại để tránh giật
  // xung giả
  if (!initialized) {
    prev_cnt1 = current_cnt1;
    prev_cnt2 = current_cnt2;
    initialized = 1;
    return;
  }

  // Tính chênh lệch xung từ lần đọc trước (dùng uint16_t ép kiểu sang int16_t
  // để xử lý tràn số chuẩn)
  int16_t diff1 = (int16_t)(current_cnt1 - prev_cnt1);
  int16_t diff2 = (int16_t)(current_cnt2 - prev_cnt2);

  // Đảo chiều đếm cho Encoder 1 (do dây A/B hoặc chiều quay động cơ 1 ngược với
  // quy ước tiến)
  diff1 = -diff1;

  // Cộng dồn vào tổng vị trí
  total_encoder1 += diff1;
  total_encoder2 += diff2;

  // Cập nhật giá trị cũ
  prev_cnt1 = current_cnt1;
  prev_cnt2 = current_cnt2;
}

/* Hàm truyền/nhận 1 byte qua Hardware SPI1 và tạo trễ nhỏ giữa các byte */
uint8_t PS2_SPI_Transfer(uint8_t data) {
  uint8_t rx_data = 0;
  // Sử dụng thư viện HAL để vừa gửi byte `data` đi, vừa nhận byte hồi đáp về
  // `rx_data`
  if (HAL_SPI_TransmitReceive(&hspi1, &data, &rx_data, 1, 100) != HAL_OK) {
    return 0xFF; // Trả về 0xFF nếu truyền nhận SPI thất bại
  }
  // Vòng lặp trễ khoảng 40-50 micro giây để MCU bên trong tay cầm PS2 kịp xử lý
  // byte vừa nhận
  for (volatile uint32_t i = 0; i < 1000; i++)
    ;
  return rx_data;
}

/* Hàm gửi một gói lệnh điều khiển (cmd) và nhận về gói phản hồi (rx) từ tay cầm
 */
void PS2_SendCmd(uint8_t *cmd, uint8_t *rx, uint8_t len) {
  // Kéo chân chọn chip SS (PA4) xuống mức THẤP để bắt đầu phiên truyền nhận dữ
  // liệu
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  for (volatile uint32_t i = 0; i < 200; i++)
    ; // Chờ tín hiệu ổn định

  // Gửi lần lượt từng byte trong gói lệnh và lưu lại byte phản hồi tương ứng
  for (uint8_t i = 0; i < len; i++) {
    rx[i] = PS2_SPI_Transfer(cmd[i]);
  }

  // Kéo chân SS (PA4) lên mức CAO để kết thúc phiên truyền nhận dữ liệu
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  for (volatile uint32_t i = 0; i < 500; i++)
    ; // Chờ giải phóng dòng truyền
}

/* Khởi tạo tay cầm PS2: kích hoạt chế độ Analog và khóa cố định chế độ này */
void PS2_Init(void) {
  uint8_t rx[9];

  // Đảm bảo chân CS (PA4) bắt đầu ở mức CAO (chưa chọn chip) trước khi giao
  // tiếp
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  HAL_Delay(100); // Chờ tay cầm và đầu thu ổn định nguồn

  // Bước 1: Gửi lệnh yêu cầu tay cầm vào chế độ cấu hình (Config Mode)
  // Lệnh: 0x01 (Bắt đầu), 0x43 (Vào config), 0x00 (Dummy), 0x01 (Bật config),
  // tiếp theo là các byte dummy 0x00
  uint8_t cmd_enter_config[] = {0x01, 0x43, 0x00, 0x01, 0x00,
                                0x00, 0x00, 0x00, 0x00};
  PS2_SendCmd(cmd_enter_config, rx, 9);
  HAL_Delay(20);

  // Bước 2: Cho phép chế độ Analog Mode và MỞ KHÓA (Unlock) nút MODE để người
  // dùng bấm nút MODE chuyển qua lại Digital/Analog tự do Byte 4 = 0x00 (Unlock
  // MODE button)
  uint8_t cmd_set_analog[] = {0x01, 0x44, 0x00, 0x01, 0x00,
                              0x00, 0x00, 0x00, 0x00};
  PS2_SendCmd(cmd_set_analog, rx, 9);
  HAL_Delay(20);

  // Bước 3: Thoát khỏi chế độ cấu hình để tay cầm bắt đầu gửi dữ liệu joystick
  // Lệnh: 0x01 (Bắt đầu), 0x43 (Cấu hình mode), 0x00, 0x00 (Thoát config), tiếp
  // theo gửi 0x5A để xác nhận thoát
  uint8_t cmd_exit_config[] = {0x01, 0x43, 0x00, 0x00, 0x5A,
                               0x5A, 0x5A, 0x5A, 0x5A};
  PS2_SendCmd(cmd_exit_config, rx, 9);
  HAL_Delay(20);
}

/* Đọc dữ liệu thô từ tay cầm PS2 và cập nhật vào các biến trạng thái */
void PS2_Read(void) {
  // Lệnh yêu cầu gửi dữ liệu: 0x01 (Bắt đầu), 0x42 (Yêu cầu data), còn lại
  // dummy
  uint8_t cmd_poll[] = {0x01, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  PS2_SendCmd(cmd_poll, ps2_rx_raw, 9);

  // Kiểm tra tính hợp lệ của phản hồi từ tay cầm (Xác nhận Mode ID 0x41/0x73/0x79)
  if (ps2_rx_raw[1] == 0x41 || ps2_rx_raw[1] == 0x73 || ps2_rx_raw[1] == 0x79) {
    ps2_connected = 1; // Đánh dấu tay cầm đang kết nối tốt
    ps2_buttons = (ps2_rx_raw[4] << 8) |
                  ps2_rx_raw[3]; // Gộp 2 byte nút bấm thành biến 16-bit

    // Kiểm tra chế độ hoạt động hiện tại (0x73 hoặc 0x79 là Analog Mode)
    if (ps2_rx_raw[1] == 0x73 || ps2_rx_raw[1] == 0x79) {
      ps2_rx = ps2_rx_raw[5]; // Analog phải trục X
      ps2_ry = ps2_rx_raw[6]; // Analog phải trục Y
      ps2_lx = ps2_rx_raw[7]; // Analog trái trục X
      ps2_ly = ps2_rx_raw[8]; // Analog trái trục Y
    } else {                  // Chế độ Digital Mode (ps2_rx_raw[1] == 0x41)
      // Đặt các cần gạt về vị trí trung tâm (128) để nhường quyền cho các nút
      // bấm D-Pad
      ps2_rx = 128;
      ps2_ry = 128;
      ps2_lx = 128;
      ps2_ly = 128;
    }
  } else {
    // Nếu không nhận được mã phản hồi 0x5A -> Tay cầm bị ngắt kết nối hoặc mất
    // nguồn
    ps2_connected = 0;
    smart_turn_active = 0;   // Hủy ngay Smart Turn khi mất kết nối tay cầm
    heading_hold_active = 0; // Hủy ngay Heading Hold khi mất kết nối tay cầm
    // Đưa tất cả cần gạt về trung tâm và nhả toàn bộ nút để xe dừng lại ngay
    // lập tức (Fail-safe)
    ps2_lx = 128;
    ps2_ly = 128;
    ps2_rx = 128;
    ps2_ry = 128;
    ps2_buttons = 0xFFFF;
  }
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1,
                    TIM_CHANNEL_1); // Bật xung PWM ra chân PA8 (ENA) để điều
                                    // khiển tốc độ Động cơ Trái (Motor 1)
  HAL_TIM_PWM_Start(&htim1,
                    TIM_CHANNEL_4); // Bật xung PWM ra chân PA11 (ENB) để điều
                                    // khiển tốc độ Động cơ Phải (Motor 2)
  HAL_TIM_Encoder_Start(&htim2,
                        TIM_CHANNEL_ALL); // Bật bộ đếm Encoder phần cứng trên
                                          // TIM2 (Đọc Encoder Động cơ Trái)
  HAL_TIM_Encoder_Start(&htim4,
                        TIM_CHANNEL_ALL); // Bật bộ đếm Encoder phần cứng trên
                                          // TIM4 (Đọc Encoder Động cơ Phải)

  PS2_Init();           // Khởi tạo tay cầm PS2
  MPU6050_Init(&hi2c2); // Khởi tạo và hiệu chỉnh MPU6050 qua I2C2
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    PS2_Read(); // Đọc dữ liệu từ tay cầm PS2

    uint8_t is_analog_mode = (ps2_rx_raw[1] == 0x73 || ps2_rx_raw[1] == 0x79);

    // Bảo vệ Fail-Safe 2 tầng: Nếu mất kết nối PS2 hoặc MPU6050, lập tức ngắt toàn bộ chế độ tự động
    if (!ps2_connected || !MPU6050.IsConnected) {
      smart_turn_active = 0;
      heading_hold_active = 0;
    }

    // --- Xử lý dữ liệu cần gạt Analog và áp dụng Vùng Chết (Deadzone) ---
    // Mặc định cần gạt ở giữa sẽ đọc ra khoảng 128. Trừ đi 128 để chuyển đổi về
    // gốc tọa độ 0. Dải giá trị lúc này: từ -128 (hết cỡ về một phía) tới +127
    // (hết cỡ về phía ngược lại)
    int ly_val = (int)ps2_ly - 128; // Trục Y Cần Trái (Tiến/Lùi)
    int rx_val = (int)ps2_rx - 128; // Trục X Cần Phải (Rẽ)

    // Áp dụng Vùng chết (Deadzone) chuẩn +/- 15
    if (ly_val >= -15 && ly_val <= 15)
      ly_val = 0;
    if (rx_val >= -15 && rx_val <= 15)
      rx_val = 0;

    int speed = 0;
    int steer = 0;

    // --- PHÂN CHIA 2 CHẾ ĐỘ HOẠT ĐỘNG RIÊNG BIỆT ---
    if (is_analog_mode) {
      // === CHẾ ĐỘ 1: ANALOG MODE (Đèn Đỏ sáng) ===
      // Cần Trái: Tiến/Lùi, Cần Phải: Trái/Phải, KHÔNG dùng các nút D-Pad để
      // lái trực tiếp
      speed = -ly_val;
      steer = rx_val;
    } else {
      // === CHẾ ĐỘ 2: DIGITAL MODE (Tắt Đèn Đỏ) ===
      // KHÔNG dùng các cần Joystick. CHỈ dùng 4 nút D-Pad để lái!
      if (!(ps2_buttons & 0x0010))
        speed = 128; // Nút LÊN (UP)    -> Tiến full công suất
      if (!(ps2_buttons & 0x0040))
        speed = -128; // Nút XUỐNG (DOWN) -> Lùi full công suất
      if (!(ps2_buttons & 0x0080))
        steer = -128; // Nút TRÁI (LEFT)  -> Rẽ Trái full công suất (999 PWM)
      if (!(ps2_buttons & 0x0020))
        steer = 128; // Nút PHẢI (RIGHT) -> Rẽ Phải full công suất (999 PWM)
    }

    // --- CẬP NHẬT GÓC TỪ MPU6050 VỚI CHU KỲ THỜI GIAN ĐỘNG dt (Dùng
    // HAL_GetTick) ---
    static uint32_t last_tick = 0;
    uint32_t current_tick = HAL_GetTick();
    float dt = (float)(current_tick - last_tick) / 1000.0f;
    if (dt <= 0.0001f)
      dt = 0.025f; // Dự phòng khi vừa khởi động hoặc tràn tick
    else if (dt > 0.1f)
      dt = 0.1f; // Giới hạn max 100ms tránh nhảy góc khi bị nghẽn
    last_tick = current_tick;

    if (MPU6050.IsConnected) {
      MPU6050_UpdateAngle(&hi2c2, &MPU6050, dt);
    } else {
      smart_turn_active = 0;   // Hủy ngay Smart Turn khi MPU6050 mất kết nối
      heading_hold_active = 0; // Hủy ngay Heading Hold khi MPU6050 mất kết nối
    }

    // --- TÍNH NĂNG 2: ANTI-FLIP / PITCH GUARD (Bảo vệ chống lật xe) ---
    if (!is_anti_flip && MPU6050.IsConnected && (MPU6050.Pitch > 35.0f || MPU6050.Pitch < -35.0f)) {
      is_anti_flip = 1;
      anti_flip_timer = HAL_GetTick();
      smart_turn_active = 0;   // Hủy ngay Smart Turn khi ngóc đầu khẩn cấp
      heading_hold_active = 0; // Hủy ngay Heading Hold khi ngóc đầu khẩn cấp
    }

    if (is_anti_flip) {
      heading_hold_active = 0; // Nhả cờ giữ thẳng khi đang hãm lùi khẩn cấp

      // Nếu đã lùi tối thiểu 500ms VÀ góc Pitch đã hạ xuống dưới 30 độ -> Thoát
      // trạng thái khẩn cấp
      if ((HAL_GetTick() - anti_flip_timer >= 500) && (MPU6050.Pitch < 30.0f)) {
        is_anti_flip = 0;
      } else {
        // Vẫn trong trạng thái khẩn cấp: Tiếp tục hãm lùi
        speed = -64;
        steer = 0;
      }
    } else {
      // --- TÍNH NĂNG 3: SMART TURNING (Rẽ 90 độ chuẩn xác bằng D-Pad
      // Trái/Phải) ---
      static uint16_t prev_buttons = 0xFFFF;
      uint8_t btn_left_pressed = (!(ps2_buttons & 0x0080)) &&
                                 (prev_buttons & 0x0080); // Nút D-Pad Left
      uint8_t btn_right_pressed = (!(ps2_buttons & 0x0020)) &&
                                  (prev_buttons & 0x0020); // Nút D-Pad Right
      prev_buttons = ps2_buttons;

      if (is_analog_mode && btn_left_pressed && MPU6050.IsConnected) {
        smart_turn_active = 1;
        smart_turn_target = MPU6050.Yaw + smart_turn_angle; // Đặt mục tiêu quay trái 90 độ thực tế
      } else if (is_analog_mode && btn_right_pressed && MPU6050.IsConnected) {
        smart_turn_active = 1;
        smart_turn_target = MPU6050.Yaw - smart_turn_angle; // Đặt mục tiêu quay phải 90 độ thực tế
      }

      // Ưu tiên lái tay: Chỉ hủy Smart Turn khi người dùng gạt Joystick rõ ràng (vượt deadzone rộng +/-25)
      int ly_drift = (int)ps2_ly - 128;
      int rx_drift = (int)ps2_rx - 128;
      if (ly_drift > 25 || ly_drift < -25 || rx_drift > 25 || rx_drift < -25) {
        smart_turn_active = 0;
      }

      if (smart_turn_active && MPU6050.IsConnected) {
        heading_hold_active = 0; // Nhả cờ giữ thẳng khi đang rẽ 90 độ tự động
        float turn_error = smart_turn_target - MPU6050.Yaw;

        // Chuẩn hóa góc sai số về [-180, 180]
        while (turn_error > 180.0f)
          turn_error -= 360.0f;
        while (turn_error < -180.0f)
          turn_error += 360.0f;

        if (fabs(turn_error) > 3.0f) {
          speed = 0;
          // Tính lực quay có ngưỡng tối thiểu (min 75) để thắng ma sát nghỉ của xích xe tăng
          float error_mag = fabs(turn_error);
          int turn_power = (int)(error_mag * 3.0f);
          if (turn_power < 75)
            turn_power = 75;   // Thắng ma sát nghỉ xích xe (tránh động cơ bị kêu mà không quay)
          if (turn_power > 110)
            turn_power = 110;  // Giới hạn công suất tối đa (~86% PWM)

          if (turn_error > 0.0f) {
            steer = -turn_power; // Quay Trái
          } else {
            steer = turn_power;  // Quay Phải
          }
        } else {
          smart_turn_active = 0; // Đã xoay xong góc 90 độ
          steer = 0;
        }
      }
      // --- TÍNH NĂNG 1: HEADING HOLD (Tự động giữ thẳng hướng khi đi tiến/lùi)
      // ---
      else if (speed != 0 && steer == 0 && MPU6050.IsConnected) {
        static float prev_heading_error = 0.0f;

        if (!heading_hold_active) {
          heading_hold_active = 1;
          target_heading = MPU6050.Yaw; // Ghi nhận ngay góc hướng chuẩn tại
                                        // thời điểm bắt đầu đi thẳng
          prev_heading_error = 0.0f; // Reset sai số vi phân tránh giật đột ngột
        }

        float heading_error = target_heading - MPU6050.Yaw;

        // 1. Chuẩn hóa dải góc về [-180, 180] độ tránh bị tràn góc khi xoay
        // tròn
        while (heading_error > 180.0f)
          heading_error -= 360.0f;
        while (heading_error < -180.0f)
          heading_error += 360.0f;

        // 2. Vùng chết Deadband +/- 1.0 độ: Nếu xe lệch dưới 1 độ thì coi như
        // đã đi thẳng chuẩn, không vi chỉnh
        if (fabs(heading_error) < 1.0f) {
          heading_error = 0.0f;
        }

        // Tính chênh lệch sai số và chuẩn hóa tránh vọt đỉnh spike khi tràn góc
        // qua [-180, 180]
        float error_delta = heading_error - prev_heading_error;
        while (error_delta > 180.0f)
          error_delta -= 360.0f;
        while (error_delta < -180.0f)
          error_delta += 360.0f;

        float heading_derivative = 0.0f;
        if (dt > 0.0001f) {
          heading_derivative = error_delta / dt;
        }
        prev_heading_error = heading_error;

        float correction =
            (Kp_heading * heading_error) + (Kd_heading * heading_derivative);

        // 3. Khống chế lực vi chỉnh tối đa MAX = 40 (trên dải 999 PWM) -> Đảm
        // bảo chỉ nhích nhẹ mượt mà, KHÔNG BAO GIỜ BỊ QUÁ LỐ
        if (correction > 40.0f)
          correction = 40.0f;
        if (correction < -40.0f)
          correction = -40.0f;

        // Đảo dấu (-correction) cho cả Tiến lẫn Lùi để Phản Hồi Âm luôn dập tắt
        // sai số, không bị khóa bánh khi lùi
        steer = (int)(-correction);
      } else {
        smart_turn_active = 0;   // Nhả cờ Smart Turn khi thoát trạng thái tự động
        heading_hold_active = 0; // Nhả cờ Heading Hold khi chủ động bẻ lái hoặc dừng
      }
    }

    // --- Thuật toán pha trộn lái xe tăng (Arcade Drive / Differential Drive)
    // --- Động cơ bên trái: Tiến khi tiến, rẽ phải thì quay tiến nhanh hơn
    int left_motor_speed = speed + steer;
    // Động cơ bên phải: Tiến khi tiến, rẽ phải thì quay lùi hoặc tiến chậm hơn
    int right_motor_speed = speed - steer;

    // Quy đổi tốc độ từ dải điều khiển cần gạt [-128, 127] lên dải duty của PWM
    // [-999, 999]
    int left_pwm = left_motor_speed * 999 / 128;
    int right_pwm = right_motor_speed * 999 / 128;

    // Giới hạn giá trị PWM đầu ra không vượt quá chu kỳ tối đa của timer (ARR =
    // 999)
    if (left_pwm > 999)
      left_pwm = 999;
    if (left_pwm < -999)
      left_pwm = -999;
    if (right_pwm > 999)
      right_pwm = 999;
    if (right_pwm < -999)
      right_pwm = -999;

    // --- Thực thi điều khiển Động cơ 1 (Bên Trái) ---
    if (left_pwm > 0) {                                    // Động cơ quay THUẬN
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); // IN1 = 1
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); // IN2 = 0
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,
                            left_pwm); // Cấp xung tốc độ tương ứng
    } else if (left_pwm < 0) {         // Động cơ quay NGHỊCH (Lùi)
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // IN1 = 0
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);   // IN2 = 1
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,
                            -left_pwm); // Truyền trị tuyệt đối của xung
    } else {                            // DỪNG động cơ (thả trôi)
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // IN1 = 0
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); // IN2 = 0
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);       // Duty = 0%
    }

    // --- Thực thi điều khiển Động cơ 2 (Bên Phải) ---
    if (right_pwm > 0) {                                   // Động cơ quay THUẬN
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); // IN3 = 1
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); // IN4 = 0
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,
                            right_pwm); // Cấp xung tốc độ tương ứng
    } else if (right_pwm < 0) {         // Động cơ quay NGHỊCH (Lùi)
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); // IN3 = 0
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);   // IN4 = 1
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,
                            -right_pwm); // Truyền trị tuyệt đối của xung
    } else {                             // DỪNG động cơ (thả trôi)
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); // IN3 = 0
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); // IN4 = 0
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);       // Duty = 0%
    }

    // Đọc phản hồi số vòng quay (xung Encoder) phục vụ cho thuật toán điều
    // khiển PID nếu cần sau này
    encoderCount1 = -(int16_t)__HAL_TIM_GET_COUNTER(
        &htim2); // Đảo dấu đồng bộ với total_encoder1
    encoderCount2 = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
    Update_Encoder_Position(); // Cập nhật tổng vị trí tích lũy encoder

    HAL_Delay(25); // Tạo chu kỳ quét dữ liệu đều đặn mỗi ~25 mili giây (tần số
                   // quét ~40Hz)
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
