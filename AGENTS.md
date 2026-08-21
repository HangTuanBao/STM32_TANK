
# Karpathy Coding Rules & Safety Standards for Tank Project (Embedded Robotics)

## 📌 PROJECT CONTEXT

- **Domain:** Embedded Systems, Firmware & Robotics (Tank Control).
- **Core Focus:** Real-time motor control, sensor feedback, communication protocols (UART/SPI/I2C/Bluetooth/RF), and autonomous navigation logic.
- **Critical Requirement:** Maintain strict hardware safety, clean memory management, and deterministic logic.

---

## 🎯 4 CORE KARPATHY PRINCIPLES

### 1. Think Before Coding (Suy nghĩ trước khi viết code)

- **State Assumptions Explicitly:** Trước khi viết bất kỳ đoạn code nào, hãy nêu rõ các giả định về phần cứng (Dòng vi điều khiển, Driver động cơ, sơ đồ chân Pinout, điện áp).
- **Ask Before Guessing:** Nếu câu lệnh của người dùng còn mơ hồ (chưa rõ dòng chip, loại driver L298N/BTS7960, hay chuẩn giao tiếp), hãy **HỎI LẠI** để làm rõ thay vì tự đoán mò.
- **Surface Tradeoffs:** Nêu rõ các ưu/nhược điểm (ví dụ: dùng ngắt Timer vs Polling, dùng thư viện HAL vs đọc ghi thanh ghi trực tiếp Register).
- **Propose Simpler Alternatives:** Nếu nhận thấy cách làm của người dùng quá phức tạp, hãy chủ động đề xuất giải pháp phần cứng/phần mềm tối ưu và đơn giản hơn.

### 2. Simplicity First (Đơn giản là trên hết)

- **Minimum Code Needed:** Chỉ viết số lượng dòng code tối thiểu để giải quyết đúng vấn đề được yêu cầu. Không suy đoán các tính năng tương lai.
- **No Overengineering:** - Không tự ý tạo các lớp trừu tượng (abstraction), struct/class phức tạp cho các tác vụ chỉ dùng 1 lần.
  - Không thêm biến cấu hình hoặc tính năng linh hoạt khi không được yêu cầu.
  - Không viết code xử lý lỗi cho các kịch bản phần cứng không thể xảy ra.
- **Concise Test:** Nếu 200 dòng code có thể tối ưu còn 50 dòng mà vẫn đảm bảo an toàn, hãy viết lại. Code đơn giản là code ít bug nhất.

### 3. Surgical Changes (Chỉnh sửa chuẩn xác / Cục bộ)

- **Touch Only What You Must:** Chỉ sửa đúng hàm, đúng file liên quan trực tiếp đến tác vụ được giao.
- **Respect Existing Code:** Không tự ý "cải tiến", reformat, hoặc xóa comment/code của các module xung quanh không liên quan.
- **Clean Up Own Mess Only:** - Tự dọn dẹp các biến, thư viện (`#include`), hoặc hàm thừa do chính thay đổi mới của AI tạo ra.
  - Không xóa các đoạn code thừa cũ của dự án trừ khi được yêu cầu trực tiếp.
- **Match Project Style:** Tuân thủ chuẩn đặt tên biến/hàm và phong cách định dạng code hiện có của dự án.

### 4. Goal-Driven Execution (Làm việc theo mục tiêu & Vòng lặp kiểm thử)

- **Define Clear Success Criteria:** Biến các câu lệnh chung chung thành tiêu chí nghiệm thu có thể kiểm chứng được:
  - Thay vì *"Viết code chạy động cơ"* $\rightarrow$ *"Viết hàm PWM điều khiển 2 động cơ, kiểm tra giới hạn xung từ 0-100%, đảm bảo đảo chiều không bị chập mạch"*.
  - Thay vì *"Sửa lỗi cảm biến siêu âm"* $\rightarrow$ *"Tạo hàm đọc cảm biến HC-SR04 không gây nghẽn (non-blocking), trả về khoảng cách cm và lọc nhiễu"*.
- **Plan -> Execute -> Verify Loop:** Với các tác vụ phức tạp, lập kế hoạch ngắn gồm 3 bước:
  1. `[Bước 1]` -> Kiểm chứng: `[Biên dịch không warning]`
  2. `[Bước 2]` -> Kiểm chứng: `[Chạy mô phỏng/Test logic]`
  3. `[Bước 3]` -> Kiểm chứng: `[Xác nhận tín hiệu PWM/Đầu ra phần cứng an toàn]`

---

## 🛡️ EMBEDDED & HARDWARE SAFETY RULES (Bắt buộc cho dự án Xe Tăng)

### 1. Hardware Fail-Safe (An toàn phần cứng tuyệt đối)

- **Emergency Stop / Disconnect Guard:** Nếu mất tín hiệu điều khiển (mất Bluetooth/RF/UART), hệ thống phải **TỰ ĐỘNG DỪNG TẤT CẢ ĐỘNG CƠ** (`PWM = 0`) ngay lập tức.
- **PWM Bounding Check:** Luôn kiểm tra giới hạn đầu vào của PWM (ví dụ: ép giá trị trong khoảng `0 - 255` hoặc `0 - 100%`). Không để xảy ra tràn số gây quay max công suất đột ngột.
- **Direction Switching Delay:** Khi đổi chiều quay động cơ (Tiến $\leftrightarrow$ Lùi), bắt buộc phải có một khoảng thời gian trễ ngắn (Dead-time delay nhỏ hoặc đưa PWM về 0 trước khi đảo chân DIR) để tránh dòng điện ngược làm cháy Driver động cơ.

### 2. Memory & Real-Time Constraints (Quản lý bộ nhớ & Thời gian thực)

- **Zero Dynamic Memory Allocation:** Tuyệt đối **KHÔNG dùng `malloc()`, `free()`, `new`, `delete`** trong quá trình xe đang vận hành (Runtime) để tránh phân mảnh RAM và tràn bộ nhớ Heap. Hãy dùng mảng/bộ đệm tĩnh (Static/Global Allocation).
- **Non-blocking Execution:** Tuyệt đối **KHÔNG dùng các hàm delay cản trở hệ thống** (như `delay()` trong Arduino) bên trong vòng lặp chính (`main loop`). Phải dùng Timer, cờ ngắt (Interrupt flags), hoặc `millis()` / `micros()` để quản lý thời gian đa nhiệm.
- **Interrupt Safety (ISR):**
  - Các biến dùng chung giữa Hàm ngắt (ISR) và Vòng lặp chính bắt buộc phải khai báo từ khóa `volatile`.
  - Hàm ngắt ISR phải ngắn gọn tối đa: Chỉ bật/tắt cờ (flag) hoặc lưu dữ liệu vào Ring Buffer, không xử lý tính toán toán học phức tạp hay in chuỗi (No `printf` in ISR).

### 3. Code Style & Types (Chuẩn viết code C/C++ Nhúng)

- Sử dụng kiểu dữ liệu có độ rộng cố định từ thư viện `<stdint.h>`: Dùng `uint8_t`, `int16_t`, `uint32_t` thay vì `int`, `long` chung chung.
- Sử dụng **Finite State Machine (FSM)** với kiểu liệt kê `enum` để quản lý các trạng thái hoạt động của xe (ví dụ: `STATE_STOP`, `STATE_MANUAL_CONTROL`, `STATE_AVOID_OBSTACLE`).
