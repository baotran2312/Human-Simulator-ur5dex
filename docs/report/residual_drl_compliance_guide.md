# Hướng Dẫn Từng Bước Triển Khai Hệ Thống Trở Kháng Dư (Residual DRL Compliance Guide)

Tài liệu này vạch ra lộ trình chi tiết từng bước để tái cấu trúc hệ thống học tăng cường (RL) từ dạng học từ con số 0 (from-scratch) sang hệ thống **Residual DRL Compliance (Trở kháng dư)**, giúp giải quyết triệt để lỗi kẹt reward `-10` và tối ưu hóa thời gian hội tụ của PPO xuống dưới 2 tiếng.

---

## 1. Bản Chất Giải Thuật Đề Xuất
Thay vì bắt mạng neural học đồng thời cả động học (gập ngón tay đúng lúc) và động lực học (dập tắt xung lực va chạm), chúng ta phân rã bài toán:
1.  **Macro-Micro Heuristic (Đảm bảo bắt bóng 100%):** Sử dụng cảm biến khoảng cách hình học trong mô phỏng để tự động gửi lệnh gập ngón tay (Heuristic Grasp) khi bóng đến gần lòng bàn tay.
2.  **DRL Compliance Adapter (Học cách bắt mềm):** Mạng neural chỉ học cách tinh chỉnh ma trận stiffness động cơ cáp kéo ($\Delta \mathbf{K}_a$) để giảm thiểu tối đa lực va chạm tiếp xúc ($\mathbf{f}_{\text{ext}}$) và triệt tiêu độ nảy của bóng.

```
                  +--------------------------------+
                  |  3D Ball Distance Sensor (Sim) |
                  +---------------+----------------+
                                  |
                   [Khoảng cách < 10cm từ lòng bàn tay?]
                                  |
                       +----------+----------+
                       |                     |
                    [ĐÚNG]                [SAI]
                       |                     |
        +--------------v-------------+       +-----v-----+
        | Kích hoạt khép ngón tay    |             | Mở tay    |
        |  (Heuristic Position)      |             | chờ bóng  |
        +--------------+-------------+             +-----------+
                       |
        +--------------v-------------+
        | DRL PPO Agent điều biến   |
        |  stiffness động cơ Ka(t)  |
        +----------------------------+
```

---

## 2. Các Bước Triển Khai Chi Tiết

### Bước 1: Kiểm tra vật lý va chạm của bóng và tay máy
*   **Mục tiêu:** Đảm bảo bóng không bị xuyên qua lòng bàn tay (Ghost collision) và không nảy văng ra ngoài khi chạm tay.
*   **Hành động:** 
    *   Mở tệp [`src/rl/dh_hand_catch_env.py`](file:///D:/NCKH/Humanoid/Human-Simulator-ur5dex/src/rl/dh_hand_catch_env.py) và xác nhận quả bóng đã được gán thuộc tính va chạm (`collision_props`).
    *   Độ nảy (`restitution`) trong `physics_material` của bóng phải được đặt bằng `0.0` (Zero bounce) và độ ma sát (`friction`) đặt bằng `1.0`.

### Bước 2: Tích hợp Bộ kích hoạt khép ngón (Heuristic Grasp Trigger)
*   **Mục tiêu:** Tạo quỹ đạo chuyển động khép ngón tay tự động dựa trên khoảng cách.
*   **Hành động:** Chỉnh sửa hàm `_apply_action(self)` trong `dh_hand_catch_env.py`:
    1.  Tính khoảng cách Euclid giữa gốc lòng bàn tay (`self.palm_link_idx`) và quả bóng.
    2.  Nếu khoảng cách $< 0.10\,\text{m}$, gán vị trí mục tiêu cho toàn bộ 19 khớp ngón tay bằng giá trị gập tối đa ($0.75\,\text{rad}$ hoặc $1.309\,\text{rad}$ tùy theo giới hạn khớp của URDF).
    3.  Ngược lại, gán vị trí mục tiêu bằng $0.0$ (giữ tay mở rộng).

### Bước 3: Ánh xạ không gian hành động DRL thành Stiffness ($\Delta \mathbf{K}_a$)
*   **Mục tiêu:** Chuyển đổi Action Space của DRL từ điều khiển vị trí sang điều khiển trở kháng thực sự.
*   **Hành động:**
    1.  Đặt cấu hình Action Space của Agent là 5 chiều (tương ứng với 5 động cơ cáp kéo của bàn tay DH).
    2.  Nhân đầu ra của mạng neural (trong khoảng $[-1.0, 1.0]$) với hệ số tỷ lệ $\pm 20.0\,\text{N.m/rad}$ để tạo ra $\Delta \mathbf{K}_a$.
    3.  Cập nhật ma trận stiffness của bàn tay: $\mathbf{K}_a = \mathbf{K}_{a,0} + \text{diag}(\Delta \mathbf{K}_a)$.
    4.  Sử dụng API của Isaac Lab/Isaac Sim để cập nhật trực tiếp độ cứng khớp: `self.robot.set_joint_stiffness(stiffness_vector)`.

### Bước 4: Điều chỉnh Quan Sát (Observation) và Phần Thưởng (Reward)
*   **Mục tiêu:** Đồng bộ hóa code thực tế khớp với toán học của Manuscript và loại bỏ nhiễu tín hiệu.
*   **Hành động:**
    *   *Observations:* Đưa vị trí bóng, vận tốc bóng, vị trí khớp ngón tay và lực tiếp xúc đo được vào vector quan sát.
    *   *Reward:* Thay thế reward cũ bằng hàm tối ưu lực va chạm: 
        $$r_t = - w_2 \|\mathbf{f}_{\text{ext}}(t)\|^2 - w_3 \|\boldsymbol{\tau}(t)\|^2$$
        Trong đó $\mathbf{f}_{\text{ext}}$ đọc trực tiếp từ cảm biến lực tiếp xúc hoặc ước lượng từ dòng điện động cơ.

### Bước 5: Chạy thử nghiệm không học (Verification Playback)
*   **Mục tiêu:** Kiểm tra độ vững chắc của bộ kích hoạt Heuristic trước khi train.
*   **Hành động:** 
    *   Khởi chạy kịch bản mô phỏng ở chế độ thủ công (chưa bật thuật toán PPO).
    *   Quan sát trực quan: Khi quả bóng rơi xuống, bàn tay có tự động gập lại ôm trọn quả bóng và giữ bóng nằm im trong lòng bàn tay hay không.
    *   **Không tiến hành train nếu bước này chưa hoạt động chuẩn xác.**

### Bước 6: Kích hoạt PPO và Áp dụng Tiêu chí Kiểm tra nhanh 15 phút (Quick-Check)
*   **Mục tiêu:** Huấn luyện mạng và ngăn chặn việc lãng phí thời gian nếu giải thuật bị kẹt.
*   **Hành động:**
    1.  Chạy tệp [`train_ppo.py`](file:///D:/NCKH/Humanoid/Human-Simulator-ur5dex/src/rl/train_ppo.py).
    2.  Theo dõi đồ thị học (Reward curve) trên TensorBoard hoặc log in ra màn hình trong vòng **15 phút đầu (khoảng 50 Epochs)**.
    3.  **Tiêu chí đánh giá:**
        *   *Thành công:* Giá trị reward trung bình tăng dần (lực va chạm tiếp xúc trung bình giảm đi) và độ lệch chuẩn của reward thu hẹp lại. $\rightarrow$ Tiếp tục để hệ thống chạy đến khi hội tụ (thường dưới 1.5 tiếng).
        *   *Thất bại:* Đồ thị reward đi ngang hoàn toàn hoặc sụt giảm không phanh. $\rightarrow$ Tắt tiến trình ngay lập tức để rà soát lỗi logic, không để treo máy qua đêm gây lãng phí tài nguyên.
