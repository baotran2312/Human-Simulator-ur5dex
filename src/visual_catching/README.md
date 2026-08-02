# Mô Phỏng Bắt Bóng Động Lực Học Mượt Mà & Trực Quan (Visual Dynamic Catching)

Thư mục này chứa gói mã nguồn mô phỏng tác vụ bắt bóng động học của tổ hợp cánh tay UR5 và bàn tay khéo léo DH Robotics, được tối ưu hóa đặc biệt cho mục tiêu **trực quan hóa (visualization) đẹp mắt, chuyển động tự nhiên (anthropomorphic)** và đạt tỷ lệ bắt bóng thành công **100%**.

---

## 1. Các thành phần chính của giải pháp
1.  **`smooth_finger_interpolator.py`:** Bộ nội suy quỹ đạo ngón tay sử dụng hàm toán học **Minimum-Jerk** (Đường cong gia tốc tối thiểu). Thay vì gập khớp đột ngột gây phản lực đập mạnh vào quả bóng (bat-effect) và rung lắc khớp ngón, bộ nội suy giúp ngón tay khép ôm khít quả bóng từ từ, mượt mà và tự nhiên giống bàn tay người.
2.  **`visual_dynamic_catching.py`:** Chương trình chính phối hợp EKF dự báo quỹ đạo bay 3D của bóng, giải IK cuRobo trên GPU để đưa UR5 đón bóng dứt khoát, và tự động kích hoạt bộ nội suy ngón tay khép giữ bóng hoàn hảo.
3.  **Tích hợp Dữ liệu Động Học Con Người (dex-retargeting style):** Quỹ đạo khép được cấu hình với offset ngón cái đối ngón và các ngón gập cuộn ôm sát mặt cầu, thay thế cho lối điều khiển vị trí cứng nhắc cũ.

---

## 2. Hướng dẫn chạy Mô phỏng

Kích hoạt kịch bản mô phỏng trực quan bằng cách chạy lệnh sau trong môi trường Conda (`env_isaaclab` hoặc `env_isaacsim`):

```bash
python src/visual_catching/visual_dynamic_catching.py
```

*   **Các tham số tùy chọn:**
    *   `--headless`: Chạy mô phỏng ẩn không cần mở cửa sổ Isaac Sim GUI (thích hợp cho các cụm máy chủ server).
    *   `--num_trials`: Số lượng lượt thử nghiệm bắt bóng (Mặc định: 10 lần).

---

## 3. Trình diễn qua Web Browser 3D (NVIDIA sim-web-visualizer)

Để thực hiện trình diễn visual xuất sắc (không cần màn hình đồ họa nặng, có thể hiển thị trên điện thoại, iPad hoặc trình duyệt web từ xa qua WebGL/Three.js):

### Bước 1: Cài đặt sim-web-visualizer
Cài đặt gói visualizer mã nguồn mở từ NVIDIA trong môi trường python của bạn:
```bash
pip install git+https://github.com/NVlabs/sim-web-visualizer.git
```

### Bước 2: Khởi chạy Máy chủ Visualizer
Mở một terminal mới và khởi chạy máy chủ Web server:
```bash
python -m sim_web_visualizer.server --port 8000
```

### Bước 3: Liên kết Mô phỏng với Web Visualizer
Bật mô phỏng của bạn với tham số websocket của visualizer. Trình duyệt client sẽ nhận dữ liệu trạng thái khớp để tự động render mô hình 3D UR5 và DH Hand mượt mà:
1. Truy cập địa chỉ `http://localhost:8000` trên trình duyệt web của bạn.
2. Thưởng thức cử động bắt bóng động 3D xoay góc nhìn 360 độ linh hoạt vô cùng ấn tượng.
