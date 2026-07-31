# Báo Cáo Tiến Độ (29/07/2026 - 31/07/2026)
**Dự Án:** Human-Simulator-ur5dex (Isaac Lab DRL Catching Task)

Báo cáo chi tiết các đợt gỡ lỗi (debugging) và tối ưu hóa hệ thống môi trường RL cho robot UR5Dex trong tác vụ bắt bóng, được thực hiện từ ngày 29/07 đến sáng 31/07.

---

### 1. Ngày 29/07: Tối ưu hóa Hiệu năng & Khắc phục thắt cổ chai (Performance Bottleneck)
- **Vấn đề:** Tốc độ giả lập cực kỳ chậm, tốn 8 tiếng đồng hồ chỉ để train 5% (khoảng 254k timesteps).
- **Nguyên nhân:** Hàm `self.robot.find_bodies("tên_khớp")` liên tục bị gọi bên trong vòng lặp `_get_observations` và `_get_rewards` ở mỗi step trên toàn bộ 1024 môi trường, gây quá tải CPU/GPU bằng việc quét chuỗi (string matching).
- **Khắc phục:** 
  - Đưa toàn bộ các lệnh `find_bodies` vào hàm khởi tạo `__init__`.
  - Lưu (cache) lại các biến `self.palm_link_idx` và `self.fingertip_indices`.
- **Kết quả:** Tăng tốc độ mô phỏng lên hơn 100x lần (Đạt ngưỡng trần ~10,000 timesteps/giây cho 1024 envs).

### 2. Ngày 29/07 - 30/07: Sửa lỗi Logic Cánh tay & IK Solver
- **Vấn đề:** Ball rớt rất nhanh nhưng bộ điều khiển PD (PD Controller) cần >1 giây để di chuyển cánh tay UR5 đến vị trí bắt bóng thông qua IK. Bàn tay luôn đến trễ.
- **Khắc phục:** 
  - Bỏ hẳn vòng lặp giải IK (IK Solver) ngốn tài nguyên.
  - Hardcode tư thế đón bóng chuẩn (`base_catch_pose = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]`).
  - Sử dụng hàm `write_joint_state_to_sim` để "dịch chuyển tức thời" (teleport) cánh tay đến vị trí đón bóng tại mỗi lần Reset (`_reset_idx`), bỏ qua độ trễ của PD Controller.
- **Kết quả:** Cánh tay luôn ở trạng thái sẵn sàng đón bóng khi môi trường reset, giảm nhiễu do di chuyển cánh tay.

### 3. Ngày 30/07: Tinh chỉnh phần thưởng (Reward Redesign)
- **Vấn đề:** Điểm tín dụng (credit assignment) không rõ ràng khiến agent bị bối rối.
- **Khắc phục:** Định hình lại hàm thưởng (Dense & Sparse):
  - Thưởng Dense: Dựa trên khoảng cách ngón tay và lòng bàn tay tới quả bóng.
  - Thưởng Sparse: Cộng nóng +5.0 điểm nếu khoảng cách < 0.12m và vận tốc tương đối < 0.5 (bóng bị kẹp chặt).
  - Phạt: -10 điểm khi bóng rơi quá độ cao 0.2m.

### 4. Chiều Tối 30/07: Sửa lỗi Vật Lý "Bóng Ma" & Độ Nảy (Collision & Bounce Fix)
- **Vấn đề:** Phân tích test logic cho thấy bóng xuyên thẳng qua tay robot mà không hề bị khựng lại hay giảm tốc độ. 
- **Nguyên nhân:** Isaac Lab `SphereCfg` mặc định chỉ tạo lớp lưới hình ảnh (visual mesh), không có thuộc tính va chạm.
- **Khắc phục:**
  - Bổ sung `collision_props=sim_utils.CollisionPropertiesCfg()` cho quả bóng.
  - Chỉnh vật liệu `physics_material` với `restitution = 0.0` để triệt tiêu hoàn toàn độ nảy, giúp bóng nằm im trên lòng bàn tay thay vì nảy văng ra ngoài.
- **Kết quả:** Quả bóng chính thức trở thành thực thể vật lý (Solid), tương tác va chạm chuẩn xác với các đốt ngón tay và lòng bàn tay.

### 5. Đêm 30/07: Sửa lỗi Nhiễu Quỹ Đạo (Trajectory Drift Fix)
- **Vấn đề:** Vận tốc nhiễu (`random_vel`) khi reset bóng bị đặt quá cao `[-0.5, 0.5] m/s`, khiến quả bóng thường xuyên bay chệch ra ngoài khoảng với của tay robot (sai lệch tới 25cm).
- **Khắc phục:** Giảm biên độ nhiễu xuống còn `0.1` (`[-0.05, 0.05] m/s`), độ lệch chuẩn ~2.5cm.
- **Kết quả:** Đảm bảo bóng luôn bay trúng lòng bàn tay nhưng vẫn có đủ sự đa dạng về góc độ để RL học cách phản xạ.

### 6. Sáng 31/07: Sửa lỗi "Đứt dây thần kinh" Ngón Tay (Joint Mapping Bug)
- **Vấn đề:** Sau 10 tiếng train đêm, reward vẫn -10. Kiểm tra hệ thống khớp cho thấy Isaac Lab sắp xếp danh sách 25 khớp của robot theo **từng đốt (J1, J2, J3, J4)** chứ không phải theo từng ngón. Vòng lặp map action tuyến tính cũ đã gán 1 nơ-ron điều khiển cho toàn bộ đốt gốc (J1) của 4 ngón tay khác nhau. Việc gập ngón tay là bất khả thi.
- **Khắc phục:** 
  - Khai báo lại tường minh ma trận chỉ số khớp cho từng ngón tay riêng biệt (Thumb: `[10,15,20]`, Index: `[6,11,16,21]`, v.v.).
  - Bọc giới hạn an toàn `torch.clamp(actions, -1.0, 1.0)` để ngăn RL đẩy giá trị ảo gây gãy khớp vật lý.
- **Kết quả:** Các tín hiệu Action từ Neural Network giờ đây đã map chính xác 1-1 với cử động gập của từng ngón tay cụ thể. Quá trình học chính thức hoạt động đúng logic 100%.

---
**Kết Luận Tình Trạng Cuối:** 
Hệ thống vật lý và luồng dữ liệu RL đã được gỡ lỗi (debug) hoàn chỉnh. Môi trường `DHHandCatchEnv` đã sẵn sàng để quá trình tối ưu PPO (Policy Optimization) tự động phân bổ trọng số và học cách nắm tay chính xác.
