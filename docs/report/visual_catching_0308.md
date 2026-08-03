# Báo cáo Cập nhật hệ thống Bắt bóng Động (Visual Dynamic Catching)
**Ngày báo cáo:** 03/08/2026

## Tổng quan
Quá trình gỡ lỗi và cải tiến hệ thống mô phỏng đón bóng trong môi trường Isaac Sim 5.1 / IsaacLab. Robot UR5 cùng bàn tay Dexterous Hand (DH) đã được sửa lỗi định hướng hệ trục tọa độ, cải thiện logic kích hoạt các ngón tay (trigger closing) và nâng cấp tốc độ cũng như biên độ nắm của bàn tay để thực hiện cú bắt bóng mượt mà, chính xác.

## Chi tiết công việc theo mốc thời gian (Timestamp)

### 02/08/2026 23:34 
- **Cải tiến tốc độ và biên độ tay nắm:**
  - File: `src/visual_catching/smooth_finger_interpolator.py`
  - Tăng tốc độ bóp ngón tay (duration) từ `0.4s` xuống `0.15s` nhằm phản ứng kịp với vận tốc rơi lớn của quả bóng.
  - Tăng góc gập của các ngón tay (thumb, index, middle, ring, pinky) lên mức ~1.5 - 1.6 rad (gần 90 độ), đảm bảo quả bóng được khóa chặt trong lòng bàn tay.
- **Kích hoạt ngón tay dự đoán thông minh (Proactive Grasp Trigger):**
  - File: `src/visual_catching/visual_dynamic_catching.py`
  - Thay thế điều kiện kích hoạt ngón tay từ vị trí tĩnh (khoảng cách `< 0.18m`) sang dùng thời gian dự đoán va chạm EKF `t_catch < 0.25s`. Bàn tay sẽ khép dần trước khi quả bóng chạm vào lòng bàn tay.

### 02/08/2026 23:41 - 23:57
- **Phân tích nguyên nhân báo cáo sai (False Positive) từ mô phỏng:**
  - Chỉ ra lỗi đánh giá `SUCCESS` của hệ thống: Quả bóng không rơi xuống đất mà đọng lại trên lưng bàn tay hoặc cẳng tay của robot (chiều cao `Z > 0.2m`) khiến code ghi nhận thành công dù visualize thất bại.
- **Trích xuất và kiểm tra động học (Kinematic Dump) của hệ trục tọa độ DH:**
  - Xây dựng và chạy kịch bản trích xuất trực tiếp tọa độ (World Coordinates) của `wrist_3_link`, lòng bàn tay (`DH_base_link`) và các đầu ngón tay (`index_Link4`, `thumb_Link3`) trong môi trường USD của Isaac Sim.
  - Bóc trần sai lầm của thiết lập trục tọa độ cũ: 
    - **Trục Z (cục bộ):** Hướng dọc theo chiều dài của các ngón tay.
    - **Trục Y (cục bộ):** Trỏ thẳng ra khỏi mặt phẳng lòng bàn tay (Palm Normal).
    - **Trục X (cục bộ):** Trỏ sang phía ngón cái.

### 02/08/2026 23:58
- **Cấu trúc lại thuật toán Định hướng trong IK Solver (Orientation Fix):**
  - File: `src/sim/curobo_ik_solver.py` (Hàm `compute_palm_orientation()`).
  - Xóa bỏ ma trận đảo lộn `R_flip` gây ra hiện tượng robot úp ngược bàn tay 180 độ.
  - Định hướng lại ma trận đón bóng mục tiêu: 
    - Trục Y (Lòng bàn tay) hướng thẳng đứng lên trời `[0, 0, 1]` để tạo mặt phẳng ngang đón bóng.
    - Trục Z (Các ngón tay) chĩa song song với mặt đất về phía trước `[1, 0, 0]`.
  - Kết quả mô phỏng ngầm xác nhận khoảng cách từ tâm quả bóng đến tâm tay (Dist) duy trì ở mức tối ưu 0.017m với tư thế tay hoàn toàn ngửa lên trời.

### 03/08/2026 07:28
- Soạn thảo báo cáo tài liệu `visual_catching_0308.md`.
- Thực hiện đóng gói (`git commit`) và đẩy thay đổi lên server (`git push`).
