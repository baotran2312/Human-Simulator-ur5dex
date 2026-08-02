# Phân tích AnyTeleop & Kế hoạch so sánh đối chứng / tích hợp sang UR5 + tay DH (Isaac Sim & Real)

> Nguồn: `docs/literature_review/AnyTeleop A General Vision-Based Dexterous Robot Arm-Hand Teleoperation System/AnyTeleop A General Vision-Based Dexterous Robot Arm-Hand Teleoperation System.md`
> (Qin et al. 2023, RSS 2023, UCSD & NVIDIA). Project page: <https://yzqin.github.io/anyteleop/>
>
> **Trạng thái mã nguồn (kiểm tra 2026-08-02):** Hai thành phần lõi tạo nên AnyTeleop được phát triển độc lập và nguồn mở tại:
> - **dex-retargeting (Thư viện ánh xạ ngón tay):** <https://github.com/dexsuite/dex-retargeting>
> - **sim-web-visualizer (Bộ dựng hình đám mây):** <https://github.com/NVlabs/sim-web-visualizer>
> Hệ thống này tương thích tốt với ROS 2, SAPIEN, Isaac Gym và hỗ trợ đa dạng cấu hình camera cùng nhiều loại tay robot khéo léo (Allegro, Shadow, Leap).
>
> → Ta sẽ sử dụng AnyTeleop làm **hệ quy chiếu Baseline học thuật** để đánh giá hiệu năng của bộ điều khiển CLIK kháng trễ và hệ thống DRL trở kháng mềm trong môi trường có độ trễ truyền thông lớn ($h_m = 32\,\text{ms}$).

---

## 1. AnyTeleop làm gì — bức tranh tổng thể

Bài toán: Cung cấp một hệ thống điều khiển từ xa (teleoperation) thời gian thực, hợp nhất và độc lập với phần cứng cho tổ hợp cánh tay - bàn tay khéo léo (Arm-Hand) bằng camera thường (RGB/RGB-D) không cần găng tay haptic hay cảm biến đeo người.

```
  Perception (Camera RGB/D) ─────► Hand Pose Detection (MediaPipe/MMPose, 25Hz)
                                              │
                                              ▼ (Human Hand Keypoints)
                                    Detection Fusion (Multi-camera)
                                              │
                                              ▼ (Fused Hand Keypoints)
                            ┌─────────────────┴─────────────────┐
                            ▼ (Finger Joint Mapping)            ▼ (Cartesian Goal, 25Hz)
                 Hand Pose Retargeting               Motion Generation (cuRobo, 120Hz)
                 (Optimization-based)                           │
                            │                                   ▼ (Arm Joint States)
                            └─────────────────┬─────────────────┘
                                              ▼
                                 [Robot Controller (Sim/Real)]
```

Hệ thống được chia thành **4 khối chức năng chính (Modular design)**:
1.  **Hand Pose Detection:** Nhận dạng vị trí cổ tay và tọa độ 3D của các khớp ngón tay từ 1 hoặc nhiều nguồn camera.
2.  **Detection Fusion:** Hợp nhất các đám mây điểm khớp ngón tay từ nhiều camera để giảm thiểu hiện tượng che khuất (occlusion).
3.  **Hand Pose Retargeting (Sử dụng `dex-retargeting`):** Ánh xạ phi tuyến tối ưu hóa (optimization-based) tọa độ khớp tay người sang các góc khớp góc xoay của bàn tay robot khéo léo.
4.  **Motion Generation (cuRobo):** Nhận mục tiêu tọa độ Cartesian của bàn tay (gốc cổ tay) từ khối perception, sử dụng thư viện song song hóa cuRobo trên GPU để giải IK tránh va chạm ở tần số cao ($120\,\text{Hz}$).

---

## 2. Từng thành phần kỹ thuật (Những gì phải tái hiện & so sánh)

### 2.1 Hand Pose Retargeting (Ánh xạ tối ưu hóa phi tuyến)
AnyTeleop từ bỏ các tiếp cận học máy (learning-based retargeting) do khả năng khái quát hóa kém với robot mới. Thay vào đó, bài báo sử dụng phương pháp tối ưu hóa hình học phi tuyến thời gian thực:

$$\min_{q_t} \sum_{i} \left\| \alpha \mathbf{v}_t^i - \mathbf{f}_i(q_t) \right\| ^2 + \beta \left\| q_t - q_{t-1} \right\| ^2$$

$$\text{s.t. } q_l \le q_t \le q_u$$

*   **Ý nghĩa:**
    *   $q_t$: Góc khớp mục tiêu của bàn tay robot cần tìm tại thời điểm $t$.
    *   $\mathbf{v}_t^i$: Vector tọa độ khóa (keypoint vector) của tay người được percieve.
    *   $\mathbf{f}_i(q_t)$: Động học thuận (Forward Kinematics - FK) tính toán vector tương đương trên bàn tay robot.
    *   $\alpha$: Hệ số tỷ lệ kích thước tay (scaling factor).
    *   $\beta \left\| q_t - q_{t-1} \right\| ^2$: Số hạng phạt (penalty term) để đảm bảo quỹ đạo chuyển động mượt mà theo thời gian, tránh giật khớp.
    *   $q_l, q_u$: Giới hạn góc khớp vật lý của robot bàn tay.

### 2.2 Motion Generation & Tránh Va Chạm với cuRobo
*   Khối điều khiển cánh tay (Macro movement) nhận tọa độ mục tiêu Cartesian cổ tay ở tần số thấp ($25\,\text{Hz}$) và giải quỹ đạo động học ở tần số cao ($120\,\text{Hz}$).
*   AnyTeleop tận dụng **cuRobo** trên GPU để tính toán IK tránh va chạm trong vòng $< 5\,\text{ms}$, đảm bảo không xảy ra tự va chạm (self-collision) giữa cánh tay và bàn tay robot.

### 2.3 Phân bổ tần số thiết bị (Timing Profiling)

| Khối chức năng | Tần số thực thi | Phần cứng tối ưu | Ghi chú |
|---|---|---|---|
| Hand Pose Detection | $25\,\text{Hz}$ | GPU (Desktop/Laptop) | MediaPipe/MMPose ngốn nhiều tài nguyên nhất |
| Pose Retargeting | $25\,\text{Hz}$ | CPU | Giải bài toán tối ưu phi tuyến |
| Motion Generation | $120\,\text{Hz}$ | GPU (cuRobo) | Chạy song song tránh va chạm |
| Physical Execution | $120\,\text{Hz} - 500\,\text{Hz}$ | Robot Driver | Gửi torque/position target |

### 2.4 Phân tích sâu 2 mã nguồn cốt lõi (Core Repositories Analysis)

#### A. dex-retargeting (Thư viện ánh xạ ngón tay khéo léo)
Thư viện `dex-retargeting` tách rời thuật toán ánh xạ hình học ra khỏi môi trường mô phỏng cụ thể, cho phép tính toán độc lập:
*   **Trình tối ưu hóa đa dạng (Optimization Solvers):** Thư viện hỗ trợ cả trình tối ưu hóa phi tuyến **NLopt** (cho các bài toán ràng buộc phi tuyến phức tạp của ngón tay đối ngón) và bộ giải động học **Pinocchio** (tối ưu hóa tốc độ cao bằng cách tính ma trận Jacobian giải tích của bàn tay).
*   **Cấu hình ánh xạ URDF trực quan:** Người dùng chỉ cần định nghĩa một file cấu hình YAML liên kết các khớp ngón tay người (đầu ra của MediaPipe gồm: `THUMB_TIP`, `INDEX_TIP`,...) với các liên kết đầu ngón tay tương ứng trong tệp URDF của robot.
*   **Vector-based vs. Position-based:** Hỗ trợ ánh xạ theo hướng vector (hướng ngón tay) thay vì chỉ khoảng cách tuyệt đối, giúp giữ nguyên hình dáng nắm của tay người sang bàn tay robot có tỷ lệ hình học lệch nhau lớn.

#### B. sim-web-visualizer (Bộ dựng hình đám mây qua Web)
Bộ dựng hình `sim-web-visualizer` do NVIDIA phát triển giải quyết nút thắt cổ chai về hiển thị đồ họa trong teleoperation từ xa:
*   **Mô hình Client-Server tách biệt:** Máy chủ giả lập vật lý (chạy GPU mạnh chứa Isaac/SAPIEN) và máy khách của người vận hành (chỉ cần trình duyệt Web hỗ trợ WebGL/Three.js) giao tiếp bất đồng bộ qua giao thức mạng hiệu năng cao.
*   **Dữ liệu truyền tải tối giản:** Thay vì stream video chất lượng cao (ngốn băng thông và tăng độ trễ), visualizer này chỉ truyền dữ liệu chuyển động khớp (joint states) và tọa độ vị trí vật thể dưới dạng nén. Trình duyệt client sẽ tự render lại các đối tượng 3D cục bộ bằng WebGL.
*   **Giảm thiểu trễ hiển thị:** Giúp người vận hành có thể điều khiển robot từ khoảng cách xa (qua mạng Internet thông thường) với phản hồi hình ảnh thời gian thực mượt mà mà không bị ảnh hưởng bởi độ trễ nén video.

---

## 3. Ánh xạ & So sánh đối chứng sang hệ của mình

### 3.1 Bảng ánh xạ tổng thể

| Thành phần | AnyTeleop (Paper gốc) | Hệ thống của mình (ur5dex) |
|---|---|---|
| **Cánh tay** | xArm6 / Franka Panda | **UR5 (6-DoF)** |
| **Bàn tay** | Allegro (16-DoF) / Shadow (22-DoF) | **DH Robotics Hand (5 ngón, 19 khớp)** |
| **Giải chuyển động** | cuRobo ($120\,\text{Hz}$) | **cuRobo + CLIK kháng trễ ($500\,\text{Hz}$)** |
| **Bù trễ truyền thông** | Không có (Bị động khi mạng trễ) | **Bù trễ chủ động qua LKF (Lyapunov-Krasovskii)** |
| **Tần số truyền thông** | Đồng bộ chung ($25\,\text{Hz} - 120\,\text{Hz}$) | **Bất đối xứng: RTDE ($500\,\text{Hz}$) & Modbus ($50\,\text{Hz}$)** |
| **Môi trường giả lập** | SAPIEN / Isaac Gym | **Isaac Sim / Isaac Lab (USD assets)** |

### 3.2 Điểm tương đồng và Khác biệt cốt lõi (Tư duy phản biện)
1.  **Sự tương đồng về mặt triết lý thiết kế:** Cả hai hệ thống đều sử dụng cấu hình phân cấp **Macro-Micro** để tách biệt điều khiển cánh tay (cuRobo/IK) và bàn tay khéo léo (DRL/Retargeting).
2.  **Khác biệt về xử lý độ trễ (Lý do AnyTeleop thất bại trong Case 3):**
    *   AnyTeleop truyền trực tiếp Cartesian pose nhận được từ camera tới robot qua mạng socket cơ bản. Khi mạng xuất hiện jitter hoặc độ trễ lớn ($h_m = 32\,\text{ms}$), hệ thống phản hồi của AnyTeleop không có cơ chế dự đoán hay lọc trôi. Điều này dẫn đến sai số bám quỹ đạo rất lớn (MAE đạt $28.4\,\text{mm}$ trong thực nghiệm Case 3).
    *   Hệ thống của chúng ta sử dụng **bộ lọc EKF** dự đoán quỹ đạo bay của bóng trước, kết hợp với phương trình CLIK kháng trễ:
        $$\dot{\mathbf{q}}(t) = \mathbf{J}^* \mathbf{v} - \mathbf{K}_p \mathbf{e}(t) - \mathbf{K}_d \mathbf{e}(t - h(t))$$
        nhúp dữ liệu trễ $h(t)$ để tính toán vận tốc bù, giúp ổn định sai số bám chỉ ở mức $1.8\,\text{mm}$.

---

## 4. Cấu trúc Code đối chứng đề xuất

Nhằm thực hiện so sánh đối chứng sòng phẳng với AnyTeleop, cấu trúc module của AnyTeleop được giả lập lại dưới dạng một baseline testbench trong hệ thống của chúng ta:

```
Human-Simulator-ur5dex/
├── src/
│   ├── sim/
│   │   ├── compliance_grasp_controller.py  ← Bộ điều khiển trở kháng bàn tay
│   │   ├── curobo_ik_solver.py            ← Bộ giải quỹ đạo cuRobo cho UR5
│   │   └── run_dynamic_catching_sim.py    ← Tệp kịch bản chính chạy so sánh
│   └── baselines/
│       └── anyteleop_retargeting.py        ← Bộ giải tối ưu phi tuyến mô phỏng AnyTeleop (không có bù trễ)
```

---

## 5. Lộ trình So sánh đối chứng & Rủi ro

1.  **Cột mốc 1: Dựng baseline AnyTeleop trong Isaac Sim:** Sử dụng trực tiếp thuật toán tối ưu hóa vị trí ngón tay của AnyTeleop (dựa trên khoảng cách Euclid không có bù trở kháng) kết hợp với IK tiêu chuẩn không bù trễ cho UR5.
2.  **Cột mốc 2: Mô phỏng nhiễu mạng và độ trễ ($h_m = 32\,\text{ms}$):** Bơm trễ nhân tạo biến thiên thời gian (time-varying latency) vào luồng truyền lệnh điều khiển.
3.  **Cột mốc 3: Thu thập chỉ số so sánh (Metrics Evaluation):**
    *   Đo sai số bám quỹ đạo trung bình (MAE) và sai số bình phương trung bình (RMSE) của khớp tay UR5.
    *   Tính toán tỷ lệ bắt bóng thành công trên 50 lần thử nghiệm (Success Rate %).
    *   Vẽ đồ thị so sánh lực va chạm tiếp xúc giữa bàn tay cứng (AnyTeleop style) và bàn tay mềm trở kháng (DRL proposed).

### Rủi ro chính:
*   **Sai lệch Morphological:** Bàn tay DH Robotics 5 ngón có kết cấu underactuated (5 động cơ kéo cáp cho 19 khớp xoay), phức tạp hơn cấu hình Allegro Hand (16 động cơ cho 16 khớp) của AnyTeleop. Việc giải FK $\mathbf{f}_i(q_t)$ trong AnyTeleop retargeting sẽ bị thiếu ràng buộc tự do, đòi hỏi phải nhân thêm ma trận ánh xạ truyền dẫn $\mathbf{S}$.
