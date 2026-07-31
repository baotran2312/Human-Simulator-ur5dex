# Hướng Dẫn Từng Bước Triển Khai Hệ Thống Trở Kháng Dư (Residual DRL Compliance Guide)

Tài liệu này vạch ra lộ trình chi tiết và cấu trúc cụ thể để tái thiết kế giải thuật học máy từ mô hình học từ đầu (from-scratch) sang **Mô hình Trở kháng dư (Residual DRL Compliance)**. Mục tiêu là định hình các mốc bàn giao cụ thể (milestones) kèm theo đầu ra định lượng để kiểm soát tiến độ và ngăn chặn việc lãng phí tài nguyên tính toán.

---

## I. KIẾN TRÚC HỆ THỐNG (SYSTEM ARCHITECTURE)

Hệ thống hoạt động theo mô hình phối hợp Macro-Micro phân cấp:
1.  **Macro Controller (Khép ngón Heuristic):** Chịu trách nhiệm về mặt hình học (đóng khép tay đúng thời điểm để bắt giữ bóng).
2.  **Micro Controller (DRL Impedance):** Chịu trách nhiệm về mặt lực học (điều biến độ cứng động cơ cáp kéo $\mathbf{K}_a$ để dập tắt xung lực va chạm và triệt tiêu lực nảy).

```
[3D Vision Camera]
       │
       ▼ (Tọa độ bóng x_ball, v_ball)
[EKF Ball Tracker] 
       │
       ▼ (Dự đoán điểm đón x_intercept & Thời điểm T_catch)
[cuRobo Motion Planner] ──────► [UR5 Arm Actuator (RTDE 500Hz)] (Macro Control)
       │
       ▼ (Tính khoảng cách Palm-to-Ball)
[Heuristic Trigger] ──────────► [Target Joint Pos (Close/Open)] (Position Control)
       │                                       │
       ▼ (DRL State Space s_t)                 ▼
[DRL Compliance Policy (PPO)] ──► [Joint Impedance Controller] ──► [DH Hand (Modbus 50Hz)] (Micro Control)
                                  (Ka = Ka0 + diag(delta_Ka))
```

---

## II. CHI TIẾT CÁC CỘT MỐC TRIỂN KHAI (MILESTONES) & ĐẦU RA (DELIVERABLES)

---

### Milestone 1: Xác Thực Vật Lý Va Chạm & Lưới Colliders (Collision & Restitution Setup)
*   **Mô tả công việc:** 
    1.  Cấu hình quả bóng `DynamicBall` trong Isaac Lab thành thực thể vật lý cứng có collider. Lưới hình học (`visual mesh`) và lưới va chạm (`collision mesh`) phải trùng khớp hoàn toàn.
    2.  Gán vật liệu vật lý (`physics_material`) cho quả bóng và lòng bàn tay robot: Độ nảy `restitution = 0.0` (Zero bounce) và hệ số ma sát `static_friction = 1.0` để tối ưu lực bám dính khi va chạm.
*   **Các bước thực hiện:**
    *   Mở file [`src/rl/dh_hand_catch_env.py`](file:///D:/NCKH/Humanoid/Human-Simulator-ur5dex/src/rl/dh_hand_catch_env.py), cập nhật định nghĩa class `DHHandCatchSceneCfg`:
        ```python
        collision_props=sim_utils.CollisionPropertiesCfg()
        physics_material=sim_utils.RigidBodyMaterialCfg(restitution=0.0, static_friction=1.0)
        ```
*   **ĐẦU RA CỤ THỂ (DELIVERABLE):** 
    *   *Visual Check:* Chạy mô phỏng Isaac Sim ở chế độ GUI, bật tính năng hiển thị lớp lưới va chạm (`Show Colliders`). Quả bóng khi rơi tự do xuống lòng bàn tay phải bị khựng lại ngay lập tức (không xuyên qua tay) và nằm im (không bị nảy văng ra ngoài).

---

### Milestone 2: Bộ Kích Hoạt Khép Ngón Tự Động (Heuristic Catch Trigger)
*   **Mô tả công việc:**
    1.  Lập trình cảm biến khoảng cách hình học trong môi trường: Tính khoảng cách Euclid giữa quả bóng và gốc lòng bàn tay robot (`DH_base_link`).
    2.  Xây dựng logic trigger: Nếu khoảng cách $d_{\text{ball-palm}} < 0.10\,\text{m}$, gửi lệnh vị trí khớp khép ngón tay ($0.75\,\text{rad}$ hoặc góc tối đa tùy URDF) cho toàn bộ 5 ngón. Nếu khoảng cách $> 0.10\,\text{m}$, mở rộng bàn tay ($0.0\,\text{rad}$).
*   **Các bước thực hiện:**
    *   Cập nhật hàm `_apply_action(self)` trong `dh_hand_catch_env.py` để tính khoảng cách vector hóa bằng PyTorch:
        ```python
        palm_pos = self.robot.data.body_pos_w[:, self.palm_link_idx] - self.scene.env_origins
        ball_pos = self.ball.data.root_pos_w - self.scene.env_origins
        palm_dist = torch.norm(ball_pos - palm_pos, dim=-1)
        target_finger_joint_pos = torch.where(palm_dist.unsqueeze(-1) < 0.10, 0.75, 0.0)
        ```
*   **ĐẦU RA CỤ THỂ (DELIVERABLE):**
    *   *Simulation Replay:* Khi bóng rơi tự do, hệ thống tự động gập các khớp ngón tay chính xác $100\%$ để giữ bóng lại mà chưa cần bật mô hình DRL. Terminal in ra log: `[Env 0] Ball caught! Dist: X.XX m, Target joint state: 0.75` khi bóng đi vào khu vực đón.

---

### Milestone 3: Ánh Xạ Action Space Thành Trở Kháng Lực (Impedance Action Mapping)
*   **Mô tả công việc:**
    1.  Cấu hình lại Action Space của DRL Agent là 5 chiều (đại diện cho 5 ngón tay cáp kéo).
    2.  Lập trình bộ ánh xạ tuyến tính chuyển đổi Action từ mạng neural (khoảng $[-1.0, 1.0]$) thành độ lệch trở kháng $\Delta \mathbf{K}_a \in [-20.0, 20.0]\,\text{N.m/rad}$.
    3.  Áp dụng cấu hình trở kháng động vào Simulator bằng hàm API `set_joint_stiffness` và `set_joint_damping` của Isaac Lab.
*   **Các bước thực hiện:**
    *   Tích hợp vào hàm `_apply_action(self)` trong `dh_hand_catch_env.py`:
        ```python
        delta_K = self.actions * 20.0 
        stiffness = 50.0 + delta_K
        damping = 5.0 + 0.1 * delta_K
        self.robot.set_joint_stiffness(stiffness, joint_indices=list(range(6, 25)))
        self.robot.set_joint_damping(damping, joint_indices=list(range(6, 25)))
        ```
*   **ĐẦU RA CỤ THỂ (DELIVERABLE):**
    *   *Telemetry Verification:* Chạy 1 test episode ngẫu nhiên và in ra giá trị trở kháng khớp. Log in ra màn hình phải chứng minh các thông số stiffness/damping của các ngón tay thay đổi động theo từng bước thời gian dựa trên đầu ra mạng neural.

---

### Milestone 4: Đồng Bộ Hóa State Space & Reward Với Bản Thảo (Academic Sync)
*   **Mô tả công việc:**
    1.  Đồng bộ không gian quan sát (Observation Space) để bao gồm: vị trí/vận tốc bóng, vị trí/vận tốc khớp ngón tay và lịch sử lực tiếp xúc từ bước trước (tổng 41 chiều như bản thảo công bố).
    2.  Thiết kế lại hàm thưởng (Reward Function) tập trung phạt lực tiếp xúc để ép robot học cách làm mềm tay:
        $$r_t = - w_2 \|\mathbf{f}_{\text{ext}}(t)\|^2 - w_3 \|\boldsymbol{\tau}(t)\|^2$$
        Trong đó $\mathbf{f}_{\text{ext}}$ đọc từ sensor tiếp xúc vật lý của robot trong Isaac Lab.
*   **Các bước thực hiện:**
    *   Cập nhật `_get_observations(self)` và `_get_rewards(self)` trong `dh_hand_catch_env.py` để sử dụng dữ liệu từ cảm biến PhysX `ContactSensor` hoặc mô-men động cơ.
*   **ĐẦU RA CỤ THỂ (DELIVERABLE):**
    *   *Observation Check:* In ra hình dáng (shape) của observation vector đảm bảo đúng $[N_{\text{envs}}, 41]$ và không chứa giá trị `NaN` hay `0.0` tĩnh.

---

### Milestone 5: Huấn Luyện PPO & Tiêu Chí Kiểm Tra Nhanh 15 Phút (Quick-Check Fail-Fast)
*   **Mô tả công việc:**
    1.  Khởi chạy tiến trình huấn luyện bằng thuật toán PPO thông qua `train_ppo.py`.
    2.  Sử dụng TensorBoard để giám sát trực tuyến đồ thị học tập của PPO.
    3.  Thực thi quy trình **Fail-Fast (Kiểm tra nhanh)** trong vòng 15 phút đầu tiên (khoảng 50 Epochs).
*   **Tiêu chí đánh giá định lượng:**
    *   *ĐẠT YÊU CẦU:* Đường cong reward trung bình (Mean Reward) có xu hướng dốc lên liên tục, và độ lớn của lực va chạm cực đại ghi nhận giảm tối thiểu **$30\%$** sau 50 epochs. Tiếp tục train đến khi hội tụ.
    *   *THẤT BẠI:* Reward đi ngang hoặc giảm mạnh về âm vô hạn. **Tắt tiến trình ngay lập tức** để kiểm tra lại ma trận Jacobian truyền dẫn khớp $\mathbf{S}$.
*   **ĐẦU RA CỤ THỂ (DELIVERABLE):**
    *   *TensorBoard Log:* Đồ thị huấn luyện biểu diễn hàm Reward dốc lên, Loss giảm và đạt hội tụ về mức ổn định trở kháng mềm. Thời gian train thực tế kỳ vọng: **$45 - 90\,\text{phút}$**.
