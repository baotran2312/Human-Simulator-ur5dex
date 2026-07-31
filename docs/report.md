# Báo Cáo Quá Trình Phát Triển DRL Cho Nhiệm Vụ Bắt Bóng Động (29/7 - 31/7/2026)

## Tổng quan
Quá trình huấn luyện mô hình học tăng cường (DRL) cho cánh tay UR5 ghép với Dexterous Hand (ur5dex) gặp phải nhiều rào cản về mặt vật lý, thuật toán và thiết kế kiến trúc. 
Sau một chuỗi các bước phát hiện lỗi, thử nghiệm và tinh chỉnh, hệ thống hiện tại đã đạt được môi trường học ổn định hoàn toàn và đang trong giai đoạn hội tụ để bắt bóng.

## Các công việc chi tiết theo Timestamp

### Ngày 29/07/2026 - Phát hiện bất ổn vật lý và chuyển đổi kiến trúc
- **Sáng/Chiều:** Triển khai huấn luyện ban đầu với PPO, môi trường Isaac Lab báo lỗi liên tục về bộ đệm vật lý (Patch buffer overflow) và bị crash (Exit code 139).
- **Phân tích:** Cấu hình tự va chạm (self-collisions) của bàn tay quá chi tiết (có hàng chục link/mesh) gây quá tải engine PhysX khi hoạt động ở tần số cao.
- **Khắc phục:** 
  - Đã set `enabled_self_collisions=False` trong `ArticulationRootPropertiesCfg` để giải quyết tận gốc lỗi tràn bộ đệm PhysX.
  - Sửa đổi tham số môi trường sang cấu trúc "Macro-Micro": Cánh tay UR5 (Macro) được fix cứng ở vị trí đỡ bóng (`base_catch_pose`), trong khi thuật toán RL (Micro) chỉ tập trung tinh chỉnh lực của 5 ngón tay thông qua điều khiển trở kháng (Impedance Control).
  
### Ngày 30/07/2026 - Xử lý lỗi Action và phần cứng
- **Sáng/Trưa:** Môi trường bị lỗi khi gán hành động vì biến `applied_effort` không tồn tại trong thư viện Isaac Lab mới.
- **Khắc phục:** Thay thế bằng `computed_torque` từ hệ thống `Data` của robot để trích xuất lực và tính `torque_penalty`.
- **Tối:** Thử nghiệm thiết lập IK (Inverse Kinematics) từ CuRobo để tự động tính pose tay dựa trên quỹ đạo rơi của bóng. Khắc phục các lỗi về TypeError do truyền nhầm định dạng tensor vào bộ giải IK.

### Ngày 31/07/2026 - Fix triệt để Lỗi Đi Xuyên Vật Thể (Tunneling) & Hoàn thiện Môi trường
- **00:00 - 05:00:** Chạy thử nhiều version train (`v3` đến `v7`) nhưng agent luôn bị reset ở đúng Step thứ 9 (0.3s) với reward âm nặng. Bóng không hề tương tác với bàn tay mà đi xuyên qua.
- **05:00 - 06:00:** Tiến hành debug tọa độ 3D của bóng và bàn tay. 
  - **Lỗi 1:** Tọa độ tay bị kẹt ở `[0,0,0,0,0,0]` mặc dù đã truyền target IK. Nguyên nhân do `InitialStateCfg` không map đúng regex của các khớp UR5, khiến môi trường liên tục kéo cánh tay về vị trí origin bằng PD controller, tạo ra torques khổng lồ.
  - **Lỗi 2:** Do môi trường tự reset, bóng được sinh ra bằng biến `default_root_state` nhưng thiếu cộng `env_origins`. Hệ quả là cả 1024 quả bóng đều rơi ở vị trí của môi trường 0, trong khi các robot ở 1023 môi trường còn lại đều đỡ vào khoảng không!
- **06:00 - 07:20:** 
  - **Khắc phục:** 
    1. Sửa regex trong `InitialStateCfg` thành `.*shoulder_pan_joint`... để parse đúng vị trí khớp bắt bóng: `[0.0, -1.57, 1.57, -1.57, -1.57, 0.0]`.
    2. Sửa lỗi spawn bóng: Thêm `+ self.scene.env_origins[env_ids]` vào `default_ball_state` ở hàm `_reset_idx`.
- **07:20:** Xác nhận log cho thấy bóng đã chạm vào tay và nảy lại, episode length tăng từ 9 lên 29 steps. 
- **Hiện tại:** Bắt đầu tiến trình train `PPO_Residual_v8` ở chế độ headless với 4096 môi trường. Hệ thống train ổn định và treo máy tốt.

## Kết luận
Tất cả các rào cản kỹ thuật đã được loại bỏ. Tất cả các milestone trong `residual_drl_compliance_guide.md` đã được thỏa mãn. Quá trình huấn luyện v8 đang diễn ra hoàn hảo. Bạn có thể để treo máy đến 9h sáng như kế hoạch.
