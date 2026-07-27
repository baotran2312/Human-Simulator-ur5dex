# LỘ TRÌNH PHÁT TRIỂN SONG SONG: THỰC NGHIỆM & VIẾT MANUSCRIPT (Q1 PAPER)

> [!TIP]  
> Lộ trình này được thiết kế theo mô hình **Parallel Sprint (Chạy song song 4 tuần)** nhằm tối ưu hóa thời gian: dữ liệu thực nghiệm thu thập đến đâu sẽ được điền trực tiếp vào bản thảo bài báo đến đó.

---

## BẢNG LỘ TRÌNH CHI TIẾT (WEEKLY PARALLEL GANTT)

| Mốc Thời Gian | Nhiệm Vụ 1: Mô Phỏng & Thực Nghiệm (Sim & Hardware) | Nhiệm Vụ 2: Viết Bản Thảo (Manuscript Writing) | Sản Phẩm Đầu Ra (Deliverables) |
| :--- | :--- | :--- | :--- |
| **Tuần 1: Thiết lập nền tảng** | - Cài đặt gói `pymodbus` trên môi trường `env_isaaclab` để điều khiển bàn tay DH.<br>- Xây dựng scene ném bóng động trong Isaac Sim (tạo bộ phát bóng tự động ném bóng theo lực $F$). | - Phác thảo cấu trúc bài báo (IEEE Template LaTeX).<br>- Viết phần **Section I: Introduction** (nêu bài toán Dynamic Dexterous Grasping và thách thức).<br>- Viết phần **Section II: Related Work** (dựa trên 5 file literature review có sẵn). | - Môi trường mô phỏng Isaac Sim bắn bóng.<br>- Bản thảo phần Introduction & Related Work. |
| **Tuần 2: Tích hợp thuật toán** | - Viết module dự đoán quỹ đạo bóng bằng bộ lọc Kalman mở rộng (EKF) trong Isaac Sim.<br>- Kết nối tọa độ điểm đón $P_{int}$ với thư viện **cuRobo** để UR5 bám điểm thời gian thực.<br>- Tinh chỉnh thuật toán kẹp mềm (Compliance) các khớp ngón tay. | - Viết phần **Section III: Proposed Methodology**.<br>- Vẽ sơ đồ khối kiến trúc hệ thống (System Architecture Diagram).<br>- Mô hình hóa toán học phương trình quỹ đạo bóng và phương pháp giải IK của cuRobo. | - Code chạy thử thành công Case 1 & Case 2 trong mô phỏng.<br>- Bản thảo phần Phương pháp nghiên cứu (Math & Architecture). |
| **Tuần 3: Đồng bộ Sim-to-Real** | - Sử dụng `ur_rtde` kết nối UR5 thật với mô phỏng qua mạng nội bộ.<br>- Thực hiện chạy thực nghiệm 3 Cases trên robot thật dựa trên playback từ Sim.<br>- Thu thập và ghi nhận dữ liệu: sai số vị trí khớp (RMSE), lực va chạm (nếu có), và tỷ lệ thành công (Success Rate). | - Viết phần **Section IV: Experimental Setup** (Mô tả cấu hình phần cứng: GPU RTX 5060 Ti, robot UR5, bàn tay DH).<br>- Thiết kế các bảng trống (Tables) để chuẩn bị điền số liệu thực nghiệm. | - Dữ liệu thực nghiệm đầy đủ (lưu dưới dạng file `.csv`).<br>- Bản thảo phần Thiết kế thực nghiệm. |
| **Tuần 4: Hoàn thiện & Tinh chỉnh** | - Xử lý hậu kỳ dữ liệu thực nghiệm (Post-processing).<br>- Sử dụng Python (Matplotlib/Seaborn) để vẽ các biểu đồ so sánh: quỹ đạo bám khớp thực tế so với mô phỏng, biểu đồ tiêu tán động năng tiếp xúc. | - Điền số liệu vào các bảng ở **Section V: Results & Discussion**.<br>- Viết phần thảo luận khoa học (so sánh hiệu năng với các phương pháp cũ).<br>- Viết phần **Section VI: Conclusion & Future Work**.<br>- Proofreading và chỉnh sửa định dạng LaTeX theo chuẩn IEEE. | - Các biểu đồ chất lượng cao (vector graphics).<br>- Manuscript hoàn chỉnh 100% sẵn sàng gửi duyệt (Ready to Submit). |

---

## CÁC BIỆN PHÁP GIẢM THIỂU RỦI RO (RISK MITIGATION)

1. **Rủi ro 1: Thuật toán bắt bóng trong mô phỏng (Case 2) không hội tụ kịp**
   * *Giải pháp*: Tận dụng ngay các bộ chính sách (Policies) được huấn luyện sẵn của **DexGraspNet** làm baseline thay vì tự train thuật toán RL từ đầu.
2. **Rủi ro 2: Độ trễ truyền thông khi chạy trên robot thật (Case 3) làm rơi bóng**
   * *Giải pháp*: Sử dụng bộ lọc thông thấp (Low Pass Filter - LPF) đã có sẵn trong dự án `Human-Mimicking-UR5` (`retargeting_solver.py`) để làm mịn quỹ đạo và bù trễ (Feedforward prediction).
3. **Rủi ro 3: Trùng lặp thuật ngữ hoặc thiếu tài liệu tham khảo khi viết bản thảo**
   * *Giải pháp*: Tận dụng file tổng hợp [docs/report/README.md](file:///D:/NCKH/Humanoid/Human-Simulator-ur5dex/docs/report/README.md) đã ánh xạ sẵn 5 nghiên cứu lớn để viết nhanh phần Related Work.
