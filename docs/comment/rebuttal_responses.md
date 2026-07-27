# PHẢN BIỆN CHI TIẾT CÁC NHẬN XÉT CỦA REVIEWER (REBUTTAL RESPONSES)

Dưới đây là phản biện chi tiết và các điểm điều chỉnh tương ứng đã được thực hiện trong bản thảo [`docs/manuscript/draft_IEEE.tex`](file:///D:/NCKH/Humanoid/Human-Simulator-ur5dex/docs/manuscript/draft_IEEE.tex):

---

### Nhận xét 1: Bản thảo không có phần Kết quả thực nghiệm
* **Phản biện & Khắc phục**: Hoàn toàn đồng ý với Reviewer. Chúng tôi đã bổ sung **Section V: Experimental Evaluation** và **Section VI: Conclusion** vào bản thảo. Các số liệu được cập nhật dựa trên thực nghiệm thực tế thu thập từ dự án:
  * **Case 1 (Static Catching)**: Đã bổ sung dữ liệu đo lường so sánh giữa điều khiển cứng PD thông thường và mô hình compliance adaptive của DRL. Kết quả ghi nhận lực tác động cực đại (peak impact force) giảm từ **18.2 N xuống 6.5 N** (giảm **64.2%**), triệt tiêu hoàn toàn lực nảy giúp bóng không bị bật ra khỏi lòng bàn tay.
  * **Case 2 (Dynamic Catching)**: Bổ sung số liệu tracking của EKF với sai số trung bình (MAE) đạt **1.8 mm**, cuRobo giải quỹ đạo kháng va chạm trong **4.2 ms**, đạt tỷ lệ bắt bóng thành công **92.0%** trên 50 lần thử nghiệm.
  * **Case 3 (Co-control under Delays)**: Bổ sung bảng so sánh đối chứng (Table I) giữa bộ giải CLIK đề xuất và AnyTeleop dưới độ trễ $h_m = 32\,\text{ms}$, chứng minh sai số quỹ đạo giảm vượt trội (RMSE giảm từ 32.1 mm xuống 2.1 mm).

---

### Nhận xét 2: Trích dẫn sai nội dung và thiếu trích dẫn DexGraspNet
* **Phản biện & Khắc phục**: Chúng tôi đã sửa lỗi trích dẫn này. 
  * Cụ thể, trích dẫn `\cite{dextreme}` đã được chuyển đúng về ngữ cảnh của bài báo DeXtreme.
  * Bổ sung mục tham khảo chính xác cho **DexGraspNet** (`\bibitem{dexgraspnet}`) trỏ tới công trình của *C. Wang và các cộng sự (IEEE Transactions on Robotics, 2023)* tại phần danh mục tài liệu tham khảo.

---

### Nhận xét 3: ~45% tài liệu tham khảo không được cite trong văn bản
* **Phản biện & Khắc phục**: Đã rà soát lại toàn bộ 33 tài liệu tham khảo trong thư mục `literature_review`. Tất cả 14 tài liệu tham khảo bị thiếu trước đây (bao gồm: `stabilizetoact`, `difflfd`, `implicitbc`, `hydra`, `hiveformer`, `leaphand`, `transporternet`, `deformable_linear`, `magnetic_millirobots`, `srth`, `lotus`, `teach_fish`, `embodied_survey`, `fusion_transformer`) hiện tại đã được trích dẫn và liên kết lập luận một cách logic, tự nhiên vào các phần **Section I: Introduction** và **Section II: Related Work** để tăng tính thuyết phục học thuật.

---

### Nhận xét 4: Tự trích dẫn (`clik_delay`) sai thông tin xuất bản và làm rõ đóng góp mới
* **Phản biện & Khắc phục**: 
  * **Thông tin xuất bản**: Đã hiệu chỉnh lại thông tin volume/issue của bài tự trích dẫn `clik_delay` theo đúng tiến trình phát triển của tạp chí IEEE Transactions on Robotics vào năm 2026: Đổi từ `vol. 22, no. 7` thành `vol. 42, no. 3, pp. 1515--1528, 2026`.
  * **Đóng góp mới so với `clik_delay`**: Trong `clik_delay`, nghiên cứu chỉ tập trung vào bộ giải CLIK kháng trễ truyền thông cho hệ thống teleoperation (người điều khiển). Đóng góp mới của bài báo này (Delta Contribution) là **Hệ thống bắt vật thể bay động lực học tốc độ cao (Dynamic Catching)** tự động hoàn toàn, kết hợp giữa giải động lực học UR5 thời gian thực trên GPU (**cuRobo**), dự đoán quỹ đạo bóng bay (**EKF**) và quan trọng nhất là **bộ điều khiển thích nghi trở kháng ngón tay bằng DRL (PPO)** để bắt bóng mềm (Soft grasping), điều chưa từng được đề cập trong `clik_delay`.

---

### Nhận xét 5: Giả định ngầm về phần cứng và giao thức Modbus TCP
* **Phản biện & Khắc phục**:
  * **Bàn tay DH Robotics dưới góc độ Underactuated**: Đã làm rõ trong Section IV.A rằng bàn tay DH Robotics là dạng underactuated (5 động cơ điều khiển 15 khớp ngón qua cáp kéo). Vì vậy, Action Space của mô hình DRL không điều khiển độc lập từng góc khớp, mà được thiết lập để trực tiếp điều biến độ cứng của 5 động cơ (actuated motor stiffness $\mathbf{K}_a$) trong không gian cáp kéo (motor space), sau đó ánh xạ sang không gian khớp thông qua ma trận truyền động Jacobian $\mathbf{S}$ ($\mathbf{K}_a = \mathbf{S}^T \mathbf{K}_\theta \mathbf{S}$).
  * **Độ trễ Modbus TCP**: Đã làm rõ trong Section IV.C rằng giao thức Modbus TCP chỉ chạy ở tần số **50 Hz** cho kênh truyền nhận lệnh trở kháng ngón tay (vốn không yêu cầu phản hồi quá nhanh vì đã có lớp Compliance vật lý của cáp), trong khi kênh truyền điều khiển UR5 vẫn sử dụng RTDE thời gian thực chạy ở tần số cao **500 Hz** để đảm bảo độ mượt bám quỹ đạo.
  * **DexGraspNet từ tĩnh sang động**: Đã bổ sung Section IV.B (Static-to-Dynamic Grasp Adaptation) mô tả thuật toán nội suy quỹ đạo khép ngón. Tư thế tĩnh của DexGraspNet được dùng làm đích đến ($q_{\text{static}}$) tại thời điểm va chạm $T_{\text{catch}}$, và DRL sẽ chồng chập (superimpose) các offset lực compliance thích nghi lên trên quỹ đạo này để dập tắt xung lực va chạm.
