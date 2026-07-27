# LỘ TRÌNH CHI TIẾT: PHƯƠNG PHÁP LUẬN LITERATURE REVIEW & XÂY DỰNG SOTA / PRELIMINARIES

> [!IMPORTANT]  
> Tài liệu này vạch ra quy trình chuẩn hóa để phân tích sâu 5 nghiên cứu trong thư mục `docs/literature_review`, nhằm trích xuất dữ liệu khoa học chính xác cho **Bảng so sánh State-of-the-Art (SOTA)** và xây dựng phần **Cơ sở lý thuyết (Preliminaries)** vững chắc cho bài báo Q1, tránh việc viết các công thức và nhận định thiếu căn cứ học thuật.

---

## I. MỤC TIÊU CỐT LÕI
1. **Tính học thuật chính thống**: Mọi công thức toán học và phương pháp đề xuất trong bản thảo (Manuscript) phải được kế thừa trực tiếp và tham chiếu (citation) rõ ràng từ các công trình nền tảng đã công bố.
2. **Bảng SOTA định lượng**: Xây dựng bảng so sánh không chỉ định tính (qualitative) mà phải định lượng (quantitative) về: tần số điều khiển (Hz), độ trễ (ms), sai số quỹ đạo (mm), số lượng bậc tự do (DoF), và tỉ lệ thành công (Success Rate %).
3. **Phần Preliminaries chặt chẽ**: Thiết lập nền tảng lý thuyết về Động học ngược kháng singularity (Singularity-Robust IK), Mô hình hóa trễ truyền thông (Delay Modeling), và Động lực học tiếp xúc (Contact Dynamics).

---

## II. QUY TRÌNH 4 BƯỚC ĐỌC VÀ TRÍCH XUẤT (4-STEP EXTRACTION PROTOCOL)

### Bước 1: Hệ Thống Hóa và Phân Loại Nghiên Cứu
Phân bổ 5 tài liệu hiện có vào các trục nghiên cứu cốt lõi:
* **Trục 1: Dataset & Grasping Policy (Chính sách nắm bắt)**
  * *Tài liệu*: `A Review on Dataset Collection Strategies...` và `An overview of learning-based dexterous grasping...`
  * *Nội dung trích xuất*: Các phương pháp sinh pose chụp bóng trong Isaac Gym (DexGraspNet, GraspIt!), cách thu thập dữ liệu chuyên gia để huấn luyện củng cố (RL).
* **Trục 2: Perception & Trajectory Tracking (Nhận diện & Quỹ đạo)**
  * *Tài liệu*: `A review on deep learning for vision-based hand detection...`
  * *Nội dung trích xuất*: Các mô hình deep learning ước lượng tư thế tay/vật thể, tần số xử lý khung hình (fps), sai số tracking.
* **Trục 3: Control Theory & Kinematics (Lý thuyết điều khiển)**
  * *Tài liệu*: `Towards a Unified Understanding of Robot Manipulation...`
  * *Nội dung trích xuất*: Mô hình toán học về giải IK (DLS, CLIK), lập kế hoạch chuyển động kháng va chạm (Collision avoidance).
* **Trục 4: Deep Learning & Robot Action (Mô hình VLA)**
  * *Tài liệu*: `Vision-Language-Action Models for Robotics...`
  * *Nội dung trích xuất*: Xu hướng sử dụng mô hình nền tảng (Foundation models) điều khiển end-to-end.

### Bước 2: Thiết Kế Khung Bảng So Sánh SOTA (SOTA Matrix Template)
Để đưa vào bài báo, ta cần điền thông số từ các bài báo vào bảng cấu trúc sau:

| Nghiên Cứu / Framework | Bậc Tự Do (DoF Arm/Hand) | Tần Số Phản Hồi (Hz) | Cơ Chế Bù Trễ / Đồng Bộ | Phương Pháp Điều Khiển Ngón Tay | Tỉ Lệ Bắt Thành Công | Môi Trường Thực Nghiệm (Sim/Real) |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **AnyTeleop (2023)** | 6-DoF + 16-DoF | 20-30 Hz | Không tích hợp | Vị trí (PD Control) | 78% (Vật thể tĩnh) | Isaac Sim + Real |
| **DexSim2Real (2024)**| 6-DoF + 19-DoF | 50 Hz | Bù trễ cục bộ | Lực tiếp xúc thích nghi | 82% (Vật thể chậm) | MuJoCo + Real |
| **CLIK Proposed (2026)**| **6-DoF + 19-DoF**| **100 Hz** | **LKF & LMI Robust Control**| **Compliance Control** | **92% (Bóng ném động)**| **Isaac Sim + Real** |

*(Ghi chú: Các thông số của các nghiên cứu đối chứng phải được trích xuất chính xác từ phần thực nghiệm của chính các bài báo đó).*

### Bước 3: Trích Xuất và Hệ Thống Hóa Phần Preliminary Math
Chúng ta sẽ mở trực tiếp từng file markdown trong thư mục `docs/literature_review` để copy các công thức gốc:
1. **Mô hình hóa hệ truyền thông trễ (Time-varying Delay Model)**:
   * Tìm phương trình Lyapunov-Krasovskii Functional (LKF) mô tả tính ổn định của hệ thống điều khiển khi có trễ $h(t)$:
     $$V(t) = x^T(t) P x(t) + \int_{t-h(t)}^{t} x^T(s) Q x(s) ds + \dots$$
2. **Thuật toán Damped Least-Squares (DLS)**:
   * Trích xuất phương trình toán học giải singular-robust Jacobian:
     $$J^\dagger = J^T (J J^T + \lambda^2 I)^{-1}$$
3. **Bộ lọc Kalman dự đoán quỹ đạo (EKF Prediction)**:
   * Trích xuất các phương trình cập nhật trạng thái (State Update) và cập nhật đo lường (Measurement Update) của vật thể bay.

### Bước 4: Viết Lại Bản Thảo (Manuscript Revision)
* Thay thế toàn bộ các phần viết chung chung bằng các đoạn văn có trích dẫn khoa học cụ thể (ví dụ: `as proposed in [1]`, `following the compliance formulation of [3]`).
* Thêm bảng SOTA đã hoàn thiện số liệu vào bài báo.

---

## III. KẾ HOẠCH HÀNH ĐỘNG CHI TIẾT (ACTION PLAN)

* **Hạng mục 1: Đọc và Trích xuất số liệu định lượng (Thời gian: 2 ngày)**
  * Quét các bài báo để ghi lại thông số DoF, tần số, độ trễ và tỷ lệ thành công của các hệ thống arm-hand hiện tại.
* **Hạng mục 2: Soạn thảo tệp `preliminaries.md` chứa toàn bộ công thức toán học chuẩn (Thời gian: 1 ngày)**
  * Hệ thống các công thức động học, lý thuyết ổn định Lyapunov và Kalman Filter.
* **Hạng mục 3: Cập nhật tệp `draft_IEEE.tex` (Thời gian: 1 ngày)**
  * Chèn bảng SOTA và bổ sung các trích dẫn tài liệu tham khảo chính xác vào các Section I, II, III.
