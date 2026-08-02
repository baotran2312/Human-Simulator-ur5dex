# Phân tích Unified MPC Framework & Kế hoạch so sánh đối chứng / tích hợp sang UR5 + tay DH (Isaac Sim & Real)

> Nguồn: `docs/literature_review/A Unified MPC Framework for Whole-Body Dynamic Locomotion and Manipulation.md`
> (Sleiman et al. 2021, IEEE Robotics and Automation Letters). Project page: <https://sls-manipulation.github.io/>
>
> **Trạng thái mã nguồn (kiểm tra 2026-08-02):** Giải thuật được tích hợp và nguồn mở trong thư viện tối ưu hóa điều khiển **OCS2** (Optimal Control for Switched Systems) của RSL, ETH Zurich tại:
> - **OCS2 Repository:** <https://github.com/leggedrobotics/ocs2>
> Đây là một framework tối ưu hóa điều khiển dạng C++ hiệu năng cao dành cho các hệ thống chuyển động đa tiếp xúc (multi-contact).
>
> → Ta sẽ sử dụng lý thuyết động lực học liên kết vật thể (augmented object dynamics) từ bài báo này để tối ưu hóa mô hình EKF dự đoán quỹ đạo bay của bóng và lập kế hoạch bắt bóng đồng bộ.

---

## 1. Unified MPC Framework làm gì — bức tranh tổng thể

Bài toán: Hoạch định và điều khiển toàn thân (whole-body control) đồng thời cho cả locomotor (bộ phận di chuyển như chân) và manipulator (bộ phận thao tác như cánh tay) khi tương tác vật lý trực tiếp với vật thể môi trường (ví dụ: đẩy/kéo cửa nặng).

```
   High-level Task Goal (e.g., Pull Door)
                    │
                    ▼
   [Augmented Centroidal & Object Dynamics] ◄─── State Feedback (q, v, f_ext)
                    │
                    ▼
   [Optimal Control Problem (OCP) Formulation]
                    │
                    ▼
   [OCS2 Solver - Sequential Linear Quadratic (SLQ)] (Real-time, ~100Hz)
                    │
                    ▼ (Reference joint torques & contact forces)
   [Low-level Whole-Body Tracking Controller] (Operational-Space ID, ~1kHz)
                    │
                    ▼
          [Robot Actuators (Sim/Real)]
```

Hệ thống hoạt động dựa trên triết lý **Hợp nhất mô hình động lực học (Unified Dynamics)** thay vì chia tách cơ học:
1.  **Augmented Robot-Object Model:** Trạng thái của vật thể bị tương tác được gộp trực tiếp vào phương trình trạng thái của hệ thống Robot.
2.  **Optimal Control Problem (OCP):** Thiết lập một bài toán điều khiển tối ưu hóa đa tiếp xúc duy nhất cho toàn bộ hệ thống Arm-Hand-Object.
3.  **OCS2 SLQ Solver:** Giải thuật Sequential Linear Quadratic (SLQ) giải trực tiếp OCP thời gian thực trên máy tính nhúng của robot để xuất ra quỹ đạo lực tiếp xúc tham chiếu.

---

## 2. Từng thành phần kỹ thuật (Những gì phải tái hiện & so sánh)

### 2.1 Phương trình động lực học liên kết (Augmented Dynamics)
Thay vì xem lực tiếp xúc là nhiễu ngoại lực, bài báo gộp động lực học robot và vật thể thành hệ phương trình liên kết:

$$\mathbf{M}_{\text{sys}}(\mathbf{x}) \dot{\mathbf{v}}_{\text{sys}} + \mathbf{C}_{\text{sys}}(\mathbf{x}, \mathbf{v}) \mathbf{v}_{\text{sys}} + \mathbf{G}_{\text{sys}}(\mathbf{x}) = \mathbf{S}^T \boldsymbol{\tau} + \mathbf{J}_c^T \boldsymbol{\lambda}$$

*   **Ý nghĩa:**
    *   $\mathbf{x} = [\mathbf{q}_r^T, \mathbf{q}_o^T]^T$: Vector tọa độ tổng quát tích hợp cả khớp robot ($\mathbf{q}_r$) và bậc tự do của vật thể tương tác ($\mathbf{q}_o$).
    *   $\mathbf{v}_{\text{sys}} = [\mathbf{v}_r^T, \mathbf{v}_o^T]^T$: Vector vận tốc tổng quát liên kết.
    *   $\mathbf{M}_{\text{sys}}, \mathbf{C}_{\text{sys}}, \mathbf{G}_{\text{sys}}$: Các ma trận khối quán tính, Coriolis và trọng lực được mở rộng để chứa cả thuộc tính vật lý của vật thể (khối lượng vật thể, ma sát bản lề).
    *   $\boldsymbol{\lambda}$: Lực tiếp xúc (contact forces) tại các điểm chạm.

### 2.2 OCS2 Solver (Sequential Linear Quadratic - SLQ)
*   Bộ giải tối ưu hóa của OCS2 chuyển đổi bài toán tối ưu phi tuyến thành một chuỗi các bài toán xấp xỉ tuyến tính-bậc hai (Linear-Quadratic approximations) dọc theo quỹ đạo danh nghĩa.
*   Thuật toán SLQ giải quyết ràng buộc tiếp xúc dưới dạng **ràng buộc mềm (soft constraints)** thông qua hàm phạt barrier, cho phép đạt tần số thực thi lên tới $100\,\text{Hz}$ trên máy tính nhúng onboard.

---

## 3. Ánh xạ & So sánh đối chứng sang hệ của mình

### 3.1 Bảng ánh xạ tổng thể

| Thành phần | Unified MPC (Paper gốc) | Hệ thống của mình (ur5dex) |
|---|---|---|
| **Cấu hình Robot** | ANYmal C + Arm (7-DoF) | **UR5 (6-DoF) + DH Hand (19-DoF)** |
| **Vật thể tương tác** | Cửa nặng, Van xoay (Liên kết khớp) | **Bóng bay tự do (Không liên kết)** |
| **Mô hình hóa vật thể** | Tích hợp vào Dynamics ($\mathbf{q}_o$) | **Dự đoán qua bộ lọc khí động học EKF** |
| **Thuật toán OCP** | OCS2 SLQ Solver ($100\,\text{Hz}$) | **cuRobo Planner ($240\,\text{Hz}$) + DRL Compliance** |
| **Bộ điều khiển lực** | Operational Space ID ($\boldsymbol{\tau}$) | **Trở kháng cơ học chủ động ($\mathbf{K}_a$)** |

### 3.2 Điểm tương đồng và Khác biệt cốt lõi (Tư duy phản biện)
1.  **Sự tương đồng về triết lý lực học:** Cả hai hệ thống đều nhận định rằng để thao tác thành công trong môi trường tương tác lực lớn, việc chỉ kiểm soát vị trí hình học (Position tracking) là thất bại. Cần phải kiểm soát lực/trở kháng tiếp xúc trực tiếp.
2.  **Khác biệt về đặc tính tiếp xúc (Switched System vs. Dynamic Catching):**
    *   Unified MPC xử lý tương tác tiếp xúc kéo dài (continuous contact như đẩy cửa, đi bộ). Vì tiếp xúc là cố định hoặc tuần hoàn theo gait schedule, bộ giải SLQ có thể hội tụ tốt.
    *   Tác vụ **bắt bóng động (Dynamic Catching)** của chúng ta là va chạm tức thời (impact collision). Thời gian va chạm cực ngắn ($\sim 0.05\,\text{s}$) và phi tuyến lớn khiến các bộ giải MPC cổ điển dựa trên vi phân quỹ đạo như OCS2 bị sập (singularity) do lực va chạm nhảy bậc thang.
    *   *Giải pháp của chúng ta:* Decouple bài toán. Để **cuRobo** lo IK hình học bám điểm bắt, và nhường việc "hấp thụ lực va chạm tức thời" cho mạng **DRL (PPO)** học chính sách trở kháng mềm ngoại tuyến.

---

## 4. Lộ trình So sánh đối chứng & Rủi ro

1.  **Cột mốc 1: Cấu hình EKF liên kết động lực học:** Cập nhật bộ lọc EKF bám bóng tích hợp cả khối lượng và lực cản không khí của bóng ($C_d$) để tối ưu thời điểm chạm $T_{\text{catch}}$, đóng vai trò như một bộ "tiền MPC".
2.  **Cột mốc 2: Mô phỏng dập va chạm bằng trở kháng:** Đánh giá độ lệch lực va chạm tiếp xúc. So sánh lực phản hồi đỉnh (peak contact force) khi bắt bóng giữa MPC cứng và DRL mềm của chúng ta.

### Rủi ro chính:
*   **OCS2 Integration Overhead:** Tích hợp OCS2 đòi hỏi toàn bộ hệ thống phải viết bằng C++ và khai báo mô hình Switched System phức tạp. Rủi ro trễ tiến độ lớn do đó ta chỉ kế thừa **triết lý tích hợp động lực học vật thể** để xây dựng mô hình mô phỏng Isaac Sim, thay vì giải OCP toàn phần bằng OCS2.
