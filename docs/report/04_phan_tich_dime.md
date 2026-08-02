# Phân tích DIME (Dexterous Imitation Made Easy) & Kế hoạch so sánh đối chứng / tích hợp sang UR5 + tay DH (Isaac Sim & Real)

> Nguồn: `docs/literature_review/Dexterous Imitation Made Easy A Learning-Based Framework for Efficient Dexterous Manipulation.md`
> (Arunachalam et al. 2023, ICRA 2023, NYU). Project page: <https://nyu-robot-learning.github.io/dime/>
>
> **Trạng thái mã nguồn (kiểm tra 2026-08-02):** DIME **đã công bố toàn bộ mã nguồn** và dữ liệu trình diễn chuyên nghiệp tại:
> - **DIME Git Repository:** <https://github.com/nyu-robot-learning/dime>
> Thư viện tích hợp tốt bộ thư viện handpose perception của MediaPipe để trích xuất dữ liệu tay người thời gian thực.
>
> → Ta sẽ sử dụng cấu trúc phối hợp dữ liệu trình diễn (Demonstration Guidance) của DIME để thiết kế bộ kích hoạt chuyển động khép ngón và tăng tốc độ học PPO cho bàn tay DH.

---

## 1. DIME làm gì — bức tranh tổng thể

Bài toán: Làm thế nào huấn luyện thành công các tác vụ thao tác khéo léo trong lòng bàn tay (in-hand manipulation như xoay, lật vật thể) một cách tiết kiệm mẫu (sample-efficient) mà không đòi hỏi hệ thống camera phức tạp hay thiết bị teleop đắt tiền.

```
  Human Operator (1x RGB Camera) ─────► MediaPipe Hand Tracker
                                              │
                                              ▼ (Fingertip 2D Coordinates)
                                     Geometric Retargeting (to 3D Robot Frame)
                                              │
                                              ▼
                               [Demonstrations Database (Zarr)]
                                              │
                       ┌──────────────────────┴──────────────────────┐
                       ▼ (Simulation Training)                       ▼ (Real Robot Execution)
            DAPG (Policy Gradient + Demos)                VINN (Visual Nearest Neighbors)
                       │                                             │
                       ▼                                             ▼
          [Allegro Hand Policy (Sim)]                    [Allegro Hand Controller (Real)]
```

Hệ thống hoạt động dựa trên quy trình 2 giai đoạn:
1.  **Thu thập dữ liệu trình diễn dễ dàng (DIME Teleop):** Người vận hành chỉ cần khua tay trước 1 camera thường (RGB). MediaPipe trích xuất tọa độ 2D của các đầu ngón tay và ánh xạ hình học sang tọa độ 3D trong hệ tọa độ robot để ghi lại file Zarr.
2.  **Huấn luyện & Thực thi chính sách (Policy Learning):**
    *   *Trong giả lập:* Sử dụng thuật toán DAPG (Demonstration Augmented Policy Gradient) để tích hợp dữ liệu trình diễn vào hàm mất mát chính sách của RL, giúp giải quyết bài toán thưa thớt reward.
    *   *Trên robot thật:* Sử dụng thuật toán VINN (Visual Imitation through Nearest Neighbors) phi tham số để truy vấn trực tiếp hành động từ cơ sở dữ liệu trình diễn dựa trên ảnh quan sát hiện tại (KNN matching).

---

## 2. Từng thành phần kỹ thuật (Những gì phải tái hiện & so sánh)

### 2.1 Thuật toán tối ưu hóa DAPG (Demonstration Augmented Policy Gradient)
DAPG cải tiến thuật toán chính sách REINFORCE truyền thống bằng cách bổ sung một số hạng tối ưu hành vi bắt chước (Behavior Cloning loss) được scale theo trọng số suy giảm:

$$g(\theta) = \nabla_{\theta} J(\theta) + \lambda_t \sum_{(s, a) \in \mathcal{D}_{\text{demo}}} \nabla_{\theta} \log \pi_{\theta}(a | s)$$

$$\lambda_t = \lambda_0 \gamma^t$$

*   **Ý nghĩa:**
    *   $\nabla_{\theta} J(\theta)$: Gradient chính sách tính từ RL (dựa trên reward môi trường thu được).
    *   $\mathcal{D}_{\text{demo}}$: Tập hợp các cặp trạng thái - hành động chuyên gia (expert demonstrations) thu thập từ teleoperation.
    *   $\lambda_t$: Trọng số phạt bắt chước, tự động giảm dần theo thời gian (epoch $t$) qua hệ số suy giảm $\gamma < 1$.
    *   *Vai trò:* Giai đoạn đầu, gradient BC $\lambda_t$ dẫn dắt chính sách nhanh chóng học được tư thế khép ngón chuẩn của con người. Giai đoạn sau khi robot đã biết bắt bóng sơ bộ, gradient RL $\nabla_{\theta} J(\theta)$ sẽ chiếm ưu thế để tinh chỉnh lực tiếp xúc tối ưu.

### 2.2 VINN (Visual Imitation through Nearest Neighbors)
*   Trên robot thật, DIME chứng minh rằng không cần mạng neural sâu để suy luận. 
*   Một mạng encoder (như ResNet-18 tự giám sát) mã hóa hình ảnh quan sát hiện tại thành một vector đặc trưng (latent code). 
*   Sau đó, giải thuật tìm kiếm K-lân cận gần nhất (KNN) sẽ đối chiếu vector này với cơ sở dữ liệu trình diễn để lấy ra hành động tương ứng $\mathbf{a}_t$.

---

## 3. Ánh xạ & So sánh đối chứng sang hệ của mình

### 3.1 Bảng ánh xạ tổng thể

| Thành phần | DIME (Paper gốc) | Hệ thống của mình (ur5dex) |
|---|---|---|
| **Tay máy khéo léo** | Allegro Hand (16-DoF, Allegro) | **DH Robotics Hand (19-DoF, underactuated)** |
| **Nguồn Demonstrations** | Teleoperation từ tay người (MediaPipe) | **FSM Scripted Trajectory (Tự động sinh)** |
| **Độ phức tạp Teleop** | Thấp (1 camera RGB) | **Cực thấp (Không cần operator, chạy code auto)** |
| **Giải thuật RL Sim** | DAPG (PPO + BC loss) | **PPO + Residual Compliance Control** |
| **Tác vụ thử nghiệm** | Xoay quả bóng, lật cốc (Chậm, tĩnh) | **Chụp bóng bay tự do tốc độ cao (Động)** |

### 3.2 Điểm tương đồng và Khác biệt cốt lõi (Tư duy phản biện)
1.  **Sự tương đồng về giải quyết bài toán khám phá (Exploration):** Cả DIME và hệ thống trở kháng dư của chúng ta đều đồng ý rằng việc dùng RL thuần để học chuyển động khéo léo từ con số 0 là không khả thi. Phải "bơm" tri thức bắt chước chuyển động (demonstration/heuristic) vào để định hướng.
2.  **Sự khác biệt về cách tích hợp dữ liệu mẫu (BC Loss vs. Residual Scripted):**
    *   DIME sử dụng bộ giải DAPG để ép mạng neural học lại quỹ đạo của con người thông qua loss hàm log. Điều này đòi hỏi mạng phải học cả quỹ đạo hình học ngón tay, ngốn nhiều thời gian train sim (mất khoảng 2 ngày giả lập).
    *   *Phương pháp trở kháng dư của chúng ta:* Chúng ta không ép mạng DRL học chuyển động khép ngón hình học qua hàm loss BC. Chúng ta **gán cứng chuyển động khép ngón** bằng một bộ Heuristic Scripted đơn giản dựa trên khoảng cách. DRL chỉ cần học cách **tinh chỉnh độ lệch trở kháng (residual stiffness)**. Nhờ đó, bài toán học được thu hẹp về không gian tối ưu hóa lực va chạm liên tục, giảm thời gian train từ **48 tiếng** (của DIME) xuống **dưới 2 tiếng**.

---

## 4. Lộ trình So sánh đối chứng & Rủi ro

1.  **Cột mốc 1: Xây dựng bộ sinh dữ liệu tự động (Auto-demonstrations):** Sử dụng FSM Scripted khép ngón tay bắt bóng thành công để tự động ghi lại dataset quỹ đạo dạng `.npz` hoặc `.zarr` giống DIME.
2.  **Cột mốc 2: So sánh hiệu quả huấn luyện:** So sánh thời gian hội tụ và độ mượt mà của lực tiếp xúc giữa thuật toán DAPG bắt chước vị trí khớp (kiểu DIME) và thuật toán Trở kháng dư (đề xuất của chúng ta).

### Rủi ro chính:
*   **MediaPipe Noise:** Khối perception của DIME (MediaPipe) có độ trễ lớn và độ nhiễu cao khi ngón tay che khuất lẫn nhau trong quá trình bắt bóng động. Vì vậy, ta ưu tiên sử dụng dữ liệu ground-truth từ Isaac Sim để huấn luyện trong sim nhằm tránh nhiễu perception phá hỏng chính sách DRL.
