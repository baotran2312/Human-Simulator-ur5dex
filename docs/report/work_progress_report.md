# BÁO CÁO TIẾN ĐỘ THỰC HIỆN DỰ ÁN (WORK PROGRESS REPORT)
## Dự Án: Human-Simulator-ur5dex
**Thời gian cập nhật báo cáo**: `2026-07-27T14:23:23+07:00`  
**Repository**: [baotran2312/Human-Simulator-ur5dex](https://github.com/baotran2312/Human-Simulator-ur5dex)

---

## I. MỤC TIÊU & TỔNG QUAN HẠNG MỤC

Báo cáo này lưu trữ chi tiết lịch sử thực hiện, các mốc thời gian (Timestamps) chính xác và sản phẩm đầu ra đã hoàn thành tính đến thời điểm hiện tại cho dự án **Human-Simulator-ur5dex** (Hệ thống mô phỏng song sinh kỹ thuật số và điều khiển cánh tay UR5 ghép bàn tay khéo léo DH Robotics).

---

## II. NHẬT KÝ THỰC HIỆN CHI TIẾT & TIMESTAMPS

### 1. Thiết Lập Workspace & Clone Repository
- **Mốc thời gian (Timestamp)**: `2026-07-27T13:55:58+07:00`
- **Nội dung thực hiện**:
  - Khảo sát thư mục làm việc `/home/nhglab/Baro/Human-Simulator-ur5dex`.
  - Thực hiện `git clone` toàn bộ mã nguồn repo chính chủ [Human-Simulator-ur5dex](https://github.com/baotran2312/Human-Simulator-ur5dex) vào workspace.
- **Sản phẩm đầu ra**: Workspace được khởi tạo sạch sẽ, đầy đủ nhánh `main` và tài liệu gốc.

### 2. Đánh Giá Khảo Sát Tài Nguyên Hệ Thống & Dự Án Sẵn Có
- **Mốc thời gian (Timestamp)**: `2026-07-27T13:58:16+07:00` – `2026-07-27T13:59:01+07:00`
- **Nội dung thực hiện**:
  - **Phần cứng**: Ghi nhận GPU NVIDIA GeForce RTX 5060 Ti (16 GB VRAM), RAM 31 GiB, NVMe SSD 621 GB free, Ubuntu 24.04 LTS.
  - **Môi trường Conda**: Đã xác nhận `env_isaacsim` (Python 3.11 + Isaac Sim `omni`), `env_isaaclab` (Python 3.10 + Isaac Lab + cuRobo 0.8.0 + `ur_rtde` 1.6.3 + `pyrealsense2`), và `hil_env` (`MediaPipe` 0.10.14).
  - **Dự án liên quan**: Khảo sát 6 repository có sẵn trên máy (`Human-Mimicking-UR5`, `Seqhandisaac`, `curobo`, `IsaacLab`, `digitaltwin_warehouse`, `ros2_ur5dex`).

### 3. Lập Báo Cáo Đánh Giá Tính Khả Thi Nghiên Cứu (Feasibility Assessment)
- **Mốc thời gian (Timestamp)**: `2026-07-27T13:59:20+07:00`
- **Nội dung thực hiện**:
  - Biên soạn báo cáo đánh giá khả thi chi tiết tại [`docs/report/feasibility_report.md`](file:///home/nhglab/Baro/Human-Simulator-ur5dex/docs/report/feasibility_report.md).
  - Đánh giá khả năng đáp ứng đạt **92% (HOÀN TOÀN KHẢ THI)** đối với đề xuất bài báo Q1 trong [`docs/report/README.md`](file:///home/nhglab/Baro/Human-Simulator-ur5dex/docs/report/README.md).
- **Sản phẩm đầu ra**: File [`docs/report/feasibility_report.md`](file:///home/nhglab/Baro/Human-Simulator-ur5dex/docs/report/feasibility_report.md).

### 4. Cấu Hình Git Identity & Commit Push Đầu Tiên
- **Mốc thời gian (Timestamp)**: `2026-07-27T14:02:31+07:00`
- **Nội dung thực hiện**:
  - Đặt thông tin người dùng Git: `user.name = "baotran2312"`, `user.email = "baotran2312@users.noreply.github.com"`.
  - Thực hiện commit (`2d9c7a0`) và push file báo cáo khả thi lên nhánh `main` của GitHub.

### 5. Cập Nhật Roadmap & Đồng Bộ Repo Remote
- **Mốc thời gian (Timestamp)**: `2026-07-27T14:07:24+07:00`
- **Nội dung thực hiện**:
  - Chạy `git pull origin main` để nạp tài liệu lộ trình [`docs/report/roadmap.md`](file:///home/nhglab/Baro/Human-Simulator-ur5dex/docs/report/roadmap.md).
  - Phân tích yêu cầu **Tuần 1 - Nhiệm vụ 1: Mô phỏng & Thực nghiệm (Physical Simulation)**.

### 6. Cài Đặt Gói Cần Thiết & Xây Dựng Bộ Mã Nguồn Trong `src/` (Tuần 1 Task 1)
- **Mốc thời gian (Timestamp)**: `2026-07-27T14:07:30+07:00` – `2026-07-27T14:10:06+07:00`
- **Nội dung thực hiện**:
  - **Cài đặt thư viện**: `pip install pymodbus` thành công vào môi trường `env_isaaclab`.
  - **Mô phỏng vật lý (`src/sim/`)**:
    - [`src/sim/physics_config.py`](file:///home/nhglab/Baro/Human-Simulator-ur5dex/src/sim/physics_config.py): Định nghĩa tham số vật lý bóng ($m=0.15\text{kg}$, $r=0.035\text{m}$), trọng lực, hệ số ma sát/đàn hồi và cảm biến tiếp xúc.
    - [`src/sim/ball_launcher.py`](file:///home/nhglab/Baro/Human-Simulator-ur5dex/src/sim/ball_launcher.py): Mô-đun phát bóng động tự động tác động lực $F$ và vận tốc $v_0$.
    - [`src/sim/grasp_scene_ball.py`](file:///home/nhglab/Baro/Human-Simulator-ur5dex/src/sim/grasp_scene_ball.py): Script chạy mô phỏng vật lý hoàn chỉnh độc lập trong NVIDIA Isaac Sim.
  - **Driver phần cứng (`src/hardware/`)**:
    - [`src/hardware/dh_hand_modbus.py`](file:///home/nhglab/Baro/Human-Simulator-ur5dex/src/hardware/dh_hand_modbus.py): Driver điều khiển Modbus TCP cho bàn tay khéo léo DH Robotics.
    - [`src/hardware/ur5_rtde_client.py`](file:///home/nhglab/Baro/Human-Simulator-ur5dex/src/hardware/ur5_rtde_client.py): Driver giao tiếp RTDE tần số cao (125Hz-500Hz) cho cánh tay UR5.
- **Sản phẩm đầu ra**: Bộ mã nguồn hoàn chỉnh trong thư mục `src/`.

### 7. Tự Động Push Mã Nguồn Tuần 1 Lên Repo GitHub
- **Mốc thời gian (Timestamp)**: `2026-07-27T14:10:24+07:00`
- **Nội dung thực hiện**:
  - Commit (`d54e804`) với thông điệp `feat(sim): implement Week 1 Task 1 physical simulation scene and ball launcher in src/`.
  - Đã đẩy thành công 10 file mã nguồn mới trong `src/` lên remote repository.

### 8. Thực Hiện Tuần 2 Task 1: Tích Hợp Thuật Toán Trong Môi Trường Mô Phỏng Vật Lý
- **Mốc thời gian (Timestamp)**: `2026-07-27T14:15:08+07:00` – `2026-07-27T14:15:44+07:00`
- **Nội dung thực hiện**:
  - **Bộ lọc EKF dự đoán quỹ đạo bóng (`src/sim/ekf_ball_tracker.py`)**: Lập mô hình động lực học không gian 3D của bóng dưới tác động của trọng lực và lực cản không khí. Tính toán điểm va chạm $P_{\text{int}}$ và thời gian tới $T_{\text{catch}}$.
  - **Giải Động học ngược cuRobo / IK (`src/sim/curobo_ik_solver.py`)**: Tính toán 6 góc khớp UR5 thời gian thực đón điểm $P_{\text{int}}$ với mặt phẳng lòng bàn tay hướng ngược lại vectơ vận tốc bóng $-\hat{v}_{\text{ball}}$.
  - **Điều khiển kẹp mềm tuân thủ lực (`src/sim/compliance_grasp_controller.py`)**: Xây dựng chính sách bắt bóng 2 pha (Phase 1: Pre-shaping, Phase 2: Soft Compliance Enclosure khi lực tiếp xúc $F \ge 0.8\text{N}$).
  - **Script mô phỏng vật lý bắt bóng động (`src/sim/run_dynamic_catching_sim.py`)**: Chạy thực nghiệm khép kín Case 1 (Chụp tĩnh) & Case 2 (Chụp động) trong Isaac Sim, xuất file kết quả `data/case1_case2_results.csv`.
- **Sản phẩm đầu ra**: Bộ mã nguồn thuật toán hoàn chỉnh trong `src/sim/`.

### 9. Tự Động Push Mã Nguồn Thuật Toán Tuần 2 Lên Repo GitHub
- **Mốc thời gian (Timestamp)**: `2026-07-27T14:20:06+07:00`
- **Nội dung thực hiện**:
  - Thực hiện `git pull --rebase origin main` để đồng bộ remote.
  - Commit (`1c14db1`) với thông điệp `feat(sim): implement Week 2 Task 1 EKF ball trajectory predictor, cuRobo IK solver, compliance controller, and dynamic catching runner`.
  - Đẩy thành công toàn bộ mã nguồn Tuần 2 lên remote repository.

---

## III. BẢNG TỔNG HỢP TIẾN ĐỘ HẠNG MỤC (SUMMARY TABLE)

| STT | Hạng Mục | Mốc Thời Gian (Timestamp) | Trạng Thái | File/Sản Phẩm Liên Quan |
| :---: | :--- | :---: | :---: | :--- |
| 1 | Khởi tạo workspace & Git clone | `2026-07-27T13:55:58+07:00` | **Hoàn thành** | Repository Root |
| 2 | Khảo sát hạ tầng & 6 dự án máy | `2026-07-27T13:59:01+07:00` | **Hoàn thành** | NVIDIA RTX 5060 Ti, Isaac Sim, cuRobo |
| 3 | Lập Báo cáo Khả thi (Feasibility) | `2026-07-27T13:59:20+07:00` | **Hoàn thành** | [`docs/report/feasibility_report.md`](file:///home/nhglab/Baro/Human-Simulator-ur5dex/docs/report/feasibility_report.md) |
| 4 | Initial Git Push (Commit `2d9c7a0`) | `2026-07-27T14:02:31+07:00` | **Hoàn thành** | Remote `main` branch |
| 5 | Git Pull & Nạp Roadmap | `2026-07-27T14:07:24+07:00` | **Hoàn thành** | [`docs/report/roadmap.md`](file:///home/nhglab/Baro/Human-Simulator-ur5dex/docs/report/roadmap.md) |
| 6 | Cài `pymodbus` & Viết Code Tuần 1 `src/` | `2026-07-27T14:10:06+07:00` | **Hoàn thành** | [`src/sim/`](file:///home/nhglab/Baro/Human-Simulator-ur5dex/src/sim), [`src/hardware/`](file:///home/nhglab/Baro/Human-Simulator-ur5dex/src/hardware) |
| 7 | Source Code Tuần 1 Git Push (Commit `d54e804`)| `2026-07-27T14:10:24+07:00` | **Hoàn thành** | Remote `main` branch |
| 8 | Viết Code Thuật Toán Tuần 2 `src/sim/` | `2026-07-27T14:15:44+07:00` | **Hoàn thành** | [`src/sim/ekf_ball_tracker.py`](file:///home/nhglab/Baro/Human-Simulator-ur5dex/src/sim/ekf_ball_tracker.py), [`src/sim/curobo_ik_solver.py`](file:///home/nhglab/Baro/Human-Simulator-ur5dex/src/sim/curobo_ik_solver.py), [`src/sim/compliance_grasp_controller.py`](file:///home/nhglab/Baro/Human-Simulator-ur5dex/src/sim/compliance_grasp_controller.py), [`src/sim/run_dynamic_catching_sim.py`](file:///home/nhglab/Baro/Human-Simulator-ur5dex/src/sim/run_dynamic_catching_sim.py) |
| 9 | Source Code Tuần 2 Git Push (Commit `1c14db1`)| `2026-07-27T14:20:06+07:00` | **Hoàn thành** | Remote `main` branch |

---
*Báo cáo được cập nhật tự động vào lúc 2026-07-27T14:23:23+07:00.*
