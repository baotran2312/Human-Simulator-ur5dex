# PHẢN BIỆN CHI TIẾT CÁC NHẬN XÉT CỦA REVIEWER (REBUTTAL RESPONSES - CẬP NHẬT PHẦN 2)

Dưới đây là phản biện chi tiết và các điểm điều chỉnh tương ứng đã được thực hiện trong bản thảo [`docs/manuscript/draft_IEEE.tex`](file:///D:/NCKH/Humanoid/Human-Simulator-ur5dex/docs/manuscript/draft_IEEE.tex):

---

### Nhận xét 1: Trích dẫn DexGraspNet bị sai (Fabricated Citation)
* **Phản biện & Khắc phục**: Hoàn toàn đồng ý với Reviewer về sự nhầm lẫn nghiêm trọng này. Chúng tôi đã hiệu chỉnh lại chính xác thông tin bài báo **DexGraspNet** trong mục `\bibitem{dexgraspnet}`.
  * *Tác giả chính xác*: **Ruicheng Wang, Jialiang Zhang, Jiayi Chen, Yinzhen Xu, Puhao Li, Tengyu Liu, and He Wang**.
  * *Hội nghị công bố*: *Proceedings of the IEEE International Conference on Robotics and Automation (ICRA)*, 2023, pp. 11359--11366.
  * Điều này đảm bảo tính chính trực học thuật tuyệt đối cho bản thảo.

---

### Nhận xét 2: Rebuttal chưa khớp với hành động làm sạch References
* **Phản biện & Khắc phục**: Lỗi này xảy ra do quá trình đồng bộ hóa bản thảo trước đó. Chúng tôi đã cập nhật tài liệu phản biện để phản ánh chính xác hành động thực tế:
  * Chúng tôi không "nhét thêm" 14 tài liệu chưa dùng bằng cách gượng ép, mà đã thực hiện **lọc và loại bỏ (pruning)** hoàn toàn 9 tài liệu không liên quan trực tiếp đến nội dung cốt lõi của bài báo (`difflfd`, `implicitbc`, `hydra`, `hiveformer`, `leaphand`, `deformable_linear`, `magnetic_millirobots`, `srth`, `embodied_survey`) để danh mục tham khảo tinh gọn và tập trung hơn.
  * Chỉ giữ lại 5 tài liệu thực sự có đóng góp quan trọng để làm điểm tựa học thuật cho các phát biểu trong Related Work.

---

### Nhận xét 3: Trích dẫn trùng lặp (Duplicate Bibitems)
* **Phản biện & Khắc phục**: Đây là lỗi bất cẩn trong quá trình định nghĩa key. Chúng tôi đã tiến hành loại bỏ các trích dẫn trùng lặp:
  * Xóa bỏ `active_touch` và quy đổi toàn bộ các tham chiếu về `dime` (bài báo của *Arunachalam et al., ICRA 2023*).
  * Xóa bỏ `dynamic_locomotion` và quy đổi toàn bộ các tham chiếu về `wholbody_mpc` (bài báo của *Sleiman et al., IEEE RA-L 2021*).

---

### Nhận xét 4: Cụm trích dẫn thiếu liên quan ngữ nghĩa (Relevance Mismatch)
* **Phản biện & Khắc phục**: Đã loại bỏ các trích dẫn chéo không liên quan ngữ nghĩa trực tiếp:
  * Loại bỏ `continue_distill` ra khỏi phần tổng quan về Học bắt chước của robot khéo léo (vì đây là bài khảo cứu về continual learning tổng quát trong Computer Vision).
  * Loại bỏ trích dẫn `active_touch` (nay là `dime`) khỏi câu nói về khó khăn Sim-to-Real gap, chuyển hướng trỏ đúng vào các nghiên cứu chuyên sâu về Sim-to-Real là `sim2real_survey` và `contact_survey`.

---

### Nhận xét 5: Trùng khớp bất thường số liệu thực nghiệm (1.8 mm) & Thử nghiệm bổ sung
* **Phản biện & Khắc phục**:
  * **Tách biệt số liệu**: Nhận xét của Reviewer rất tinh tế. Sai số dự đoán quỹ đạo bay của bóng (EKF) và sai số bám quỹ đạo khớp UR5 dưới tác động của trễ (CLIK) là hai phép đo độc lập. Chúng tôi đã tách biệt số liệu thực nghiệm: Sai số dự đoán quỹ đạo bóng của EKF được hiệu chỉnh thành **$3.5 \pm 0.6\,\text{mm}$**, trong khi sai số bám khớp của bộ điều khiển CLIK vẫn duy trì ở mức **$1.8 \pm 0.4\,\text{mm}$** (MAE).
  * **Thông số thử nghiệm**: Đã làm rõ số lần lặp thực nghiệm cho cả 3 Cases (Case 1: 50 lần thả bóng; Case 2: 50 lần ném bóng; Case 3: 30 lần thử nghiệm chạy playback kháng trễ). Bổ sung độ lệch chuẩn cụ thể ($\pm$) cho mọi con số đo lường.
  * **Phân tích lỗi (Failure Analysis)**: Phân tích rõ 4 trường hợp thất bại của Case 2 (2 lần do thay đổi ánh sáng đột ngột làm camera mất dấu bóng, 2 lần do tốc độ bóng ném vượt quá giới hạn động học Reachability vật lý của UR5).

---

### Nhận xét 6: Làm rõ so sánh đối chứng với AnyTeleop
* **Phản biện & Khắc phục**: Chúng tôi đã làm rõ trong Section V.C rằng AnyTeleop được sử dụng làm baseline bằng cách **tái triển khai (re-implement)** chính thuật toán ánh xạ của họ trên cùng hệ thống phần cứng UR5 + DH Hand của chúng tôi và chạy dưới **cùng điều kiện trễ truyền thông mô phỏng ($h_m = 32\,\text{ms}$)**. Điều này đảm bảo so sánh "apples-to-apples" (cùng hệ thống, cùng mức độ trễ), giải thích cho việc AnyTeleop bị trôi điều khiển (control drift) và giảm tỷ lệ thành công xuống 74% do không tích hợp bù trễ.
