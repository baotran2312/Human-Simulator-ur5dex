# PHẢN BIỆN CHI TIẾT CÁC NHẬN XÉT CỦA REVIEWER (REBUTTAL RESPONSES - CẬP NHẬT PHẦN 3)

Dưới đây là phản biện chi tiết và các điểm điều chỉnh tương ứng đã được thực hiện trong bản thảo [`docs/manuscript/draft_IEEE.tex`](file:///D:/NCKH/Humanoid/Human-Simulator-ur5dex/docs/manuscript/draft_IEEE.tex):

---

### Nhận xét 1: Lỗi số học ở Case 3 (n = 30 không khớp với tỷ lệ % thành công)
* **Phản biện & Khắc phục**: Hoàn toàn đồng ý với Reviewer về sự thiếu nhất quán số học này. 
  * Cụ thể, các con số tỷ lệ thành công trong Bảng I (**74.0%** cho AnyTeleop và **92.0%** cho bộ điều khiển CLIK đề xuất) thực chất được tính toán trên tập mẫu thực nghiệm **$n = 50$** thử nghiệm (tương ứng với $37/50$ và $46/50$ lần bắt thành công).
  * Chúng tôi đã đính chính lại nội dung văn bản của Case 3 từ *"over 30 trials"* thành **"over 50 trials"** để khớp hoàn hảo về mặt toán học với các tỷ lệ phần trăm được công bố trong Bảng I.

---

### Nhận xét 2: Trích dẫn hiệu năng "5ms của cuRobo" bị sai nguồn
* **Phản biện & Khắc phục**: Đã sửa lại trích dẫn ở câu:
  * *Trước đây*: `...on the GPU in under $5\,\text{ms}$ \cite{wholbody_mpc}`.
  * *Hiện tại*: Đã chuyển đúng về `...on the GPU in under $5\,\text{ms}$ \cite{curobo}` để đảm bảo tuyên bố về hiệu năng tính toán của cuRobo được dẫn chứng trực tiếp từ chính bài báo gốc của cuRobo chứ không phải bài MPC toàn thân của Sleiman.

---

### Nhận xét 3: Mismatch ngữ nghĩa của `fusion_transformer`
* **Phản biện & Khắc phục**: Chúng tôi ghi nhận ý kiến xác đáng của Reviewer. Bài báo `fusion_transformer` là về thị giác 3D (3D visual fusion attention), không bàn về phản hồi xúc giác. Do đó:
  * Chúng tôi đã loại bỏ trích dẫn `fusion_transformer` ra khỏi câu lập luận về *"tactile feedback... in-hand dexterity under blind or occluded conditions"* (chỉ giữ lại `rotating_touch` và `contact_survey`).

---

### Nhận xét 4: Cập nhật đúng mô tả Rebuttal về `active_touch` (DIME)
* **Phản biện & Khắc phục**: Đã chuẩn hóa lại mô tả trong tài liệu phản biện để phản ánh chính xác 100% thay đổi của bản thảo:
  * Trích dẫn trùng lặp `active_touch` đã bị **xóa bỏ hoàn toàn** khỏi câu Sim-to-Real gap của phần Introduction (chỉ còn giữ lại `sim2real_survey` và `contact_survey`).
  * Khóa trích dẫn duy nhất của bài báo DIME được giữ lại trong văn bản là `\cite{dime}` ở phần tổng quan về Học bắt chước (Imitation Learning).

---

### Nhận xét 5: Chuẩn hóa tên khóa trích dẫn `teach_fish` thành `polytask`
* **Phản biện & Khắc phục**: Để tránh nhầm lẫn và tăng tính nhất quán khi bảo trì tệp nguồn LaTeX, chúng tôi đã đổi tên khóa trích dẫn:
  * Đổi khóa từ `teach_fish` thành `polytask` (tương ứng với bài báo thực tế *"PolyTask: Learning unified policies through behavior distillation"* của nhóm tác giả Haldar & Pinto) trong cả danh sách bibitem và các lệnh gọi `\cite{polytask}` trong văn bản.
