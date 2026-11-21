# Mô phỏng làn sóng xanh (Green Wave) trên tuyến 5 nút giao N1–N5

Mã nguồn mô phỏng cho **Đề tài 9** (SUMO + Python/TraCI), bám theo thiết kế ở Chương 3–4 của báo cáo.

## Cài đặt

```bash
pip install -r requirements.txt        # eclipse-sumo cài kèm cả sumo, sumo-gui, netconvert
# Nếu đã cài SUMO bản chính thức: đặt biến môi trường SUMO_HOME trỏ tới thư mục SUMO
```

## Cấu trúc

| Tệp | Vai trò |
|---|---|
| `corridor.py` | Hình học tuyến, **thuật toán tính offset (Mục 3.5)**, sinh tệp `corridor.nod/edg/net/rou/add.xml` và `corridor.sumocfg` |
| `runner.py` | Chạy **một** kịch bản qua TraCI, tính 5 chỉ số MOEs, ghi `tripinfo.xml`, `metrics.json` |
| `run_experiments.py` | Chạy **toàn bộ KB1–KB4**, nhiều seed, xuất `results/summary.csv` và bảng so sánh |
| `plot_time_space.py` | Vẽ biểu đồ không gian – thời gian (Hình 2.6 / 3.3) cho KB1 và KB2 |

## Chạy nhanh

```bash
python corridor.py                         # in bảng offset (khớp Bảng 3.5.6: 0, 32, 76, 22, 61)

python runner.py --mode independent        # KB1 - đèn độc lập (đối chứng)
python runner.py --mode greenwave          # KB2 - làn sóng xanh
python runner.py --mode greenwave --gui    # xem bằng sumo-gui

python runner.py --mode greenwave --flow 1500    # KB3: 600 / 900 / 1200 / 1500 xe/h
python runner.py --mode greenwave --cycle 100    # KB4-A: 80 / 90 / 100 s (offset tự tính lại)
python runner.py --mode greenwave --speed 60     # KB4-B: 40 / 50 / 60 km/h (offset tự tính lại)

python run_experiments.py --seeds 3        # toàn bộ KB1-KB4 (khoảng 13 s / lượt trên 1 CPU)
python run_experiments.py --jobs 4         # chạy song song
python plot_time_space.py                  # xuất time_space.png
```

## Các quyết định mô hình hóa (cần nắm khi viết báo cáo)

* **Đèn**: chu kỳ 90 s = xanh chính 50 + vàng 3 + xanh phụ 34 + vàng 3, tức đỏ trục chính = 37 s (đúng Mục 3.4).
  Khi đổi chu kỳ, xanh chính = round(C·50/90), xanh phụ = phần còn lại.
* **Offset**: dùng thuộc tính `offset` của `<tlLogic>`; pha xanh chính tại nút k bắt đầu ở thời điểm `t = O_k (mod C)`.
  Đã kiểm tra bằng TraCI: N2 bắt đầu xanh tại giây 32, N3 tại 76, N4 tại 22, N5 tại 61.
* **KB1** (đối chứng): chu kỳ 90 s như KB2 nhưng offset là bộ ngẫu nhiên cố định (`--kb1-offsets random`, seed 2025
  -> `[0, 71, 10, 82, 61]`) hoặc toàn 0 (`--kb1-offsets zero`, phối hợp đồng thời). Cùng seed => cùng lịch phát xe.
* **Mốc đo**: tâm N1 (x = 0) và tâm N5 (x = 2100 m). Tuyến thẳng theo trục x nên vị trí x chính là cự ly dọc tuyến;
  mạng được tạo với `--offset.disable-normalization` để x của TraCI trùng toạ độ nút.
  Thời gian chờ đèn đỏ *trước* tâm N1 không tính vào TT; chờ ở N2–N5 thì có.
* **Nhóm thống kê**: xe phát trong [300 s, 3000 s] (bỏ 300 s khởi động), theo dõi đến hết hành trình.
  Thông lượng đo trong cửa sổ [300 s, 3600 s]. Số xe chưa hoàn thành được báo cáo riêng.
* **Hàng chờ**: `Qmax (xe)` = tổng số xe dừng (v < 0,1 m/s) trên 15 làn tiếp cận N1–N5 ở mỗi bước, lấy max
  (đúng mô tả `runner.py` ở Mục 3.7); `Q (m)` = chiều dài hàng lớn nhất từ vạch dừng đến đuôi xe dừng cuối.
* **Tốc độ mong muốn**: giữ `speedDev = 0.1` mặc định của SUMO nên đoàn xe tự phân tán dần;
  độ trễ D tính theo `v_des` của từng xe như công thức 2.8. Đặt `speed_dev=0` trong `Config` nếu muốn dòng xe đồng nhất.
* `time-to-teleport = -1` để SUMO không "dịch chuyển" xe khi kẹt (tránh làm sai thông lượng/hàng chờ).
* Chiều ngược N5 → N1 mặc định không có xe (`--reverse-flow` để bật); khi đó đèn chính hai chiều xanh cùng lúc.

## Đầu ra mỗi lượt chạy

`metrics.json`, `tripinfo.xml`, `scenario.json` (cấu hình + offset đã dùng) và `trajectories.csv` (nếu `--trajectories`).

## Kết quả chạy thử (3 seed, trường hợp cơ sở: 1200 xe/h, C = 90 s, 50 km/h)

| Chỉ số | KB1 độc lập | KB2 làn sóng xanh | Thay đổi |
|---|---|---|---|
| Số lần dừng S (lần/xe) | 2,67 | 0,20 | −92,3% |
| Độ trễ D (s/xe) | 76,8 | 19,2 | −75,0% |
| Thời gian hành trình TT (s/xe) | 235,0 | 177,4 | −24,5% |
| Số xe dừng cực đại Qmax (xe) | 56,7 | 18,3 | −67,6% |
| Hàng chờ dài nhất Q (m) | 75,7 | 40,6 | −46,4% |
| Thông lượng TH (xe/h) | 1211 | 1197 | −1,2% |
| Tỷ lệ xe đi hết tuyến không dừng | 0% | 79,8% | |

Đây là số liệu chạy trong môi trường của người viết mã (SUMO cài bằng pip); khi chạy lại trên máy bạn
có thể lệch nhẹ do phiên bản SUMO khác. Chạy `python run_experiments.py --seeds 5` để có số liệu ổn định hơn.
