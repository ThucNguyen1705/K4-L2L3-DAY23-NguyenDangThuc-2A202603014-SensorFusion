# Báo cáo bài nộp — Day 23 Sensor Fusion Lab

> Điền file này rồi commit. Cách nộp: [hướng dẫn nộp](../SUBMISSION.md).

## Thông tin học viên

- Họ tên: Nguyễn Đăng Thực
- MSSV: 2A202603014
- Email: thuc13999@gmail.com
- Link repo (fork): https://github.com/ThucNguyen1705/K4-L2L3-DAY23-NguyenDangThuc-2A202603014-SensorFusion
- Commit hash nộp (`git rev-parse HEAD`): hash của commit CP6 cuối cùng sau khi push (ghi trên LMS)

## Tóm tắt kết quả

- `fusion_mode` (bắt buộc `compare`), `frames`, `segment`, `seed`: `compare`, `[0, 198]` (199 frame),
  `training_segment-1005081002024129653_5313_150_5333_150_with_camera_labels.tfrecord`, `0`
- `detection.precision`, `detection.recall`, `detection.tp/fp/fn`: precision = 0.9701, recall = 0.7004,
  tp/fp/fn = 519 / 16 / 222
- `tracking.lidar.rmse`, `matches`, `sum_sq_err`, `ghost_track_frames`, `missed_gt_frames`, `mean_confirmed_tracks`:
  0.1503 m, 502, 11.3436, 0, 239, 2.5226
- `tracking.fused.rmse`, `matches`, `sum_sq_err`, `ghost_track_frames`, `missed_gt_frames`, `mean_confirmed_tracks`:
  0.1359 m, 502, 9.2668, 0, 239, 2.5226
- Giải thích khác biệt hai mode, đọc RMSE cùng số ghép và ghost/miss: xem bảng và phân tích bên dưới.

| Mode | RMSE (m) | matches | sum_sq_err (m²) | ghost | miss | precision_track | coverage = matches/det_tp |
|---|---|---|---|---|---|---|---|
| lidar | 0.1503 | 502 | 11.3436 | 0 | 239 | 502/502 = 1.000 | 502/519 = 0.967 |
| fused | 0.1359 | 502 | 9.2668 | 0 | 239 | 502/502 = 1.000 | 502/519 = 0.967 |

`rmse_fused − rmse_lidar = −0.0145 m` (≤ 0.05 m). Cả hai mode đạt RMSE ≤ 0.45 m,
`precision_track` ≥ 0.75, `coverage` ≥ 0.70.

**Đọc RMSE cùng matches/ghost/miss:**

- **Camera chỉ đổi độ chính xác, không đổi vòng đời.** `matches`, `ghosts`, `misses`,
  `confirmed` giống hệt nhau ở **cả 199 frame** giữa hai mode (đối chiếu từng cặp
  record `lidar`/`fused` trong `grade_run.log`); chỉ `sum_sq_err` khác. Vì cùng số
  cặp (502) nên so RMSE là công bằng: fused giảm `sum_sq_err` 11.34 → 9.27 m² (−18%),
  RMSE 0.150 → 0.136 m (−9.6%). Fused có sai số nhỏ hơn ở 137/195 frame có ghép,
  lớn hơn ở 58 frame. Theo đoạn frame: 0–49 fused kém hơn (0.144 vs 0.129 m), từ frame
  50 trở đi fused tốt hơn (50–99: 0.136 vs 0.165; 100–149: 0.142 vs 0.161;
  150–198: 0.124 vs 0.139). Đầu segment track còn mới, `P` vận tốc lớn, nên một đo
  2D chỉ có hướng có thể kéo state lệch (ví dụ frame 4: 0.032 vs 0.011 m²); khi
  track đã hội tụ, đo góc từ camera bổ sung thông tin ngang/dọc độc lập với LiDAR.
- **0 ghost:** detector có 16 FP nhưng không FP nào thành track confirmed. Chuỗi FP dài
  nhất là 4 frame liên tiếp (frame 26–29): sinh với score 1/6 rồi 3 hit → 4/6 = 0.667
  < 0.8, chưa đủ để confirmed (cần sinh + 4 hit liên tiếp = 5/6).
- **239 miss** = 222 FN của detector (xe detector không thấy, ví dụ frame 186–190 có
  12 GT hợp lệ nhưng detector chỉ thấy 2–3) + 17 miss do tracker. 17 = 22 − 5:
  - 8 ở frame 0–3: hai xe được sinh ở frame 0, chỉ confirmed từ frame 4 (độ trễ xác nhận).
  - 4 ở frame 44–47: detector bắt đầu thấy xe thứ ba ở frame 44 (GT có từ frame 40),
    track sinh ở frame 44 và được confirmed ở frame 48.
  - 10 ở frame 60–97: detector bắt xe thứ ba chập chờn (`det_tp` dao động 2↔3), chưa
    lần nào đủ 5 frame liên tiếp để track vượt 0.8, nên chỉ confirmed lại từ frame 98.
  - −5 ở frame 50, 55, 56, 58, 188: detector bỏ sót nhưng track confirmed vẫn được
    predict và ghép đúng GT (`matches` > `det_tp`). Đây là lợi ích của tracking so với
    detection từng frame.
- **Giới hạn của đo camera:** đo camera là tâm hộp 2D **ground-truth** FRONT cộng nhiễu
  Gaussian σ = 0.5 px theo seed (`run_lab.py:97`), trong khi `R` của EKF giả định σ = 5 px.
  Mức giảm RMSE vì thế một phần là do nhãn GT 2D "rò" thông tin vào tracker, rồi lại
  được chấm với GT 3D. Kết quả **không** chứng minh một camera detector thật sẽ cải
  thiện tracking. Detector thật có nhiễu lớn hơn, có FP/FN và hộp lệch theo che khuất,
  nên cần đo lại khi thay bằng detector ảnh. Chỉ 1 camera (FRONT) được dùng, nên track
  ngoài FOV FRONT không được camera update.

Chạy từ root repo:

```bash
fusion-run-lab --config student/config/paths.yaml --fusion compare --seed 0
```

`rmse = sqrt(sum_sq_err/matches)` trên vị trí 3D của confirmed tracks ghép
một-một với GT xe trong cửa sổ BEV, gate XY **2.0 m**; `null` nếu không có cặp.
Camera dùng tâm hộp 2D ground-truth FRONT có nhiễu seeded, **không** dùng camera
detector. Kết quả này không đo hiệu quả một perception system độc lập với GT.

`grade_run.log` là JSONL, mỗi `(mode,frame)` đúng một record với các trường:
`mode`, `frame`, `det_tp`, `det_fp`, `det_fn`, `valid_gt`, `confirmed`, `matches`,
`sum_sq_err`, `ghosts`, `misses`. Đảm bảo `matches+ghosts==confirmed` và
`matches+misses==valid_gt`; tổng/trung bình record phải khớp `metrics.json`.
File per-mode `metrics_lidar.json`, `metrics_fused.json`, `grade_run_lidar.log`,
`grade_run_fused.log` được giữ để đối chiếu.

## Giải thích ngắn (Parts E–H — tự viết)

1. **Khác biệt đo lidar 3D và camera 2D trong EKF (`z`, `R`)?**
   - LiDAR: `z = (x, y, z)` tâm hộp 3D (m), `R = diag(0.1², 0.1², 0.1²)` m²
     (`Measurement`, `sensors.py`). `h(x) = R_vs·p + t` tuyến tính nên `H` là
     ma trận hằng 3×6 (khối xoay + 3 cột vận tốc bằng 0). Đo LiDAR quan sát
     trực tiếp cả ba trục vị trí, kể cả độ sâu.
   - Camera: `z = (u, v)` pixel, `R = diag(σ_cam_i², σ_cam_j²) = diag(25, 25)` px²
     (`build_camera_measurement`, `camera_fusion.py:84`). `h(x)` là phép chiếu
     pinhole phi tuyến `u = c_i − f_i·y_s/x_s`, `v = c_j − f_j·z_s/x_s`
     (`camera_fusion.py:79-80`), nên EKF dùng Jacobian `H` 2×6 do platform tính
     tại `x` hiện tại. Camera chỉ đo hướng (góc ngang/dọc), không đo độ sâu:
     hàng Jacobian theo `x_s` nhỏ (∝ 1/x_s²), nên update camera chủ yếu kéo
     vị trí ngang/dọc, còn độ sâu phải dựa vào LiDAR. Đơn vị của `R` khác nhau
     (m² vs px²), và `S = H P Hᵀ + R` đưa bất định của track sang không gian đo
     của từng sensor (`kalman.py:96`).
2. **Vì sao cần gating Mahalanobis trước khi gán?**
   `d² = γᵀ S⁻¹ γ` (`association.py:27-40`) chuẩn hoá innovation theo `S`, tức
   gộp cả bất định của track (`P`) lẫn nhiễu sensor (`R`). Track mới sinh có
   `P` vận tốc 50² nên `S` rộng — một đo cách vài mét vẫn hợp lý; track đã
   hội tụ có `S` hẹp — cùng khoảng cách Euclid có thể là cặp sai. Euclid không
   phân biệt được hai trường hợp đó, và cũng không so được m với pixel. Với
   cặp đúng, `d²` ~ χ² với `dim_meas` bậc tự do, nên cổng
   `d² < chi2.ppf(0.995, dim_meas)` (12.84 cho LiDAR 3D, 10.60 cho camera 2D;
   `association.py:55`) loại các cặp quá xa trước khi greedy chọn cặp cost nhỏ
   nhất. Không có cổng, greedy sẽ gán cả clutter/ghost vào track (kéo lệch
   state và giữ track ma sống), hoặc cướp đo của track khác.
   Cặp ngoài FOV bị loại trước cả khi tính `d²` (`association.py:75`) để
   không chiếu điểm nằm sau camera.
3. **Track-then-fuse hay fuse-then-track?** — **Track-then-fuse.** Chỉ có một
   `TrackManager`; mỗi frame `run_lab.py:200` predict mọi track **một lần**,
   `run_lab.py:202` gán + update LiDAR (AssocL, kết thúc bằng `manage_tracks`
   của LiDAR), rồi `run_lab.py:209` gán + update camera (AssocC) trên cùng danh
   sách track. Không có bước gộp raw point cloud với ảnh trước detection.
   Trên log: cặp record cùng frame của hai mode có `det_tp/det_fp/det_fn/valid_gt` giống nhau (cùng một lượt detection, ví dụ frame 100: `det_tp=3, det_fp=0, det_fn=0` ở cả `lidar` và `fused`), và `confirmed/matches/ghosts/misses` cũng giống nhau ở cả 199 frame. Chỉ `sum_sq_err` khác (frame 100: 0.0601 ở `lidar`, 0.0488 ở `fused`). Nghĩa là camera không tạo/xoá/xác nhận track nào mà chỉ tinh chỉnh state của track do LiDAR tạo ra. Track đầu tiên confirmed ở frame 4 trong cả hai mode (sinh ở frame 0 + 4 hit LiDAR). Nếu là fuse-then-track (gộp đo trước khi tracking), danh sách track và số ghép sẽ phụ thuộc vào camera.
4. **Nếu camera lệch calibration, triệu chứng gì trên innovation/residual?**
   Extrinsic lệch làm `h(x)` chiếu track tới pixel sai một lượng có hệ thống.
   Innovation `γ = z − h(x)` của camera không còn quanh 0 mà có **bias** cùng
   dấu qua nhiều frame và nhiều track (ví dụ lệch yaw → `u` lệch đều về một
   phía, lệch pitch → `v` lệch); `d²` trung bình tăng vượt kỳ vọng χ²₂ (≈2).
   Lệch nhỏ: vẫn qua cổng, EKF bị kéo vị trí ngang về phía sai, rồi lượt LiDAR
   frame sau kéo lại → residual LiDAR cũng tăng và RMSE fused xấu hơn LiDAR.
   Lệch lớn: `d²` vượt ngưỡng 10.60 nên cổng chặn hầu hết đo camera — fused
   suy biến về LiDAR-only (an toàn nhưng mất tác dụng của camera). Vì lỗi có
   hệ thống, gating không phát hiện được lệch nhỏ; cần theo dõi trung bình
   innovation (NIS) để phát hiện.
5. **Vì sao `associate_and_update(..., sensor)` cần sensor tường minh ở frame
   rỗng? Vì sao LiDAR quyết định score/init/delete còn camera chỉ EKF update?**
   Khi `meas_list = []` không thể suy ra sensor từ phần tử đo nào, nhưng
   `manage_tracks` vẫn phải biết lượt này là LiDAR hay camera
   (`association.py:144`). Frame LiDAR rỗng nghĩa là mọi track trong FOV LiDAR
   bị **miss** → trừ score và có thể xoá (`manager.py:111-121`); một lượt camera
   rỗng (hoặc không có nhóm FRONT) thì không được làm gì với vòng đời. Nếu bỏ
   qua `manage_tracks` ở frame rỗng, track ma không bao giờ bị trừ điểm.
   LiDAR quyết định vòng đời vì nó cho vị trí 3D đầy đủ (có độ sâu), phủ
   ±90° phía trước, và là nguồn detection thật của lab; một đo camera 2D không
   xác định được độ sâu nên không khởi tạo được state 3D, FOV FRONT hẹp hơn
   (track ngoài FOV camera sẽ bị "miss" oan), và đo camera trong lab lại là nhãn
   GT có nhiễu — nếu để camera cộng score, nhãn GT sẽ "nuôi" track, làm sai lệch
   đánh giá. Vì thế `handle_updated_track` chỉ cộng score khi `sensor.name ==
   "lidar"` (`manager.py:95`) và `manage_tracks` return ngay với camera
   (`manager.py:111`); camera chỉ gọi `filter_obj.update` để tinh chỉnh state.
6. **Điều kiện xác nhận, giữ confirmed sau miss, điều kiện xóa track.**
   (`track_management.py:48-96`, `window = 6`)
   - Khởi tạo: từ đo LiDAR chưa ghép, `score = 1/6`, `state = "initialized"`,
     `P` vị trí = `R` xoay sang hệ xe, `P` vận tốc = diag(50², 50², 5²).
   - Mỗi lượt LiDAR: hit `+1/6` (tối đa 1); miss trong FOV LiDAR `−1/6`.
     Hit đầu tiên chuyển `initialized → tentative`.
   - Xác nhận khi `score > 0.8`: từ 1/6 cần 4 hit liên tiếp (5/6 ≈ 0.833).
   - Đã `confirmed` thì không bị hạ trạng thái: một miss đưa 1.0 → 0.833, vẫn
     confirmed (`track_management.py:68`).
   - Xoá (OR): `P[0,0] > 9` hoặc `P[1,1] > 9` (bất định > 3 m) bất kể score;
     confirmed có `score < 0.6` (từ 1.0 cần 3 miss liên tiếp → 0.5);
     chưa confirmed có `score <= 0` (track mới sinh mà miss ngay frame sau).
## Bonus (không bắt buộc)

Liệt kê phần bonus đã làm, file bằng chứng trong `student/bonus/` và kết quả chính
(xem [RUBRIC.md](../RUBRIC.md) mục 2). Không làm thì ghi "Không".

- Không

## Khai báo sử dụng AI (bắt buộc)

Ghi rõ, kể cả khi không dùng ("Không dùng AI"). Xem [RULES.md](../RULES.md) mục 2.

- Công cụ đã dùng (ChatGPT, Copilot, Claude, …): Claude Code (Claude Opus 5.5)
- Dùng cho phần nào (hàm, câu hỏi, debug): Hỗ trợ cài đặt môi trường (uv, Python 3.12, torch CPU) và tải dữ liệu; viết Part E (`kalman.py`), G (`camera_fusion.py`), F (`association.py`), H (`track_management.py`) theo các dòng `# vi: TODO`; debug lỗi `float()` trên ma trận 1×1 của NumPy mới trong `mahalanobis_distance`; soạn nháp sáu câu giải thích và phần diễn giải kết quả trong file này.
- Cách bạn đã kiểm tra lại (pytest, chạy Waymo, đối chiếu công thức): Chạy `pytest student/tests -q` sau từng checkpoint (kết quả cuối 128 passed, 0 failed/xfailed); đối chiếu công thức EKF (`F`, `Q`, `S = HPHᵀ+R`, `K = PHᵀS⁻¹`) với test số học trong `test_kalman.py`; chạy thử tracker trên kịch bản tổng hợp (xe 8 m/s, đo nhiễu 0.1 m, mất đo định kỳ, một clutter) để kiểm tra vận tốc hội tụ ≈ 7.8 m/s, clutter bị xoá, track confirmed sống qua frame mất đo; chạy `fusion-run-lab --fusion compare --seed 0` frame 0–198 và đối chiếu mọi số liệu trong báo cáo với `metrics.json`/`grade_run.log`; chạy `python tools/check_submission.py`.

## Checklist nộp

- [x] **Part E–H** trong `workspace/` đã implement; `pytest student/tests -q` không còn `failed`/`xfailed`
- [x] Part A–D: không bắt buộc sửa (hoặc ghi chú nếu bạn đã sửa)
- [x] Lần chạy chấm điểm: `--fusion compare --seed 0`, `frame_start: 0`, `frame_end: 198`
- [x] Đã commit `student/artifacts/metrics*.json` và `student/artifacts/grade_run*.log` (không sửa tay)
- [x] Đã điền đủ file này, gồm khai báo AI
- [x] Không commit dữ liệu Waymo, weights, `paths.yaml`, API key
- [x] `python tools/check_submission.py` báo `KẾT QUẢ: SẴN SÀNG NỘP`
- [ ] Đã push và nộp link repo + commit hash trên LMS ([hướng dẫn nộp](../SUBMISSION.md))
