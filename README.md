# camsense_x1_web — xem ảnh quét lidar Camsense X1 trên trình duyệt bằng ESP32

Chương trình mẫu ESP-IDF: ESP32 đọc lidar Camsense X1 qua UART, gom vòng quét
360° và phát web server hiển thị radar qua WiFi. Cắm dây, nạp code, mở trình
duyệt trên điện thoại là thấy ảnh quét.

Bài hướng dẫn: https://vinexus.dev/blog/doc-lidar-camsense-x1-esp32/

## Cần chuẩn bị

| Món | Ghi chú |
|---|---|
| ESP32 DevKit (WROOM) | Module WROVER: xem mục Nối dây |
| Lidar Camsense X1 | |
| Nguồn 5V ≥ 500mA | Cấp cho động cơ quay của lidar |
| ESP-IDF v5.x hoặc v6.x | Đã kiểm tra trên v6.0.2 |

## Nối dây

| Dây lidar | ESP32 |
|---|---|
| **TX** | **GPIO 16** |
| **5V** | 5V / VIN |
| **GND** | GND |

Chân TX có mức logic 3.3V, nối thẳng vào GPIO. Module WROVER dùng GPIO 16/17
cho PSRAM — chọn chân khác (ví dụ GPIO 4) và sửa `LIDAR_RX_PIN` trong
`main/main.c`.

## Nạp và chạy

```bash
git clone https://github.com/dungtsbb/camsense_x1_web.git
cd camsense_x1_web
# Sửa WIFI_SSID và WIFI_PASS ở đầu main/main.c
idf.py set-target esp32
idf.py -p /dev/ttyUSB0 flash monitor
```

Màn hình `monitor` in địa chỉ trang web và trạng thái mỗi giây:

```
I (2870) x1: Mở trình duyệt tại: http://192.168.1.42
I (4000) x1: vòng quét=28  gói sai checksum=0  rpm=324
```

Mở địa chỉ đó bằng trình duyệt trên thiết bị cùng mạng WiFi — ảnh quét 360°
cập nhật liên tục.

## Dùng dữ liệu trong chương trình của bạn

Web server có endpoint JSON trả về vòng quét mới nhất:

```
GET /scan  →  {"rpm":324,"seq":28,"err":0,"d":[360 khoảng cách, mm]}
```

Phần tử thứ `i` của mảng `d` là khoảng cách ở góc `i` độ; `0` nghĩa là không
có vật ở hướng đó. Có thể đọc từ Python, Node, hay bất kỳ ứng dụng nào trong
cùng mạng.

Trong firmware, vòng quét hoàn chỉnh nằm ở buffer `s_complete` (360 ô, mỗi ô
1°) — đặt logic xử lý của bạn (tránh vật cản, đo khoảng cách...) đọc từ đây,
sao chép ra bản riêng trong critical section giống hàm `scan_get()`.

## Các hằng số cấu hình

| Hằng số (`main/main.c`) | Mặc định | Ý nghĩa |
|---|---|---|
| `WIFI_SSID` / `WIFI_PASS` | — | Mạng WiFi để ESP32 kết nối |
| `LIDAR_RX_PIN` | 16 | Chân ESP32 nối với TX của lidar |
| `LIDAR_BAUD` | 115200 | Tốc độ UART của lidar |
| `SCAN_BINS` | 360 | Số ô góc mỗi vòng (1°/ô) |

## Xử lý sự cố

| Hiện tượng | Kiểm tra |
|---|---|
| `vòng quét=0` | Dây TX lidar phải vào GPIO 16; lidar có quay không (nguồn 5V) |
| `gói sai checksum` tăng đều | Nguồn 5V và dây GND |
| Không vào được trang web | Điện thoại và ESP32 cùng mạng WiFi; đúng `WIFI_SSID`/`WIFI_PASS` |
| Ảnh quét bị xoay | Cộng offset vào biến `rad` trong `main/index.html` theo hướng lắp lidar |

## Cấu trúc

```
camsense_x1_web/
├── CMakeLists.txt
├── sdkconfig.defaults
└── main/
    ├── CMakeLists.txt
    ├── main.c          ← đọc lidar + web server
    └── index.html      ← trang radar, nhúng vào firmware
```

## License

MIT
