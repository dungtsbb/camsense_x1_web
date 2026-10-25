# camsense_x1_web — LiDAR Camsense X1 + ESP32, bản đồ 2D trên trình duyệt

ESP32 đọc LiDAR Camsense X1 qua UART và tự phát WiFi. Kết nối điện thoại hoặc
máy tính vào WiFi của ESP32, mở trình duyệt là thấy bản đồ quét 360° cập nhật
liên tục. Có 2 bản, chọn theo môi trường bạn quen dùng:

| Thư mục | Nền tảng |
|---|---|
| [`arduino/`](arduino/esp32_camsense_x1_radar) | Arduino IDE / Arduino CLI — không cần cài thư viện ngoài |
| [`esp-idf/`](esp-idf) | ESP-IDF v5.x / v6.x |

Hai bản cùng giao diện web và cùng API.

Bài hướng dẫn: https://vinexus.dev/blog/doc-lidar-camsense-x1-esp32/

## Nối dây

| Camsense X1 | ESP32 |
|---|---|
| **5V** | 5V / VIN |
| **GND** | GND |
| **TX** | **GPIO 16** |

Chân TX mức logic 3.3V, nối thẳng vào ESP32. Module WROVER: đổi sang chân khác
(ví dụ GPIO 4) và sửa `LIDAR_RX_PIN`.

## Nạp chương trình

**Arduino IDE:** mở `arduino/esp32_camsense_x1_radar/esp32_camsense_x1_radar.ino`,
chọn board ESP32 Dev Module, Upload.

**Arduino CLI:**

```bash
git clone https://github.com/dungtsbb/camsense_x1_web.git
cd camsense_x1_web
arduino-cli compile --fqbn esp32:esp32:esp32 arduino/esp32_camsense_x1_radar
arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32 arduino/esp32_camsense_x1_radar
```

**ESP-IDF:**

```bash
git clone https://github.com/dungtsbb/camsense_x1_web.git
cd camsense_x1_web/esp-idf
idf.py set-target esp32
idf.py -p /dev/ttyUSB0 flash monitor
```

## Xem bản đồ

1. Kết nối vào WiFi **`ESP32_LiDAR_X1`**, mật khẩu **`12345678`**.
2. Mở trình duyệt: **http://192.168.4.1**
3. Chọn chế độ vẽ: điểm (point cloud), đa giác (đường bao phòng) hoặc tia quét.

Serial Monitor (115200) in trạng thái mỗi 2 giây:

```
vong quet=80  goi sai checksum=0  rpm=315
```

## API dữ liệu

```
GET /scan  →  {"rpm":315,"seq":80,"err":0,"d":[1240,1245,0,890,...]}
```

`d` gồm 360 phần tử — phần tử thứ `i` là khoảng cách (mm) ở góc `i` độ, `0` là
không có vật. Đọc được từ bất kỳ thiết bị nào kết nối vào WiFi của ESP32.

## Cấu hình

| Hằng số | Mặc định | Ý nghĩa |
|---|---|---|
| `AP_SSID` / `AP_PASS` | `ESP32_LiDAR_X1` / `12345678` | Tên và mật khẩu WiFi ESP32 phát ra |
| `LIDAR_RX_PIN` | 16 | Chân ESP32 nối với TX của LiDAR |
| `LIDAR_BAUD` | 115200 | Tốc độ UART |
| `SCAN_BINS` | 360 | Số ô góc mỗi vòng (1°/ô) |

## Xử lý sự cố

| Hiện tượng | Kiểm tra |
|---|---|
| `vong quet=0`, `rpm=0` | Dây TX vào GPIO 16; LiDAR có quay không |
| `rpm` dưới 250, quay giật | Nguồn 5V yếu — dùng cáp USB tốt hoặc nguồn 5V ngoài (chung GND) |
| `goi sai checksum` tăng liên tục | Nối chung GND; dây TX chắc chắn |
| Bản đồ bị xoay | Cộng góc bù vào biến `rad` trong hàm `renderMap()` của trang HTML |

## License

MIT
