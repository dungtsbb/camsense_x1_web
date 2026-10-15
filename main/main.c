/*
 * Camsense X1 + ESP32: đọc lidar qua UART và vẽ ảnh quét lên web ngay trên ESP32.
 *
 * Toàn bộ chương trình trong một file:
 *   1. Kết nối WiFi (chế độ station)
 *   2. Task đọc UART, kiểm checksum, gom gói thành một vòng quét 360°
 *   3. Web server: "/" trả trang HTML, "/scan" trả dữ liệu quét dạng JSON
 *
 * Nối dây — Camsense X1 chỉ có 3 dây:
 *   TX lidar -> GPIO16 (UART2 RX)      5V -> 5V      GND -> GND
 *
 * Build:  idf.py set-target esp32 && idf.py build flash monitor
 */
#include <stdio.h>
#include <string.h>

#include "driver/uart.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

/* ==== SỬA HAI DÒNG NÀY CHO ĐÚNG WIFI CỦA BẠN ==== */
#define WIFI_SSID  "TenWifiCuaBan"
#define WIFI_PASS  "MatKhauWifi"

#define LIDAR_UART      UART_NUM_2
#define LIDAR_RX_PIN    16
#define LIDAR_BAUD      115200   /* Camsense X1 cố định 115200, không đổi được */

static const char *TAG = "x1";

/* ---------------------------------------------------------------------------
 * 1. Giao thức Camsense X1
 *
 * Lidar tự phát liên tục ngay khi có điện — không có lệnh khởi động, không có
 * chân điều khiển động cơ. Cắm vào là nó quay.
 *
 * Mỗi gói dài đúng 36 byte và chứa 8 điểm đo:
 *
 *   0x55 0xAA 0x03 0x08 | speed(2) | góc đầu(2)
 *                       | 8 × [khoảng cách(2) + chất lượng(1)]
 *                       | góc cuối(2) | checksum(2)
 *
 * Mọi số nhiều byte đều little-endian — trùng với ESP32, nên chỉ cần khai báo
 * struct packed là đọc được, không phải ghép byte tay.
 *
 * BA CHỖ DỄ SAI, vì X1 không theo quy ước của phần lớn lidar khác:
 *   - Góc KHÔNG phải đơn vị 0.01°. Công thức: độ = raw/64 − 640.
 *   - Tốc độ raw = Hz × 3840, nên vòng/phút = raw/64.
 *   - Kiểm toàn vẹn KHÔNG phải CRC mà là tổng dịch bit 15-bit (xem bên dưới).
 * ------------------------------------------------------------------------- */
#define PKT_SIZE    36
#define PKT_POINTS  8
#define ANGLE_ZERO  0xA000   /* raw ứng với 0°; 0xA000/64 = 640 */

/* X1 không có bit "không hợp lệ" riêng cho khoảng cách, nên phải tự chặn giá trị
 * vô lý. Tầm thật của X1 khoảng 8m; cái gì xa hơn nhiều là rác. */
#define DIST_SANE_MAX_MM 12000

typedef struct __attribute__((packed)) {
    uint16_t distance_mm;
    uint8_t  quality;       /* độ mạnh tín hiệu 0..255 */
} sample_raw_t;

typedef struct __attribute__((packed)) {
    uint8_t      start_byte0;   /* 0x55 */
    uint8_t      start_byte1;   /* 0xAA */
    uint8_t      start_byte2;   /* 0x03 */
    uint8_t      samples;       /* 0x08 — số điểm mỗi gói */
    uint16_t     speed;         /* Hz × 3840 */
    uint16_t     start_angle;
    sample_raw_t sample[PKT_POINTS];
    uint16_t     end_angle;
    uint16_t     checksum;      /* 15-bit */
} x1_packet_t;

/* Nếu trình biên dịch chèn byte đệm thì mọi phép đọc sau đây đều lệch.
 * Chốt cứng kích thước ngay lúc biên dịch, thay vì đi tìm lỗi lúc chạy. */
_Static_assert(sizeof(x1_packet_t) == PKT_SIZE, "gói phải đúng 36 byte");

/* Checksum 15-bit: gộp từng word 16-bit little-endian, mỗi bước dịch trái tổng
 * đang gom 1 bit rồi cộng word mới; cuối cùng gập 32-bit xuống 15-bit.
 * Phủ 34 byte đầu (17 word), không tính 2 byte checksum.
 *
 * Đây KHÔNG phải CRC. Đừng mang hàm CRC8/CRC16 nào về dùng — sẽ không bao giờ
 * khớp, và biểu hiện giống hệt đứt dây: lidar "im lặng". */
static uint16_t x1_checksum(const uint8_t *buf, size_t len_bytes)
{
    uint32_t chk32 = 0;
    for (size_t i = 0; i < len_bytes / 2; i++) {
        uint16_t word = (uint16_t)buf[2 * i] | ((uint16_t)buf[2 * i + 1] << 8);
        chk32 = (chk32 << 1) + word;
    }
    uint16_t sum = (uint16_t)((chk32 & 0x7FFF) + (chk32 >> 15));
    return sum & 0x7FFF;
}

static bool packet_valid(const uint8_t *pkt)
{
    uint16_t stored = (uint16_t)pkt[PKT_SIZE - 2] |
                      ((uint16_t)pkt[PKT_SIZE - 1] << 8);
    return x1_checksum(pkt, PKT_SIZE - 2) == stored;
}

/* Đổi giá trị góc thô sang độ.
 *
 * Trừ trong miền SỐ NGUYÊN 16-bit trước khi chia: gói vắt qua mốc 0° có raw nhỏ
 * hơn ANGLE_ZERO, trừ trong miền số thực sẽ ra góc âm rất lớn. Số nguyên 16-bit
 * tự quấn vòng đúng chỗ ta cần. */
static float raw_to_deg(uint16_t raw)
{
    return (float)(uint16_t)(raw - ANGLE_ZERO) / 64.0f;
}

/* ---------------------------------------------------------------------------
 * 2. Gom gói thành một vòng quét 360°
 *
 * Mỗi gói chỉ là một cung ~7°. X1 gửi khoảng 50 gói mỗi vòng × 8 điểm = ~400
 * điểm/vòng. Ta chia vòng tròn thành 360 ô, mỗi ô 1°, rồi đổ điểm vào ô tương ứng.
 * ------------------------------------------------------------------------- */
#define SCAN_BINS 360

typedef struct {
    uint16_t dist_mm[SCAN_BINS];
    uint8_t  quality[SCAN_BINS];
    uint16_t rpm;
    uint32_t seq;
} scan_t;

/* Buffer kép: task lidar ghi vào s_building, web server đọc s_complete.
 * Dùng chung một buffer thì trang web sẽ bắt được ảnh nửa cũ nửa mới và hiện ra
 * một vệt rách chạy vòng quanh màn hình. */
static scan_t s_building, s_complete;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static volatile bool s_has_scan = false;
static volatile uint32_t s_scan_count, s_err_count;

static float s_last_start_deg = -1.0f;

static void finish_scan(void)
{
    /* Chỉ là memcpy ~1.5KB (vài µs), đủ ngắn để dùng critical section.
     * Việc nặng thì phải dùng mutex — khoá kiểu này chặn cả bộ định thời. */
    taskENTER_CRITICAL(&s_lock);
    s_building.seq = ++s_scan_count;
    memcpy(&s_complete, &s_building, sizeof(scan_t));
    memset(&s_building, 0, sizeof(scan_t));
    taskEXIT_CRITICAL(&s_lock);
    s_has_scan = true;
}

static void handle_packet(const uint8_t *raw)
{
    /* memcpy chứ không ép con trỏ: buffer đệm không đảm bảo địa chỉ chẵn, mà
     * ESP32 đọc uint16 ở địa chỉ lẻ sẽ lấy sai dữ liệu. */
    x1_packet_t p;
    memcpy(&p, raw, sizeof(p));

    float start = raw_to_deg(p.start_angle);
    float end   = raw_to_deg(p.end_angle);
    float span  = end - start;
    if (span < 0.0f) {
        span += 360.0f;      /* gói vắt qua mốc 0° */
    }
    float step = span / (PKT_POINTS - 1);

    /* Lidar không có cờ "hết vòng". Góc đầu gói nhỏ hơn gói trước nghĩa là kim
     * vừa quay qua mốc 0°, tức xong một vòng. */
    if (s_last_start_deg >= 0.0f && start < s_last_start_deg) {
        finish_scan();
    }
    s_last_start_deg = start;

    s_building.rpm = (uint16_t)(p.speed / 64);   /* Hz×3840 -> vòng/phút */

    for (int i = 0; i < PKT_POINTS; i++) {
        uint16_t d = p.sample[i].distance_mm;
        if (d == 0 || d > DIST_SANE_MAX_MM) {
            continue;        /* không có vật phản xạ, hoặc giá trị rác */
        }
        float ang = start + step * i;
        while (ang >= 360.0f) ang -= 360.0f;
        while (ang < 0.0f)    ang += 360.0f;

        int bin = (int)(ang + 0.5f) % SCAN_BINS;
        /* Nhiều điểm rơi cùng một ô: giữ điểm có tín hiệu mạnh hơn */
        if (s_building.dist_mm[bin] == 0 ||
            p.sample[i].quality > s_building.quality[bin]) {
            s_building.dist_mm[bin] = d;
            s_building.quality[bin] = p.sample[i].quality;
        }
    }
}

/* Tìm các gói hợp lệ trong buffer, trả về số byte đã tiêu thụ.
 * UART trả về từng khúc tuỳ ý, không theo ranh giới gói, nên phải tự dò header
 * và tự giữ lại phần đuôi chưa đủ một gói. */
static size_t parse_buffer(const uint8_t *buf, size_t len)
{
    size_t i = 0;
    while (i + PKT_SIZE <= len) {
        if (buf[i] != 0x55 || buf[i + 1] != 0xAA ||
            buf[i + 2] != 0x03 || buf[i + 3] != 0x08) {
            i++;
            continue;
        }
        if (!packet_valid(buf + i)) {
            s_err_count++;
            i++;         /* checksum sai: có thể là header giả, trượt 1 byte */
            continue;
        }
        handle_packet(buf + i);
        i += PKT_SIZE;
    }
    return i;
}

static void lidar_task(void *arg)
{
    uint8_t buf[512];
    size_t len = 0;

    while (1) {
        int r = uart_read_bytes(LIDAR_UART, buf + len, sizeof(buf) - len,
                                pdMS_TO_TICKS(100));
        if (r <= 0) {
            continue;
        }
        len += r;
        size_t used = parse_buffer(buf, len);
        if (used > 0) {
            memmove(buf, buf + used, len - used);
            len -= used;
        } else if (len == sizeof(buf)) {
            len = 0;     /* toàn rác, không có gói nào: xả để đồng bộ lại */
        }
    }
}

static void lidar_start(void)
{
    uart_config_t cfg = {
        .baud_rate  = LIDAR_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(LIDAR_UART, 2048, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(LIDAR_UART, &cfg));
    /* Lidar chỉ phát, không nhận: chỉ cần khai báo chân RX */
    ESP_ERROR_CHECK(uart_set_pin(LIDAR_UART, UART_PIN_NO_CHANGE, LIDAR_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    /* Ưu tiên 13 (cao hơn mức thường 5): buffer 2KB chỉ chứa được ~0.2 giây dữ
     * liệu, task đọc bị chen là mất gói ngay. */
    xTaskCreate(lidar_task, "lidar", 4096, NULL, 13, NULL);
}

/* ---------------------------------------------------------------------------
 * 3. Web server
 * ------------------------------------------------------------------------- */
extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[]   asm("_binary_index_html_end");

static esp_err_t index_get(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, (const char *)index_html_start,
                           index_html_end - index_html_start - 1);
}

/* Trả về {"rpm":..,"seq":..,"err":..,"d":[360 số mm]}.
 * JSON cho dễ đọc khi học; muốn nhẹ hơn thì gửi thẳng 720 byte nhị phân. */
static esp_err_t scan_get(httpd_req_t *req)
{
    static scan_t snap;      /* static: ~1.5KB, để không làm tràn stack task web */
    if (!s_has_scan) {
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_sendstr(req, "{\"rpm\":0,\"seq\":0,\"err\":0,\"d\":[]}");
    }
    taskENTER_CRITICAL(&s_lock);
    memcpy(&snap, &s_complete, sizeof(snap));
    taskEXIT_CRITICAL(&s_lock);

    httpd_resp_set_type(req, "application/json");

    char head[96];
    int n = snprintf(head, sizeof(head),
                     "{\"rpm\":%u,\"seq\":%lu,\"err\":%lu,\"d\":[",
                     snap.rpm, (unsigned long)snap.seq,
                     (unsigned long)s_err_count);
    httpd_resp_send_chunk(req, head, n);

    /* Gửi theo từng khúc: nối cả 360 số vào một chuỗi thì cần ~2.5KB RAM,
     * còn cách này chỉ tốn một buffer nhỏ trên stack. */
    char part[256];
    int pos = 0;
    for (int i = 0; i < SCAN_BINS; i++) {
        pos += snprintf(part + pos, sizeof(part) - pos, "%s%u",
                        i ? "," : "", snap.dist_mm[i]);
        if (pos > (int)sizeof(part) - 8) {
            httpd_resp_send_chunk(req, part, pos);
            pos = 0;
        }
    }
    pos += snprintf(part + pos, sizeof(part) - pos, "]}");
    httpd_resp_send_chunk(req, part, pos);

    return httpd_resp_send_chunk(req, NULL, 0);   /* NULL = kết thúc phản hồi */
}

static void web_start(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t srv = NULL;
    ESP_ERROR_CHECK(httpd_start(&srv, &cfg));

    httpd_uri_t u_index = { .uri = "/", .method = HTTP_GET, .handler = index_get };
    httpd_uri_t u_scan  = { .uri = "/scan", .method = HTTP_GET, .handler = scan_get };
    httpd_register_uri_handler(srv, &u_index);
    httpd_register_uri_handler(srv, &u_scan);
}

/* ---------------------------------------------------------------------------
 * 4. WiFi
 * ------------------------------------------------------------------------- */
static void wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "Mất WiFi, đang kết nối lại...");
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "Mở trình duyệt tại: http://" IPSTR, IP2STR(&e->ip_info.ip));
    }
}

static void wifi_start(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t ic = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&ic));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                        wifi_event, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                        wifi_event, NULL, NULL));
    wifi_config_t wc = {
        .sta = { .ssid = WIFI_SSID, .password = WIFI_PASS },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());
}

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    lidar_start();
    wifi_start();
    web_start();

    /* In trạng thái mỗi 2 giây để biết lidar có sống hay không */
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        ESP_LOGI(TAG, "vòng quét=%lu  gói sai checksum=%lu  rpm=%u",
                 (unsigned long)s_scan_count, (unsigned long)s_err_count,
                 s_complete.rpm);
    }
}
