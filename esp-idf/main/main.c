/*
 * Camsense X1 + ESP32 (Chế độ WiFi Access Point tự phát):
 *   - Đọc LiDAR Camsense X1 qua UART2 (GPIO 16, 115200 bps)
 *   - Tự phát WiFi "ESP32_LiDAR_X1" (Mật khẩu: "12345678")
 *   - Phục vụ Local Web hiển thị bản đồ 2D tại http://192.168.4.1
 */
#include <stdio.h>
#include <string.h>

#include "driver/uart.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#define AP_SSID         "ESP32_LiDAR_X1"
#define AP_PASS         "12345678"
#define AP_MAX_CONN     4

#define LIDAR_UART      UART_NUM_2
#define LIDAR_RX_PIN    16
#define LIDAR_BAUD      115200

#define PKT_SIZE         36
#define PKT_POINTS       8
#define ANGLE_ZERO       0xA000
#define DIST_SANE_MAX_MM 12000
#define SCAN_BINS        360

static const char *TAG = "x1_ap";

typedef struct __attribute__((packed)) {
    uint16_t distance_mm;
    uint8_t  quality;
} sample_raw_t;

typedef struct __attribute__((packed)) {
    uint8_t      start_byte0;   /* 0x55 */
    uint8_t      start_byte1;   /* 0xAA */
    uint8_t      start_byte2;   /* 0x03 */
    uint8_t      samples;       /* 0x08 */
    uint16_t     speed;         /* Hz * 3840 */
    uint16_t     start_angle;
    sample_raw_t sample[PKT_POINTS];
    uint16_t     end_angle;
    uint16_t     checksum;      /* 15-bit */
} x1_packet_t;

_Static_assert(sizeof(x1_packet_t) == PKT_SIZE, "Packet size must be 36 bytes");

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

static float raw_to_deg(uint16_t raw)
{
    return (float)(uint16_t)(raw - ANGLE_ZERO) / 64.0f;
}

typedef struct {
    uint16_t dist_mm[SCAN_BINS];
    uint8_t  quality[SCAN_BINS];
    uint16_t rpm;
    uint32_t seq;
} scan_t;

static scan_t s_building, s_complete;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static volatile bool s_has_scan = false;
static volatile uint32_t s_scan_count, s_err_count;
static float s_last_start_deg = -1.0f;

static void finish_scan(void)
{
    taskENTER_CRITICAL(&s_lock);
    s_building.seq = ++s_scan_count;
    memcpy(&s_complete, &s_building, sizeof(scan_t));
    memset(&s_building, 0, sizeof(scan_t));
    taskEXIT_CRITICAL(&s_lock);
    s_has_scan = true;
}

static void handle_packet(const uint8_t *raw)
{
    x1_packet_t p;
    memcpy(&p, raw, sizeof(p));

    float start = raw_to_deg(p.start_angle);
    float end   = raw_to_deg(p.end_angle);
    float span  = end - start;
    if (span < 0.0f) {
        span += 360.0f;
    }
    float step = span / (PKT_POINTS - 1);

    if (s_last_start_deg >= 0.0f && start < s_last_start_deg) {
        finish_scan();
    }
    s_last_start_deg = start;

    s_building.rpm = (uint16_t)(p.speed / 64);

    for (int i = 0; i < PKT_POINTS; i++) {
        uint16_t d = p.sample[i].distance_mm;
        if (d == 0 || d > DIST_SANE_MAX_MM || p.sample[i].quality == 0) {
            continue;
        }
        float ang = start + step * i;
        while (ang >= 360.0f) ang -= 360.0f;
        while (ang < 0.0f)    ang += 360.0f;

        int bin = (int)(ang + 0.5f) % SCAN_BINS;
        if (s_building.dist_mm[bin] == 0 ||
            p.sample[i].quality > s_building.quality[bin]) {
            s_building.dist_mm[bin] = d;
            s_building.quality[bin] = p.sample[i].quality;
        }
    }
}

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
            i++;
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
            len = 0;
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
    ESP_ERROR_CHECK(uart_set_pin(LIDAR_UART, UART_PIN_NO_CHANGE, LIDAR_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    xTaskCreate(lidar_task, "lidar", 4096, NULL, 13, NULL);
}

extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[]   asm("_binary_index_html_end");

static esp_err_t index_get(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, (const char *)index_html_start,
                           index_html_end - index_html_start - 1);
}

static esp_err_t scan_get(httpd_req_t *req)
{
    static scan_t snap;
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

    return httpd_resp_send_chunk(req, NULL, 0);
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

static void wifi_ap_start(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t ic = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&ic));

    wifi_config_t wc = {
        .ap = {
            .ssid = AP_SSID,
            .ssid_len = strlen(AP_SSID),
            .channel = 1,
            .password = AP_PASS,
            .max_connection = AP_MAX_CONN,
            .authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "Da phat WiFi AP: %s", AP_SSID);
    ESP_LOGI(TAG, "Mat khau       : %s", AP_PASS);
    ESP_LOGI(TAG, "Truy cap Web   : http://192.168.4.1");
    ESP_LOGI(TAG, "==========================================");
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
    wifi_ap_start();
    web_start();

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        ESP_LOGI(TAG, "vong quet=%lu  goi sai checksum=%lu  rpm=%u",
                 (unsigned long)s_scan_count, (unsigned long)s_err_count,
                 s_complete.rpm);
    }
}
