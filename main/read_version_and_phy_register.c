#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "../components/Ethernet/wizchip_conf.h"
#include "../components/Ethernet/W5500/w5500.h"
#include "../components/Ethernet/socket.h"

// ================================================================
// --- ĐỊNH NGHĨA PHẦN CỨNG VÀ MẠNG ---
// ================================================================
#define PIN_MOSI 11
#define PIN_MISO 13
#define PIN_SCLK 12
#define PIN_CS   10
#define PIN_INT  GPIO_NUM_4
#define LED_GPIO GPIO_NUM_2 // Chân GPIO kết nối với LED

#define SPI_HOST SPI2_HOST 

// Cài đặt cho TCP Server
#define TCP_SOCKET_NUM  0
#define TCP_PORT        5000
#define DATA_BUF_SIZE   2048

static spi_device_handle_t spi_handle;
static const char *TAG = "W5500_TCP_SERVER";
static uint8_t g_w5500_irq_sem = 0;
uint8_t g_data_buf[DATA_BUF_SIZE];


// ================================================================
// --- CÁC HÀM CALLBACK CHO THƯ VIỆN WIZNET ---
// ================================================================
void my_cs_select(void) { gpio_set_level(PIN_CS, 0); }
void my_cs_deselect(void) { gpio_set_level(PIN_CS, 1); }

uint8_t my_spi_readbyte(void) {
    uint8_t rx_data = 0;
    spi_transaction_t t = {.length = 8, .tx_buffer = NULL, .rx_buffer = &rx_data};
    assert(spi_device_polling_transmit(spi_handle, &t) == ESP_OK);
    return rx_data;
}

void my_spi_writebyte(uint8_t wb) {
    spi_transaction_t t = {.length = 8, .tx_buffer = &wb, .rx_buffer = NULL};
    assert(spi_device_polling_transmit(spi_handle, &t) == ESP_OK);
}

// ================================================================
// --- TRÌNH PHỤC VỤ NGẮT (ISR) ---
// ================================================================
static void IRAM_ATTR w5500_isr_handler(void* arg) {
    g_w5500_irq_sem = 1;
}

// ================================================================
// --- TASK XỬ LÝ LOGIC TCP SERVER ---
// ================================================================
void tcp_server_task(void* pvParameters) {
    // --- KHỞI TẠO MẠNG CHO W5500 ---
    uint8_t version = getVERSIONR();
    if (version != 0x04) {
        ESP_LOGE(TAG, "SPI communication failed. Halting task.");
        vTaskDelete(NULL);
    }
    ESP_LOGI(TAG, "W5500 Version: 0x%02X", version);

    ESP_LOGI(TAG, "Checking for Ethernet link...");
    while (!(getPHYCFGR() & PHYCFGR_LNK_ON)) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGI(TAG, "Ethernet link is UP.");
    
    wiz_NetInfo net_info = {
        .mac = {0x00, 0x08, 0xDC, 0x12, 0x34, 0x56},
        .ip = {192, 168, 1, 101},
        .sn = {255, 255, 255, 0},
        .gw = {192, 168, 1, 1},
        .dns = {0, 0, 0, 0},
        .dhcp = NETINFO_STATIC
    };
    wizchip_setnetinfo(&net_info);
    ESP_LOGI(TAG, "Network configured. IP: %d.%d.%d.%d", net_info.ip[0], net_info.ip[1], net_info.ip[2], net_info.ip[3]);
    
    ESP_LOGI(TAG, "Starting TCP Server on port %d...", TCP_PORT);

    // --- VÒNG LẶP MÁY TRẠNG THÁI ---
    while (1) {
        switch (getSn_SR(TCP_SOCKET_NUM)) {
            
            case SOCK_CLOSED:
                socket(TCP_SOCKET_NUM, Sn_MR_TCP, TCP_PORT, 0x00);
                break;

            case SOCK_INIT:
                listen(TCP_SOCKET_NUM);
                ESP_LOGI(TAG, "Socket is listening...");

                // ĐỌC GIÁ TRỊ BAN ĐẦU: Đọc Sn_IR ngay sau khi listen để xóa mọi cờ cũ (nếu có) và xem giá trị sạch.
                // Thường thì giá trị này sẽ là 0x00.
                uint8_t initial_sock_ir = getSn_IR(TCP_SOCKET_NUM);
                setSn_IR(TCP_SOCKET_NUM, initial_sock_ir); // Xóa mọi cờ cũ để bắt đầu sạch
                ESP_LOGW(TAG, "Giá trị Sn_IR ban đầu (sau khi listen): 0x%02X", getSn_IR(TCP_SOCKET_NUM));

                // Cấu hình ngắt cho các sự kiện mong muốn
                uint8_t socket_irq_mask = (Sn_IR_CON | Sn_IR_RECV | Sn_IR_DISCON);
                setSn_IMR(TCP_SOCKET_NUM, socket_irq_mask);
                break;

            case SOCK_ESTABLISHED:
            case SOCK_CLOSE_WAIT:
                // Task sẽ ngủ ở đây, đợi tín hiệu từ ISR.
                if (g_w5500_irq_sem == 1) {

                    g_w5500_irq_sem = 0;
                    
                    // ĐỌC GIÁ TRỊ KHI CÓ SỰ KIỆN: Đọc Sn_IR ngay sau khi được ngắt đánh thức.
                    // Đây là nơi chúng ta thấy được cờ ngắt đã được bật.
                    uint8_t event_sock_ir = getSn_IR(TCP_SOCKET_NUM);
                    ESP_LOGW(TAG, "Sự kiện ngắt xảy ra! Giá trị Sn_IR lúc này: 0x%02X", event_sock_ir);

                    // Xử lý sự kiện KẾT NỐI MỚI
                    if (event_sock_ir & Sn_IR_CON) {
                        uint8_t client_ip[4];
                        getSn_DIPR(TCP_SOCKET_NUM, client_ip);
                        ESP_LOGI(TAG, "Client connected: %d.%d.%d.%d", client_ip[0], client_ip[1], client_ip[2], client_ip[3]);
                    }

                    // Xử lý sự kiện NHẬN DỮ LIỆU
                    if (event_sock_ir & Sn_IR_RECV) {
                        uint16_t recv_size = getSn_RX_RSR(TCP_SOCKET_NUM);
                        if (recv_size > 0) {
                            int32_t ret = recv(TCP_SOCKET_NUM, g_data_buf, recv_size);
                            if (ret > 0) {
                                g_data_buf[ret] = '\0';
                                ESP_LOGI(TAG, "Received %d bytes: '%s'", ret, (char*)g_data_buf);
                                
                                char* response_msg = "Unknown command";
                                if (strstr((char*)g_data_buf, "LED_ON") != NULL) {
                                    gpio_set_level(LED_GPIO, 1);
                                    response_msg = "OK: LED is ON";
                                } else if (strstr((char*)g_data_buf, "LED_OFF") != NULL) {
                                    gpio_set_level(LED_GPIO, 0);
                                    response_msg = "OK: LED is OFF";
                                }
                                
                                send(TCP_SOCKET_NUM, (uint8_t*)response_msg, strlen(response_msg));
                                ESP_LOGI(TAG, "Sent response: '%s'", response_msg);
                            }
                        }
                    }

                    // Xử lý sự kiện NGẮT KẾT NỐI
                    if (event_sock_ir & Sn_IR_DISCON) {
                        ESP_LOGI(TAG, "Client disconnected.");
                        disconnect(TCP_SOCKET_NUM);
                    }

                    // DỌN DẸP: Xóa các cờ ngắt đã xử lý trên W5500
                    setSn_IR(TCP_SOCKET_NUM, event_sock_ir);
                    ESP_LOGW(TAG, "Giá trị Sn_IR sau khi xoa co ngat (sau khi listen): 0x%02X", getSn_IR(TCP_SOCKET_NUM));
                    
                }
                break;

            case SOCK_LISTEN:
                 vTaskDelay(pdMS_TO_TICKS(100)); // Trạng thái chờ, không cần làm gì
                 break;
                 
            default:
                ESP_LOGW(TAG, "Unexpected socket status (0x%02X), closing socket.", getSn_SR(TCP_SOCKET_NUM));
                close(TCP_SOCKET_NUM);
                break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ================================================================
// --- HÀM MAIN: KHỞI TẠO HỆ THỐNG ---
// ================================================================
void app_main(void) {
    esp_err_t ret;

    // --- BƯỚC 1: KHỞI TẠO GPIO VÀ SPI ---
    ESP_LOGI(TAG, "Initializing GPIO and SPI bus...");
    gpio_config_t led_gpio_config = {.pin_bit_mask = (1ULL << LED_GPIO), .mode = GPIO_MODE_OUTPUT};
    gpio_config(&led_gpio_config);

    gpio_config_t cs_gpio_config = {.pin_bit_mask = (1ULL << PIN_CS), .mode = GPIO_MODE_OUTPUT, .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&cs_gpio_config);
    gpio_set_level(PIN_CS, 1);

    spi_bus_config_t bus_cfg = {.mosi_io_num=PIN_MOSI, .miso_io_num=PIN_MISO, .sclk_io_num=PIN_SCLK, .quadwp_io_num=-1, .quadhd_io_num=-1};
    ret = spi_bus_initialize(SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);

    spi_device_interface_config_t dev_cfg = {.clock_speed_hz=8*1000*1000, .mode=0, .spics_io_num=-1, .queue_size=7};
    ret = spi_bus_add_device(SPI_HOST, &dev_cfg, &spi_handle);
    ESP_ERROR_CHECK(ret);

    // --- BƯỚC 2: KHỞI TẠO W5500 ---
    ESP_LOGI(TAG, "Initializing W5500 chip...");
    reg_wizchip_cs_cbfunc(my_cs_select, my_cs_deselect);
    reg_wizchip_spi_cbfunc(my_spi_readbyte, my_spi_writebyte);
    wizchip_init(NULL, NULL);
    
    // BƯỚC CẤU HÌNH NGẮT 2: Bật "cầu dao tổng" của W5500
    setIMR(0xFF); 
    ESP_LOGI(TAG, "W5500 global interrupt mask enabled.");

    // --- BƯỚC 3: CẤU HÌNH NGẮT TRÊN ESP32 ---
    ESP_LOGI(TAG, "Configuring GPIO interrupt on ESP32...");
    gpio_config_t int_gpio_config = {.pin_bit_mask=(1ULL<<PIN_INT), .mode=GPIO_MODE_INPUT, .pull_up_en=GPIO_PULLUP_ENABLE, .intr_type=GPIO_INTR_NEGEDGE};
    gpio_config(&int_gpio_config);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_INT, w5500_isr_handler, NULL);

    // --- BƯỚC 4: TẠO TASK CHÍNH CỦA ỨNG DỤNG ---
    xTaskCreate(tcp_server_task, "tcp_server_task", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "Initialization complete. TCP Server task started.");
}