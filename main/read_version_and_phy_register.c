#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

// ====================================================================
// === GIẢI PHÁP XUNG ĐỘT THƯ VIỆN ===
// Chỉ định đường dẫn tương đối để trình biên dịch ưu tiên file socket.h của WIZnet
// ====================================================================
#include "../components/Ethernet/wizchip_conf.h"
#include "../components/Ethernet/W5500/w5500.h"
#include "../components/Ethernet/socket.h"

// 1. ĐỊNH NGHĨA PHẦN CỨNG VÀ CALLBACKS
//================================================================
#define PIN_MOSI 11
#define PIN_MISO 13
#define PIN_SCLK 12
#define PIN_CS   10
#define SPI_HOST SPI2_HOST
#define LED_GPIO GPIO_NUM_2 // Chân GPIO để kết nối đèn LED

static spi_device_handle_t spi_handle;
static const char *TAG = "W5500_LED_CONTROL";

// Khai báo các hàm callback (được định nghĩa ở cuối file)
void my_cs_select(void);
void my_cs_deselect(void);
uint8_t my_spi_readbyte(void);
void my_spi_writebyte(uint8_t wb);

// ĐỊNH NGHĨA CHO TCP SERVER
//================================================================
#define TCP_SOCKET_NUM  0
#define TCP_PORT        5000
#define DATA_BUF_SIZE   2048

uint8_t g_data_buf[DATA_BUF_SIZE];


// 3. HÀM MAIN CỦA ỨNG DỤNG
//================================================================
void app_main(void) {
    // --- KHỞI TẠO PHẦN CỨNG VÀ DRIVER SPI ---
    ESP_LOGI(TAG, "Initializing W5500 hardware...");
    gpio_config_t cs_gpio_config = {.pin_bit_mask = (1ULL << PIN_CS), .mode = GPIO_MODE_OUTPUT};
    gpio_config(&cs_gpio_config);
    gpio_set_level(PIN_CS, 1);
    spi_bus_config_t bus_cfg = {.mosi_io_num=PIN_MOSI, .miso_io_num=PIN_MISO, .sclk_io_num=PIN_SCLK, .quadwp_io_num=-1, .quadhd_io_num=-1, .max_transfer_sz=4094};
    ESP_ERROR_CHECK(spi_bus_initialize(SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO));
    spi_device_interface_config_t dev_cfg = {.clock_speed_hz = 8 * 1000 * 1000, .mode = 0, .spics_io_num = -1, .queue_size=7};
    ESP_ERROR_CHECK(spi_bus_add_device(SPI_HOST, &dev_cfg, &spi_handle));

    // Cấu hình chân GPIO cho đèn LED
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

    // --- ĐĂNG KÝ CALLBACK VÀ KHỞI TẠO CHIP ---
    reg_wizchip_cs_cbfunc(my_cs_select, my_cs_deselect);
    reg_wizchip_spi_cbfunc(my_spi_readbyte, my_spi_writebyte);
    wizchip_init(NULL, NULL);

    // --- BƯỚC 1: KIỂM TRA GIAO TIẾP SPI ---
    uint8_t version = getVERSIONR();
    if (version != 0x04) {
        ESP_LOGE(TAG, "SPI communication failed. Read version: 0x%02X. Halting.", version);
        return;
    }
    ESP_LOGI(TAG, "SPI communication successful. W5500 Version: 0x%02X", version);

    // --- BƯỚC 2: KIỂM TRA KẾT NỐI VẬT LÝ ETHERNET ---
    ESP_LOGI(TAG, "Checking for Ethernet link...");
    while (!(getPHYCFGR() & PHYCFGR_LNK_ON)) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGI(TAG, "Ethernet link is UP.");

    // --- BƯỚC 3: CẤU HÌNH THÔNG TIN MẠNG (IP TĨNH) ---
    ESP_LOGI(TAG, "Configuring static IP address...");
    wiz_NetInfo net_info = {
        .mac = {0x00, 0x08, 0xDC, 0x12, 0x34, 0x56},
        .ip = {192, 168, 1, 101},
        .sn = {255, 255, 255, 0},
        .gw = {0, 0, 0, 0},
        .dns = {0, 0, 0, 0},
        .dhcp = NETINFO_STATIC
    };
    wizchip_setnetinfo(&net_info);
    
    wiz_NetInfo verify_net_info;
    wizchip_getnetinfo(&verify_net_info);
    ESP_LOGI(TAG, "Network configured:");
    ESP_LOGI(TAG, "  IP:  %d.%d.%d.%d", verify_net_info.ip[0], verify_net_info.ip[1], verify_net_info.ip[2], verify_net_info.ip[3]);
    ESP_LOGI(TAG, "  MAC: %02X:%02X:%02X:%02X:%02X:%02X", verify_net_info.mac[0], verify_net_info.mac[1], verify_net_info.mac[2], verify_net_info.mac[3], verify_net_info.mac[4], verify_net_info.mac[5]);
    
    // --- BƯỚC 4: BẮT ĐẦU VÒNG LẶP TCP SERVER ---
    ESP_LOGI(TAG, "Starting TCP LED Control Server on port %d...", TCP_PORT);
    int32_t ret;
    uint16_t size = 0;

    while (1) {
        switch (getSn_SR(TCP_SOCKET_NUM)) {
            case SOCK_CLOSED:
                if (socket(TCP_SOCKET_NUM, Sn_MR_TCP, TCP_PORT, 0x00) != TCP_SOCKET_NUM) {
                    ESP_LOGE(TAG, "socket() failed.");
                }
                break;

            case SOCK_INIT:
                if (listen(TCP_SOCKET_NUM) != SOCK_OK) {
                    ESP_LOGE(TAG, "listen() failed.");
                } else {
                    ESP_LOGI(TAG, "Socket is listening on port %d...", TCP_PORT);
                }
                break;
            
            case SOCK_ESTABLISHED:
                if (getSn_IR(TCP_SOCKET_NUM) & Sn_IR_CON) {
                    uint8_t client_ip[4];
                    getSn_DIPR(TCP_SOCKET_NUM, client_ip);
                    ESP_LOGI(TAG, "Client connected: %d.%d.%d.%d", client_ip[0], client_ip[1], client_ip[2], client_ip[3]);
                    setSn_IR(TCP_SOCKET_NUM, Sn_IR_CON);
                }

                if ((size = getSn_RX_RSR(TCP_SOCKET_NUM)) > 0) {
                    if (size > DATA_BUF_SIZE) size = DATA_BUF_SIZE;
                    ret = recv(TCP_SOCKET_NUM, g_data_buf, size);
                    if (ret <= 0) break;

                    // LOGIC XỬ LÝ LỆNH ĐIỀU KHIỂN LED
                    g_data_buf[ret] = '\0';
                    ESP_LOGI(TAG, "Received command: '%s'", (char*)g_data_buf);

                    char* response_msg = "Unknown command"; // Tin nhắn phản hồi mặc định

                    // So sánh chuỗi nhận được với các lệnh đã biết
                    if (strcmp((char*)g_data_buf, "LED_ON") == 0) {
                        gpio_set_level(LED_GPIO, 1); // Bật đèn LED
                        ESP_LOGI(TAG, "Action: LED turned ON");
                        response_msg = "OK: LED is ON";
                    } else if (strcmp((char*)g_data_buf, "LED_OFF") == 0) {
                        gpio_set_level(LED_GPIO, 0); // Tắt đèn LED
                        ESP_LOGI(TAG, "Action: LED turned OFF");
                        response_msg = "OK: LED is OFF";
                    } else {
                        ESP_LOGW(TAG, "Warning: Command not recognized.");
                    }

                    // Gửi lại tin nhắn phản hồi cho client
                    send(TCP_SOCKET_NUM, (uint8_t*)response_msg, strlen(response_msg));
                    ESP_LOGI(TAG, "Sent response: '%s'", response_msg);
                }
                break;

            case SOCK_CLOSE_WAIT:
                if (disconnect(TCP_SOCKET_NUM) != SOCK_OK) {
                    ESP_LOGE(TAG, "disconnect() failed.");
                } else {
                    ESP_LOGI(TAG, "Client disconnected gracefully.");
                }
                break;

            case SOCK_LISTEN:
                // Trạng thái bình thường, không cần làm gì, chỉ chờ kết nối
                break;

            default:
                // Nếu có lỗi không mong muốn, đóng socket để bắt đầu lại
                ESP_LOGW(TAG, "Unexpected socket status (0x%02X), closing socket.", getSn_SR(TCP_SOCKET_NUM));
                close(TCP_SOCKET_NUM);
                break;
        }

        // Nhường quyền cho các tác vụ khác của hệ thống để tránh lỗi watchdog
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ================================================================
// ĐỊNH NGHĨA CÁC HÀM CALLBACK
// ================================================================
void my_cs_select(void) {
    gpio_set_level(PIN_CS, 0);
}

void my_cs_deselect(void) {
    gpio_set_level(PIN_CS, 1);
}

uint8_t my_spi_readbyte(void) {
    uint8_t rx_data = 0;
    spi_transaction_t t = {.length = 8, .tx_buffer = NULL, .rx_buffer = &rx_data};
    // Sử dụng assert để phát hiện lỗi nghiêm trọng trong quá trình phát triển
    assert(spi_device_polling_transmit(spi_handle, &t) == ESP_OK);
    return rx_data;
}

void my_spi_writebyte(uint8_t wb) {
    spi_transaction_t t = {.length = 8, .tx_buffer = &wb, .rx_buffer = NULL};
    assert(spi_device_polling_transmit(spi_handle, &t) == ESP_OK);
}