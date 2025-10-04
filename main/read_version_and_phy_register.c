#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "../components/Ethernet/wizchip_conf.h"
#include "../components/Ethernet/W5500/w5500.h"
#include "../components/Ethernet/socket.h"

//
// Definition
//
#define PIN_MOSI 11
#define PIN_MISO 13
#define PIN_SCLK 12
#define PIN_CS   10
#define PIN_INT  GPIO_NUM_14
#define PIN_RST  GPIO_NUM_21
#define PIN_LED  GPIO_NUM_2

#define SPI_HOST SPI2_HOST

// Cài đặt cho TCP Server
#define TCP_SOCKET_NUM  0
#define TCP_PORT        5000
#define DATA_BUF_SIZE   2048

//
// Variables
//
static spi_device_handle_t spi_handle;
static const char *TAG = "W5500_TCP_SERVER";
uint8_t g_data_buf[DATA_BUF_SIZE];
volatile bool g_w5500_irq_flag = false; //flag interrupt

//
// Function
//
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

// interrupt function
static void IRAM_ATTR w5500_isr_handler(void* arg)
{
    g_w5500_irq_flag = true;
}

void app_main()
{
    esp_err_t ret;

    gpio_config_t led_gpio_config = {.pin_bit_mask = (1ULL << PIN_LED), .mode = GPIO_MODE_OUTPUT};
    gpio_config(&led_gpio_config);
    gpio_set_level(PIN_LED, 0);

    // --- BƯỚC 1: KHỞI TẠO GPIO VÀ SPI ---
    ESP_LOGI(TAG, "Initializing GPIO and SPI bus...");
    gpio_config_t rst_gpio_config = {.pin_bit_mask = (1ULL << PIN_RST), .mode = GPIO_MODE_OUTPUT};
    gpio_config(&rst_gpio_config);
    gpio_set_level(PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_RST, 1);

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

    // --- BƯỚC 3: CẤU HÌNH NGẮT TRÊN ESP32 ---
    ESP_LOGI(TAG, "Configuring GPIO interrupt on ESP32...");
    gpio_config_t int_gpio_config = {.pin_bit_mask=(1ULL<<PIN_INT), .mode=GPIO_MODE_INPUT, .pull_up_en=GPIO_PULLUP_DISABLE, .intr_type=GPIO_INTR_NEGEDGE};
    gpio_config(&int_gpio_config);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_INT, w5500_isr_handler, NULL);

    uint8_t version = getVERSIONR();
    if(version != 0x04) {
        ESP_LOGE(TAG, "SPI communication failed. Halting task.");
        while(1);
    } else {
        ESP_LOGI(TAG, "W5500 Version: 0x%02X", version);
    }

    ESP_LOGI(TAG, "Checking for Ethernet link...");
    while (!(getPHYCFGR() & PHYCFGR_LNK_ON)) {
        ESP_LOGI(TAG, "Waiting for link...");
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

    // --- BƯỚC 4: KÍCH HOẠT HỆ THỐNG NGẮT CỦA W5500 ---

    // *** THAY ĐỔI 1: Cấu hình mặt nạ ngắt cho Socket ***
    // Bật ngắt cho các sự kiện: CON, DISCON, RECV
    uint8_t socket_irq_mask = (Sn_IR_CON | Sn_IR_DISCON | Sn_IR_RECV);
    setSn_IMR(TCP_SOCKET_NUM, socket_irq_mask);
    ESP_LOGI(TAG, "Socket %d interrupt mask configured to 0x%02X.", TCP_SOCKET_NUM, socket_irq_mask);

    // Kích hoạt ngắt toàn cục cho Socket
    setSIMR(1 << TCP_SOCKET_NUM);
    ESP_LOGI(TAG, "W5500 global interrupt mask enabled for Socket %d.", TCP_SOCKET_NUM);

    while(1)
    {
        uint8_t sn_sr = getSn_SR(TCP_SOCKET_NUM);

        switch(sn_sr)
        {
            case SOCK_CLOSED:
                socket(TCP_SOCKET_NUM, Sn_MR_TCP, TCP_PORT, SF_TCP_NODELAY);
                ESP_LOGI(TAG, "Socket %d: SOCK_CLOSED -> Initializing...", TCP_SOCKET_NUM);
                break;

            case SOCK_INIT:
                listen(TCP_SOCKET_NUM);
                ESP_LOGI(TAG, "Socket %d: SOCK_INIT -> Listening on port %d...", TCP_SOCKET_NUM, TCP_PORT);
                break;

            case SOCK_ESTABLISHED:
                // Xử lý ngắt CON ngay sau khi vào trạng thái ESTABLISHED
                // *** THAY ĐỔI 2: Thêm đoạn kiểm tra ngắt CON ***
                uint8_t initial_ir = getSn_IR(TCP_SOCKET_NUM);
                if (initial_ir & Sn_IR_CON)
                {
                    uint8_t client_ip[4];
                    getSn_DIPR(TCP_SOCKET_NUM, client_ip);
                    ESP_LOGI(TAG, "+++ Client connected from: %d.%d.%d.%d +++", client_ip[0], client_ip[1], client_ip[2], client_ip[3]);
                    
                    // In giá trị Sn_IR trước khi xóa
                    ESP_LOGW(TAG, "[CONNECT] Sn_IR before clear: 0x%02X", initial_ir);
                    // Xóa cờ ngắt CON
                    setSn_IR(TCP_SOCKET_NUM, Sn_IR_CON);
                }

                if (g_w5500_irq_flag == true)
                {
                    g_w5500_irq_flag = false;
                    uint8_t ir = getSn_IR(TCP_SOCKET_NUM); // Đọc thanh ghi ngắt

                    // *** THAY ĐỔI 3: Thêm log cho từng loại ngắt ***

                    if (ir & Sn_IR_RECV)
                    {
                        // In giá trị Sn_IR trước khi xóa
                        ESP_LOGW(TAG, "[RECV] Sn_IR before clear: 0x%02X", ir);
                        setSn_IR(TCP_SOCKET_NUM, Sn_IR_RECV); // Xóa cờ ngắt RECV

                        int32_t recv_len = getSn_RX_RSR(TCP_SOCKET_NUM);
                        if (recv_len > 0)
                        {
                            recv(TCP_SOCKET_NUM, g_data_buf, recv_len);
                            g_data_buf[recv_len] = '\0';
                            ESP_LOGI(TAG, "Received %d bytes: %s", recv_len, (char*)g_data_buf);

                            if (strcmp((char*)g_data_buf, "led on") == 0) {
                                gpio_set_level(PIN_LED, 1);
                                ESP_LOGI(TAG, "LED is ON");
                            } else if (strcmp((char*)g_data_buf, "led off") == 0) {
                                gpio_set_level(PIN_LED, 0);
                                ESP_LOGI(TAG, "LED is OFF");
                            }
                        }
                    }

                    if (ir & Sn_IR_DISCON)
                    {
                        // In giá trị Sn_IR trước khi xóa
                        ESP_LOGW(TAG, "[DISCONNECT] Sn_IR before clear: 0x%02X", ir);
                        setSn_IR(TCP_SOCKET_NUM, Sn_IR_DISCON); // Xóa cờ ngắt DISCON

                        ESP_LOGI(TAG, "Socket %d: Disconnected by client.", TCP_SOCKET_NUM);
                        close(TCP_SOCKET_NUM);
                    }
                }
                break;

            case SOCK_CLOSE_WAIT:
                ESP_LOGI(TAG, "Socket %d: SOCK_CLOSE_WAIT -> Disconnecting...", TCP_SOCKET_NUM);
                disconnect(TCP_SOCKET_NUM);
                break;

            default:
                break;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}