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

//
// Definition
//
#define PIN_MOSI 11
#define PIN_MISO 13
#define PIN_SCLK 12
#define PIN_CS   10
#define PIN_INT  GPIO_NUM_14
#define PIN_RST  GPIO_NUM_21

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
// static uint8_t g_w5500_irq_sem = 0;
uint8_t g_data_buf[DATA_BUF_SIZE];
uint8_t g_w5500_irq_sem = 0;

//
//Function
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

static void IRAM_ATTR w5500_isr_handler(void* arg) 
{
    setSn_IR(TCP_SOCKET_NUM, 0);
    g_w5500_irq_sem = 1;
}

//
//main
//
void app_main()
{
    esp_err_t ret;
    // --- BƯỚC 1: KHỞI TẠO GPIO VÀ SPI ---
    ESP_LOGI(TAG, "Initializing GPIO and SPI bus...");
    gpio_config_t led_gpio_config = {.pin_bit_mask = (1ULL << PIN_RST), .mode = GPIO_MODE_OUTPUT};
    gpio_config(&led_gpio_config);
    gpio_set_level(PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_RST, 1);


    gpio_config_t int_gpio_config = {
        .pin_bit_mask = (1ULL << PIN_INT), 
        .mode = GPIO_MODE_INPUT, 
        .intr_type = GPIO_INTR_NEGEDGE,
        .pull_up_en = GPIO_PULLUP_DISABLE};
    gpio_config(&int_gpio_config);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_INT, w5500_isr_handler, NULL);


    gpio_config_t cs_gpio_config = {.pin_bit_mask = (1ULL << PIN_CS), .mode = GPIO_MODE_OUTPUT, .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&cs_gpio_config);
    gpio_set_level(PIN_CS, 1);

    spi_bus_config_t bus_cfg = {.mosi_io_num=PIN_MOSI, .miso_io_num=PIN_MISO, .sclk_io_num=PIN_SCLK, .quadwp_io_num=-1, .quadhd_io_num=-1};
    ret = spi_bus_initialize(SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);

    spi_device_interface_config_t dev_cfg = {.clock_speed_hz=50*1000*1000, .mode=0, .spics_io_num=-1, .queue_size=7};
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
    // ESP_LOGI(TAG, "Configuring GPIO interrupt on ESP32...");
    // gpio_config_t int_gpio_config = {.pin_bit_mask=(1ULL<<PIN_INT), .mode=GPIO_MODE_INPUT, .pull_up_en=GPIO_PULLUP_ENABLE, .intr_type=GPIO_INTR_NEGEDGE};
    // gpio_config(&int_gpio_config);
    // gpio_install_isr_service(0);
    // gpio_isr_handler_add(PIN_INT, w5500_isr_handler, NULL);

    uint8_t version = getVERSIONR();
    if(version != 0x04)
    {
        ESP_LOGE(TAG, "SPI communication failed. Halting task.");
        while(1);
    }
    else
    {
        ESP_LOGI(TAG, "W5500 Version: 0x%02X", version);
    }
    ESP_LOGI(TAG, "Checking for Ethernet link...");
    while (!(getPHYCFGR() & PHYCFGR_LNK_ON)) {
        ESP_LOGI(TAG, "Waiting ...\n");
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

    uint8_t check = 0;
    while(1)
    {
        uint8_t sn_sr = getSn_SR(TCP_SOCKET_NUM);
        if(sn_sr == SOCK_CLOSED)
        {
            socket(TCP_SOCKET_NUM, Sn_MR_TCP, TCP_PORT, 0x00);
        }
        else if (sn_sr == SOCK_INIT)
        {
            listen(TCP_SOCKET_NUM);
            uint8_t initial_sock_ir = getSn_IR(TCP_SOCKET_NUM);
            setSn_IR(TCP_SOCKET_NUM, initial_sock_ir); // Xóa mọi cờ cũ để bắt đầu sạch
            ESP_LOGW(TAG, "Giá trị Sn_IR ban đầu (sau khi listen): 0x%02X", getSn_IR(TCP_SOCKET_NUM));
            
            // Cấu hình ngắt cho các sự kiện mong muốn
            uint8_t socket_irq_mask = Sn_IR_RECV ;
            setSn_IMR(TCP_SOCKET_NUM, socket_irq_mask);
            setSIMR(1);


        }
        else if(sn_sr == SOCK_ESTABLISHED)
        {
            check = 1;
            uint8_t sn_cr = getSn_CR(TCP_SOCKET_NUM);
            uint8_t sn_ir = getSn_IR(TCP_SOCKET_NUM);
            uint8_t sn_imr = getSn_IMR(TCP_SOCKET_NUM);
            uint16_t intlevel = getINTLEVEL();

            int int_status = gpio_get_level(PIN_INT);
            if(g_w5500_irq_sem == 1)
            {
                uint16_t recv_size = getSn_RX_RSR(TCP_SOCKET_NUM);
                if (recv_size > 0) 
                {
                    int32_t ret = recv(TCP_SOCKET_NUM, g_data_buf, recv_size);
                    if(ret  > 0)
                    {
                        g_data_buf[ret] = '\0';
                        ESP_LOGI(TAG, "Received %d bytes: '%s'", ret, (char*)g_data_buf);
                        char* response_msg = "OK: LED is ON";
                        send(TCP_SOCKET_NUM, (uint8_t*)response_msg, strlen(response_msg));
                    }

                }
            }   
            ESP_LOGE(TAG, "int_status: %d, sn_cr: 0x%02X, sn_ir: 0x%02X, sn_ỉmr: 0x%02X, intlevel: %d", 
                     int_status, sn_cr, sn_ir, sn_imr, intlevel);
                    
            // ESP_LOGI(TAG, "Sn_SR: 0x%02X", sn_sr);
            // ESP_LOGI(TAG, "Sn_CR: 0x%02X", sn_ir);

        }
        else if(sn_sr == SOCK_CLOSE_WAIT)
        {
            setSn_CR(TCP_SOCKET_NUM, Sn_CR_DISCON);
            check = 0;
        }


        // if(check == 0)
        // {
        //     ESP_LOGI(TAG, "Sn_SR: 0x%02X", sn_sr);
        // }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

}




