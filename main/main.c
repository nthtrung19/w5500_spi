#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "wizchip_conf.h"
#include "w5500.h"

// 1. ĐỊNH NGHĨA PHẦN CỨNG
//================================================================
#define PIN_MOSI 11
#define PIN_MISO 13
#define PIN_SCLK 12
#define PIN_CS   10
#define SPI_HOST SPI2_HOST

// Biến toàn cục cho thiết bị SPI
static spi_device_handle_t spi_handle;

// 2. CÁC HÀM GIAO TIẾP (CALLBACK) CHO THƯ VIỆN WIZNET
//================================================================

// Hàm Chọn Chip (CS) - Bây giờ chúng ta sẽ viết code điều khiển GPIO ở đây
void manual_wizchip_cs_select(void) {
    gpio_set_level(PIN_CS, 0); // Kéo chân CS xuống mức thấp để chọn chip
}

// Hàm Bỏ Chọn Chip (CS) - Tương tự, điều khiển GPIO ở đây
void manual_wizchip_cs_deselect(void) {
    gpio_set_level(PIN_CS, 1); // Kéo chân CS lên mức cao để bỏ chọn
}

// Hàm đọc 1 byte từ W5500
uint8_t manual_spi_read_byte(void) {
    uint8_t rx_data = 0;
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = NULL,
        .rx_buffer = &rx_data
    };
    esp_err_t ret = spi_device_polling_transmit(spi_handle, &t);
    assert(ret == ESP_OK);
    return rx_data;
}

// Hàm ghi 1 byte tới W5500
void manual_spi_write_byte(uint8_t wb) {
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &wb,
        .rx_buffer = NULL
    };
    esp_err_t ret = spi_device_polling_transmit(spi_handle, &t);
    assert(ret == ESP_OK);
}

// 3. HÀM MAIN CỦA ỨNG DỤNG
//================================================================
void app_main(void) {
    printf("Initializing W5500 with Manual CS control...\n");

    // --- BƯỚC 1: TỰ CẤU HÌNH CHÂN CS ---
    gpio_config_t cs_gpio_config = {
        .pin_bit_mask = (1ULL << PIN_CS),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&cs_gpio_config);
    gpio_set_level(PIN_CS, 1); // Đặt CS ở mức cao (không hoạt động) lúc ban đầu

    // --- BƯỚC 2: CẤU HÌNH VÀ KHỞI TẠO BUS SPI ---
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = PIN_MISO,
        .sclk_io_num = PIN_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4094
    };
    esp_err_t ret = spi_bus_initialize(SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);

    // --- BƯỚC 3: CẤU HÌNH THIẾT BỊ W5500 ---
    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = 4 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = -1, // QUAN TRỌNG: Đặt là -1 để vô hiệu hóa quản lý CS tự động
        .queue_size = 7
    };
    ret = spi_bus_add_device(SPI_HOST, &dev_cfg, &spi_handle);
    ESP_ERROR_CHECK(ret);

    printf("SPI device configured.\n");

    // --- BƯỚC 4: ĐĂNG KÝ CÁC HÀM CALLBACK VỚI THƯ VIỆN WIZNET ---
    reg_wizchip_cs_cbfunc(manual_wizchip_cs_select, manual_wizchip_cs_deselect);
    reg_wizchip_spi_cbfunc(manual_spi_read_byte, manual_spi_write_byte);

    // --- BƯỚC 5: KHỞI TẠO VÀ KIỂM TRA W5500 ---
    printf("Initializing W5500 chip...\n");
    wizchip_init(NULL, NULL);
    uint8_t version = getVERSIONR();

    printf("============================================\n");
    printf("Attempting to read W5500 Version Register...\n");
    printf(">> Value read: 0x%02X\n", version);

    if (version == 0x04) {
        printf(">> SUCCESS! Communication with W5500 is established.\n");
    } else {
        printf(">> ERROR! Failed to communicate correctly.\n");
        printf("   Expected value: 0x04\n");
    }
    printf("============================================\n");
}