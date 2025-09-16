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

static spi_device_handle_t spi_handle;

// 2. CÁC HÀM CALLBACK 
//================================================================
void my_cs_select(void) { gpio_set_level(PIN_CS, 0); }
void my_cs_deselect(void) { gpio_set_level(PIN_CS, 1); }
uint8_t my_spi_readbyte(void) {
    uint8_t rx_data = 0;
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = NULL,
        .rx_buffer = &rx_data
    };
    assert(spi_device_polling_transmit(spi_handle, &t) == ESP_OK);
    return rx_data;
}
void my_spi_writebyte(uint8_t wb) {
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &wb,
        .rx_buffer = NULL
    };
    assert(spi_device_polling_transmit(spi_handle, &t) == ESP_OK);
}

// 3. HÀM MAIN CỦA ỨNG DỤNG
//================================================================
void app_main(void) {
    // --- KHỞI TẠO ---
    printf("Initializing W5500...\n");
    gpio_config_t cs_gpio_config = {.pin_bit_mask = (1ULL << PIN_CS), .mode = GPIO_MODE_OUTPUT};
    gpio_config(&cs_gpio_config);
    gpio_set_level(PIN_CS, 1);

    spi_bus_config_t bus_cfg = {
        .mosi_io_num=PIN_MOSI, .miso_io_num=PIN_MISO, .sclk_io_num=PIN_SCLK,
        .quadwp_io_num=-1, .quadhd_io_num=-1, .max_transfer_sz=4094
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = 4 * 1000 * 1000, .mode = 0, .spics_io_num = -1, .queue_size=7
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI_HOST, &dev_cfg, &spi_handle));
    
    reg_wizchip_cs_cbfunc(my_cs_select, my_cs_deselect);
    reg_wizchip_spi_cbfunc(my_spi_readbyte, my_spi_writebyte);
    
    wizchip_init(NULL, NULL);

    // --- KIỂM TRA GIAO TIẾP SPI ---
    uint8_t version = getVERSIONR();
    printf("============================================\n");
    printf("Step 1: Checking SPI Communication...\n");
    printf(">> W5500 Version Register value: 0x%02X\n", version);
    if (version != 0x04) {
        printf(">> ERROR: SPI communication failed. Halting.\n");
        return;
    }
    printf(">> SUCCESS: SPI communication is OK.\n");
    printf("============================================\n\n");

    // --- KIỂM TRA KẾT NỐI VẬT LÝ ETHERNET ---
    printf("Step 2: Continuously Checking Ethernet Physical Link Status...\n\n");

    while(1) {
        uint8_t phy_status = getPHYCFGR();

        // Kiểm tra trạng thái Link bằng hằng số PHYCFGR_LNK_ON
        if (phy_status & PHYCFGR_LNK_ON) {
            printf(">>> Trạng thái Link: UP (Đã kết nối)\n");
            
            // Kiểm tra tốc độ bằng hằng số PHYCFGR_SPD_100
            if (phy_status & PHYCFGR_SPD_100) {
                printf("    - Tốc độ: 100Mbps\n");
            } else {
                printf("    - Tốc độ: 10Mbps\n");
            }
            // Kiểm tra chế độ duplex bằng hằng số PHYCFGR_DPX_FULL
            if (phy_status & PHYCFGR_DPX_FULL) {
                printf("    - Chế độ: Full Duplex\n");
            } else {
                printf("    - Chế độ: Half Duplex\n");
            }
        } else {
            printf(">>> Trạng thái Link: DOWN (Đã ngắt kết nối)\n");
        }

        vTaskDelay(2000 / portTICK_PERIOD_MS);
        printf("---\n");
    }
}