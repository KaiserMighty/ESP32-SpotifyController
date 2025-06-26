#include <stdio.h>
#include <ssd1306.h>
#include <driver/i2c_master.h>
#include "esp_log.h"

#define I2C_MASTER_SDA_IO       23
#define I2C_MASTER_SCL_IO       22

void app_main()
{
    i2c_master_bus_config_t i2c_config =
    {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = 0,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t i2c_bus;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_config, &i2c_bus));

    ssd1306_config_t panel_cfg = I2C_SSD1306_128x32_CONFIG_DEFAULT;
    ssd1306_handle_t panel_hdl;

    ssd1306_init(i2c_bus, &panel_cfg, &panel_hdl);
    if (panel_hdl == NULL)
    {
        ESP_LOGE("SSD1306", "ssd1306 handle init failed");
        assert(panel_hdl);
    }

    ESP_LOGI("SSD1306", "Hello World!");
    ssd1306_clear_display(panel_hdl, false);
    ssd1306_set_contrast(panel_hdl, 0xff);
    ssd1306_display_text(panel_hdl, 0, "SSD1306 128x32", false);
    ssd1306_display_text(panel_hdl, 1, "Hello World!", false);
    ssd1306_display_text(panel_hdl, 2, "This is a test!", false);
    ssd1306_display_text(panel_hdl, 3, "Lorem Ipsum!!", false);
}