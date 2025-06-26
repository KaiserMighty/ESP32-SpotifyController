#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <ssd1306.h>
#include <driver/i2c_master.h>
#include <driver/gpio.h>
#include "esp_log.h"
#include "string.h"

#define I2C_MASTER_SDA_IO       23
#define I2C_MASTER_SCL_IO       22

#define ROT_ENC_VOLUP           16
#define ROT_ENC_VOLDOWN         17

#define ROT_ENC_BUTTON          4
#define BUTTON_NEXT             21
#define BUTTON_PLAY             19
#define BUTTON_PREV             18

gpio_num_t button_pins[4] =
{
    ROT_ENC_BUTTON,
    BUTTON_NEXT,
    BUTTON_PLAY,
    BUTTON_PREV
};

void button_init()
{
    gpio_config_t io_conf =
    {
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    for (int i = 0; i < 4; i++)
    {
        io_conf.pin_bit_mask = 1ULL << button_pins[i];
        gpio_config(&io_conf);
    }
}

int check_buttons(void)
{
    for (int i = 0; i < 4; i++)
    {
        if (gpio_get_level(button_pins[i]) == 0)
        {
            switch (i)
            {
                case 0:
                    ESP_LOGI("BUTTON", "Rotary pressed!");
                    break;
                case 1:
                    ESP_LOGI("BUTTON", "Skip pressed!");
                    break;
                case 2:
                    ESP_LOGI("BUTTON", "Play pressed!");
                    break;
                case 3:
                    ESP_LOGI("BUTTON", "Back pressed!");
                    break;
                default:
                    ESP_LOGI("BUTTON", "Button %d pressed!", i);
            }

            return i;
        }
    }
    return -1;
}

static int8_t rotary_table[16] =
{
    0, -1,  1,  0,
    1,  0,  0, -1,
   -1,  0,  0,  1,
    0,  1, -1,  0
};

void rotary_init(void)
{
    gpio_config_t io_conf =
    {
        .pin_bit_mask = (1ULL << ROT_ENC_VOLUP) | (1ULL << ROT_ENC_VOLDOWN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
}

void check_rotary_encoder(void)
{
    static uint8_t prev_state = 0;
    static int8_t steps_accum = 0;

    uint8_t a = gpio_get_level(ROT_ENC_VOLUP);
    uint8_t b = gpio_get_level(ROT_ENC_VOLDOWN);
    uint8_t current_state = (a << 1) | b;

    uint8_t index = (prev_state << 2) | current_state;
    int8_t delta = rotary_table[index];

    if (delta != 0)
    {
        steps_accum += delta;
        if (steps_accum >= 4)
        {
            steps_accum = 0;
            ESP_LOGI("ENCODER", "Volume Up!");
        }
        else if (steps_accum <= -4)
        {
            steps_accum = 0;
            ESP_LOGI("ENCODER", "Volume Down!");
        }
    }
    prev_state = current_state;
}

i2c_master_bus_handle_t i2c_init()
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
    return i2c_bus;
}

ssd1306_handle_t display_init(i2c_master_bus_handle_t i2c_bus)
{
    ssd1306_config_t panel_cfg = I2C_SSD1306_128x32_CONFIG_DEFAULT;
    ssd1306_handle_t panel_hdl;

    ssd1306_init(i2c_bus, &panel_cfg, &panel_hdl);
    if (panel_hdl == NULL)
    {
        ESP_LOGE("SSD1306", "ssd1306 handle init failed");
        assert(panel_hdl);
    }

    ssd1306_clear_display(panel_hdl, false);
    ssd1306_set_contrast(panel_hdl, 0xff);
    return panel_hdl;
}

void app_main()
{
    button_init();
    rotary_init();
    i2c_master_bus_handle_t i2c = i2c_init();
    ssd1306_handle_t display = display_init(i2c);
    ssd1306_display_text(display, 0, "Spotify Controller", false);
    ssd1306_display_text(display, 1, "Debug Mode", false);

    while (1)
    {
        check_rotary_encoder();
        int btn = check_buttons();
        if (btn != -1)
        {
            ssd1306_display_text(display, 2, "Input Detected!", false);
            char buf[32];
            snprintf(buf, sizeof(buf), "Button %d pressed", btn);
            ssd1306_display_text(display, 3, buf, false);
            vTaskDelay(pdMS_TO_TICKS(500));
            ssd1306_clear_display_page(display, 2, false);
            ssd1306_clear_display_page(display, 3, false);
        }
    }
}