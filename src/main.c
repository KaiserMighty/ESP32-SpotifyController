#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <ssd1306.h>
#include <driver/i2c_master.h>
#include <driver/gpio.h>
#include "esp_log.h"
#include "string.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include <sys/socket.h>
#include <netinet/in.h>

#define I2C_MASTER_SDA_IO       23
#define I2C_MASTER_SCL_IO       22

#define ROT_ENC_VOLUP           16
#define ROT_ENC_VOLDOWN         17

#define ROT_ENC_BUTTON          4
#define BUTTON_NEXT             21
#define BUTTON_PLAY             19
#define BUTTON_PREV             18

#define WIFI_SSID               "SSID"
#define WIFI_PASS               "PASS"
#define BROADCAST_PORT          17388
#define BROADCAST_IP            "255.255.255.255"

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
        esp_wifi_connect();
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        ESP_LOGW("NETWORK", "Disconnected from WiFi. Trying to reconnect...");
        esp_wifi_connect();
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI("NETWORK", "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
    }
}

void wifi_init(void)
{
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL);

    wifi_config_t wifi_config =
    {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();
    esp_wifi_connect();
}

void send_udp_broadcast(const char *message)
{
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0)
    {
        ESP_LOGI("NETWORK", "Failed to create socket: errno %d", errno);
        return;
    }

    int broadcastEnable = 1;
    if (setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable)) < 0)
    {
        ESP_LOGE("NETWORK", "Failed to set broadcast option: errno %d", errno);
        close(sock);
        return;
    }

    struct sockaddr_in addr =
    {
        .sin_family = AF_INET,
        .sin_port = htons(BROADCAST_PORT),
        .sin_addr.s_addr = inet_addr(BROADCAST_IP),
    };

    wifi_ap_record_t ap_info;
    if (esp_wifi_sta_get_ap_info(&ap_info) != ESP_OK)
    {
        ESP_LOGE("NETWORK", "ESP32 is not connected to Wi-Fi");
        return;
    }

    if (sendto(sock, message, strlen(message), 0, (struct sockaddr *)&addr, sizeof(addr)) < 0)
        ESP_LOGE("NETWORK", "Error occurred during sending: errno %d", errno);

    close(sock);
}

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
                    send_udp_broadcast("SPOTIFY_CONTROLLER:ROTARY");
                    break;
                case 1:
                    ESP_LOGI("BUTTON", "Skip pressed!");
                    send_udp_broadcast("SPOTIFY_CONTROLLER:SKIP");
                    break;
                case 2:
                    ESP_LOGI("BUTTON", "Play pressed!");
                    send_udp_broadcast("SPOTIFY_CONTROLLER:PLAY");
                    break;
                case 3:
                    ESP_LOGI("BUTTON", "Back pressed!");
                    send_udp_broadcast("SPOTIFY_CONTROLLER:BACK");
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

int check_rotary_encoder(void)
{
    static uint8_t prev_state = 0;
    static int8_t steps_accum = 0;

    uint8_t a = gpio_get_level(ROT_ENC_VOLUP);
    uint8_t b = gpio_get_level(ROT_ENC_VOLDOWN);
    uint8_t current_state = (a << 1) | b;

    uint8_t index = (prev_state << 2) | current_state;
    int8_t delta = rotary_table[index];
    int ret = -1;

    if (delta != 0)
    {
        steps_accum += delta;
        if (steps_accum >= 4)
        {
            steps_accum = 0;
            ESP_LOGI("ENCODER", "Volume Up!");
            send_udp_broadcast("SPOTIFY_CONTROLLER:VOLUP");
            ret = 4;
        }
        else if (steps_accum <= -4)
        {
            steps_accum = 0;
            ESP_LOGI("ENCODER", "Volume Down!");
            send_udp_broadcast("SPOTIFY_CONTROLLER:VOLDOWN");
            ret = 5;
        }
    }
    prev_state = current_state;
    return ret;
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

void update_display(ssd1306_handle_t handle, int msg_id)
{
    switch (msg_id)
    {
        case 0:
            ssd1306_display_text(handle, 2, "Input Detected!", false);
            ssd1306_display_text(handle, 3, "Rotary Pressed", false);
            vTaskDelay(pdMS_TO_TICKS(300));
            ssd1306_clear_display_page(handle, 2, false);
            ssd1306_clear_display_page(handle, 3, false);
            break;
        case 1:
            ssd1306_display_text(handle, 2, "Input Detected!", false);
            ssd1306_display_text(handle, 3, "Next Pressed", false);
            vTaskDelay(pdMS_TO_TICKS(300));
            ssd1306_clear_display_page(handle, 2, false);
            ssd1306_clear_display_page(handle, 3, false);
            break;
        case 2:
            ssd1306_display_text(handle, 2, "Input Detected!", false);
            ssd1306_display_text(handle, 3, "Play Pressed", false);
            vTaskDelay(pdMS_TO_TICKS(300));
            ssd1306_clear_display_page(handle, 2, false);
            ssd1306_clear_display_page(handle, 3, false);
            break;
        case 3:
            ssd1306_display_text(handle, 2, "Input Detected!", false);
            ssd1306_display_text(handle, 3, "Back Pressed", false);
            vTaskDelay(pdMS_TO_TICKS(300));
            ssd1306_clear_display_page(handle, 2, false);
            ssd1306_clear_display_page(handle, 3, false);
            break;
        case 4:
            ssd1306_display_text(handle, 2, "Input Detected!", false);
            ssd1306_display_text(handle, 3, "Volume Up", false);
            vTaskDelay(pdMS_TO_TICKS(2));
            ssd1306_clear_display_page(handle, 2, false);
            ssd1306_clear_display_page(handle, 3, false);
            break;
        case 5:
            ssd1306_display_text(handle, 2, "Input Detected!", false);
            ssd1306_display_text(handle, 3, "Volume Down", false);
            vTaskDelay(pdMS_TO_TICKS(2));
            ssd1306_clear_display_page(handle, 2, false);
            ssd1306_clear_display_page(handle, 3, false);
            break;
        default:
            ssd1306_display_text(handle, 2, "Input Detected!", false);
            ssd1306_display_text(handle, 3, "Undefined Button", false);
            vTaskDelay(pdMS_TO_TICKS(300));
            ssd1306_clear_display_page(handle, 2, false);
            ssd1306_clear_display_page(handle, 3, false);
    }
}

void app_main()
{
    button_init();
    rotary_init();
    i2c_master_bus_handle_t i2c = i2c_init();
    ssd1306_handle_t display = display_init(i2c);
    ssd1306_display_text(display, 0, "Spotify Controller", false);
    ssd1306_display_text(display, 1, "Debug Mode", false);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    wifi_init();

    while (1)
    {
        int msg = check_rotary_encoder();
        if (msg != -1)
        {
            update_display(display, msg);
        }
        msg = check_buttons();
        if (msg != -1)
        {
            update_display(display, msg);
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}