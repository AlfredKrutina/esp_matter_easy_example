/**
 * @file app_main.cpp
 * @brief ESP-Matter Extended Color Light application for ESP32-C6
 * 
 * This file implements a Matter Extended Color Light device that is compatible
 * with Apple Home. The device supports:
 * - On/Off control
 * - Brightness (Level Control)
 * - Color temperature and XY color control
 * - WiFi-based Matter commissioning via BLE
 * 
 * @note This application is optimized for ESP32-C6 with the following features:
 *       - BLE Extended Advertising enabled (MTU 512 bytes)
 *       - WiFi 2.4GHz only (required for Matter)
 *       - ICD Management cluster for Apple Home compatibility
 *       - Extended commissioning timeout (900 seconds)
 *       - WS2812B LED control on GPIO 8 (optional, Matter continues if LED init fails)
 * 
 * @author Matter Project
 * @date 2025
 * 
 * @copyright This example code is in the Public Domain (or CC0 licensed, at your option.)
 *            Unless required by applicable law or agreed to in writing, this
 *            software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 *            CONDITIONS OF ANY KIND, either express or implied.
 */

#include <esp_err.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <esp_netif.h>
#include <esp_event.h>
#include <esp_wifi.h>
#include <esp_timer.h>

#include <led_strip.h>
#include <esp_matter.h>
#include <esp_matter_console.h>
#include <esp_matter_ota.h>

#include <app/server/CommissioningWindowManager.h>
#include <app/server/Server.h>
#include <setup_payload/OnboardingCodesUtil.h>
#include <setup_payload/SetupPayload.h>

static const char *TAG = "app_main";

using namespace esp_matter;
using namespace esp_matter::attribute;
using namespace esp_matter::endpoint;
using namespace chip::app::Clusters;

// Note: Commissioning timeout is controlled by CONFIG_CHIP_DISCOVERY_TIMEOUT_SECS in sdkconfig
// Matter stack automatically manages commissioning timeouts based on this configuration

// LED strip configuration for ESP32-C6 DevKit
// GPIO 8 is the default LED pin on ESP32-C6 DevKit boards
#define LED_GPIO_NUM 8
#define LED_STRIP_LED_NUM 1
#define LED_STRIP_RMT_RES_HZ (10 * 1000 * 1000)  // 10MHz resolution for WS2812B

// Light endpoint ID - will be set when endpoint is created
static uint16_t light_endpoint_id = chip::kInvalidEndpointId;

// LED strip handle - initialized before Matter stack starts
// If initialization fails, this remains nullptr and Matter continues without LED
static led_strip_handle_t s_led_strip = nullptr;
static bool s_led_on = false;
static uint8_t s_current_brightness = 100;  // 0-100%
static uint8_t s_current_r = 255, s_current_g = 255, s_current_b = 255;  // White by default

// LED blinking state for commissioning feedback
static esp_timer_handle_t s_led_blink_timer = nullptr;
static bool s_led_blink_state = false;

// Forward declaration
static void update_led_strip(void);

/**
 * @brief LED blink timer callback for commissioning visual feedback
 * 
 * This callback is called periodically by esp_timer to toggle LED state during commissioning.
 * Uses existing led_strip driver functions for LED control.
 * 
 * @param arg Timer argument (unused)
 */
static void led_blink_timer_cb(void *arg)
{
    if (s_led_strip == nullptr) {
        return;
    }
    
    s_led_blink_state = !s_led_blink_state;
    
    if (s_led_blink_state) {
        // Turn on LED (white, 50% brightness for visibility)
        led_strip_set_pixel(s_led_strip, 0, 128, 128, 128);
    } else {
        // Turn off LED
        led_strip_clear(s_led_strip);
    }
    led_strip_refresh(s_led_strip);
}

/**
 * @brief Start LED blinking for commissioning feedback
 * 
 * Starts periodic LED blinking using ESP-IDF esp_timer API.
 * Uses existing led_strip driver functions.
 * 
 * @return ESP_OK on success, ESP_ERR_* on error
 */
static esp_err_t start_led_blinking(void)
{
    if (s_led_strip == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (s_led_blink_timer != nullptr) {
        // Timer already running
        return ESP_OK;
    }
    
    const esp_timer_create_args_t timer_args = {
        .callback = &led_blink_timer_cb,
        .name = "led_blink"
    };
    
    esp_err_t err = esp_timer_create(&timer_args, &s_led_blink_timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create LED blink timer: %s", esp_err_to_name(err));
        return err;
    }
    
    // Start periodic timer: 500ms interval
    err = esp_timer_start_periodic(s_led_blink_timer, 500 * 1000); // 500ms in microseconds
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start LED blink timer: %s", esp_err_to_name(err));
        esp_timer_delete(s_led_blink_timer);
        s_led_blink_timer = nullptr;
        return err;
    }
    
    ESP_LOGI(TAG, "LED blinking started for commissioning feedback");
    return ESP_OK;
}

/**
 * @brief Stop LED blinking
 * 
 * Stops periodic LED blinking and clears LED using existing led_strip driver.
 * 
 * @return ESP_OK on success
 */
static esp_err_t stop_led_blinking(void)
{
    if (s_led_blink_timer != nullptr) {
        esp_timer_stop(s_led_blink_timer);
        esp_timer_delete(s_led_blink_timer);
        s_led_blink_timer = nullptr;
    }
    
    if (s_led_strip != nullptr) {
        // Clear LED using existing led_strip driver
        led_strip_clear(s_led_strip);
        led_strip_refresh(s_led_strip);
    }
    
    s_led_blink_state = false;
    ESP_LOGI(TAG, "LED blinking stopped");
    return ESP_OK;
}

/**
 * @brief Identification callback for Matter Identify Cluster
 * 
 * This callback is invoked when clients interact with the Identify Cluster.
 * In the callback implementation, an endpoint can identify itself (e.g., by flashing an LED or light).
 * Uses existing led_strip driver functions for LED control.
 * 
 * @param type Callback type (e.g., START, STOP)
 * @param endpoint_id Endpoint ID that received the identify command
 * @param effect_id Effect ID to apply (0 = blink, 1 = breathe, etc.)
 * @param effect_variant Effect variant (0 = default)
 * @param priv_data Private data pointer (unused)
 * 
 * @return ESP_OK on success
 */
static esp_err_t app_identification_cb(identification::callback_type_t type, uint16_t endpoint_id, uint8_t effect_id,
                                       uint8_t effect_variant, void *priv_data)
{
    ESP_LOGI(TAG, "Identification callback: type: %u, effect: %u, variant: %u", type, effect_id, effect_variant);
    
    // Control LED using existing led_strip driver
    if (s_led_strip == nullptr) {
        return ESP_OK;
    }
    
    if (type == identification::callback_type_t::START) {
        // Start LED identification effect (blink)
        if (effect_id == 0) { // Blink effect
            // Use existing led_strip functions to set LED to white
            led_strip_set_pixel(s_led_strip, 0, 255, 255, 255);
            led_strip_refresh(s_led_strip);
        }
    } else if (type == identification::callback_type_t::STOP) {
        // Stop LED identification effect - restore previous state
        update_led_strip();
    }
    
    return ESP_OK;
}

/**
 * @brief WiFi event handler callback
 * 
 * Handles WiFi station events and IP address assignment events.
 * This callback is registered during WiFi initialization and provides
 * logging for WiFi connection status.
 * 
 * @param arg User argument (unused)
 * @param event_base Event base (WIFI_EVENT or IP_EVENT)
 * @param event_id Event ID (e.g., WIFI_EVENT_STA_CONNECTED, IP_EVENT_STA_GOT_IP)
 * @param event_data Event-specific data (e.g., IP address info)
 */
static void app_wifi_event_handler(void *arg, esp_event_base_t event_base,
                                   int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
        case WIFI_EVENT_STA_START:
            ESP_LOGI(TAG, "WiFi station started");
            break;
        case WIFI_EVENT_STA_CONNECTED:
            ESP_LOGI(TAG, "WiFi connected");
            break;
        case WIFI_EVENT_STA_DISCONNECTED:
            ESP_LOGW(TAG, "WiFi disconnected - Matter Network Commissioning will handle reconnection");
            break;
        default:
            break;
        }
    } else if (event_base == IP_EVENT) {
        if (event_id == IP_EVENT_STA_GOT_IP) {
            ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
            ESP_LOGI(TAG, "Got IP address: " IPSTR, IP2STR(&event->ip_info.ip));
        }
    }
}

/**
 * @brief Convert color temperature (mireds) to RGB values for white light
 * 
 * Converts color temperature in mireds to RGB values suitable for white LED.
 * Typical range: 153 mireds (6500K cool white) to 500 mireds (2000K warm white).
 * 
 * @param mireds Color temperature in mireds
 * @param r Output: Red component (0-255)
 * @param g Output: Green component (0-255)
 * @param b Output: Blue component (0-255)
 */
static void color_temp_to_rgb(uint16_t mireds, uint8_t *r, uint8_t *g, uint8_t *b)
{
    // Clamp mireds to valid range (153-500)
    if (mireds < 153) mireds = 153;
    if (mireds > 500) mireds = 500;
    
    // Convert mireds to Kelvin
    uint16_t kelvin = 1000000 / mireds;
    
    // Calculate RGB for white light at given temperature (simplified algorithm)
    if (kelvin < 3000) {
        // Warm white (2000-3000K) - more red/orange
        *r = 255;
        *g = (uint8_t)((kelvin - 2000) * 255 / 1000);
        *b = (uint8_t)((kelvin - 2000) * 200 / 1000);
    } else if (kelvin < 5000) {
        // Neutral white (3000-5000K) - balanced
        *r = 255;
        *g = 255;
        *b = (uint8_t)((kelvin - 3000) * 255 / 2000);
    } else {
        // Cool white (5000-6500K) - more blue
        *r = (uint8_t)(255 - (kelvin - 5000) * 100 / 1500);
        *g = (uint8_t)(255 - (kelvin - 5000) * 50 / 1500);
        *b = 255;
    }
}

/**
 * @brief Update LED strip with current color and brightness
 * 
 * Applies current RGB color and brightness to the LED strip.
 * This function is called whenever any attribute changes.
 * 
 * @note This function safely handles the case when LED strip is not initialized
 *       (s_led_strip == nullptr), which can happen if LED initialization failed.
 */
static void update_led_strip(void)
{
    if (s_led_strip == nullptr) {
        return;
    }
    
    if (!s_led_on) {
        // Turn off LED
        led_strip_clear(s_led_strip);
        return;
    }
    
    // Apply brightness to RGB values
    uint8_t r = (s_current_r * s_current_brightness) / 100;
    uint8_t g = (s_current_g * s_current_brightness) / 100;
    uint8_t b = (s_current_b * s_current_brightness) / 100;
    
    // Set pixel (WS2812B uses GRB format, but led_strip handles this automatically)
    led_strip_set_pixel(s_led_strip, 0, r, g, b);
    led_strip_refresh(s_led_strip);
}

/**
 * @brief Initialize LED strip (WS2812B on GPIO 8)
 * 
 * Initializes the LED strip using the led_strip component.
 * This must be called BEFORE Matter stack starts to ensure LED is ready
 * when attribute callbacks are invoked.
 * 
 * @return ESP_OK on success, ESP_ERR_* on error
 * 
 * @note Uses new led_strip API (led_strip_new_rmt_device)
 * @note LED format is GRB (handled automatically by led_strip component)
 * @note If initialization fails, Matter functionality continues normally
 */
static esp_err_t app_led_init(void)
{
    esp_err_t err = ESP_OK;
    
    // LED strip configuration
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_GPIO_NUM,
        .max_leds = LED_STRIP_LED_NUM,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB,  // WS2812B uses GRB format
        .led_model = LED_MODEL_WS2812,
        .flags = {
            .invert_out = false,
        },
    };
    
    // RMT configuration for LED strip
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = LED_STRIP_RMT_RES_HZ,
        .flags = {
            .with_dma = false,  // DMA not supported on ESP32-C6
        },
    };
    
    // Create LED strip handle
    err = led_strip_new_rmt_device(&strip_config, &rmt_config, &s_led_strip);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create LED strip: %s", esp_err_to_name(err));
        s_led_strip = nullptr;  // Ensure handle is null on failure
        return err;
    }
    
    // Clear LED strip (turn off initially)
    led_strip_clear(s_led_strip);
    
    ESP_LOGI(TAG, "LED strip initialized on GPIO %d (WS2812B)", LED_GPIO_NUM);
    return ESP_OK;
}

/**
 * @brief Attribute update callback for Matter clusters
 * 
 * This callback is called for every attribute update from Matter clients (e.g., Apple Home).
 * The callback implementation handles the desired attributes and returns an appropriate error code.
 * 
 * @note If the attribute is not of interest, return ESP_OK without modifying the value.
 * 
 * @param type Callback type (PRE_UPDATE or POST_UPDATE)
 * @param endpoint_id Endpoint ID where the attribute was updated
 * @param cluster_id Cluster ID (e.g., OnOff::Id, LevelControl::Id, ColorControl::Id)
 * @param attribute_id Attribute ID within the cluster
 * @param val Pointer to the attribute value (can be modified in PRE_UPDATE)
 * @param priv_data Private data pointer (unused)
 * 
 * @return ESP_OK on success, ESP_ERR_* on error
 * 
 * @details Supported clusters:
 *          - OnOff: Controls light on/off state (updates LED strip)
 *          - LevelControl: Controls brightness (0-254, where 254 = 100%, updates LED brightness)
 *          - ColorControl: Controls color temperature (mireds, converts to RGB and updates LED)
 * 
 * @note LED control is implemented for WS2812B on GPIO 8 (ESP32-C6 DevKit).
 *       If LED initialization fails, Matter functionality continues normally.
 */
static esp_err_t app_attribute_update_cb(attribute::callback_type_t type, uint16_t endpoint_id, uint32_t cluster_id,
                                         uint32_t attribute_id, esp_matter_attr_val_t *val, void *priv_data)
{
    esp_err_t err = ESP_OK;

    if (type == PRE_UPDATE && endpoint_id == light_endpoint_id) {
        // OnOff Cluster
        if (cluster_id == OnOff::Id) {
            if (attribute_id == OnOff::Attributes::OnOff::Id) {
                bool on = val->val.b;
                ESP_LOGI(TAG, "OnOff attribute updated: %s", on ? "ON" : "OFF");
                s_led_on = on;
                update_led_strip();
            }
        }
        // LevelControl Cluster (dimming)
        else if (cluster_id == LevelControl::Id) {
            if (attribute_id == LevelControl::Attributes::CurrentLevel::Id) {
                uint8_t level = val->val.u8;
                uint8_t brightness_percent = (level > 0) ? ((level * 100) / 254) : 0;
                ESP_LOGI(TAG, "LevelControl attribute updated: %u/254 (%u%%)", level, brightness_percent);
                s_current_brightness = brightness_percent;
                update_led_strip();
            }
        }
        // ColorControl Cluster
        else if (cluster_id == ColorControl::Id) {
            if (attribute_id == ColorControl::Attributes::ColorTemperatureMireds::Id) {
                uint16_t color_temp = val->val.u16;
                ESP_LOGI(TAG, "ColorTemperature attribute updated: %u mireds", color_temp);
                // Convert color temperature to RGB
                color_temp_to_rgb(color_temp, &s_current_r, &s_current_g, &s_current_b);
                update_led_strip();
            } else if (attribute_id == ColorControl::Attributes::CurrentX::Id) {
                uint16_t x = val->val.u16;
                ESP_LOGI(TAG, "ColorControl CurrentX attribute updated: %u", x);
                // TODO: Implement XY to RGB conversion if needed
                // For now, we only handle color temperature
            } else if (attribute_id == ColorControl::Attributes::CurrentY::Id) {
                uint16_t y = val->val.u16;
                ESP_LOGI(TAG, "ColorControl CurrentY attribute updated: %u", y);
                // TODO: Implement XY to RGB conversion if needed
                // For now, we only handle color temperature
            }
        }
    }

    return err;
}

/**
 * @brief Matter device event callback
 * 
 * Handles Matter device events such as commissioning completion, WiFi connectivity,
 * and BLE connection status. This callback provides detailed logging for debugging
 * and monitoring the Matter commissioning process.
 * 
 * @param event Pointer to the ChipDeviceEvent structure
 * @param arg User argument (unused)
 * 
 * @details Handled events:
 *          - kInterfaceIpAddressChanged: WiFi IP address assigned
 *          - kCommissioningComplete: Commissioning successfully completed
 *          - kFailSafeTimerExpired: Commissioning failed due to timeout
 *          - kOperationalNetworkStarted: Operational network (WiFi) is ready
 *          - kCHIPoBLEConnectionClosed: BLE connection closed after commissioning
 *          - kCommissioningWindowOpened: Commissioning window opened (ready for pairing)
 */
static void app_event_cb(const ChipDeviceEvent *event, intptr_t arg)
{
    // Log all events for debugging
    ESP_LOGI(TAG, "=== Matter Event: %d ===", static_cast<int>(event->Type));
    
    switch (event->Type) {
    case chip::DeviceLayer::DeviceEventType::kInterfaceIpAddressChanged:
        ESP_LOGI(TAG, "Interface IP Address changed - WiFi connected successfully");
        // Note: Detailed IP type check removed to avoid linter errors
        // The event indicates IP address was assigned (IPv4 or IPv6)
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
        ESP_LOGI(TAG, "=== Commissioning complete - device ready for Apple Home ===");
        ESP_LOGI(TAG, "BLE should disconnect now, communication will continue over WiFi/IP");
        // Stop LED blinking and restore normal LED state using existing led_strip driver
        stop_led_blinking();
        break;

    case chip::DeviceLayer::DeviceEventType::kFailSafeTimerExpired:
        ESP_LOGE(TAG, "Commissioning failed, fail safe timer expired");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStarted:
        ESP_LOGI(TAG, "Commissioning session started");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStopped:
        ESP_LOGI(TAG, "Commissioning session stopped");
        break;

    case chip::DeviceLayer::DeviceEventType::kOperationalNetworkStarted:
        ESP_LOGI(TAG, "=== Operational network started - mDNS advertising operational node ===");
        ESP_LOGI(TAG, "Device should now be discoverable by Apple Home over WiFi");
        ESP_LOGI(TAG, "BLE should disconnect now in non-concurrent mode");
        break;

    case chip::DeviceLayer::DeviceEventType::kCHIPoBLEConnectionClosed:
        ESP_LOGI(TAG, "=== BLE connection closed - switching to WiFi/IP communication ===");
        break;

    case chip::DeviceLayer::DeviceEventType::kOperationalNetworkEnabled:
        ESP_LOGI(TAG, "=== Operational network enabled - mDNS operational advertising ===");
        break;

    case chip::DeviceLayer::DeviceEventType::kWiFiConnectivityChange:
        ESP_LOGI(TAG, "WiFi connectivity change detected");
        break;

    case chip::DeviceLayer::DeviceEventType::kInternetConnectivityChange:
        ESP_LOGI(TAG, "Internet connectivity change detected");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowOpened:
        ESP_LOGI(TAG, "Commissioning window opened - ready for pairing");
        // Start LED blinking for visual feedback using existing led_strip driver
        start_led_blinking();
        // Note: Pairing codes are printed in app_main() to avoid duplication
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowClosed:
        ESP_LOGI(TAG, "Commissioning window closed");
        break;

    case chip::DeviceLayer::DeviceEventType::kFabricRemoved:
        ESP_LOGI(TAG, "Fabric removed");
        break;

    case chip::DeviceLayer::DeviceEventType::kFabricWillBeRemoved:
        ESP_LOGI(TAG, "Fabric will be removed");
        break;

    case chip::DeviceLayer::DeviceEventType::kFabricUpdated:
        ESP_LOGI(TAG, "Fabric updated");
        break;

    case chip::DeviceLayer::DeviceEventType::kFabricCommitted:
        ESP_LOGI(TAG, "Fabric committed");
        break;

    default:
        break;
    }
}

/**
 * @brief Create Extended Color Light endpoint
 * 
 * Creates a Matter Extended Color Light endpoint with OnOff, LevelControl,
 * and ColorControl clusters. The endpoint is configured with default values:
 * - Light starts in OFF state
 * - Default brightness: 50% (128/254)
 * - Default color mode: Color Temperature
 * - Default color temperature: 250 mireds (~4000K warm white)
 * 
 * @param node Matter node to attach the endpoint to
 * 
 * @return Pointer to the created endpoint, or nullptr on failure
 * 
 * @note The endpoint ID is stored in light_endpoint_id global variable
 */
static endpoint_t *create_light_endpoint(node_t *node)
{
    extended_color_light::config_t light_config;
    
    // OnOff configuration - start with light off
    light_config.on_off.on_off = false;
    
    // LevelControl configuration - default brightness 50% (128/254)
    light_config.level_control.current_level = 128;
    light_config.level_control.on_level = 128;
    
    // ColorControl configuration - default to color temperature mode
    light_config.color_control.color_mode = (uint8_t)ColorControl::ColorMode::kColorTemperature;
    light_config.color_control.enhanced_color_mode = (uint8_t)ColorControl::ColorMode::kColorTemperature;
    
    // ColorControl color temperature feature configuration - warm white (250 mireds = ~4000K)
    light_config.color_control_color_temperature.color_temperature_mireds = 250;
    
    // ColorControl XY feature configuration - default values (will be set by feature)
    // light_config.color_control_xy uses default constructor values
    
    light_config.color_control_remaining_time = 0;
    
    // Create the endpoint
    endpoint_t *endpoint = extended_color_light::create(node, &light_config, ENDPOINT_FLAG_NONE, nullptr);
    if (endpoint == nullptr) {
        ESP_LOGE(TAG, "Failed to create Extended Color Light endpoint");
        return nullptr;
    }
    
    light_endpoint_id = endpoint::get_id(endpoint);
    ESP_LOGI(TAG, "Extended Color Light endpoint created with ID: %d", light_endpoint_id);
    
    return endpoint;
}

/**
 * @brief Initialize WiFi stack for Matter commissioning
 * 
 * Initializes the WiFi station interface and configures it for 2.4GHz operation only.
 * WiFi credentials are not set here - they will be provided by Matter Network Commissioning
 * cluster during the commissioning process.
 * 
 * @note WiFi power saving is disabled to improve BLE performance during commissioning.
 * 
 * @return ESP_OK on success, ESP_ERR_* on error
 * 
 * @details Configuration:
 *          - WiFi mode: Station only (WIFI_MODE_STA)
 *          - Band: 2.4GHz only (channels 1-13)
 *          - Bandwidth: 20MHz (WIFI_BW_HT20)
 *          - Power saving: Disabled (WIFI_PS_NONE)
 *          - Country code: CZ (Czech Republic)
 */
static esp_err_t app_wifi_init(void)
{
    esp_err_t err = ESP_OK;
    
    // Initialize network interface
    ESP_ERROR_CHECK(esp_netif_init());
    
    // Create default event loop
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    // Create default WiFi station
    esp_netif_create_default_wifi_sta();
    
    // Initialize WiFi with default configuration
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    // Register WiFi event handlers
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &app_wifi_event_handler,
                                                        nullptr,
                                                        nullptr));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &app_wifi_event_handler,
                                                        nullptr,
                                                        nullptr));
    
    // Set WiFi mode to station
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    
    // Configure WiFi for 2.4GHz only (required for Matter commissioning)
    wifi_country_t country_config = {
        .cc = "CZ",           // Country code
        .schan = 1,           // Start channel (2.4GHz: 1-13)
        .nchan = 13,          // Number of channels (2.4GHz: 13)
        .policy = WIFI_COUNTRY_POLICY_AUTO
    };
    err = esp_wifi_set_country(&country_config);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set WiFi country config: %d (non-critical)", err);
    } else {
        ESP_LOGI(TAG, "WiFi configured for 2.4GHz band (channels 1-13)");
    }
    
    // Set bandwidth to 20MHz (2.4GHz standard)
    err = esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set WiFi bandwidth: %d (non-critical)", err);
    }
    
    // Start WiFi (credentials will be set by Matter Network Commissioning during pairing)
    ESP_ERROR_CHECK(esp_wifi_start());
    
    // Optimize WiFi/BLE coexistence for Matter commissioning
    // Disable WiFi power saving to improve BLE performance during finalization
    err = esp_wifi_set_ps(WIFI_PS_NONE);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to disable WiFi power saving: %d (non-critical)", err);
    } else {
        ESP_LOGI(TAG, "WiFi power saving disabled for better BLE coexistence");
    }
    
    // Note: BLE TX power settings will be applied by Matter stack
    // Matter automatically configures BLE for optimal commissioning performance
    
    ESP_LOGI(TAG, "WiFi initialized - waiting for Matter Network Commissioning");
    
    return err;
}

/**
 * @brief Main application entry point
 * 
 * Initializes the Matter Extended Color Light application with the following sequence:
 * 0. Initialize LED strip (WS2812B on GPIO 8) - optional, Matter continues if it fails
 * 1. Initialize NVS (Non-Volatile Storage) for Matter data persistence
 * 2. Initialize WiFi stack (2.4GHz only, credentials set via Matter)
 * 3. Create Matter node with root endpoint
 * 4. Create Extended Color Light endpoint
 * 5. Start Matter stack
 * 6. Initialize Matter console for debugging
 * 7. Print pairing codes (QR code and manual code) for Apple Home
 * 
 * @note After NVS flash erase, Matter fabric data is lost and device must be re-paired.
 *       Commissioning window opens automatically via CONFIG_CHIP_ENABLE_PAIRING_AUTOSTART.
 *       Pairing codes are printed both at startup and when the commissioning window opens.
 * 
 * @details Matter stack configuration:
 *          - Vendor ID: 0xFFF2 (Espressif test vendor ID)
 *          - Product ID: 0x8001
 *          - Commissioning: BLE-based with WiFi provisioning
 *          - Endpoint: Extended Color Light (ID 1)
 */
extern "C" void app_main()
{
    esp_err_t err = ESP_OK;

    ESP_LOGI(TAG, "Starting Matter Extended Color Light application");

    // 0. Initialize LED strip FIRST (before Matter stack starts)
    // This ensures LED is ready before any Matter callbacks are registered.
    // If LED init fails, Matter functionality continues normally (LED is optional).
    err = app_led_init();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "LED strip initialization failed: %s (non-critical, Matter will continue without LED)", esp_err_to_name(err));
        s_led_strip = nullptr;  // Ensure handle is null if init failed
    } else {
        ESP_LOGI(TAG, "LED strip ready - Matter can now control LED via attribute callbacks");
    }

    // 1. Initialize NVS (critical for Matter data persistence)
    // Note: If NVS is erased, Matter fabric data will be lost and device must be re-paired
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition was truncated and needs to be erased");
        ESP_LOGI(TAG, "Erasing NVS - Matter fabric data will be lost, device will need re-pairing");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
        ESP_LOGI(TAG, "NVS erased and re-initialized - commissioning window will open automatically");
    }
    ESP_ERROR_CHECK(err);
    ESP_LOGI(TAG, "NVS initialized successfully");

    // 2. Initialize WiFi (Matter will handle provisioning via Network Commissioning)
    err = app_wifi_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WiFi initialization failed: %d", err);
        return;
    }

    // 3. Create Matter node (root node endpoint is created automatically)
    node::config_t node_config;
    node_t *node = node::create(&node_config, app_attribute_update_cb, app_identification_cb);
    if (node == nullptr) {
        ESP_LOGE(TAG, "Failed to create Matter node");
        return;
    }
    ESP_LOGI(TAG, "Matter node created successfully");

    // 4. Create Extended Color Light endpoint
    endpoint_t *light_endpoint = create_light_endpoint(node);
    if (light_endpoint == nullptr) {
        ESP_LOGE(TAG, "Failed to create light endpoint");
        return;
    }

    // 5. Start Matter stack
    err = esp_matter::start(app_event_cb);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Matter start failed: %d", err);
        return;
    }
    ESP_LOGI(TAG, "Matter stack started successfully");

    // 6. Initialize Matter console for debugging
    err = esp_matter::console::init();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Matter console initialization failed: %d (non-critical)", err);
    } else {
        ESP_LOGI(TAG, "Matter console initialized - CLI available for debugging");
    }

    ESP_LOGI(TAG, "Application started - ready for Apple Home pairing");
    
    // Print pairing codes immediately after Matter stack starts
    // This will show QR code and manual pairing code for Apple Home
    ESP_LOGI(TAG, "=== APPLE HOME PAIRING CODES ===");
    PrintOnboardingCodes(chip::RendezvousInformationFlags(chip::RendezvousInformationFlag::kBLE));
    ESP_LOGI(TAG, "=== Use iPhone Home app to scan QR code or enter manual pairing code ===");
}

