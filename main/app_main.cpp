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
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <esp_heap_caps.h>
#include <esp_task_wdt.h>

#include <led_strip.h>
#include <esp_matter.h>
#include <esp_matter_console.h>
#include <esp_matter_ota.h>

#include <app/server/CommissioningWindowManager.h>
#include <app/server/Server.h>
#include <setup_payload/OnboardingCodesUtil.h>
#include <setup_payload/SetupPayload.h>
#include <lib/support/BitFlags.h>
#include <credentials/FabricTable.h>
#include <app/InteractionModelEngine.h>
#include <system/SystemClock.h>
#include <platform/PlatformManager.h>

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

// PILLAR 2: Thread-safety mutexes for LED state (CRITICAL for multi-client stability)
static SemaphoreHandle_t s_led_state_mutex = nullptr;   // Protects LED state variables
static SemaphoreHandle_t s_led_hw_mutex = nullptr;      // Protects LED hardware access
// FIX #3: Atomic flag to prevent race condition during mutex initialization
static volatile bool s_led_init_complete = false;       // Signals LED init is fully complete

// LED blinking state for commissioning feedback
static esp_timer_handle_t s_led_blink_timer = nullptr;
static bool s_led_blink_state = false;

// WiFi reconnection state for power loss recovery
static bool s_wifi_was_connected = false;
static uint32_t s_wifi_reconnect_attempts = 0;
static const uint32_t MAX_WIFI_RECONNECT_ATTEMPTS = 10;
static esp_timer_handle_t s_wifi_reconnect_timer = nullptr;

// PILLAR 4: WiFi instability detection (for active monitoring)
static uint32_t s_wifi_disconnect_count = 0;
static uint32_t s_wifi_last_disconnect_time = 0;
static const uint32_t WIFI_INSTABILITY_THRESHOLD = 5;  // 5 disconnects
static const uint32_t WIFI_INSTABILITY_WINDOW_MS = 30000;  // within 30 seconds
// FIX #7: Flag to prevent race condition during WiFi restart
static volatile bool s_wifi_restarting = false;  // Set during WiFi restart to prevent race

// Async LED update queue for non-blocking LED updates (stability improvement)
// This prevents blocking Matter callbacks when multiple controllers update attributes simultaneously
// FIX #8: Increased from 8 to 16 to prevent buffer overflow with multiple rapid commands
#define LED_UPDATE_QUEUE_SIZE 16
static QueueHandle_t s_led_update_queue = nullptr;
static TaskHandle_t s_led_update_task_handle = nullptr;

// PILLAR 6: NVS persistence state
static const char* NVS_NAMESPACE_LED = "led_state";
static bool s_led_state_dirty = false;  // Flag to indicate state needs saving

// Matter server ready flag - indicates ExchangeManager is fully initialized
static bool s_matter_server_ready = false;

// LED update message structure
typedef struct {
    bool on;
    uint8_t brightness;  // 0-100%
    uint8_t r, g, b;
} led_update_msg_t;

// Forward declarations
static void update_led_strip(void);
static void led_update_task(void *pvParameters);
static void wifi_reconnect_timer_cb(void *arg);

// PILLAR 1: LED init with retry
static esp_err_t app_led_init(void);  // Forward declaration
static esp_err_t app_led_init_with_retry(void);

// PILLAR 2: Thread-safe LED state access
static void update_led_state_safe(bool on, uint8_t r, uint8_t g, uint8_t b, uint8_t brightness);
static esp_err_t update_led_strip_safe(void);
static void get_led_state_safe(bool *on, uint8_t *r, uint8_t *g, uint8_t *b, uint8_t *brightness);

// PILLAR 3: Error recovery
typedef enum {
    RECOVERY_NONE,
    RECOVERY_RETRY,
    RECOVERY_RESTART_COMPONENT,
    RECOVERY_RESTART_DEVICE,
    RECOVERY_GRACEFUL_DEGRADE,
} recovery_strategy_t;
static recovery_strategy_t get_recovery_strategy(esp_err_t err);
static void handle_error(const char *subsystem, esp_err_t err);

// PILLAR 5: Health monitoring
static void health_monitor_task(void *arg);

// PILLAR 6: NVS persistence
static void save_led_state_to_nvs(void);
static void restore_led_state_from_nvs(void);
static void led_state_persistence_task(void *arg);

/**
 * @brief WiFi reconnect timer callback
 * 
 * Attempts to reconnect WiFi after power loss or disconnection.
 * Uses exponential backoff to avoid overwhelming the network.
 * 
 * @param arg Timer argument (unused)
 */
static void wifi_reconnect_timer_cb(void *arg)
{
    ESP_LOGI(TAG, "Attempting WiFi reconnection...");
    esp_err_t err = esp_wifi_connect();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "WiFi reconnect attempt failed: %s", esp_err_to_name(err));
    }
}

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
            {
                ESP_LOGI(TAG, "WiFi connected");
                s_wifi_was_connected = true;
                s_wifi_reconnect_attempts = 0;  // Reset reconnect attempts on successful connection
                s_wifi_disconnect_count = 0;    // PILLAR 4: Reset disconnect count
                
                // Stop reconnect timer if running
                if (s_wifi_reconnect_timer != nullptr) {
                    esp_timer_stop(s_wifi_reconnect_timer);
                }
            }
            break;
    case WIFI_EVENT_STA_DISCONNECTED:
        {
            wifi_event_sta_disconnected_t *disconnected = (wifi_event_sta_disconnected_t *)event_data;
            ESP_LOGW(TAG, "WiFi disconnected (reason: %d)", disconnected->reason);
            
            // PILLAR 4: Active WiFi instability detection
            uint32_t now = esp_timer_get_time() / 1000;  // Convert to milliseconds
            uint32_t elapsed = (s_wifi_last_disconnect_time > 0) ? (now - s_wifi_last_disconnect_time) : 0;
            s_wifi_last_disconnect_time = now;
            
            s_wifi_disconnect_count++;
            
            // FIX #7: Check for WiFi instability pattern (multiple disconnects in short time)
            // Use atomic flag to prevent race condition if another disconnect event arrives during restart
            if (!s_wifi_restarting && s_wifi_disconnect_count >= WIFI_INSTABILITY_THRESHOLD && elapsed < WIFI_INSTABILITY_WINDOW_MS) {
                ESP_LOGE(TAG, "⚠️ WiFi INSTABILITY DETECTED: %u disconnects in %lu ms - restarting WiFi", 
                         s_wifi_disconnect_count, elapsed);
                
                // Set flag to prevent race condition
                s_wifi_restarting = true;
                
                // Restart WiFi stack to recover from instability
                esp_wifi_stop();
                vTaskDelay(pdMS_TO_TICKS(1000));
                esp_wifi_start();
                
                // Reset counters AFTER restart
                s_wifi_disconnect_count = 0;
                s_wifi_reconnect_attempts = 0;
                
                // Clear flag after restart completes
                s_wifi_restarting = false;
                
                // Continue with normal reconnect logic
            } else if (s_wifi_restarting) {
                // Another disconnect event arrived during restart - ignore it
                ESP_LOGD(TAG, "WiFi disconnect event ignored (restart in progress)");
            }
            
            // Log disconnection reason for debugging stability issues
            // Common reasons: 2=authentication failed, 3=association failed, 4=handshake timeout, 8=beacon timeout
            if (disconnected->reason == WIFI_REASON_ASSOC_LEAVE) {
                ESP_LOGI(TAG, "WiFi disconnection: device left network (normal)");
            } else if (disconnected->reason == WIFI_REASON_AUTH_EXPIRE || 
                       disconnected->reason == WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT) {
                ESP_LOGW(TAG, "WiFi disconnection: authentication/security issue (reason: %d)", disconnected->reason);
            } else if (disconnected->reason == WIFI_REASON_BEACON_TIMEOUT ||
                       disconnected->reason == WIFI_REASON_NO_AP_FOUND) {
                ESP_LOGW(TAG, "WiFi disconnection: network/power loss issue (reason: %d) - will auto-reconnect", disconnected->reason);
            } else {
                ESP_LOGW(TAG, "WiFi disconnection: network issue (reason: %d) - will auto-reconnect", disconnected->reason);
            }
            
            // Auto-reconnect logic for power loss recovery
            // Only attempt reconnect if WiFi was previously connected (not during initial commissioning)
            if (s_wifi_was_connected) {
                s_wifi_reconnect_attempts++;
                
                if (s_wifi_reconnect_attempts <= MAX_WIFI_RECONNECT_ATTEMPTS) {
                    // Exponential backoff: 1s, 2s, 4s, 8s, 16s, 30s, 30s, ...
                    uint32_t delay_ms = (s_wifi_reconnect_attempts <= 5) ? 
                                        (1 << (s_wifi_reconnect_attempts - 1)) * 1000 : 30000;
                    
                    ESP_LOGI(TAG, "WiFi auto-reconnect attempt %u/%u in %lu ms", 
                             s_wifi_reconnect_attempts, MAX_WIFI_RECONNECT_ATTEMPTS, delay_ms);
                    
                    // Schedule reconnect attempt
                    if (s_wifi_reconnect_timer == nullptr) {
                        const esp_timer_create_args_t timer_args = {
                            .callback = wifi_reconnect_timer_cb,
                            .name = "wifi_reconnect"
                        };
                        esp_err_t timer_err = esp_timer_create(&timer_args, &s_wifi_reconnect_timer);
                        if (timer_err != ESP_OK) {
                            ESP_LOGE(TAG, "Failed to create WiFi reconnect timer: %s", esp_err_to_name(timer_err));
                            s_wifi_reconnect_timer = nullptr;
                        }
                    }
                    
                    esp_timer_stop(s_wifi_reconnect_timer);
                    esp_timer_start_once(s_wifi_reconnect_timer, delay_ms * 1000);
                } else {
                    ESP_LOGE(TAG, "WiFi reconnection failed after %u attempts - Matter Network Commissioning may be needed", 
                             MAX_WIFI_RECONNECT_ATTEMPTS);
                    // Reset flag to allow Matter Network Commissioning to handle it
                    s_wifi_was_connected = false;
                    s_wifi_reconnect_attempts = 0;
                }
            } else {
                // WiFi was never connected - Matter Network Commissioning will handle it
                ESP_LOGI(TAG, "WiFi not previously connected - Matter Network Commissioning will handle connection");
            }
        }
        break;
        default:
            break;
        }
    } else if (event_base == IP_EVENT) {
        if (event_id == IP_EVENT_STA_GOT_IP) {
            ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
            ESP_LOGI(TAG, "Got IP address: " IPSTR, IP2STR(&event->ip_info.ip));
            
            // WiFi reconnected successfully - reset reconnect attempts
            s_wifi_reconnect_attempts = 0;
            s_wifi_was_connected = true;
            
            // Stop reconnect timer if running
            if (s_wifi_reconnect_timer != nullptr) {
                esp_timer_stop(s_wifi_reconnect_timer);
            }
            
            ESP_LOGI(TAG, "WiFi reconnected successfully after power loss");
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
    // FIX #5: Clamp mireds to SAFE range to prevent integer underflow
    if (mireds < 153) mireds = 153;  // 6500K (cool white)
    if (mireds > 500) mireds = 500;  // 2000K (warm white)
    
    // FIX #5: Use float for accurate calculation and prevent integer underflow
    // Now kelvin is guaranteed to be in range [2000, 6500]
    float kelvin = 1000000.0f / (float)mireds;
    
    // FIX #5: Use signed intermediate calculations with clamping for safety
    if (kelvin < 3000.0f) {
        // Warm white (2000-3000K) - more red/orange
        *r = 255;
        int g_val = (int)((kelvin - 2000.0f) * 255.0f / 1000.0f);
        int b_val = (int)((kelvin - 2000.0f) * 200.0f / 1000.0f);
        *g = (uint8_t)((g_val < 0) ? 0 : ((g_val > 255) ? 255 : g_val));  // Clamp to [0, 255]
        *b = (uint8_t)((b_val < 0) ? 0 : ((b_val > 255) ? 255 : b_val));  // Clamp to [0, 255]
    } else if (kelvin < 5000.0f) {
        // Neutral white (3000-5000K) - balanced
        *r = 255;
        *g = 255;
        int b_val = (int)((kelvin - 3000.0f) * 255.0f / 2000.0f);
        *b = (uint8_t)((b_val < 0) ? 0 : ((b_val > 255) ? 255 : b_val));  // Clamp to [0, 255]
    } else {
        // Cool white (5000-6500K) - more blue
        int r_val = (int)(255.0f - (kelvin - 5000.0f) * 100.0f / 1500.0f);
        int g_val = (int)(255.0f - (kelvin - 5000.0f) * 50.0f / 1500.0f);
        *r = (uint8_t)((r_val < 0) ? 0 : ((r_val > 255) ? 255 : r_val));  // Clamp to [0, 255]
        *g = (uint8_t)((g_val < 0) ? 0 : ((g_val > 255) ? 255 : g_val));  // Clamp to [0, 255]
        *b = 255;
    }
}

/**
 * @brief PILLAR 2: Thread-safe LED state update
 * 
 * Updates LED state variables with mutex protection.
 * This prevents race conditions when multiple Matter clients update attributes simultaneously.
 * 
 * @param on LED on/off state
 * @param r Red component (0-255)
 * @param g Green component (0-255)
 * @param b Blue component (0-255)
 * @param brightness Brightness percentage (0-100)
 */
static void update_led_state_safe(bool on, uint8_t r, uint8_t g, uint8_t b, uint8_t brightness)
{
    // FIX #3: Check atomic flag first - prevents race condition during initialization
    if (!s_led_init_complete) {
        // During initialization - fallback is OK, but log it for debugging
        s_led_on = on;
        s_current_r = r;
        s_current_g = g;
        s_current_b = b;
        s_current_brightness = brightness;
        return;
    }
    
    if (s_led_state_mutex == nullptr) {
        // Mutex not initialized - fallback to unsafe update (should not happen after init)
        ESP_LOGW(TAG, "LED state mutex not initialized - using fallback");
        s_led_on = on;
        s_current_r = r;
        s_current_g = g;
        s_current_b = b;
        s_current_brightness = brightness;
        return;
    }
    
    if (xSemaphoreTake(s_led_state_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        s_led_on = on;
        s_current_r = r;
        s_current_g = g;
        s_current_b = b;
        s_current_brightness = brightness;
        s_led_state_dirty = true;  // Mark for NVS persistence
        xSemaphoreGive(s_led_state_mutex);
    }
}

/**
 * @brief PILLAR 2: Thread-safe LED state read
 * 
 * Reads LED state variables with mutex protection.
 * 
 * @param on Output: LED on/off state
 * @param r Output: Red component (0-255)
 * @param g Output: Green component (0-255)
 * @param b Output: Blue component (0-255)
 * @param brightness Output: Brightness percentage (0-100)
 */
static void get_led_state_safe(bool *on, uint8_t *r, uint8_t *g, uint8_t *b, uint8_t *brightness)
{
    if (s_led_state_mutex == nullptr) {
        // Mutex not initialized - fallback to unsafe read
        *on = s_led_on;
        *r = s_current_r;
        *g = s_current_g;
        *b = s_current_b;
        *brightness = s_current_brightness;
        return;
    }
    
    if (xSemaphoreTake(s_led_state_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        *on = s_led_on;
        *r = s_current_r;
        *g = s_current_g;
        *b = s_current_b;
        *brightness = s_current_brightness;
        xSemaphoreGive(s_led_state_mutex);
    }
}

/**
 * @brief PILLAR 2: Thread-safe LED hardware update
 * 
 * Updates LED hardware with mutex protection for hardware access.
 * This prevents concurrent hardware access from multiple tasks.
 * 
 * @return ESP_OK on success, ESP_ERR_* on error
 * 
 * @note This function is safe to call even if mutexes don't exist yet (during init),
 *       but LED hardware must be initialized (s_led_strip != nullptr).
 */
static esp_err_t update_led_strip_safe(void)
{
    // CRITICAL: Check LED hardware is initialized
    if (s_led_strip == nullptr) {
        return ESP_OK;  // LED not available - not an error
    }
    
    // Get current state safely (with fallback if mutexes don't exist)
    bool on;
    uint8_t r, g, b, brightness;
    get_led_state_safe(&on, &r, &g, &b, &brightness);
    
    // CRITICAL: Validate values to prevent crashes (sanity check)
    // Clamp brightness to valid range (0-100)
    if (brightness > 100) {
        ESP_LOGW(TAG, "Invalid brightness value: %u, clamping to 100", brightness);
        brightness = 100;
    }
    
    // RGB values should be 0-255 (already uint8_t, but ensure they're valid)
    // No need to clamp as uint8_t is already in valid range
    
    // Protect hardware access with mutex if available
    // If mutex doesn't exist yet (during init), use direct access (safe in single-threaded init)
    bool use_mutex = (s_led_hw_mutex != nullptr);
    
    if (use_mutex) {
        if (xSemaphoreTake(s_led_hw_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
            ESP_LOGW(TAG, "LED HW mutex timeout - skipping update");
            return ESP_ERR_TIMEOUT;
        }
    }
    
    esp_err_t result = ESP_OK;
    if (!on) {
        result = led_strip_clear(s_led_strip);
        if (result == ESP_OK) {
            result = led_strip_refresh(s_led_strip);
        }
    } else {
        // Calculate final RGB with brightness applied
        uint8_t fr = (r * brightness) / 100;
        uint8_t fg = (g * brightness) / 100;
        uint8_t fb = (b * brightness) / 100;
        
        // CRITICAL: Double-check LED strip handle is still valid before hardware access
        if (s_led_strip == nullptr) {
            if (use_mutex) {
                xSemaphoreGive(s_led_hw_mutex);
            }
            ESP_LOGW(TAG, "LED strip became invalid during update - skipping");
            return ESP_ERR_INVALID_STATE;
        }
        
        result = led_strip_set_pixel(s_led_strip, 0, fr, fg, fb);
        if (result == ESP_OK) {
            result = led_strip_refresh(s_led_strip);
        }
    }
    
    if (use_mutex) {
        xSemaphoreGive(s_led_hw_mutex);
    }
    
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "LED HW update failed: %s", esp_err_to_name(result));
        handle_error("LED_HW", result);
    }
    
    return result;
}

/**
 * @brief Update LED strip with current color and brightness (synchronous version)
 * 
 * Internal function that actually updates the LED hardware.
 * Called from the async LED update task for thread safety.
 * Uses thread-safe functions for state access and hardware update.
 * 
 * @param msg LED update message with color and brightness values
 */
static void update_led_strip_sync(const led_update_msg_t *msg)
{
    // Update state safely
    update_led_state_safe(msg->on, msg->r, msg->g, msg->b, msg->brightness);
    
    // Update hardware safely
    update_led_strip_safe();
}

/**
 * @brief Update LED strip with current color and brightness (async version)
 * 
 * Queues LED update to async task for non-blocking operation.
 * This prevents blocking Matter callbacks when multiple controllers update attributes simultaneously.
 * 
 * @note This function is thread-safe and can be called from any context (Matter callbacks, ISRs, etc.)
 * @note If queue is full, the update is dropped (fail-safe behavior)
 * @note Uses thread-safe state reading
 */
static void update_led_strip(void)
{
    // Get current state safely
    bool on;
    uint8_t r, g, b, brightness;
    get_led_state_safe(&on, &r, &g, &b, &brightness);
    
    if (s_led_update_queue == nullptr) {
        // Fallback to synchronous update if queue not initialized
        led_update_msg_t msg = {
            .on = on,
            .brightness = brightness,
            .r = r,
            .g = g,
            .b = b
        };
        update_led_strip_sync(&msg);
        return;
    }
    
    // Queue LED update for async processing
    led_update_msg_t msg = {
        .on = on,
        .brightness = brightness,
        .r = r,
        .g = g,
        .b = b
    };
    
        // Non-blocking send - if queue is full, drop the update (fail-safe)
        // FIX #8: Improved logging for queue overflow detection
        if (xQueueSend(s_led_update_queue, &msg, 0) != pdTRUE) {
            // Queue full - this indicates buffer overflow (should be rare with size 16)
            ESP_LOGE(TAG, "🔴 LED queue FULL - command DROPPED! (buffer overflow) - consider increasing LED_UPDATE_QUEUE_SIZE if this occurs frequently");
        }
}

/**
 * @brief Async LED update task
 * 
 * FreeRTOS task that processes LED updates from queue.
 * This ensures LED updates don't block Matter callbacks.
 * 
 * @param pvParameters Task parameters (unused)
 */
static void led_update_task(void *pvParameters)
{
    led_update_msg_t msg;
    
    // Feed watchdog periodically
    esp_task_wdt_add(NULL);
    
    while (1) {
        // Wait for LED update message (blocking wait with timeout for watchdog)
        if (xQueueReceive(s_led_update_queue, &msg, pdMS_TO_TICKS(1000)) == pdTRUE) {
            // Update LED hardware
            update_led_strip_sync(&msg);
        }
        
        // Feed watchdog
        esp_task_wdt_reset();
    }
}

/**
 * @brief PILLAR 3: Error recovery strategy
 * 
 * Determines recovery strategy based on error type.
 * 
 * @param err Error code
 * @return Recovery strategy to apply
 */
static recovery_strategy_t get_recovery_strategy(esp_err_t err)
{
    switch (err) {
    case ESP_ERR_NO_MEM:
        return RECOVERY_GRACEFUL_DEGRADE;
    case ESP_ERR_TIMEOUT:
        return RECOVERY_RETRY;
    case ESP_ERR_INVALID_STATE:
        return RECOVERY_RESTART_COMPONENT;
    case ESP_ERR_INVALID_CRC:  // NVS corruption
        return RECOVERY_RESTART_DEVICE;
    case ESP_ERR_NVS_NO_FREE_PAGES:
    case ESP_ERR_NVS_NEW_VERSION_FOUND:
        return RECOVERY_RESTART_DEVICE;
    default:
        return RECOVERY_NONE;
    }
}

// Flag to track if initialization is complete (prevents restart during init)
static bool s_initialization_complete = false;

/**
 * @brief PILLAR 3: Error handler with recovery strategy
 * 
 * Handles errors with appropriate recovery strategy.
 * 
 * @param subsystem Subsystem name (for logging)
 * @param err Error code
 * 
 * @note During initialization, device restart is disabled to prevent crashes.
 */
static void handle_error(const char *subsystem, esp_err_t err)
{
    ESP_LOGE(TAG, "❌ [%s] Error: %s (0x%x)", subsystem, esp_err_to_name(err), err);
    
    recovery_strategy_t strategy = get_recovery_strategy(err);
    
    switch (strategy) {
    case RECOVERY_NONE:
        ESP_LOGE(TAG, "   FATAL ERROR - no recovery");
        break;
    case RECOVERY_RETRY:
        ESP_LOGW(TAG, "   Retrying...");
        vTaskDelay(pdMS_TO_TICKS(1000));
        break;
    case RECOVERY_RESTART_COMPONENT:
        ESP_LOGW(TAG, "   Component restart required");
        break;
    case RECOVERY_RESTART_DEVICE:
        // CRITICAL: Don't restart during initialization (causes crashes)
        if (!s_initialization_complete) {
            ESP_LOGW(TAG, "   Device restart requested during init - deferring (will log error only)");
            ESP_LOGE(TAG, "   ERROR: Device restart required but deferred during initialization");
        } else {
            ESP_LOGE(TAG, "   DEVICE RESTART required");
            vTaskDelay(pdMS_TO_TICKS(3000));
            esp_restart();
        }
        break;
    case RECOVERY_GRACEFUL_DEGRADE:
        ESP_LOGW(TAG, "   Graceful degradation - continuing without feature");
        break;
    }
}

/**
 * @brief PILLAR 1: LED initialization with retry logic
 * 
 * Attempts LED initialization with retry logic (3 attempts).
 * If all attempts fail, gracefully degrades and continues without LED.
 * 
 * @return ESP_OK on success, ESP_ERR_* on error (but continues anyway)
 */
static esp_err_t app_led_init_with_retry(void)
{
    const int LED_INIT_RETRIES = 3;
    const int LED_INIT_RETRY_DELAY_MS = 500;
    esp_err_t err = ESP_OK;
    
    for (int attempt = 0; attempt < LED_INIT_RETRIES; attempt++) {
        ESP_LOGI(TAG, "LED init attempt %d/%d", attempt + 1, LED_INIT_RETRIES);
        
        err = app_led_init();
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "✅ LED initialized successfully");
            return ESP_OK;
        }
        
        ESP_LOGW(TAG, "LED init failed: %s", esp_err_to_name(err));
        
        if (attempt < LED_INIT_RETRIES - 1) {
            vTaskDelay(pdMS_TO_TICKS(LED_INIT_RETRY_DELAY_MS));
        }
    }
    
    // Graceful degradation: continue without LED
    ESP_LOGE(TAG, "❌ LED FAILED after %d attempts - starting WITHOUT LED", LED_INIT_RETRIES);
    s_led_strip = nullptr;
    return err;  // Return error, but app continues
}

/**
 * @brief PILLAR 6: Save LED state to NVS
 * 
 * Saves current LED state to NVS for persistence across power loss.
 * Called periodically or when state changes.
 */
static void save_led_state_to_nvs(void)
{
    if (!s_led_state_dirty) {
        return;  // No changes to save
    }
    
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE_LED, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to open NVS for LED state: %s", esp_err_to_name(err));
        return;
    }
    
    // Get state safely
    bool on;
    uint8_t r, g, b, brightness;
    get_led_state_safe(&on, &r, &g, &b, &brightness);
    
    // Save to NVS
    nvs_set_u8(handle, "on", on ? 1 : 0);
    nvs_set_u8(handle, "r", r);
    nvs_set_u8(handle, "g", g);
    nvs_set_u8(handle, "b", b);
    nvs_set_u8(handle, "brightness", brightness);
    
    err = nvs_commit(handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to commit LED state to NVS: %s", esp_err_to_name(err));
    } else {
        s_led_state_dirty = false;
        ESP_LOGD(TAG, "LED state saved to NVS");
    }
    
    nvs_close(handle);
}

/**
 * @brief PILLAR 6: Restore LED state from NVS
 * 
 * Restores LED state from NVS after power loss or reboot.
 * Called during initialization AFTER LED init (when mutexes exist).
 * 
 * @note This function directly sets state variables - mutexes are used if available,
 *       but if called before mutex creation, it sets variables directly (safe during init).
 */
static void restore_led_state_from_nvs(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE_LED, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        ESP_LOGD(TAG, "No saved LED state in NVS (first boot or NVS cleared)");
        return;
    }
    
    uint8_t on = 0, r = 255, g = 255, b = 255, brightness = 100;
    
    // Read with defaults (if key doesn't exist, use default value)
    nvs_get_u8(handle, "on", &on);
    nvs_get_u8(handle, "r", &r);
    nvs_get_u8(handle, "g", &g);
    nvs_get_u8(handle, "b", &b);
    nvs_get_u8(handle, "brightness", &brightness);
    
    nvs_close(handle);
    
    // CRITICAL: Use thread-safe update if mutexes exist (normal case after LED init)
    // If mutexes don't exist yet (shouldn't happen, but safe fallback), set directly
    if (s_led_state_mutex != nullptr) {
        update_led_state_safe(on == 1, r, g, b, brightness);
    } else {
        // Direct assignment during initialization (before mutexes created)
        // This is safe because we're in single-threaded init context
        s_led_on = (on == 1);
        s_current_r = r;
        s_current_g = g;
        s_current_b = b;
        s_current_brightness = brightness;
        s_led_state_dirty = false;  // Don't save immediately after restore
    }
    
    ESP_LOGI(TAG, "✅ Restored LED state from NVS: %s, RGB(%u,%u,%u), brightness %u%%", 
             on ? "ON" : "OFF", r, g, b, brightness);
}

/**
 * @brief PILLAR 6: LED state persistence task
 * 
 * Periodically saves LED state to NVS (every 30 seconds).
 * This ensures state is preserved across power loss.
 * 
 * @param arg Task argument (unused)
 * 
 * @note Watchdog timeout is 30s, so we feed it every 5s to be safe.
 */
static void led_state_persistence_task(void *arg)
{
    esp_err_t wdt_err = esp_task_wdt_add(NULL);
    if (wdt_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add led_persist task to watchdog: %s", esp_err_to_name(wdt_err));
    }
    
    // Track time for 30-second persistence interval
    uint32_t last_save_time = 0;
    const uint32_t SAVE_INTERVAL_MS = 30000;  // 30 seconds
    
    while (1) {
        // Feed watchdog every 5 seconds (much less than 30s timeout)
        esp_task_wdt_reset();
        
        // Delay 5 seconds (safe interval for watchdog)
        vTaskDelay(pdMS_TO_TICKS(5000));
        
        // Feed watchdog again
        esp_task_wdt_reset();
        
        // Check if it's time to save (every 30 seconds)
        uint32_t current_time = esp_timer_get_time() / 1000;  // Convert to milliseconds
        if (last_save_time == 0) {
            last_save_time = current_time;  // Initialize on first run
        }
        
        if ((current_time - last_save_time) >= SAVE_INTERVAL_MS) {
            // Feed watchdog before NVS operation
            esp_task_wdt_reset();
            
            if (s_led_state_dirty) {
                save_led_state_to_nvs();
            }
            
            last_save_time = current_time;
            
            // Feed watchdog after NVS operation
            esp_task_wdt_reset();
        }
    }
}

// NOTE: Commissioning window opening is handled AUTOMATICALLY by Matter SDK
// in Server::Init() if CONFIG_CHIP_ENABLE_PAIRING_AUTOSTART=y and FabricCount() == 0.
// This happens during Server::Init() when ExchangeManager is fully initialized,
// so it's safe and doesn't cause crashes.
//
// However, if fabric data still exists in NVS (even after removing from Apple Home),
// FabricCount() may not be 0, so window won't open automatically.
//
// We add a safety check after kServerReady to ensure window opens if needed.

/**
 * @brief PILLAR 5: Health monitoring task
 * 
 * Monitors system health (heap, stack, memory) and takes action if needed.
 * 
 * @param arg Task argument (unused)
 * 
 * @note Watchdog timeout is 30s, so we feed it every 3s to be safe.
 * @note Uses simple counter-based timing instead of esp_timer_get_time() to avoid blocking.
 */
static void health_monitor_task(void *arg)
{
    esp_err_t wdt_err = esp_task_wdt_add(NULL);
    if (wdt_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add health_monitor task to watchdog: %s", esp_err_to_name(wdt_err));
    }
    
    // Simple counter-based timing (every 10 seconds = 2 iterations of 5s delay)
    uint32_t check_counter = 0;
    const uint32_t CHECK_INTERVAL_ITERATIONS = 2;  // 2 * 5s = 10 seconds
    
    while (1) {
        // Feed watchdog BEFORE delay (critical - prevents timeout)
        esp_task_wdt_reset();
        
        // Delay 5 seconds (safe interval for watchdog - much less than 30s timeout)
        vTaskDelay(pdMS_TO_TICKS(5000));
        
        // Feed watchdog AFTER delay (before any operations)
        esp_task_wdt_reset();
        
        // Check if it's time to monitor (every 10 seconds = 2 iterations)
        check_counter++;
        if (check_counter >= CHECK_INTERVAL_ITERATIONS) {
            check_counter = 0;  // Reset counter
            
            // Feed watchdog before potentially long operations
            esp_task_wdt_reset();
            
            size_t free_heap = esp_get_free_heap_size();
            
            // Critical memory check
            if (free_heap < 20000) {
                ESP_LOGE(TAG, "❌ CRITICAL MEMORY: %zu bytes - restarting device", free_heap);
                esp_task_wdt_reset();  // Feed before restart
                vTaskDelay(pdMS_TO_TICKS(3000));
                esp_restart();
            } else if (free_heap < 50000) {
                ESP_LOGW(TAG, "⚠️ LOW MEMORY: %zu bytes free", free_heap);
                handle_error("Memory_Low", ESP_ERR_NO_MEM); // PILLAR 3: Error handling
            }
            
            // FIX #4: Stack monitoring - use quick check only (uxTaskGetStackHighWaterMark can block)
            // Feed watchdog BEFORE potentially blocking call
            esp_task_wdt_reset();
            UBaseType_t stack_high_water = uxTaskGetStackHighWaterMark(NULL);
            esp_task_wdt_reset();  // Feed immediately after to prevent starvation
            
            if (stack_high_water < 1024) {
                ESP_LOGW(TAG, "⚠️ LOW STACK: %u bytes remaining", stack_high_water * sizeof(StackType_t));
                handle_error("Stack_Low", ESP_FAIL); // PILLAR 3: Error handling
            }
            
            // Feed watchdog after all checks
            esp_task_wdt_reset();
        }
    }
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
 * @note Creates mutexes for thread-safety (PILLAR 2)
 */
static esp_err_t app_led_init(void)
{
    esp_err_t err = ESP_OK;
    
    // PILLAR 2: Create mutexes for thread-safety (CRITICAL for multi-client stability)
    if (s_led_state_mutex == nullptr) {
        s_led_state_mutex = xSemaphoreCreateMutex();
        if (s_led_state_mutex == nullptr) {
            ESP_LOGE(TAG, "Failed to create LED state mutex");
            return ESP_ERR_NO_MEM;
        }
    }
    
    if (s_led_hw_mutex == nullptr) {
        s_led_hw_mutex = xSemaphoreCreateMutex();
        if (s_led_hw_mutex == nullptr) {
            ESP_LOGE(TAG, "Failed to create LED HW mutex");
            vSemaphoreDelete(s_led_state_mutex);
            s_led_state_mutex = nullptr;
            return ESP_ERR_NO_MEM;
        }
    }
    
    // Create LED update queue for async processing (stability improvement)
    s_led_update_queue = xQueueCreate(LED_UPDATE_QUEUE_SIZE, sizeof(led_update_msg_t));
    if (s_led_update_queue == nullptr) {
        ESP_LOGE(TAG, "Failed to create LED update queue");
        // Continue without async queue (will use synchronous updates)
    } else {
        // Create LED update task
        BaseType_t task_result = xTaskCreate(
            led_update_task,
            "led_update",
            2048,  // Stack size (sufficient for LED operations)
            nullptr,
            5,  // Priority (lower than Matter tasks)
            &s_led_update_task_handle
        );
        
        if (task_result != pdPASS) {
            ESP_LOGE(TAG, "Failed to create LED update task");
            vQueueDelete(s_led_update_queue);
            s_led_update_queue = nullptr;
            // Continue without async queue (will use synchronous updates)
        } else {
            ESP_LOGI(TAG, "LED async update queue and task created");
        }
    }
    
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
        // Clean up queue and task if created
        if (s_led_update_queue != nullptr) {
            if (s_led_update_task_handle != nullptr) {
                vTaskDelete(s_led_update_task_handle);
                s_led_update_task_handle = nullptr;
            }
            vQueueDelete(s_led_update_queue);
            s_led_update_queue = nullptr;
        }
        return err;
    }
    
    // Clear LED strip (turn off initially)
    led_strip_clear(s_led_strip);
    led_strip_refresh(s_led_strip);
    
    // FIX #3: Set atomic flag AFTER all initialization is complete
    // This prevents race conditions if update_led_state_safe() is called during init
    s_led_init_complete = true;
    
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
    // FIX #6: NULL check to prevent crash if Matter framework passes nullptr
    if (val == nullptr) {
        ESP_LOGW(TAG, "app_attribute_update_cb: val is NULL for endpoint %u, cluster %u, attribute %u",
                 endpoint_id, cluster_id, attribute_id);
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t err = ESP_OK;

    if (type == PRE_UPDATE && endpoint_id == light_endpoint_id) {
        // OnOff Cluster
        if (cluster_id == OnOff::Id) {
            if (attribute_id == OnOff::Attributes::OnOff::Id) {
                bool on = val->val.b;
                ESP_LOGI(TAG, "OnOff attribute updated: %s", on ? "ON" : "OFF");
                
                // PILLAR 2: Use thread-safe state update
                bool current_on;
                uint8_t r, g, b, brightness;
                get_led_state_safe(&current_on, &r, &g, &b, &brightness);
                update_led_state_safe(on, r, g, b, brightness);
                update_led_strip();
            }
        }
        // LevelControl Cluster (dimming)
        else if (cluster_id == LevelControl::Id) {
            if (attribute_id == LevelControl::Attributes::CurrentLevel::Id) {
                uint8_t level = val->val.u8;
                uint8_t brightness_percent = (level > 0) ? ((level * 100) / 254) : 0;
                ESP_LOGI(TAG, "LevelControl attribute updated: %u/254 (%u%%)", level, brightness_percent);
                
                // PILLAR 2: Use thread-safe state update
                bool on;
                uint8_t r, g, b, current_brightness;
                get_led_state_safe(&on, &r, &g, &b, &current_brightness);
                update_led_state_safe(on, r, g, b, brightness_percent);
                update_led_strip();
            }
        }
        // ColorControl Cluster
        else if (cluster_id == ColorControl::Id) {
            if (attribute_id == ColorControl::Attributes::ColorTemperatureMireds::Id) {
                uint16_t color_temp = val->val.u16;
                ESP_LOGI(TAG, "ColorTemperature attribute updated: %u mireds", color_temp);
                
                // Convert color temperature to RGB
                uint8_t r, g, b;
                color_temp_to_rgb(color_temp, &r, &g, &b);
                
                // PILLAR 2: Use thread-safe state update
                bool on;
                uint8_t current_r, current_g, current_b, brightness;
                get_led_state_safe(&on, &current_r, &current_g, &current_b, &brightness);
                update_led_state_safe(on, r, g, b, brightness);
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
        ESP_LOGI(TAG, "=== Commissioning window opened - ready for pairing ===");
        ESP_LOGI(TAG, "Device is ready to be added to Apple Home");
        // Start LED blinking for visual feedback using existing led_strip driver
        start_led_blinking();
        // Print pairing codes when window opens (opened automatically by Matter SDK)
        ESP_LOGI(TAG, "=== APPLE HOME PAIRING CODES ===");
        // RendezvousInformationFlags is defined in SetupPayload.h as BitFlags<RendezvousInformationFlag, uint8_t>
        // Linter may show error but compiler resolves it correctly - matches Matter SDK pattern
        // See OnboardingCodesUtil.cpp:98 for reference
        PrintOnboardingCodes(chip::RendezvousInformationFlags(chip::RendezvousInformationFlag::kBLE));
        ESP_LOGI(TAG, "=== Use iPhone Home app to scan QR code or enter manual pairing code ===");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowClosed:
        ESP_LOGI(TAG, "Commissioning window closed");
        break;

    case chip::DeviceLayer::DeviceEventType::kFabricRemoved:
        {
            ESP_LOGI(TAG, "Fabric removed - device is now unpaired");
            ESP_LOGI(TAG, "Commissioning window will open automatically if all fabrics are removed");
            // NOTE: Matter SDK automatically opens commissioning window when FabricCount() becomes 0
            // via CONFIG_CHIP_ENABLE_PAIRING_AUTOSTART. We just need to wait for it.
            // The kCommissioningWindowOpened event will be triggered when it opens.
            // If it doesn't open automatically, the delayed check from kServerReady will handle it.
        }
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

    case chip::DeviceLayer::DeviceEventType::kServerReady:
        ESP_LOGI(TAG, "=== Matter Server Ready - ExchangeManager fully initialized ===");
        ESP_LOGI(TAG, "Server is now ready for commissioning window operations");
        // Set flag to indicate server is ready
        s_matter_server_ready = true;
        
        // NOTE: Matter SDK automatically opens commissioning window in Server::Init() if:
        // - CONFIG_CHIP_ENABLE_PAIRING_AUTOSTART=y (already set in sdkconfig)
        // - FabricCount() == 0 (no fabrics in NVS)
        // We do NOT manually open the window here because:
        // 1. OpenBasicCommissioningWindow() requires ExchangeManager to be fully initialized,
        //    which may not be the case even after kServerReady event
        // 2. Matter SDK handles this automatically in Server::Init() with proper timing
        // 3. If window doesn't open, it means FabricCount() > 0 (fabric data still in NVS)
        //    In that case, user should erase NVS or remove device from Apple Home properly
        ESP_LOGI(TAG, "Commissioning window will open automatically if device is not paired (FabricCount() == 0)");
        ESP_LOGI(TAG, "If window doesn't open, check NVS for remaining fabric data");
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
/**
 * @brief Monitor heap and stack usage (stability monitoring)
 * 
 * Logs heap and stack statistics for debugging stability issues.
 * Called periodically to detect memory leaks and stack overflows.
 * Also monitors Matter network stability (fabrics, sessions, subscriptions).
 */
static void monitor_system_resources(void)
{
    // Heap monitoring
    size_t free_heap = esp_get_free_heap_size();
    size_t min_free_heap = esp_get_minimum_free_heap_size();
    size_t largest_free_block = heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT);
    
    ESP_LOGI(TAG, "=== System Resources ===");
    ESP_LOGI(TAG, "Heap: free=%zu, min_free=%zu, largest_block=%zu", 
             free_heap, min_free_heap, largest_free_block);
    
    // Stack monitoring for main task
    UBaseType_t stack_high_water = uxTaskGetStackHighWaterMark(NULL);
    ESP_LOGI(TAG, "Main task stack high water mark: %u bytes", stack_high_water * sizeof(StackType_t));
    
    // Matter network stability monitoring
    ESP_LOGI(TAG, "=== Matter Network Status ===");
    
    // NOTE: Fabric count check removed - requires CHIP LOCK (must use ScheduleWork())
    // Matter SDK automatically opens commissioning window if FabricCount() == 0
    // (via CONFIG_CHIP_ENABLE_PAIRING_AUTOSTART), so no manual check needed
    
    // WiFi connection status
    if (s_wifi_was_connected) {
        wifi_ap_record_t ap_info;
        esp_err_t wifi_status = esp_wifi_sta_get_ap_info(&ap_info);
        if (wifi_status == ESP_OK) {
            ESP_LOGI(TAG, "WiFi: Connected (RSSI: %d dBm)", ap_info.rssi);
        } else {
            ESP_LOGW(TAG, "WiFi: Disconnected (status: %s)", esp_err_to_name(wifi_status));
        }
    } else {
        ESP_LOGI(TAG, "WiFi: Not yet connected (waiting for commissioning)");
    }
    
    // Warning if heap is getting low
    if (free_heap < 50000) {  // Less than 50KB free
        ESP_LOGW(TAG, "⚠️  Low heap memory warning: %zu bytes free", free_heap);
    }
    
    // Warning if stack is getting low
    if (stack_high_water < 1024) {  // Less than 1KB stack remaining
        ESP_LOGW(TAG, "⚠️  Low stack memory warning: %u bytes remaining", stack_high_water * sizeof(StackType_t));
    }
    
    // Warning if WiFi reconnect attempts are high
    if (s_wifi_reconnect_attempts > 0 && s_wifi_reconnect_attempts < MAX_WIFI_RECONNECT_ATTEMPTS) {
        ESP_LOGW(TAG, "⚠️  WiFi reconnect in progress: attempt %u/%u", 
                 s_wifi_reconnect_attempts, MAX_WIFI_RECONNECT_ATTEMPTS);
    } else if (s_wifi_reconnect_attempts >= MAX_WIFI_RECONNECT_ATTEMPTS) {
        ESP_LOGE(TAG, "❌ WiFi reconnection failed after %u attempts", MAX_WIFI_RECONNECT_ATTEMPTS);
    }
}

extern "C" void app_main()
{
    esp_err_t err = ESP_OK;

    ESP_LOGI(TAG, "Starting Matter Extended Color Light application");

    // Initialize watchdog timer for system stability
    // Watchdog is automatically initialized via CONFIG_ESP_TASK_WDT_INIT=y in sdkconfig
    // We just add the main task to be monitored (30s timeout configured in sdkconfig)
    err = esp_task_wdt_add(NULL);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to add main task to watchdog: %s (non-critical)", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Watchdog monitoring enabled for main task (30s timeout)");
    }
    
    // Log initial heap state
    ESP_LOGI(TAG, "Initial free heap: %zu bytes", esp_get_free_heap_size());

    // PILLAR 5: Create health monitoring task (before other initialization)
    xTaskCreate(health_monitor_task, "health_monitor", 3072, nullptr, 1, nullptr);
    ESP_LOGI(TAG, "Health monitoring task created");

    // 0. Initialize LED strip FIRST (before Matter stack starts)
    // This ensures LED is ready before any Matter callbacks are registered.
    // If LED init fails, Matter functionality continues normally (LED is optional).
    // PILLAR 1: Use retry logic for LED initialization
    err = app_led_init_with_retry();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "LED strip initialization failed after retries: %s (non-critical, Matter will continue without LED)", esp_err_to_name(err));
        s_led_strip = nullptr;  // Ensure handle is null if init failed
    } else {
        ESP_LOGI(TAG, "LED strip ready - Matter can now control LED via attribute callbacks");
    }

    // 1. Initialize NVS (critical for Matter data persistence)
    // Note: If NVS is erased, Matter fabric data will be lost and device must be re-paired
    // Enhanced error handling for stability
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition was truncated and needs to be erased");
        ESP_LOGI(TAG, "Erasing NVS - Matter fabric data will be lost, device will need re-pairing");
        
        // Erase NVS with retry logic for stability
        int retry_count = 0;
        const int max_retries = 3;
        while (retry_count < max_retries) {
            err = nvs_flash_erase();
            if (err == ESP_OK) {
                break;
            }
            retry_count++;
            ESP_LOGW(TAG, "NVS erase failed (attempt %d/%d): %s, retrying...", 
                     retry_count, max_retries, esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(100));  // Wait 100ms before retry
        }
        
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to erase NVS after %d attempts: %s", max_retries, esp_err_to_name(err));
            // Continue anyway - Matter will handle missing NVS gracefully
        } else {
            ESP_LOGI(TAG, "NVS erased successfully");
        }
        
        // Re-initialize NVS
        err = nvs_flash_init();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to re-initialize NVS after erase: %s", esp_err_to_name(err));
            // Continue anyway - Matter will handle missing NVS gracefully
        } else {
            ESP_LOGI(TAG, "NVS erased and re-initialized - commissioning window will open automatically");
        }
    } else if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS initialization failed: %s", esp_err_to_name(err));
        ESP_LOGE(TAG, "Matter functionality may be limited without NVS");
        // Continue anyway - Matter will handle missing NVS gracefully
    } else {
        ESP_LOGI(TAG, "NVS initialized successfully");
    }

    // PILLAR 6: Restore LED state from NVS (CRITICAL: must be AFTER LED init when mutexes exist)
    // Note: This must be called AFTER app_led_init() because:
    // 1. Mutexes must exist (created in app_led_init)
    // 2. LED hardware must be initialized
    // 3. State will be applied immediately after restore
    // NOTE: LED state restore is deferred until after Matter stack starts to avoid crashes
    // State will be applied later when system is fully initialized

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
    
    // Mark initialization as complete (allows error recovery to restart device if needed)
    s_initialization_complete = true;
    
    // CRITICAL: Commissioning window is opened AUTOMATICALLY by Matter SDK
    // in Server::Init() if CONFIG_CHIP_ENABLE_PAIRING_AUTOSTART=y and FabricCount() == 0.
    // 
    // This happens during Server::Init() when ExchangeManager is fully initialized,
    // so it's safe and doesn't cause crashes.
    //
    // We DO NOT manually open commissioning window - Matter SDK handles it automatically.
    // We just listen for kCommissioningWindowOpened event to start LED blinking and print codes.
    
    ESP_LOGI(TAG, "Commissioning window will open automatically if device is not paired");
    ESP_LOGI(TAG, "Listen for kCommissioningWindowOpened event for pairing codes");
    
    // FIX #2: LED State Persistence Timing - CORRECT ORDER:
    // 1. LED init creates queue (already done in app_led_init_with_retry())
    // 2. Wait for queue initialization to complete
    // 3. Create persistence task (task needs queue to be ready)
    // 4. Restore state from NVS
    // 5. Apply to hardware
    if (s_led_strip != nullptr) {
        // Wait for LED init to complete (queue is created in app_led_init())
        vTaskDelay(pdMS_TO_TICKS(500));  // Wait 500ms for queue initialization
        
        // Now create persistence task (queue is ready)
        xTaskCreate(led_state_persistence_task, "led_persist", 2048, nullptr, 1, nullptr);
        ESP_LOGI(TAG, "LED state persistence task created (queue ready)");
        
        // Wait for task to initialize
        vTaskDelay(pdMS_TO_TICKS(500));
        
        // Restore LED state from NVS
        restore_led_state_from_nvs();
        
        // Apply to hardware
        esp_err_t restore_err = update_led_strip_safe();
        if (restore_err == ESP_OK) {
            ESP_LOGI(TAG, "✅ Restored LED state from NVS and applied to hardware");
        } else {
            ESP_LOGW(TAG, "Failed to apply restored LED state: %s (non-critical, will retry later)", esp_err_to_name(restore_err));
        }
    }
    
    // Log system resources after initialization
    monitor_system_resources();
    
    // Main loop: feed watchdog, monitor system resources, and check WiFi connectivity
    while (1) {
        // Feed watchdog (must be done at least every 30 seconds)
        esp_task_wdt_reset();
        
        // Monitor system resources every 60 seconds
        static uint32_t last_monitor_time = 0;
        uint32_t current_time = esp_timer_get_time() / 1000000;  // Convert to seconds
        if (current_time - last_monitor_time >= 60) {
            monitor_system_resources();
            last_monitor_time = current_time;
            
            // Check WiFi connection status (for power loss detection)
            if (s_wifi_was_connected) {
                wifi_ap_record_t ap_info;
                esp_err_t wifi_status = esp_wifi_sta_get_ap_info(&ap_info);
                if (wifi_status != ESP_OK) {
                    ESP_LOGW(TAG, "WiFi connection lost - reconnect logic will handle it");
                }
            }
        }
        
        // Sleep for 1 second (watchdog timeout is 30s, so this is safe)
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

