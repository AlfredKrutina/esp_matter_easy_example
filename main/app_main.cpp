/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <esp_err.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <esp_netif.h>
#include <esp_event.h>
#include <esp_wifi.h>

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

// Increased timeout to 600 seconds (10 minutes) to handle slow BLE communication during finalization
constexpr auto k_timeout_seconds = 600;

// Light endpoint ID - will be set when endpoint is created
static uint16_t light_endpoint_id = chip::kInvalidEndpointId;

// This callback is invoked when clients interact with the Identify Cluster.
// In the callback implementation, an endpoint can identify itself. (e.g., by flashing an LED or light).
static esp_err_t app_identification_cb(identification::callback_type_t type, uint16_t endpoint_id, uint8_t effect_id,
                                       uint8_t effect_variant, void *priv_data)
{
    ESP_LOGI(TAG, "Identification callback: type: %u, effect: %u, variant: %u", type, effect_id, effect_variant);
    return ESP_OK;
}

// WiFi event callback
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

// This callback is called for every attribute update. The callback implementation shall
// handle the desired attributes and return an appropriate error code. If the attribute
// is not of your interest, please do not return an error code and strictly return ESP_OK.
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
                // TODO: Add hardware control here (e.g., LED driver)
            }
        }
        // LevelControl Cluster (dimming)
        else if (cluster_id == LevelControl::Id) {
            if (attribute_id == LevelControl::Attributes::CurrentLevel::Id) {
                uint8_t level = val->val.u8;
                uint8_t brightness_percent = (level > 0) ? ((level * 100) / 254) : 0;
                ESP_LOGI(TAG, "LevelControl attribute updated: %u/254 (%u%%)", level, brightness_percent);
                // TODO: Add hardware control here (e.g., LED brightness)
            }
        }
        // ColorControl Cluster
        else if (cluster_id == ColorControl::Id) {
            if (attribute_id == ColorControl::Attributes::ColorTemperatureMireds::Id) {
                uint16_t color_temp = val->val.u16;
                ESP_LOGI(TAG, "ColorTemperature attribute updated: %u mireds", color_temp);
                // TODO: Add hardware control here (e.g., LED color temperature)
            } else if (attribute_id == ColorControl::Attributes::CurrentX::Id) {
                uint16_t x = val->val.u16;
                ESP_LOGI(TAG, "ColorControl CurrentX attribute updated: %u", x);
                // TODO: Add hardware control here
            } else if (attribute_id == ColorControl::Attributes::CurrentY::Id) {
                uint16_t y = val->val.u16;
                ESP_LOGI(TAG, "ColorControl CurrentY attribute updated: %u", y);
                // TODO: Add hardware control here
            }
        }
    }

    return err;
}

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
        // Print pairing codes for Apple Home
        PrintOnboardingCodes(chip::RendezvousInformationFlags(chip::RendezvousInformationFlag::kBLE));
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

// Create Extended Color Light endpoint
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

// Initialize WiFi (Matter will handle WiFi provisioning through Network Commissioning cluster)
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

extern "C" void app_main()
{
    esp_err_t err = ESP_OK;

    ESP_LOGI(TAG, "Starting Matter Extended Color Light application");

    // 1. Initialize NVS (critical for Matter data persistence)
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition was truncated and needs to be erased");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    ESP_LOGI(TAG, "NVS initialized");

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

