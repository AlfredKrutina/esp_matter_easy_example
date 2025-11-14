/**
 * @file chip_project_config.h
 * @brief Custom Matter configuration for high-concurrency multi-device support
 * 
 * This file overrides default Matter configuration to support 25+ concurrent devices.
 * 
 * @note This file is included via CONFIG_CHIP_PROJECT_CONFIG in sdkconfig.defaults
 */

#pragma once

// Note: This file is included via CHIP_PROJECT_CONFIG_INCLUDE define
// which is set in main/CMakeLists.txt based on CONFIG_CHIP_PROJECT_CONFIG

#include <sdkconfig.h>

// CRITICAL: Increase secure session pool size for 25+ concurrent devices
// Default formula: MAX_FABRICS * 3 + 2 = 5 * 3 + 2 = 17 sessions (TOO LOW!)
// For 25+ devices, we need at least 40 sessions to handle all concurrent connections
// Each device can have 1-2 active sessions (CASE + subscription)
// Formula: (estimated_devices * 1.5) + overhead = (25 * 1.5) + 5 = 42.5 -> 40 sessions
#ifndef CHIP_CONFIG_SECURE_SESSION_POOL_SIZE
#define CHIP_CONFIG_SECURE_SESSION_POOL_SIZE 40
#endif

// Increase unauthenticated connection pool for pairing
// Default is 4, but for multiple devices pairing simultaneously we need more
#ifndef CHIP_CONFIG_UNAUTHENTICATED_CONNECTION_POOL_SIZE
#define CHIP_CONFIG_UNAUTHENTICATED_CONNECTION_POOL_SIZE 8
#endif

// Increase device max active devices (for OTA and bindings)
// Default is 4, but for 25+ devices we need more
#ifndef CHIP_CONFIG_DEVICE_MAX_ACTIVE_DEVICES
#define CHIP_CONFIG_DEVICE_MAX_ACTIVE_DEVICES 30
#endif

// Increase CASE client sessions per fabric
// Default is 2, but for multiple devices per fabric we need more
// Apple HomeKit can have multiple hubs (HomePod, Apple TV) per fabric
#ifndef CHIP_CONFIG_DEVICE_MAX_ACTIVE_CASE_CLIENTS
#define CHIP_CONFIG_DEVICE_MAX_ACTIVE_CASE_CLIENTS 10
#endif

// Increase subscription pool size for large networks
// Apple HomeKit networks can have many subscriptions (scenes, automations)
// Default is usually sufficient, but for large networks we may need more
#ifndef CHIP_CONFIG_MAX_SUBSCRIPTION_HANDLERS
#define CHIP_CONFIG_MAX_SUBSCRIPTION_HANDLERS 50
#endif

// Increase group message pool for multicast messages
// Large networks have more group communications
#ifndef CHIP_CONFIG_MAX_GROUP_MESSAGE_POOL_SIZE
#define CHIP_CONFIG_MAX_GROUP_MESSAGE_POOL_SIZE 20
#endif

