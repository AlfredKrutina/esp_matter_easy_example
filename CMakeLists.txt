# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

# Check for ESP-Matter - priority: ESP_MATTER_PATH > managed_components > idf_component.yml
# Note: managed_components may not exist yet after fullclean, but will be downloaded automatically

# First check ESP_MATTER_PATH (highest priority)
if(DEFINED ENV{ESP_MATTER_PATH})
    # Using ESP_MATTER_PATH environment variable
    set(USING_MANAGED_COMPONENTS FALSE)
    set(ESP_MATTER_PATH $ENV{ESP_MATTER_PATH})
    set(MATTER_SDK_PATH ${ESP_MATTER_PATH}/connectedhomeip/connectedhomeip)
    
    if(NOT DEFINED ENV{ESP_MATTER_DEVICE_PATH})
        # Try to get IDF_TARGET from sdkconfig if not set as CMake variable
        if(NOT DEFINED IDF_TARGET)
            if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/sdkconfig")
                file(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/sdkconfig" SDKCONFIG_CONTENT)
                foreach(line ${SDKCONFIG_CONTENT})
                    if(line MATCHES "^CONFIG_IDF_TARGET=\"([^\"]+)\"")
                        set(IDF_TARGET ${CMAKE_MATCH_1})
                        break()
                    endif()
                endforeach()
            endif()
        endif()
        
        # Default to esp32 if not set
        if(NOT DEFINED IDF_TARGET)
            set(IDF_TARGET "esp32")
        endif()
        
        if("${IDF_TARGET}" STREQUAL "esp32" OR "${IDF_TARGET}" STREQUAL "")
            set(ENV{ESP_MATTER_DEVICE_PATH} $ENV{ESP_MATTER_PATH}/device_hal/device/esp32_devkit_c)
        elseif("${IDF_TARGET}" STREQUAL "esp32c3")
            set(ENV{ESP_MATTER_DEVICE_PATH} $ENV{ESP_MATTER_PATH}/device_hal/device/esp32c3_devkit_m)
        elseif("${IDF_TARGET}" STREQUAL "esp32c2")
            set(ENV{ESP_MATTER_DEVICE_PATH} $ENV{ESP_MATTER_PATH}/device_hal/device/esp32c2_devkit_m)
        elseif("${IDF_TARGET}" STREQUAL "esp32h2")
            set(ENV{ESP_MATTER_DEVICE_PATH} $ENV{ESP_MATTER_PATH}/device_hal/device/esp32h2_devkit_c)
        elseif("${IDF_TARGET}" STREQUAL "esp32s3")
            set(ENV{ESP_MATTER_DEVICE_PATH} $ENV{ESP_MATTER_PATH}/device_hal/device/esp32s3_devkit_c)
        elseif("${IDF_TARGET}" STREQUAL "esp32c6")
            set(ENV{ESP_MATTER_DEVICE_PATH} $ENV{ESP_MATTER_PATH}/device_hal/device/esp32c6_devkit_c)
        elseif("${IDF_TARGET}" STREQUAL "esp32c5")
            set(ENV{ESP_MATTER_DEVICE_PATH} $ENV{ESP_MATTER_PATH}/device_hal/device/esp32c5_devkit_c)
        elseif("${IDF_TARGET}" STREQUAL "esp32p4")
            set(ENV{ESP_MATTER_DEVICE_PATH} $ENV{ESP_MATTER_PATH}/device_hal/device/hollow)
        else()
            message(FATAL_ERROR "Unsupported IDF_TARGET: ${IDF_TARGET}")
        endif()
        message(STATUS "ESP_MATTER_DEVICE_PATH set to: $ENV{ESP_MATTER_DEVICE_PATH} for IDF_TARGET: ${IDF_TARGET}")
    endif()
    
    message(STATUS "Using ESP-Matter from ESP_MATTER_PATH: ${ESP_MATTER_PATH}")
else()
    # Check if managed_components already exists (after first build)
    get_filename_component(MANAGED_ESP_MATTER_PATH "${CMAKE_CURRENT_SOURCE_DIR}/managed_components/espressif__esp_matter" ABSOLUTE)
    if(EXISTS "${MANAGED_ESP_MATTER_PATH}/CMakeLists.txt")
        # Using managed_components version - ESP-Matter will be included automatically
        set(USING_MANAGED_COMPONENTS TRUE)
        message(STATUS "Using ESP-Matter from managed_components")
    else()
        # Check if idf_component.yml contains esp_matter (will be downloaded automatically)
        if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/main/idf_component.yml")
            file(READ "${CMAKE_CURRENT_SOURCE_DIR}/main/idf_component.yml" COMPONENT_YML)
            if(COMPONENT_YML MATCHES "espressif/esp_matter")
                # ESP-Matter is in idf_component.yml, will be downloaded automatically
                set(USING_MANAGED_COMPONENTS TRUE)
                message(STATUS "ESP-Matter found in idf_component.yml - will be downloaded automatically")
            else()
                message(FATAL_ERROR "ESP-Matter not found. Please either:\n1. Add 'espressif/esp_matter: \"*\"' to main/idf_component.yml, or\n2. Set ESP_MATTER_PATH environment variable")
            endif()
        else()
            message(FATAL_ERROR "ESP-Matter not found. Please either:\n1. Create main/idf_component.yml with 'espressif/esp_matter: \"*\"', or\n2. Set ESP_MATTER_PATH environment variable")
        endif()
    endif()
endif()

set(PROJECT_VER "1.0")
set(PROJECT_VER_NUMBER 1)

# This should be done before using the IDF_TARGET variable.
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

if(NOT USING_MANAGED_COMPONENTS)
    include(${ESP_MATTER_PATH}/examples/common/cmake_common/components_include.cmake)
    include($ENV{ESP_MATTER_DEVICE_PATH}/esp_matter_device.cmake)
    
    set(EXTRA_COMPONENT_DIRS
        "${ESP_MATTER_PATH}/examples/common"
        "${MATTER_SDK_PATH}/config/esp32/components"
        "${ESP_MATTER_PATH}/components"
        "${ESP_MATTER_PATH}/device_hal/device"
        ${extra_components_dirs_append})
endif()

project(matter_pokus_v2)

idf_build_set_property(CXX_COMPILE_OPTIONS "-std=gnu++17;-Os;-DCHIP_HAVE_CONFIG_H;-Wno-overloaded-virtual" APPEND)
idf_build_set_property(C_COMPILE_OPTIONS "-Os" APPEND)
# For RISCV chips, project_include.cmake sets -Wno-format, but does not clear various
# flags that depend on -Wformat
idf_build_set_property(COMPILE_OPTIONS "-Wno-format-nonliteral;-Wno-format-security" APPEND)


