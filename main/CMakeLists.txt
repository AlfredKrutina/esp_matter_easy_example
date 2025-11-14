# Check if using managed_components version
get_filename_component(MANAGED_ESP_MATTER_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../managed_components/espressif__esp_matter" ABSOLUTE)
if(EXISTS "${MANAGED_ESP_MATTER_PATH}/CMakeLists.txt")
    # Using managed_components - utils should be available through esp_matter component
    idf_component_register(SRC_DIRS          "."
                           PRIV_INCLUDE_DIRS  "."
                           REQUIRES           "led_strip")
elseif(DEFINED ESP_MATTER_PATH)
    # Using ESP_MATTER_PATH
    idf_component_register(SRC_DIRS          "."
                           PRIV_INCLUDE_DIRS  "." "${ESP_MATTER_PATH}/examples/common/utils"
                           REQUIRES           "led_strip")
else()
    idf_component_register(SRC_DIRS          "."
                           PRIV_INCLUDE_DIRS  "."
                           REQUIRES           "led_strip")
endif()

target_compile_options(${COMPONENT_LIB} PRIVATE "-DCHIP_HAVE_CONFIG_H")

# Include custom chip_project_config.h for high-concurrency support (25+ devices)
# This file overrides Matter session pool sizes and other limits
get_filename_component(CHIP_PROJECT_CONFIG_PATH "${CMAKE_CURRENT_SOURCE_DIR}/chip_project_config.h" ABSOLUTE)
if(EXISTS "${CHIP_PROJECT_CONFIG_PATH}")
    # Define CHIP_PROJECT_CONFIG_INCLUDE to include the file
    # Matter SDK will include this file via #include CHIP_PROJECT_CONFIG_INCLUDE
    target_compile_definitions(${COMPONENT_LIB} PRIVATE 
        CHIP_PROJECT_CONFIG_INCLUDE="${CHIP_PROJECT_CONFIG_PATH}")
    message(STATUS "Including chip_project_config.h from: ${CHIP_PROJECT_CONFIG_PATH}")
else()
    message(WARNING "chip_project_config.h not found at: ${CHIP_PROJECT_CONFIG_PATH}")
endif()


