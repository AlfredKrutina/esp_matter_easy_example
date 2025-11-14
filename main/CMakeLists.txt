# Check if using managed_components version
get_filename_component(MANAGED_ESP_MATTER_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../managed_components/espressif__esp_matter" ABSOLUTE)
if(EXISTS "${MANAGED_ESP_MATTER_PATH}/CMakeLists.txt")
    # Using managed_components - utils should be available through esp_matter component
    idf_component_register(SRC_DIRS          "."
                           PRIV_INCLUDE_DIRS  ".")
elseif(DEFINED ESP_MATTER_PATH)
    # Using ESP_MATTER_PATH
    idf_component_register(SRC_DIRS          "."
                           PRIV_INCLUDE_DIRS  "." "${ESP_MATTER_PATH}/examples/common/utils")
else()
    idf_component_register(SRC_DIRS          "."
                           PRIV_INCLUDE_DIRS  ".")
endif()

target_compile_options(${COMPONENT_LIB} PRIVATE "-DCHIP_HAVE_CONFIG_H")


