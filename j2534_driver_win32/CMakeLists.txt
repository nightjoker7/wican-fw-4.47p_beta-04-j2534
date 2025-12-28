cmake_minimum_required(VERSION 3.15)
project(wican_j2534 VERSION 1.0.0 LANGUAGES C)

# Set C standard
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Windows-specific settings
if(WIN32)
    add_definitions(-D_WIN32_WINNT=0x0601)  # Windows 7+
    add_definitions(-DWIN32_LEAN_AND_MEAN)
    add_definitions(-DWINVER=0x0601)
endif()

# Source files for DLL
set(SOURCES
    wican_j2534.c
    wican_comm.c
)

# Header files
set(HEADERS
    j2534_api.h
    wican_comm.h
)

# Create DLL
add_library(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})

# Module definition file for exports
target_sources(${PROJECT_NAME} PRIVATE wican_j2534.def)

# Link Winsock and advapi32 (for registry) library
target_link_libraries(${PROJECT_NAME} PRIVATE ws2_32 advapi32)

# Set output name without lib prefix
set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX "")

# ============================================================================
# Configuration Utility
# ============================================================================
add_executable(wican_config WIN32 wican_config.c)
target_link_libraries(wican_config PRIVATE ws2_32 comctl32 advapi32)
set_target_properties(wican_config PROPERTIES OUTPUT_NAME "WiCAN_Config")

# Install rules
install(TARGETS ${PROJECT_NAME} wican_config
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
)

# Copy DLL to output directory with proper name
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E echo "WiCAN J2534 Driver built successfully!"
)

# Create 32-bit build configuration
if(CMAKE_SIZEOF_VOID_P EQUAL 4)
    message(STATUS "Building 32-bit J2534 driver")
    set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME "wican_j2534_32")
else()
    message(STATUS "Building 64-bit J2534 driver")
    set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME "wican_j2534_64")
endif()
