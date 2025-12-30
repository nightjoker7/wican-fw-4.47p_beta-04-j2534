# J2534 Component CMakeLists.txt
# SAE J2534 Pass-Thru Protocol Implementation for WiCAN
#
# NOTE: This component needs access to main component headers (can.h, stn_j2534.h, hw_config.h)
# but we cannot use PRIV_REQUIRES main due to circular dependency (main requires j2534).
# Instead, we add main's include directory directly via PRIV_INCLUDE_DIRS.
# The actual linking works because main provides the symbols and both link into the final ELF.

idf_component_register(
    SRCS
        "j2534_core.c"
        "j2534_channel.c"
        "j2534_msg.c"
        "j2534_isotp.c"
        "j2534_ioctl.c"
        "j2534_cmd.c"
        "stn_j2534_usdt.c"
    INCLUDE_DIRS
        "include"
    PRIV_INCLUDE_DIRS
        "${CMAKE_SOURCE_DIR}/main"
    REQUIRES
        driver
        esp_timer
        freertos
)
