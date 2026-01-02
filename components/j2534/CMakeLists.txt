# J2534 Component CMakeLists.txt
# SAE J2534 Pass-Thru Protocol Implementation for WiCAN
#
# NOTE: stn_j2534 implementation is in main/stn_j2534.c (not refactored yet)
# The stn_j2534_*.c files in this component are incomplete and not used.

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
