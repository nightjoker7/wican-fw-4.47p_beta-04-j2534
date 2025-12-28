# J2534 Component CMakeLists.txt
# SAE J2534 Pass-Thru Protocol Implementation for WiCAN

idf_component_register(
    SRCS
        "j2534_core.c"
        "j2534_channel.c"
        "j2534_msg.c"
        "j2534_isotp.c"
        "j2534_ioctl.c"
        "j2534_cmd.c"
    INCLUDE_DIRS
        "include"
    REQUIRES
        driver
        esp_timer
        freertos
    PRIV_REQUIRES
        main
)
