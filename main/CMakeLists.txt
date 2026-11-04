# wl82loader.bin is JieLi's flash loader, from kagaimiq's jl-uboot-tool (not ours to ship): tools/fetch_deps.sh
# fetches it. Checked here.
set(LOADER "${CMAKE_CURRENT_LIST_DIR}/wl82loader.bin")
set(LOADER_SHA256 "d41da6126760c9d66660bcc0cac8d27d221806c5e369a8036921efe68dca5376")
if(NOT EXISTS "${LOADER}")
    message(FATAL_ERROR "main/wl82loader.bin missing: run tools/fetch_deps.sh")
endif()
file(SHA256 "${LOADER}" got)
if(NOT got STREQUAL LOADER_SHA256)
    message(FATAL_ERROR "main/wl82loader.bin has sha256 ${got}, expected ${LOADER_SHA256}")
endif()

idf_component_register(SRCS "main.c" "log.c" "recovery.c" "host.c" "jieli_uboot.c" "proto.c"
                       INCLUDE_DIRS "."
                       REQUIRES usb esp_driver_uart esp_driver_gpio esp_timer esp_hw_support
                       EMBED_FILES "wl82loader.bin")
