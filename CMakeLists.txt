# SPDX-License-Identifier: Apache-2.0

if(CONFIG_DW3000)

# Utilisation de zephyr_library pour les fichiers sources
zephyr_library()

zephyr_library_sources(
    platform/deca_port.c
    platform/dw3000_hw.c
    platform/dw3000_spi.c
    platform/dw3000_spi_trace.c
    platform/deca_compat.c
    dwt_uwb_driver/deca_interface.c
    dwt_uwb_driver/deca_rsl.c
    dwt_uwb_driver/lib/qmath/src/qmath.c
)

zephyr_library_sources_ifdef(CONFIG_DW3000_CHIP_DW3000 dwt_uwb_driver/dw3000/dw3000_device.c)
zephyr_library_sources_ifdef(CONFIG_DW3000_CHIP_DW3720 dwt_uwb_driver/dw3720/dw3720_device.c)

zephyr_include_directories(platform)
zephyr_include_directories(dwt_uwb_driver)
zephyr_include_directories(dwt_uwb_driver/lib/qmath/include)

endif(CONFIG_DW3000)

# Chargement explicite du SDK Nordic obligatoire
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(uwb_receiver2)

# Fichier principal
target_sources(app PRIVATE src/main.c)

# Support BLE NUS (Nordic UART Service)
target_sources(app PRIVATE ${ZEPHYR_BASE}/../nrf/subsys/bluetooth/services/nus.c)
zephyr_include_directories(${ZEPHYR_BASE}/../nrf/subsys/bluetooth/services)
zephyr_include_directories(${ZEPHYR_BASE}/../nrf/include)
