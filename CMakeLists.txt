set(Python3_EXECUTABLE "C:/Users/angem/AppData/Local/Programs/Python/Python313/python.exe")
set(KCONFIG_WARN_AS_ERROR OFF CACHE BOOL "Disable Kconfig warnings as errors")
cmake_minimum_required(VERSION 3.20.0)
list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_LIST_DIR}/../nrf)

# Chargement explicite du SDK Nordic obligatoire
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(uwb_receiver2)

# Fichier principal
target_sources(app PRIVATE src/main.c)

# Ajouter manuellement les fichiers du driver ici
target_sources(app PRIVATE
    ../modules/hal/qorvo/platform/dw3000_hw.c
    ../modules/hal/qorvo/platform/dw3000_spi.c
    ../modules/hal/qorvo/platform/dw3000_spi_trace.c
    ../modules/hal/qorvo/platform/deca_port.c
    ../modules/hal/qorvo/platform/deca_compat.c
    ../modules/hal/qorvo/dwt_uwb_driver/dwt_uwb_driver/deca_interface.c
    ../modules/hal/qorvo/dwt_uwb_driver/dwt_uwb_driver/deca_rsl.c
    ../modules/hal/qorvo/dwt_uwb_driver/dwt_uwb_driver/lib/qmath/src/qmath.c
    ../modules/hal/qorvo/dwt_uwb_driver/dwt_uwb_driver/dw3000/dw3000_deca_regs.h
    ../modules/hal/qorvo/dwt_uwb_driver/dwt_uwb_driver/dw3000/dw3000_device.c
)

# Include headers
zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR})
zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../modules/hal/qorvo/dwt_uwb_driver/dwt_uwb_driver)
zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../modules/hal/qorvo/platform)
zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../modules/hal/qorvo/dwt_uwb_driver/dwt_uwb_driver/dw3000)
zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../modules/hal/qorvo/dwt_uwb_driver/dwt_uwb_driver/lib/qmath/include)

# Support BLE NUS (Nordic UART Service)
target_sources(app PRIVATE ${ZEPHYR_BASE}/../nrf/subsys/bluetooth/services/nus.c)
zephyr_include_directories(${ZEPHYR_BASE}/../nrf/subsys/bluetooth/services)
zephyr_include_directories(${ZEPHYR_BASE}/../nrf/include)
