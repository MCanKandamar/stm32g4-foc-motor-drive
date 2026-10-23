# CANopen yığını: CANopenNode (dokunulmaz) + STM32 FDCAN sürücüsü + Object Dictionary.
# Sürümler için VERSIONS.md'ye bak.
#
# Kullanılmayan özellikler (LSS, GFC, SRDO, ...) derlenmeye girse de
# CANopenNode'un kendi CO_CONFIG_* bayraklarıyla devre dışı kalır.
file(GLOB CANOPEN_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/CANopenNode/CANopen.c
    ${CMAKE_CURRENT_SOURCE_DIR}/CANopenNode/301/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/CANopenNode/303/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/CANopenNode/304/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/CANopenNode/305/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/CANopenNode_STM32/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/od/*.c
)

add_library(canopen STATIC ${CANOPEN_SOURCES})

target_include_directories(canopen PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/CANopenNode
    ${CMAKE_CURRENT_SOURCE_DIR}/CANopenNode_STM32
    ${CMAKE_CURRENT_SOURCE_DIR}/od
)

# PUBLIC: CO_driver_target.h'yi include eden uygulama kodu da aynı ayarları görsün.
target_compile_definitions(canopen PUBLIC
    CO_USE_GLOBALS                  # CANopen nesneleri statik bellekte (heap sadece 512 B)
    CO_LOCK_BASEPRI_ENABLE=1        # kilitte tüm kesmeler değil, sadece öncelik >= 5 kapanır
    CO_LOCK_BASEPRI_PRIO_LEVEL=5    # FDCAN = 5; ADC3 (FOC, 1) ve EXTI0 (nFAULT, 0) etkilenmez
)

# main.h ve HAL başlıkları (sadece include + define, kaynak eklemez)
target_link_libraries(canopen PUBLIC stm32cubemx)
