# CANopen stack: CANopenNode (not modified) + STM32 FDCAN driver + Object Dictionary.
# See VERSIONS.md for the versions.
#
# Unused features (LSS, GFC, SRDO, ...) are compiled, but
# CANopenNode's own CO_CONFIG_* flags turn them off.
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

# PUBLIC: application code that includes CO_driver_target.h sees the same settings.
target_compile_definitions(canopen PUBLIC
    CO_USE_GLOBALS                  # CANopen objects in static memory (heap is only 512 B)
    CO_LOCK_BASEPRI_ENABLE=1        # the lock masks only priority >= 5, not all interrupts
    CO_LOCK_BASEPRI_PRIO_LEVEL=5    # FDCAN = 5; ADC3 (FOC, 1) and EXTI0 (nFAULT, 0) are not affected
    # 0x1001 bit 0 (generic): set by motor faults (EMCY bits 0x40..0x4F) too, not only by the stack's own errors
    "CO_CONFIG_ERR_CONDITION_GENERIC=((em->errorStatusBits[5] != 0U) || (em->errorStatusBits[8] != 0U) || (em->errorStatusBits[9] != 0U))"
    # Only a communication error moves NMT to pre-operational; a motor fault (generic bit) is reported by CiA 402 Fault,
    # so TPDOs keep running. The port's default also had GENERIC (CO_app_STM32.c, #ifndef NMT_CONTROL).
    "NMT_CONTROL=(CO_NMT_STARTUP_TO_OPERATIONAL | CO_NMT_ERR_ON_ERR_REG | CO_ERR_REG_COMMUNICATION)"
)

# main.h and HAL headers (include + define only, no sources added)
target_link_libraries(canopen PUBLIC stm32cubemx)
