# Donanımdan bağımsız FOC kütüphanesi.
# Hem PC'de hem STM32'de aynı kaynaklarla derlenir.
#
#   Başlık dosyaları: inc/*.hpp   →  kullanırken: #include "pi_controller.hpp"
#   Kaynak dosyalar : src/*.cpp
#
# src/ içindeki tüm .cpp dosyaları otomatik bulunur; yeni dosya eklediğinde
# bu dosyaya dokunman gerekmez.
file(GLOB MOTOR_CONTROL_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

if(MOTOR_CONTROL_SOURCES)
    add_library(motor_control STATIC ${MOTOR_CONTROL_SOURCES})
    target_include_directories(motor_control PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/inc)
    target_compile_options(motor_control PRIVATE
        -Wall -Wextra -Wpedantic
        -Wdouble-promotion   # float'ın gizlice double'a dönüşmesini yakala (Cortex-M FPU'su sadece float)
        -Wfloat-conversion   # double -> float gizli dönüşümleri yakala
    )
else()
    # Henüz .cpp yok: sadece başlık dosyalarını paylaşan bir kütüphane.
    add_library(motor_control INTERFACE)
    target_include_directories(motor_control INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/inc)
endif()
