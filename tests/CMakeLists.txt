# PC'de çalışan GoogleTest testleri: tests/*.cpp
# Test dosyası yoksa hiçbir şey yapılmaz. İlk test dosyası eklenince
# GoogleTest GitHub'dan build/ klasörüne indirilir.
file(GLOB TEST_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

if(TEST_SOURCES)
    include(FetchContent)
    FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/refs/tags/v1.17.0.zip
    )
    FetchContent_MakeAvailable(googletest)

    add_executable(motor_control_tests ${TEST_SOURCES})
    target_link_libraries(motor_control_tests PRIVATE
        motor_control
        GTest::gtest_main
    )
    target_compile_options(motor_control_tests PRIVATE -Wall -Wextra)

    if(MINGW)
        # MinGW çalışma zamanı DLL'lerini exe'ye göm; testler PATH'ten bağımsız çalışsın.
        target_link_options(motor_control_tests PRIVATE -static)
    endif()

    # Testleri CTest'e ve VS Code'un Testing paneline tanıt.
    include(GoogleTest)
    gtest_discover_tests(motor_control_tests)
endif()
