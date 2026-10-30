# GoogleTest tests that run on the PC: tests/*.cpp
# With no test file, nothing happens. When the first test file is added,
# GoogleTest is downloaded from GitHub into the build/ folder.
file(GLOB TEST_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

if(TEST_SOURCES)
    include(FetchContent)
    FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/refs/tags/v1.17.0.zip
        URL_HASH SHA256=40d4ec942217dcc84a9ebe2a68584ada7d4a33a8ee958755763278ea1c5e18ff
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )
    FetchContent_MakeAvailable(googletest)

    add_executable(motor_control_tests ${TEST_SOURCES})
    target_link_libraries(motor_control_tests PRIVATE
        motor_control
        GTest::gtest_main
    )
    target_compile_options(motor_control_tests PRIVATE -Wall -Wextra)

    if(MINGW)
        # Link the MinGW runtime into the exe, so the tests run without depending on PATH.
        target_link_options(motor_control_tests PRIVATE -static)
    endif()

    # Register the tests with CTest and the VS Code Testing panel.
    include(GoogleTest)
    gtest_discover_tests(motor_control_tests)
endif()
