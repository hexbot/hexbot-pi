cmake_minimum_required(VERSION 3.1)

project(pihexbot)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(JSONCPP_WITH_TESTS "OFF" CACHE STRING "Do not need that.")

file(GLOB PI_HEXBOT_SRC
    "src/*.h"
    "src/*.cpp"
)

add_executable(pihexbot ${PI_HEXBOT_SRC})

set(HEXBOT_STATIC True)
add_subdirectory(core)
target_link_libraries(pihexbot hexbot)
include_directories(core/src)

add_custom_command(TARGET pihexbot POST_BUILD
    COMMAND cp $<TARGET_FILE:pihexbot> /opt/hexbot
)

add_custom_command(TARGET pihexbot POST_BUILD
    COMMAND cp ${CMAKE_CURRENT_SOURCE_DIR}/animations/* /opt/hexbot/animations
)