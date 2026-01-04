cmake_minimum_required(VERSION 3.14)
project(hello_world C)

include(FetchContent)

FetchContent_Declare(
    core
    GIT_REPOSITORY https://github.com/ethospathoslogos/core.git
    GIT_TAG        fe858f7
)
FetchContent_MakeAvailable(core)

add_executable(hello main.c)

# This is the key line:
target_link_libraries(hello PRIVATE grbl)
