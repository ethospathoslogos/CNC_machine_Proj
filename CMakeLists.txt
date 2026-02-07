cmake_minimum_required(VERSION 3.14)
project(grbl_core C)

# --- Build the grbl static library from all src/ files ---
file(GLOB GRBL_SOURCES CONFIGURE_DEPENDS src/*.c)

add_library(grbl STATIC ${GRBL_SOURCES})

target_include_directories(grbl PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)

# Install the library and headers so drivers can link against it
install(TARGETS grbl
    ARCHIVE DESTINATION lib
)
install(DIRECTORY src/ DESTINATION include/grbl
    FILES_MATCHING PATTERN "*.h"
)
