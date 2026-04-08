file(GLOB_RECURSE COMMON_SOURCES ${CMAKE_CURRENT_LIST_DIR}/common/*.cc)

include(add_target.cmake)

include_directories(${CMAKE_CURRENT_LIST_DIR}/common)
include_directories(${CMAKE_CURRENT_LIST_DIR}/targets)


#飞镖镖架
file(GLOB_RECURSE DART_RACK_SOURCES ${CMAKE_CURRENT_LIST_DIR}/targets/dart_rack/*.cc ${CMAKE_CURRENT_LIST_DIR}/targets/dart_rack/*.cpp)
add_exe_target(dart_rack "${DART_RACK_SOURCES};${COMMON_SOURCES}")
target_compile_definitions(dart_rack PRIVATE DART_RACK)
