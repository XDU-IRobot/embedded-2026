
file(GLOB_RECURSE COMMON_SOURCES ${CMAKE_CURRENT_LIST_DIR}/common/*.cc)

include(add_target.cmake)

include_directories(${CMAKE_CURRENT_LIST_DIR}/common)
include_directories(${CMAKE_CURRENT_LIST_DIR}/targets)

# 无人机云台
file(GLOB_RECURSE DRONE_GB_SOURCES ${CMAKE_CURRENT_LIST_DIR}/targets/drone_gb/*.cc)
add_exe_target(drone_gb "${DRONE_GB_SOURCES};${COMMON_SOURCES}")

# 舵轮底盘
file(GLOB_RECURSE STEER_INFANTRY_CS_SOURCES ${CMAKE_CURRENT_LIST_DIR}/targets/steer_infantry_cs/*.cc)
add_exe_target(steer_infantry_cs "${STEER_INFANTRY_CS_SOURCES};${COMMON_SOURCES}")
