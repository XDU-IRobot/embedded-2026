
file(GLOB_RECURSE COMMON_SOURCES ${CMAKE_CURRENT_LIST_DIR}/common/*.cc)

include(add_target.cmake)

include_directories(${CMAKE_CURRENT_LIST_DIR}/common)
include_directories(${CMAKE_CURRENT_LIST_DIR}/targets)

# 无人机云台
file(GLOB_RECURSE DRONE_GB_SOURCES ${CMAKE_CURRENT_LIST_DIR}/targets/drone_gb/*.cc)
add_exe_target(drone_gb "${DRONE_GB_SOURCES};${COMMON_SOURCES}")
target_compile_definitions(drone_gb PRIVATE DRONE_GB)

# 全向轮靶车底盘
file(GLOB_RECURSE OMNI_TARGET_SOURCES ${CMAKE_CURRENT_LIST_DIR}/targets/omni_target/*.cc)
add_exe_target(omni_target "${OMNI_TARGET_SOURCES};${COMMON_SOURCES}")

# 舵轮底盘
file(GLOB_RECURSE STEER_INFANTRY_CS_SOURCES ${CMAKE_CURRENT_LIST_DIR}/targets/steer_infantry_cs/*.cc)
add_exe_target(steer_infantry_cs "${STEER_INFANTRY_CS_SOURCES};${COMMON_SOURCES}")

# 舵轮云台
file(GLOB_RECURSE STEER_INFANTRY_GB_SOURCES ${CMAKE_CURRENT_LIST_DIR}/targets/steer_infantry_gb/*.cc)
add_exe_target(steer_infantry_gb "${STEER_INFANTRY_GB_SOURCES};${COMMON_SOURCES}")

# 老哨兵云台
file(GLOB_RECURSE OLD_SENTRY_SOURCES ${CMAKE_CURRENT_LIST_DIR}/targets/old_sentry/*.cc)
add_exe_target(old_sentry "${OLD_SENTRY_SOURCES};${COMMON_SOURCES}")
target_compile_definitions(old_sentry PRIVATE OLD_SENTRY)
