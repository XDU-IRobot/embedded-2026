
file(GLOB_RECURSE COMMON_SOURCES ${CMAKE_CURRENT_LIST_DIR}/common/*.cc)

include(add_target.cmake)

include_directories(${CMAKE_CURRENT_LIST_DIR}/common)
include_directories(${CMAKE_CURRENT_LIST_DIR}/targets)

#算法招新测试小云台
file(GLOB_RECURSE ALGORITHM_ENROLL_TEST_SOURCES ${CMAKE_CURRENT_LIST_DIR}/targets/algorithm_enroll_test/*.cc)
add_exe_target(algorithm_enroll_test "${ALGORITHM_ENROLL_TEST_SOURCES};${COMMON_SOURCES}")
target_compile_definitions(algorithm_enroll_test PRIVATE ALGORITHM_ENROLL_TEST)

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

# 老哨兵云台+底盘
file(GLOB_RECURSE OLD_SENTRY_SOURCES ${CMAKE_CURRENT_LIST_DIR}/targets/old_sentry/*.cc)
add_exe_target(old_sentry "${OLD_SENTRY_SOURCES};${COMMON_SOURCES}")
target_compile_definitions(old_sentry PRIVATE OLD_SENTRY)

# 测试平台云台
file(GLOB_RECURSE TEST_GB_SOURCES ${CMAKE_CURRENT_LIST_DIR}/targets/test_gb/*.cc)
add_exe_target(test_gb "${TEST_GB_SOURCES};${COMMON_SOURCES}")
target_compile_definitions(test_gb PRIVATE TEST_GB)

#飞镖镖架
file(GLOB_RECURSE DART_RACK_SOURCES ${CMAKE_CURRENT_LIST_DIR}/targets/dart_rack/*.cc)
add_exe_target(dart_rack "${DART_RACK_SOURCES};${COMMON_SOURCES}")
target_compile_definitions(dart_rack PRIVATE DART_RACK)

#麦轮英雄底盘
file(GLOB_RECURSE MECA_HERO_SOURCES ${CMAKE_CURRENT_LIST_DIR}/targets/meca_hero/*.cc)
add_exe_target(meca_hero "${MECA_HERO_SOURCES};${COMMON_SOURCES}")
target_compile_definitions(meca_hero PRIVATE MECA_HERO)