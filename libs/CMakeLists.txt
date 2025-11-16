set(BUILD_TESTS OFF)

add_compile_definitions(LIBRM_BYPASS_HAL_ASSERT)

add_subdirectory(librm)

add_library(${PROJECT_NAME}_libs INTERFACE)

target_link_libraries(${PROJECT_NAME}_libs INTERFACE
        rm
)