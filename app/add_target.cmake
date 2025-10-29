
function(add_exe_target TARGET_NAME TARGET_SOURCES)
    add_executable(${TARGET_NAME})
    target_sources(${TARGET_NAME} PRIVATE
            ${TARGET_SOURCES}
            $<TARGET_PROPERTY:${CMAKE_PROJECT_NAME},SOURCES>
    )
    target_include_directories(${TARGET_NAME} PRIVATE
            ./include
            $<TARGET_PROPERTY:${CMAKE_PROJECT_NAME},INCLUDE_DIRECTORIES>
    )
    target_link_libraries(${TARGET_NAME} PRIVATE
            $<TARGET_PROPERTY:${CMAKE_PROJECT_NAME},LINK_LIBRARIES>
    )
endfunction()