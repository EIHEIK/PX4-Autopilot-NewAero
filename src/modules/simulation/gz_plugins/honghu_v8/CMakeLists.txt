project(HonghuV8)

set(HONGHU_V8_COMMON HonghuV8Common.cpp)

add_library(HonghuAeroV8 SHARED HonghuAeroV8.cpp ${HONGHU_V8_COMMON})
add_library(HonghuPropulsionV8 SHARED HonghuPropulsionV8.cpp ${HONGHU_V8_COMMON})

foreach(target HonghuAeroV8 HonghuPropulsionV8)
    target_link_libraries(${target}
        PUBLIC px4_gz_msgs
        PUBLIC gz-plugin${gz-plugin_VERSION_MAJOR}::gz-plugin${gz-plugin_VERSION_MAJOR}
        PUBLIC gz-sim${gz-sim_VERSION_MAJOR}::gz-sim${gz-sim_VERSION_MAJOR}
        PUBLIC gz-transport${gz-transport_VERSION_MAJOR}::gz-transport${gz-transport_VERSION_MAJOR}
    )
    target_include_directories(${target} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR} px4_gz_msgs)
endforeach()
