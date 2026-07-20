############################################################################
# Honghu lookup-table aerodynamic plugin
############################################################################

project(HonghuAeroTable)

add_library(${PROJECT_NAME} SHARED
    HonghuAeroTable.cpp
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC px4_gz_msgs
    PUBLIC gz-sensors${gz-sensors_VERSION_MAJOR}::gz-sensors${gz-sensors_VERSION_MAJOR}
    PUBLIC gz-plugin${gz-plugin_VERSION_MAJOR}::gz-plugin${gz-plugin_VERSION_MAJOR}
    PUBLIC gz-sim${gz-sim_VERSION_MAJOR}::gz-sim${gz-sim_VERSION_MAJOR}
    PUBLIC gz-transport${gz-transport_VERSION_MAJOR}::gz-transport${gz-transport_VERSION_MAJOR}
)

target_include_directories(${PROJECT_NAME}
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
    PUBLIC ${CMAKE_CURRENT_BINARY_DIR}
    PUBLIC px4_gz_msgs
)
