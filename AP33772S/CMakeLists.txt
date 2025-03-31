add_library(ap33772S INTERFACE)

target_include_directories(ap33772S INTERFACE ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(ap33772S INTERFACE pico_stdlib hardware_i2c)

target_sources(ap33772S INTERFACE ${CMAKE_CURRENT_LIST_DIR}/ap33772S.cpp)