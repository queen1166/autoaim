# 离线测试：每个 .cpp 一个可执行文件 + 一个 ctest 用例。
# 这些测试不碰硬件，是车上时间紧张时的主要质量保证手段。

enable_testing()

file(GLOB TEST_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

foreach(src ${TEST_SOURCES})
  get_filename_component(name ${src} NAME_WE)
  add_executable(${name} ${src})
  target_link_libraries(${name} PRIVATE autoaim_core)
  add_test(NAME ${name} COMMAND ${name})
  message(STATUS "  测试: ${name}")
endforeach()
