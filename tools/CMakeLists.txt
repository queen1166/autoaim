# 每个 .cpp 工具单独构建成一个可执行文件。
# 它们各自有自己的 main()，所以不能进 autoaim_core 的 GLOB。
# tools/*.py 不受影响（Python 脚本直接跑）。

file(GLOB TOOL_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

foreach(src ${TOOL_SOURCES})
  get_filename_component(name ${src} NAME_WE)
  add_executable(tool_${name} ${src})
  target_link_libraries(tool_${name} PRIVATE autoaim_core)
  message(STATUS "  工具: tool_${name}")
endforeach()
