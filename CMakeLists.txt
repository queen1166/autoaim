cmake_minimum_required(VERSION 3.16)
project(autoaim CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-Wall -Wextra)

find_package(Threads REQUIRED)

# 必须在顶层调用，否则子目录里的 add_test 不会被 ctest 发现
enable_testing()

# ─────────────────────────────────────────────────────────────
# Eigen：优先系统安装，没有则用 third_party/eigen
# （规则 §2.3 要求不要修改系统级设置，所以 vendor 了一份）
# ─────────────────────────────────────────────────────────────
find_package(Eigen3 QUIET NO_MODULE)
if(TARGET Eigen3::Eigen)
  set(AUTOAIM_EIGEN Eigen3::Eigen)
  message(STATUS "Eigen: 使用系统安装")
else()
  add_library(autoaim_eigen INTERFACE)
  target_include_directories(autoaim_eigen INTERFACE
      ${CMAKE_SOURCE_DIR}/third_party/eigen)
  set(AUTOAIM_EIGEN autoaim_eigen)
  message(STATUS "Eigen: 使用 vendored third_party/eigen")
endif()

# ─────────────────────────────────────────────────────────────
# OpenCV：可选。没有它时跳过需要图像的模块，
# 这样 EKF / tracker / aim / 协议 仍能单独编译和测试。
# ─────────────────────────────────────────────────────────────
find_package(OpenCV QUIET COMPONENTS core imgproc calib3d dnn highgui imgcodecs)

if(OpenCV_FOUND)
  message(STATUS "OpenCV ${OpenCV_VERSION} 已找到 → 启用 detect / solver / camera")
else()
  message(WARNING
    "未找到 OpenCV → 跳过 modules/detect、modules/solver、bsp/camera。\n"
    "  Ubuntu: 用 conda 装到自己环境（【不要】sudo apt install libopencv-dev，\n"
    "          小电脑是共用设备，规则 §2.3 要求不要改系统级配置）")
endif()

# ─────────────────────────────────────────────────────────────
# 海康 MVS SDK：可选。没有它也能跑 --replay 离线模式。
# SDK 默认装在 /opt/MVS。装之前先问赛务（同样是系统级安装）。
# ─────────────────────────────────────────────────────────────
set(MVS_ROOT "/opt/MVS" CACHE PATH "海康 MVS SDK 根目录")
find_path(MVS_INCLUDE_DIR NAMES MvCameraControl.h
          PATHS ${MVS_ROOT}/include /usr/include)
find_library(MV_CAMERA_LIB NAMES MvCameraControl
             PATHS ${MVS_ROOT}/lib/64 ${MVS_ROOT}/lib /usr/lib)

if(MVS_INCLUDE_DIR AND MV_CAMERA_LIB)
  message(STATUS "海康 MVS SDK: 已找到 (${MV_CAMERA_LIB}) → 启用真机相机")
  set(AUTOAIM_HAVE_MVS TRUE)
else()
  message(STATUS "海康 MVS SDK: 未找到 → 只支持 --replay 离线模式")
  set(AUTOAIM_HAVE_MVS FALSE)
endif()

# ─────────────────────────────────────────────────────────────
# 核心库（不含 main）
# ─────────────────────────────────────────────────────────────
file(GLOB_RECURSE CORE_SOURCES CONFIGURE_DEPENDS
     ${CMAKE_SOURCE_DIR}/bsp/*.cpp
     ${CMAKE_SOURCE_DIR}/modules/*.cpp
     ${CMAKE_SOURCE_DIR}/threads/*.cpp
     ${CMAKE_SOURCE_DIR}/app/*.cpp)

# 带 main() 的文件不能进静态库
list(FILTER CORE_SOURCES EXCLUDE REGEX "/app/main[^/]*\\.cpp$")

if(NOT OpenCV_FOUND)
  list(FILTER CORE_SOURCES EXCLUDE REGEX "/(detect|solver|camera)/")
endif()

if(NOT CORE_SOURCES)
  message(FATAL_ERROR "没有找到任何源文件，检查目录结构")
endif()

add_library(autoaim_core STATIC ${CORE_SOURCES})

target_include_directories(autoaim_core PUBLIC ${CMAKE_SOURCE_DIR})
target_link_libraries(autoaim_core PUBLIC Threads::Threads ${AUTOAIM_EIGEN})

if(OpenCV_FOUND)
  target_include_directories(autoaim_core PUBLIC ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(autoaim_core PUBLIC ${OpenCV_LIBS})
  target_compile_definitions(autoaim_core PUBLIC AUTOAIM_HAVE_OPENCV=1)
endif()

if(AUTOAIM_HAVE_MVS)
  target_include_directories(autoaim_core PUBLIC ${MVS_INCLUDE_DIR})
  target_link_libraries(autoaim_core PUBLIC ${MV_CAMERA_LIB})
  target_compile_definitions(autoaim_core PUBLIC AUTOAIM_HAVE_MVS=1)
endif()

# ─────────────────────────────────────────────────────────────
# 可执行文件
# ─────────────────────────────────────────────────────────────
# 正式程序
add_executable(autoaim app/main.cpp)
target_link_libraries(autoaim PRIVATE autoaim_core)

# 阶段 1 的云台扫动测试程序（保留，现场排查串口时很有用）
add_executable(main_stage1 app/main_stage1.cpp)
target_link_libraries(main_stage1 PRIVATE autoaim_core)

add_subdirectory(tools)
add_subdirectory(tests)
