cmake_minimum_required(VERSION 3.16)
project(ImageProcessor C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ROS2 风格目录结构
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/install/bin")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/install/lib")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/install/lib")

# 源代码目录
set(SRC_DIR "${CMAKE_SOURCE_DIR}/src")

# 选项：是否构建 GTK 图形界面（默认 OFF，便于在没有 GTK 的环境下也能构建视频处理工具）
option(BUILD_GUI "Build GTK GUI imageprocessor" ON)

# 公共 C/C++ 源文件（两端共用）
set(COMMON_SOURCES
    ${SRC_DIR}/processor.c
    ${SRC_DIR}/utils.cpp
)

# 内部实现：单独静态库（默认构建，但不链接到 GUI/CLI，以实现对外隐藏）
add_library(image_internal STATIC
    ${SRC_DIR}/global_image_buffer.c
    ${SRC_DIR}/image.c
    ${SRC_DIR}/morph_binary_bitpacked.c
    ${SRC_DIR}/dynamic_log.cpp
    ${SRC_DIR}/utils.cpp
    ${SRC_DIR}/kalman.c
    ${SRC_DIR}/test_ipm.c
)
target_include_directories(image_internal PUBLIC ${SRC_DIR})

# 为 C 语言源文件强制包含 <stddef.h> 以解决 size_t 未定义问题
# 使用生成器表达式以确保跨平台兼容性 (MSVC 使用 /FI, GCC/Clang 使用 -include)
target_compile_options(image_internal PRIVATE
    $<$<AND:$<CXX_COMPILER_ID:MSVC>,$<COMPILE_LANGUAGE:C>>:/FIstddef.h>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:$<$<COMPILE_LANGUAGE:C>:-include stddef.h>>
)

# ---------------- GUI 目标（可选） ----------------
if(BUILD_GUI)
    set(SOURCES_GUI
        ${SRC_DIR}/main.cpp
        ${SRC_DIR}/csv_reader.cpp
        ${SRC_DIR}/oscilloscope.cpp
        ${SRC_DIR}/dynamic_log.cpp
        ${SRC_DIR}/utils.cpp
        ${SRC_DIR}/global_image_buffer.c
        ${COMMON_SOURCES}
    )

    find_package(PkgConfig REQUIRED)
    pkg_check_modules(GTK3 REQUIRED IMPORTED_TARGET gtk+-3.0)
    pkg_check_modules(LIBPNG REQUIRED IMPORTED_TARGET libpng)
    pkg_check_modules(LIBJPEG REQUIRED IMPORTED_TARGET libjpeg)

    # GUI 可选集成 OpenCV（用于视频导入）。未找到 OpenCV 也可以继续构建 GUI，只是禁用视频按钮。
    find_package(OpenCV QUIET)

    add_executable(imageprocessor ${SOURCES_GUI})

    target_link_libraries(imageprocessor
        PRIVATE
            PkgConfig::GTK3
            PkgConfig::LIBPNG
            PkgConfig::LIBJPEG
            image_internal
    )

    if(OpenCV_FOUND)
        target_include_directories(imageprocessor PRIVATE ${OpenCV_INCLUDE_DIRS})
        target_link_libraries(imageprocessor PRIVATE ${OpenCV_LIBS})
        target_compile_definitions(imageprocessor PRIVATE HAVE_OPENCV=1)
    endif()
endif()

# ---------------- 视频逐帧处理 CLI 目标 ----------------
# 依赖 OpenCV：videoio、imgproc、imgcodecs 等模块
option(BUILD_VIDEO_TOOL "Build video_processor CLI (OpenCV)" ON)
if(BUILD_VIDEO_TOOL)
    find_package(OpenCV QUIET)
    if(OpenCV_FOUND)
        add_executable(video_processor
            ${SRC_DIR}/video_processor.cpp
            ${SRC_DIR}/utils.cpp
            ${SRC_DIR}/global_image_buffer.c
            ${COMMON_SOURCES}
        )
        target_include_directories(video_processor PRIVATE ${OpenCV_INCLUDE_DIRS})
        target_link_libraries(video_processor PRIVATE ${OpenCV_LIBS} image_internal)
    else()
        message(WARNING "OpenCV 未找到，将跳过 video_processor 目标的构建。设置 OpenCV 环境或使用 -DOpenCV_DIR 指定后重试。")
    endif()
endif()

# 可选：构建嵌入式图像处理管线（image.c）
# 注意：image.c 依赖嵌入式环境头文件（如 Binarization.h、lcd_spi_200.h）以及全局缓冲
# 在桌面环境通常不可用，默认关闭。仅当相关依赖可用时再启用。
# 取消原先的可选嵌入式可执行，改为静态库方式隐藏实现
