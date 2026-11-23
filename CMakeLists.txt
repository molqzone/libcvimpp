cmake_minimum_required(VERSION 3.16)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(cvimpp LANGUAGES C CXX)
endif()

set(CVIMPP_MIDDLEWARE_DIR "" CACHE PATH
    "CVI middleware build directory (boards/<board>-sd/middleware)")

if(NOT CVIMPP_MIDDLEWARE_DIR)
  message(FATAL_ERROR
    "CVIMPP_MIDDLEWARE_DIR must point at the board middleware build directory")
endif()

get_filename_component(_cvi_board_build "${CVIMPP_MIDDLEWARE_DIR}/.." ABSOLUTE)

set(_cvi_include_dirs
  ${CVIMPP_MIDDLEWARE_DIR}/include
  ${CVIMPP_MIDDLEWARE_DIR}/include/isp/cv181x
  ${CVIMPP_MIDDLEWARE_DIR}/modules/isp/include/cv181x
  ${CVIMPP_MIDDLEWARE_DIR}/sample/common
  ${CVIMPP_MIDDLEWARE_DIR}/component/isp/common
  ${_cvi_board_build}/kernel/riscv/usr/include)

foreach(_include_dir IN LISTS _cvi_include_dirs)
  if(NOT IS_DIRECTORY "${_include_dir}")
    message(FATAL_ERROR "CVI middleware include directory not found: ${_include_dir}")
  endif()
endforeach()

set(_cvi_archives
  ${CVIMPP_MIDDLEWARE_DIR}/lib/libsample.a
  ${CVIMPP_MIDDLEWARE_DIR}/lib/libisp.a
  ${CVIMPP_MIDDLEWARE_DIR}/lib/libisp_algo.a
  ${CVIMPP_MIDDLEWARE_DIR}/lib/libae.a
  ${CVIMPP_MIDDLEWARE_DIR}/lib/libawb.a
  ${CVIMPP_MIDDLEWARE_DIR}/lib/libaf.a
  ${CVIMPP_MIDDLEWARE_DIR}/lib/libcvi_bin.a
  ${CVIMPP_MIDDLEWARE_DIR}/lib/libcvi_bin_isp.a
  ${CVIMPP_MIDDLEWARE_DIR}/lib/libvi.a
  ${CVIMPP_MIDDLEWARE_DIR}/lib/libvpss.a
  ${CVIMPP_MIDDLEWARE_DIR}/lib/libvenc.a
  ${CVIMPP_MIDDLEWARE_DIR}/lib/libvdec.a
  ${CVIMPP_MIDDLEWARE_DIR}/lib/libvo.a
  ${CVIMPP_MIDDLEWARE_DIR}/lib/librgn.a
  ${CVIMPP_MIDDLEWARE_DIR}/lib/libsys.a
  ${CVIMPP_MIDDLEWARE_DIR}/lib/libcvi_ive.a
  ${CVIMPP_MIDDLEWARE_DIR}/lib/libmisc.a
  ${CVIMPP_MIDDLEWARE_DIR}/lib/libgdc.a
  ${CVIMPP_MIDDLEWARE_DIR}/lib/libraw_dump.a
  ${CVIMPP_MIDDLEWARE_DIR}/lib/libjson-c.a
  ${CVIMPP_MIDDLEWARE_DIR}/lib/3rd/libini.a
  ${CVIMPP_MIDDLEWARE_DIR}/lib/3rd/libcli.a)

foreach(_archive IN LISTS _cvi_archives)
  if(NOT EXISTS "${_archive}")
    message(FATAL_ERROR "CVI middleware archive not found: ${_archive}")
  endif()
endforeach()

add_library(cvimpp STATIC
  src/cvi_status.cpp
  src/cvi_context.cpp
  src/cvi_vpss.cpp
  src/cvi_frame.cpp
  src/cvi_capture.cpp
  src/cvi_encoder.cpp)

# C++23：公共 API 用 `std::expected` 报构造失败，消费方一起升到 23。
# C++23: the public API reports construction failures with `std::expected`, so consumers
# move up with it.
target_compile_features(cvimpp PUBLIC cxx_std_23)
# 与中间件库保持**同一组传感器宏**：`SAMPLE_SNS_TYPE_E` 的编号本身是被这些宏包着的，
# 应用侧少定义一个，它看到的枚举就和库里差一位——ini 在库里解析成第 N 项，应用把同一个
# 数值传回去却对应另一个传感器，查表于是返回 NULL（"fail to find pstSnsObj"）。
# Keep the **same sensor macros as the middleware library**: `SAMPLE_SNS_TYPE_E`'s
# numbering is itself guarded by them, so an application that defines one fewer sees a
# different enum -- the ini resolves to entry N inside the library while the same number means
# another sensor when it comes back, and the lookup returns NULL.
target_compile_definitions(cvimpp PRIVATE __CV181X__ ARCH_CV181X SENSOR_SMS_SC035HGS)
target_include_directories(cvimpp
  PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include
  PRIVATE ${CMAKE_CURRENT_LIST_DIR}/src)
target_include_directories(cvimpp SYSTEM PRIVATE ${_cvi_include_dirs})
target_compile_options(cvimpp PRIVATE -Wall -Wextra -Wpedantic)

target_link_directories(cvimpp PUBLIC
  ${CVIMPP_MIDDLEWARE_DIR}/lib)

# The CVI archives are mutually recursive. Keep the complete vendor closure in
# the library target so applications only link cvimpp, while the group keeps
# archive extraction deterministic on CMake versions without LINK_GROUP.
target_link_libraries(cvimpp PUBLIC
  "-Wl,--start-group"
  ${_cvi_archives}
  "-l:libsns_full.so"
  "-l:libsns_sc035hgs.so"
  "-Wl,--end-group"
  atomic pthread dl m)
