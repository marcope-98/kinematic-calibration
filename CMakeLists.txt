cmake_minimum_required(VERSION 3.22 FATAL_ERROR)
set(LIBRARY_NAME kc)

project(${LIBRARY_NAME} VERSION 0.0.1 LANGUAGES C CXX)

list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_SOURCE_DIR}/cmake)

set(MIN_EIGEN_VERSION 3.4)
find_package(Eigen3 ${MIN_EIGEN_VERSION} REQUIRED NO_MODULE)
find_package(Ceres REQUIRED)
add_library(${LIBRARY_NAME}
  INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}/include/kc/Link.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/kc/Robot.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/kc/CostFunction.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/kc/types.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/kc/utils.hpp
)
add_library(${LIBRARY_NAME}::${LIBRARY_NAME} ALIAS ${LIBRARY_NAME})

target_include_directories(${LIBRARY_NAME}
  INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_options(${LIBRARY_NAME}
  INTERFACE
  $<$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:AppleClang>,$<CXX_COMPILER_ID:GNU>>:
  -Wall -Wextra -Wpedantic -O2 -march=native >
)

target_link_libraries(${LIBRARY_NAME}
  INTERFACE
  Ceres::ceres
  Eigen3::Eigen
)

include(GNUInstallDirs)
set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/${LIBRARY_NAME})

install(
  TARGETS ${LIBRARY_NAME}
  EXPORT ${LIBRARY_NAME}-targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

include(CMakePackageConfigHelpers)

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/${LIBRARY_NAME}ConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion
)

configure_package_config_file(
  ${CMAKE_CURRENT_LIST_DIR}/cmake/${LIBRARY_NAME}Config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${LIBRARY_NAME}Config.cmake
  INSTALL_DESTINATION ${INSTALL_CONFIGDIR}
)

install(
  FILES
  ${CMAKE_CURRENT_BINARY_DIR}/${LIBRARY_NAME}Config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/${LIBRARY_NAME}ConfigVersion.cmake
  DESTINATION ${INSTALL_CONFIGDIR}
)

export(
  EXPORT ${LIBRARY_NAME}-targets
  FILE ${CMAKE_CURRENT_BINARY_DIR}/${LIBRARY_NAME}Targets.cmake
  NAMESPACE ${LIBRARY_NAME}::
)

export(PACKAGE ${LIBRARY_NAME})

add_subdirectory(examples)
