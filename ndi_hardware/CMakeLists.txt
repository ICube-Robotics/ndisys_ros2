cmake_minimum_required(VERSION 3.5)
project(ndi_hardware)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

set(EXT_LIBS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external)

## COMPILE CAPI SHARED LIBRARY

file(GLOB NDICAPI_LIBRARY_SRC ${EXT_LIBS_DIR}/ndi_capi/src/*.cpp)

add_library(
  ndicapi_library
  SHARED
  ${NDICAPI_LIBRARY_SRC}
)

target_include_directories(ndicapi_library PUBLIC
  ${EXT_LIBS_DIR}/ndi_capi/include
  ${EXT_LIBS_DIR}/ndi_capi/src/include)

include_directories(${EXT_LIBS_DIR})
# add_library(ndicapi_library SHARED IMPORTED GLOBAL)
# set_target_properties(ndicapi_library PROPERTIES IMPORTED_LOCATION ${EXT_LIBS_DIR}/libndicapi.so)

# # COMPILE
add_library(
  ${PROJECT_NAME}
  SHARED
  src/ndi_sensor_hi.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
  ${EXT_LIBS_DIR}
)
ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

target_link_libraries(${PROJECT_NAME} ndicapi_library yaml-cpp)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "NDI_HARDWARE_BUILDING_LIBRARY")

# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(hardware_interface ndi_hardware_plugin.xml)

# INSTALL ndicapi_library
install(
  TARGETS ndicapi_library
  DESTINATION lib/external
)

# INSTALL
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)

# install(DIRECTORY launch config
# DESTINATION share/${PROJECT_NAME}
# )
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# # EXPORTS
ament_export_include_directories(
  include
)
ament_export_libraries(
  ndicapi_library
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
ament_package()
