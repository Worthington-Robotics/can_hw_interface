cmake_minimum_required(VERSION 3.5)
project(can_hw_interface)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THREADS_PREFER_PTHREAD_FLAG ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)
# ros2 client libs
find_package(rclcpp REQUIRED)
#xml parser libs
find_package(tinyxml_vendor REQUIRED)
find_package(TinyXML REQUIRED)
find_package(can_msgs REQUIRED)

file(GLOB hw_interface_sources src/interface/**.cpp src/interface/motors/**.cpp)

#declare executable
add_executable(hw_interface src/interface.cpp ${hw_interface_sources})

#trace include dirs
target_include_directories(hw_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/interfaces>
  $<INSTALL_INTERFACE:include>)

#link CTRE libs
if (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "arm-linux-gnueabihf") # Jetson TK1 / Pi
    target_link_libraries(hw_interface ${CMAKE_SOURCE_DIR}/lib/raspberry/libCTRE_Phoenix.a)
    target_link_libraries(hw_interface ${CMAKE_SOURCE_DIR}/lib/raspberry/libCTRE_PhoenixCCI.a)
	  target_link_libraries(hw_interface ${CMAKE_SOURCE_DIR}/lib/raspberry/libCTRE_PhoenixDiagnostics.a)
    target_link_libraries(hw_interface ${CMAKE_SOURCE_DIR}/lib/raspberry/libCTRE_PhoenixPlatform_socketcan.a)
    target_link_libraries(hw_interface ${CMAKE_SOURCE_DIR}/lib/raspberry/libCTRE_PhoenixCanutils.a)
    target_link_libraries(hw_interface ${CMAKE_SOURCE_DIR}/lib/raspberry/libCTRE_PhoenixCore.a)
    target_link_libraries(hw_interface -lstdc++fs)
elseif (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "aarch64-linux-gnu") # Jetson TX2
    target_link_libraries(hw_interface ${CMAKE_SOURCE_DIR}/lib/jetsontx/libCTRE_Phoenix.a)
    target_link_libraries(hw_interface ${CMAKE_SOURCE_DIR}/lib/jetsontx/libCTRE_PhoenixCCI.a)
    target_link_libraries(hw_interface ${CMAKE_SOURCE_DIR}/lib/jetsontx/libCTRE_PhoenixDiagnostics.a)
    target_link_libraries(hw_interface ${CMAKE_SOURCE_DIR}/lib/jetsontx/libCTRE_PhoenixPlatform_socketcan.a)
    target_link_libraries(hw_interface ${CMAKE_SOURCE_DIR}/lib/jetsontx/libCTRE_PhoenixCanutils.a)
    target_link_libraries(hw_interface ${CMAKE_SOURCE_DIR}/lib/jetsontx/libCTRE_PhoenixCore.a)
else()
    target_link_libraries(hw_interface ${CMAKE_SOURCE_DIR}/lib/x86-64/libCTRE_Phoenix.a)
    target_link_libraries(hw_interface ${CMAKE_SOURCE_DIR}/lib/x86-64/libCTRE_PhoenixCCI.a)
    target_link_libraries(hw_interface ${CMAKE_SOURCE_DIR}/lib/x86-64/libCTRE_PhoenixDiagnostics.a)
    target_link_libraries(hw_interface ${CMAKE_SOURCE_DIR}/lib/x86-64/libCTRE_PhoenixPlatform_socketcan.a)
    target_link_libraries(hw_interface ${CMAKE_SOURCE_DIR}/lib/x86-64/libCTRE_PhoenixCanutils.a)
    target_link_libraries(hw_interface ${CMAKE_SOURCE_DIR}/lib/x86-64/libCTRE_PhoenixCore.a)
endif()
target_link_libraries(hw_interface Threads::Threads)
target_link_libraries(hw_interface ${TinyXML_LIBRARIES})

ament_target_dependencies(hw_interface rclcpp can_msgs)

install(TARGETS hw_interface DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
