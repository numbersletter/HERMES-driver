cmake_minimum_required(VERSION 3.10)
project(hermes_driver_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── ROS 2 Dependencies ──────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# ── GPIO Library ─────────────────────────────────────────────────────
# libgpiod v2 is the current stable C library for Linux GPIO character devices.
# On Raspberry Pi OS (Bookworm): sudo apt install libgpiod-dev
# When installed from source, libgpiod lands in /usr/local/lib which is not
# on the default dynamic-linker search path.  IMPORTED_TARGET + the RPATH
# setting below ensure the installed binary carries the correct runpath so
# that libgpiod.so.3 is found at launch without any LD_LIBRARY_PATH tweak.
find_package(PkgConfig REQUIRED)
pkg_check_modules(GPIOD REQUIRED IMPORTED_TARGET gpiod>=2.0)

# Embed every non-system library directory used at link time into the
# installed binary's RPATH (handles /usr/local/lib and similar prefixes).
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# Threads are required for software PWM
find_package(Threads REQUIRED)

# ── Build ────────────────────────────────────────────────────────────
add_executable(diff_drive_controller_node
  src/diff_drive_controller.cpp
  src/tb6612fng.cpp
)

target_include_directories(diff_drive_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(diff_drive_controller_node
  rclcpp
  geometry_msgs
  std_msgs
)

target_link_libraries(diff_drive_controller_node PkgConfig::GPIOD Threads::Threads)

# ── Install ──────────────────────────────────────────────────────────
install(TARGETS diff_drive_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# ── Testing ──────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()