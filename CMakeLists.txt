cmake_minimum_required(VERSION 3.8)
project(fabrik_algorithm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(
  DIRECTORY include/
  DESTINATION include/fabrik_algorithm
)

#executables for package
add_executable(move_end_effector_test src/move_end_effector_test.cpp)
add_executable(root_pos_test src/root_pos_test.cpp)

target_include_directories(move_end_effector_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/fabrik_algorithm>
  $<INSTALL_INTERFACE:include/fabrik_algorithm>
)
target_include_directories(root_pos_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/fabrik_algorithm>
  $<INSTALL_INTERFACE:include/fabrik_algorithm>
)

install(TARGETS
  move_end_effector_test
  root_pos_test
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
