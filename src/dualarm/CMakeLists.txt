cmake_minimum_required(VERSION 3.8)
project(dualarm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch rviz urdf meshes
  DESTINATION share/${PROJECT_NAME})
  
install(CODE "execute_process(
  COMMAND ${CMAKE_COMMAND} -E create_symlink 
    ${CMAKE_SOURCE_DIR}/rviz/rviz_config.rviz
    ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/rviz/rviz.rviz
)")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
