cmake_minimum_required(VERSION 3.12)

project(a10_dual)

# aris use c++ 17 standard
set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# find aris
if(UNIX)
	add_definitions(-DUNIX)
        find_package(aris REQUIRED PATHS /usr/aris/aris-2.3.10.240826)
endif(UNIX)
IF(WIN32)
#更改路径以适配已安装aris版本
        find_package(aris REQUIRED PATHS C:/aris/aris-2.3.10.240826)
endif(WIN32)
include_directories(${aris_INCLUDE_DIRS})

add_executable(a10_dual src/main.cpp src/aris_node.cpp src/robot.cpp src/robot.hpp src/plan.cpp src/plan.hpp)
target_compile_features(a10_dual PUBLIC c_std_99 cxx_std_17)

if(UNIX)
    target_link_libraries(a10_dual ${aris_LIBRARIES} atomic)
endif(UNIX)
IF(WIN32)
    target_link_libraries(a10_dual ${aris_LIBRARIES})
endif(WIN32)

IF(WIN32)
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD        
    COMMAND ${CMAKE_COMMAND} -E copy  
        "${aris_DIR}/../../../lib/${CMAKE_BUILD_TYPE}/aris_lib.dll"      
        $<TARGET_FILE_DIR:${PROJECT_NAME}>)    
endif(WIN32)

ament_target_dependencies(a10_dual 
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
)
install(TARGETS a10_dual
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY xml
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

