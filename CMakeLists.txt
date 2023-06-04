cmake_minimum_required(VERSION 3.5)
project(grab_srvcli)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(grab_interface REQUIRED)
find_package(NLopt REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(trac_ik_lib REQUIRED)
find_package(trajectory_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(RM_INCLUDE_DIRS /opt/rm_robot/include)
set(RM_LIBRARIES /opt/rm_robot/lib/libRM_Base.so)

add_executable(grab_server src/grab_server.cpp src/udp.c)
ament_target_dependencies(grab_server rclcpp grab_interface orocos_kdl trac_ik_lib trajectory_msgs)
add_executable(grab_client src/grab_client.cpp)
ament_target_dependencies(grab_client rclcpp grab_interface)

target_include_directories(
  grab_server
  PRIVATE
  include
  ${RM_INCLUDE_DIRS}
)
target_include_directories(
  grab_client
  PRIVATE
  include
  ${RM_INCLUDE_DIRS}
)

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


#add_executable(grab_client src/grab_client.cpp)
#ament_target_dependencies(grab_client rclcpp grab_interface)

target_link_libraries(grab_server ${RM_LIBRARIES})
target_link_libraries(grab_client ${RM_LIBRARIES})

install(TARGETS
  grab_server
  grab_client
  DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
