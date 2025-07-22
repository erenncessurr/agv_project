cmake_minimum_required(VERSION 3.8)
project(agv_project)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(include)
include_directories(third_party/snap7/core)
include_directories(third_party/snap7/sys)
include_directories(third_party/snap7/lib)

add_library(plc_system SHARED
  hardware/plc_system.cpp
)
add_library(snap7_cpp SHARED
  third_party/snap7/core/s7_client.cpp
  third_party/snap7/core/s7_isotcp.cpp
  third_party/snap7/core/s7_partner.cpp
  third_party/snap7/core/s7_peer.cpp
  third_party/snap7/core/s7_server.cpp
  third_party/snap7/core/s7_micro_client.cpp
  third_party/snap7/core/s7_text.cpp
  third_party/snap7/sys/snap_msgsock.cpp
  third_party/snap7/sys/snap_sysutils.cpp
  third_party/snap7/sys/snap_tcpsrvr.cpp
  third_party/snap7/sys/snap_threads.cpp
  third_party/snap7/lib/snap7_libmain.cpp
)

include_directories(hardware/include)
ament_target_dependencies(plc_system
  hardware_interface
  pluginlib
  rclcpp)

link_directories(/usr/lib)
  
target_link_libraries(plc_system snap7_cpp pthread rt)

pluginlib_export_plugin_description_file(hardware_interface plc_plugin.xml)

ament_export_include_directories(include)
ament_export_libraries(plc_system)
ament_export_dependencies(hardware_interface pluginlib rclcpp)



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

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
  
install(DIRECTORY description
  DESTINATION share/${PROJECT_NAME})
  
install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

install(TARGETS plc_system snap7_cpp
    DESTINATION lib
  )
  
install(DIRECTORY hardware/include/ 
    DESTINATION include
  )

install(FILES
  plc_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)
  
ament_package()
