cmake_minimum_required(VERSION 3.8)
project(arm_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Aggiungi le dipendenze
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Aggiungi l'eseguibile
add_executable(talker_listener src/pub_sub.cpp)

# Dichiara le dipendenze
ament_target_dependencies(talker_listener rclcpp sensor_msgs std_msgs)

# Installa l'eseguibile
install(
  TARGETS talker_listener
  DESTINATION lib/${PROJECT_NAME}
)

# Installa le directory di configurazione e di lancio
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

