cmake_minimum_required(VERSION 3.8)
project(jarabot_sim_navigation2)

find_package(ament_cmake REQUIRED)

# launch / config / maps install
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
