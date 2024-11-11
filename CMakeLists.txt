cmake_minimum_required(VERSION 3.5)
project(bcr_bot)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(DIRECTORY launch meshes models urdf worlds rviz
    DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY scripts
        DESTINATION share/${PROJECT_NAME}
)

install( DIRECTORY config/
         DESTINATION share/${PROJECT_NAME}/config)
         
install(PROGRAMS scripts/remapper.py
        DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/point_follow.py
        DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/multi_point_follow.py
        DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/circle.py
        DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/hardware_joy.py
        DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/ultrasonic_sensor.py
        DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/bt_nav_obj.py
        DESTINATION lib/${PROJECT_NAME})

ament_package()


 