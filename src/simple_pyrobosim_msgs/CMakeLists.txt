cmake_minimum_required(VERSION 3.8)
project(simple_pyrobosim_msgs)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/SimpleRobotState.msg"
    "srv/IsDoorOpen.srv"
    "action/ApproachDoor.action"
    "action/ApproachObject.action"
    "action/Navigate.action"
    "action/Open.action"
    "action/Pick.action"
    "action/Place.action"
    )

ament_export_dependencies(rosidl_default_runtime)

ament_package()
