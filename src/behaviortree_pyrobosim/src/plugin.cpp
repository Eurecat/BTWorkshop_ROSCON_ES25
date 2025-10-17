
#include "behaviortree_cpp/bt_factory.h"
#include <behaviortree_ros2/behaviortree_ros2.hpp>

#include "simple_pyrobosim_msgs/msg/simple_robot_state.hpp"

#include "simple_pyrobosim_msgs/srv/is_door_open.hpp"

#include "simple_pyrobosim_msgs/action/navigate.hpp"
#include "simple_pyrobosim_msgs/action/approach_object.hpp"
#include "simple_pyrobosim_msgs/action/pick.hpp"
#include "simple_pyrobosim_msgs/action/place.hpp"

namespace BT_ROS
{
    // template specializations or helper functions can be placed here if needed
}

using namespace BT;

BT_REGISTER_ROS_NODES(factory, params)
{
    // Subscribers
    factory.registerNodeType<AutoSerSubscriber<simple_pyrobosim_msgs::msg::SimpleRobotState>>("UpdateRobotState",params);

    // Service clients
    factory.registerNodeType<AutoDesAutoSerServiceClient<simple_pyrobosim_msgs::srv::IsDoorOpen>>("IsDoorOpen",params);

    // Action clients
    factory.registerNodeType<AutoDesAutoSerActionClient<simple_pyrobosim_msgs::action::Navigate>>("Navigate",params);
    factory.registerNodeType<AutoDesAutoSerActionClient<simple_pyrobosim_msgs::action::ApproachObject>>("ApproachObject",params);
    factory.registerNodeType<AutoDesAutoSerActionClient<simple_pyrobosim_msgs::action::Pick>>("Pick",params);
    factory.registerNodeType<AutoDesAutoSerActionClient<simple_pyrobosim_msgs::action::Place>>("Place",params);

};