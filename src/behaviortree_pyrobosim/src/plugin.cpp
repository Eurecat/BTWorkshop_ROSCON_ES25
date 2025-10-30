
#include "behaviortree_cpp/bt_factory.h"
#include <behaviortree_ros2/behaviortree_ros2.hpp>

#include "simple_pyrobosim_msgs/msg/simple_robot_battery.hpp"

#include "simple_pyrobosim_msgs/srv/is_passage_door_open.hpp"

#include "simple_pyrobosim_msgs/action/navigate.hpp"
#include "simple_pyrobosim_msgs/action/approach_furniture.hpp"
#include "simple_pyrobosim_msgs/action/pick.hpp"
#include "simple_pyrobosim_msgs/action/place.hpp"

using namespace BT;

BT_REGISTER_ROS_NODES(factory, params)
{
    // Convenient renamings
    namespace pyrobosim_msgs = simple_pyrobosim_msgs::msg;
    namespace pyrobosim_srvs = simple_pyrobosim_msgs::srv;
    namespace pyrobosim_actions = simple_pyrobosim_msgs::action;

    // Subscribers
    factory.registerNodeType<AutoSerSubscriber<pyrobosim_msgs::SimpleRobotBattery>>("UpdateRobotBattery",params);

    // Service clients
    factory.registerNodeType<AutoDesAutoSerServiceClient<pyrobosim_srvs::IsPassageDoorOpen>>("IsPassageDoorOpen",params);

    // Action clients
    factory.registerNodeType<AutoDesAutoSerActionClient<pyrobosim_actions::Navigate>>("Navigate",params);
    factory.registerNodeType<AutoDesAutoSerActionClient<pyrobosim_actions::ApproachFurniture>>("ApproachFurniture",params);
    factory.registerNodeType<AutoDesAutoSerActionClient<pyrobosim_actions::Pick>>("Pick",params);
    factory.registerNodeType<AutoDesAutoSerActionClient<pyrobosim_actions::Place>>("Place",params);

};