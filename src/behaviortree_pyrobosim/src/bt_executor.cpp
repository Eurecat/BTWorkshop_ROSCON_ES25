#include <behaviortree_ros2/tree_execution_server.hpp>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/float32.hpp>

// Example that shows how to customize TreeExecutionServer.
//
// Here, we:
// - add an extra logger, BT::StdCoutLogger
// - add a subscriber that will continuously update a variable in the global blackboard

class PyRobosimBTActionServer : public BT::TreeExecutionServer
{
public:
  PyRobosimBTActionServer(const rclcpp::NodeOptions& options) : TreeExecutionServer(options)
  {
    // Note that the callback above and the execution of the tree accessing the
    // global blackboard happen in two different threads.
    // The former runs in the MultiThreadedExecutor, while the latter in the thread created
    // by TreeExecutionServer. But this is OK because the blackboard is thread-safe.
  }

  void onTreeCreated(BT::Tree& tree) override
  {
    logger_cout_ = std::make_shared<BT::StdCoutLogger>(tree);
  }

  std::optional<std::string> onTreeExecutionCompleted(BT::NodeStatus status,
                                                      bool was_cancelled) override
  {
    // avoid unused parameter warning
    (void)status;
    (void)was_cancelled;
    // NOT really needed, even if logger_cout_ may contain a dangling pointer of the tree
    // at this point
    logger_cout_.reset();
    return std::nullopt;
  }

private:
  std::shared_ptr<BT::StdCoutLogger> logger_cout_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto action_server = std::make_shared<PyRobosimBTActionServer>(options);

  // TODO: This workaround is for a bug in MultiThreadedExecutor where it can deadlock when spinning without a timeout.
  // Deadlock is caused when Publishers or Subscribers are dynamically removed as the node is spinning.
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 0, false,
                                                std::chrono::milliseconds(250));
  exec.add_node(action_server->node());
  const char* ros_domain_id = std::getenv("ROS_DOMAIN_ID");
  RCLCPP_INFO(action_server->node()->get_logger(), "ROS_DOMAIN_ID: %s", ros_domain_id ? ros_domain_id : "not set");
  exec.spin();
  exec.remove_node(action_server->node());

  rclcpp::shutdown();
}
