#include "tb3_multi_robot/is_topic_active_condition.hpp"

namespace tb3_multi_robot
{

IsTopicActiveCondition::IsTopicActiveCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  is_active_(false)
{
  getInput("topic_name", topic_name_);
}

BT::NodeStatus IsTopicActiveCondition::tick()
{
  if (!node_) {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      topic_name_, qos,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        is_active_ = msg->data;
      });
    
    RCLCPP_INFO(node_->get_logger(), "IsTopicActiveCondition subscribed to %s", topic_name_.c_str());
  }

  bool current_active = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    current_active = is_active_;
  }

  if (current_active) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

}  // namespace tb3_multi_robot

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<tb3_multi_robot::IsTopicActiveCondition>("IsTopicActive");
}
