#ifndef TB3_MULTI_ROBOT__IS_TOPIC_ACTIVE_CONDITION_HPP_
#define TB3_MULTI_ROBOT__IS_TOPIC_ACTIVE_CONDITION_HPP_

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "std_msgs/msg/bool.hpp"

namespace tb3_multi_robot
{

class IsTopicActiveCondition : public BT::ConditionNode
{
public:
  IsTopicActiveCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  IsTopicActiveCondition() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic_name", "Topic to check for activity")
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
  std::string topic_name_;
  bool is_active_;
  std::mutex mutex_;
};

}  // namespace tb3_multi_robot

#endif  // TB3_MULTI_ROBOT__IS_TOPIC_ACTIVE_CONDITION_HPP_
