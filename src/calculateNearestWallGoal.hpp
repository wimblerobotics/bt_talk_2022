#pragma once

#include <geometry_msgs/msg/pose.hpp>
#include <vector>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "closePointSubscriber.hpp"
#include "rclcpp/rclcpp.hpp"


class CalculateNearestWallGoal : public BT::SyncActionNode {
 public:
  CalculateNearestWallGoal(const std::string& name,
                           const BT::NodeConfiguration& cfg);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick();
};