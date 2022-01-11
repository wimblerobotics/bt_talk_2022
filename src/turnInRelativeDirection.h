#pragma once

#include "behaviortree_cpp_v3/behavior_tree.h"

class TurnInRelativeDirection : public BT::SyncActionNode {
 public:
  TurnInRelativeDirection(
      const std::string& name, const BT::NodeConfiguration& config);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts();
};