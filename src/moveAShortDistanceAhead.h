#pragma once

#include "behaviortree_cpp_v3/behavior_tree.h"

class MoveAShortDistanceAhead : public BT::ActionNodeBase {
 public:
  MoveAShortDistanceAhead(const std::string& name,
                          const BT::NodeConfiguration& config);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts();

  virtual void halt() override final;
};