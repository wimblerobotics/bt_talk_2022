#pragma once

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace BT_Talk_IsNearAWall {
BT::NodeStatus IsNearAWall();

inline void RegisterNodes(BT::BehaviorTreeFactory& factory) {
  factory.registerSimpleCondition("IsNearAWall", std::bind(IsNearAWall));
}
}  // namespace BT_Talk_IsNearAWall
