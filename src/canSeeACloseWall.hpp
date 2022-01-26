#pragma once

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace BT_Talk_CanSeeACloseWall {
BT::NodeStatus CanSeeACloseWall();

inline void RegisterNodes(BT::BehaviorTreeFactory& factory) {
  factory.registerSimpleCondition("CanSeeACloseWall",
                                  std::bind(CanSeeACloseWall));
}
}  // namespace BT_Talk
