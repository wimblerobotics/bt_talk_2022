#include "isNearAWall.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

BT::NodeStatus IsNearAWall() {
  static int calls = 0;
  // std::cout << "[ IsNearAWall: call count: " << ++calls << " ]" << std::endl;
  // return calls < 20 ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
  return BT::NodeStatus::FAILURE;
}