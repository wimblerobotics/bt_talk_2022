#include "notAboutToCollide.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

BT::NodeStatus NotAboutToCollide() {
  static int calls = 0;
  std::cout << "[ NotAboutToCollide: call count: " << ++calls << " ]" << std::endl;
  return calls > 14 ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
}