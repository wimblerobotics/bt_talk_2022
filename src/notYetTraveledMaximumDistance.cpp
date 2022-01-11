#include "notYetTraveledMaximumDistance.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

BT::NodeStatus NotYetTraveledMaximumDistance() {
  std::cout << "[ NotYetTraveledMaximumDistance: OK ]" << std::endl;
 return BT::NodeStatus::SUCCESS;
}