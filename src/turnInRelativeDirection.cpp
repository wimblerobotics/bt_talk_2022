#include "turnInRelativeDirection.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

TurnInRelativeDirection::TurnInRelativeDirection(
    const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

BT::NodeStatus TurnInRelativeDirection::tick() {
  std::string makeOppositeTurn;
  std::string turnDirection;
extern BT::Tree* g_tree;
std::cout << "--------------" << std::endl;
g_tree->blackboard_stack[0]->debugMessage();
std::cout << "--------------" << std::endl;
g_tree->blackboard_stack[1]->debugMessage();
std::cout << "--------------" << std::endl;
  if (!getInput("makeOppositeTurn", makeOppositeTurn)) {
    throw BT::RuntimeError("missing port [makeOppositeTurn]");
  }
  std::cout << "TurnInRelativeDirection makeOppositeTurn: " << makeOppositeTurn
            << std::endl;

  if (!getInput("turnDirection", turnDirection)) {
    throw BT::RuntimeError("missing port [turnDirection]");
  }
  std::cout << "TurnInRelativeDirection turnDirection: " << turnDirection
            << std::endl;
  static int calls = 0;
  std::cout << "[ TurnInRelativeDirection: call count: " << ++calls << " ]"
            << std::endl;
  return calls < 3 ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
}

BT::PortsList TurnInRelativeDirection::providedPorts() {
  // Optionally, a port can have a human readable description
  return {BT::InputPort<std::string>("makeOppositeTurn"),
          BT::InputPort<std::string>("turnDirection")};
}