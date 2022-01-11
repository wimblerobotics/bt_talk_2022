#include "moveAShortDistanceAhead.h"

#include <behaviortree_cpp_v3/behavior_tree.h>

MoveAShortDistanceAhead::MoveAShortDistanceAhead(
    const std::string& name, const BT::NodeConfiguration& config)
    : BT::ActionNodeBase(name, config) {
    }

BT::NodeStatus MoveAShortDistanceAhead::tick() {
  float distanceTraveled;
  if (!getInput<float>("distanceTraveled", distanceTraveled)) {
    throw BT::RuntimeError("missing port [distanceTraveled]");
  }
  std::cout << "MoveAShortDistanceAhead distanceTraveled: " << distanceTraveled
            << std::endl;
  static int calls = 0;
  std::cout << "[ MoveAShortDistanceAhead: call count: " << ++calls << " ]"
            << std::endl;
  float dt = distanceTraveled + 0.5;
std::cout << "dt: " << dt  << std::endl;
  setOutput("distanceTraveled", dt);
  getInput("distanceTraveled", distanceTraveled);
  std::cout << "After setting, distanceTraveled: " << distanceTraveled << std::endl;
  return BT::NodeStatus::RUNNING;
}

BT::PortsList MoveAShortDistanceAhead::providedPorts() {
  // Optionally, a port can have a human readable description
  return {BT::BidirectionalPort<float>("distanceTraveled")};
}

void MoveAShortDistanceAhead::halt() { setStatus(BT::NodeStatus::IDLE); }
