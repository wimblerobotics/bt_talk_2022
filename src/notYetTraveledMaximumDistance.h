#pragma once

#include "behaviortree_cpp_v3/action_node.h"

class NotYetTraveledMaximumDistance : public BT::SyncActionNode {
 public:
  NotYetTraveledMaximumDistance(const std::string& name,
                                const BT::NodeConfiguration& config)
      : SyncActionNode(name, config) {
    setRegistrationID("NotYetTraveledMaximumDistance");
  }

  static BT::PortsList providedPorts() {
    return {BT::InputPort<float>("distanceTraveled",
                          "Value representing distance traveled so far."),
            BT::InputPort<float>("maxDistance", "Maximum travel distance allowed")};
  }

 private:
  virtual BT::NodeStatus tick() override {
    float distanceTraveled;
    float maxDistance;
    if (!getInput<float>("distanceTraveled", distanceTraveled)) {
      throw BT::RuntimeError("missing port [distanceTraveled]");
    }
    std::cout << "NotYetTraveledMaximumDistance distanceTraveled: "
         << distanceTraveled << std::endl;

    if (!getInput<float>("maxDistance", maxDistance)) {
      throw BT::RuntimeError("missing port [maxDistance]");
    }
    std::cout << "NotYetTraveledMaximumDistance maxDistance: " << maxDistance
         << std::endl;

    static int calls = 0;
    std::cout << "[ NotYetTraveledMaximumDistance: call count: " << ++calls
              << " ]" << std::endl;
    return distanceTraveled >= maxDistance
               ? BT::NodeStatus::FAILURE
               : BT::NodeStatus::SUCCESS;
  }
};
