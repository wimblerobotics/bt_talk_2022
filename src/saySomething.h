#ifndef SIMPLE_BT_NODES_H
#define SIMPLE_BT_NODES_H

#include <geometry_msgs/msg/pose.hpp>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace DummyNodes {

BT::NodeStatus CheckBattery();

BT::NodeStatus CheckTemperature();
BT::NodeStatus SayHello();

class GripperInterface {
 public:
  GripperInterface() : _opened(true) {}

  BT::NodeStatus open();

  BT::NodeStatus close();

 private:
  bool _opened;
};

//--------------------------------------

// Example of custom SyncActionNode (synchronous action)
// without ports.
class ApproachObject : public BT::SyncActionNode {
 public:
  ApproachObject(const std::string& name) : BT::SyncActionNode(name, {}) {}

  // You must override the virtual function tick()
  BT::NodeStatus tick() override;
};

// Example of custom SyncActionNode (synchronous action)
// with an input port.
class SaySomething : public BT::SyncActionNode {
 public:
  SaySomething(const std::string& name, const BT::NodeConfiguration& config)
      : BT::SyncActionNode(name, config) {}

  // You must override the virtual function tick()
  BT::NodeStatus tick() {
    auto msg = getInput<geometry_msgs::msg::Pose>("message");
    // Check if optional is valid. If not, throw its error
    if (!msg) {
      throw BT::RuntimeError("missing required input [message]: ", msg.error());
    }

    // use the method value() to extract the valid message.
    std::cout << "Robot says: " << msg->position.x << std::endl;
    return BT::NodeStatus::SUCCESS;
  }

  // It is mandatory to define this static method.
  static BT::PortsList providedPorts() {
    return {BT::InputPort<geometry_msgs::msg::Pose>("message")};
  }
};

// Same as class SaySomething, but to be registered with SimpleActionNode
BT::NodeStatus SaySomethingSimple(BT::TreeNode& self);

inline void RegisterNodes(BT::BehaviorTreeFactory& factory) {
  static GripperInterface grip_singleton;

  factory.registerSimpleCondition("CheckBattery", std::bind(CheckBattery));
  factory.registerSimpleCondition("CheckTemperature",
                                  std::bind(CheckTemperature));
  factory.registerSimpleAction("SayHello", std::bind(SayHello));
  factory.registerSimpleAction(
      "OpenGripper", std::bind(&GripperInterface::open, &grip_singleton));
  factory.registerSimpleAction(
      "CloseGripper", std::bind(&GripperInterface::close, &grip_singleton));
  factory.registerNodeType<ApproachObject>("ApproachObject");
  factory.registerNodeType<SaySomething>("SaySomething");
}

}  // namespace DummyNodes

#endif  // SIMPLE_BT_NODES_H
