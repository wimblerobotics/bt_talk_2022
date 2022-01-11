#include <cstdio>
#include <rclcpp/rclcpp.hpp>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "calculateNearestWallGoal.h"
#include "canSeeACloseWall.h"
#include "closePointSubscriber.h"
#include "isNearAWall.h"
#include "moveToPose.hpp"
#include "notAboutToCollide.h"
#include "notYetTraveledMaximumDistance.h"
#include "saySomething.h"
#include "turnInRelativeDirection.h"


BT::Tree* g_tree;
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr g_node = rclcpp::Node::make_shared("floorbot_1_node");

  std::string xml_path;
  g_node->declare_parameter<std::string>("xml_path", "foo");
  g_node->get_parameter("xml_path", xml_path);
  BT::BehaviorTreeFactory factory;
  auto tree = factory.createTreeFromFile(xml_path);
  g_tree = &tree;

  factory.registerNodeType<CalculateNearestWallGoal>(
      "CalculateNearestWallGoal");
  factory.registerSimpleCondition("CanSeeACloseWall",
                                  std::bind(CanSeeACloseWall));
  factory.registerSimpleCondition("IsNearAWall", std::bind(IsNearAWall));
  factory.registerSimpleCondition("NotAboutToCollide",
                                  std::bind(NotAboutToCollide));
  factory.registerNodeType<MoveToPose>("MoveToPose");
  factory.registerNodeType<NotYetTraveledMaximumDistance>(
      "NotYetTraveledMaximumDistance");
  factory.registerNodeType<DummyNodes::SaySomething>("SaySomething");
  factory.registerNodeType<TurnInRelativeDirection>("TurnInRelativeDirection");

  BT::PublisherZMQ publisher_zmq(tree);
  BT::StdCoutLogger logger_cout(tree);
  BT::printTreeRecursively(tree.rootNode());

  // ClosePointSubscriber::SharedPtr cps = ClosePointSubscriber::singleton();
  std::shared_ptr<ClosePointSubscriber> cps = ClosePointSubscriber::singleton();

  {
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    rclcpp::WallRate loopRate(15);
    while (rclcpp::ok() && (status == BT::NodeStatus::RUNNING)) {
      rclcpp::spin_some(cps);
      status = tree.tickRoot();
      loopRate.sleep();
    }
  }

  printf("BT_Talk_2021 finished\n");
  return 0;
}
