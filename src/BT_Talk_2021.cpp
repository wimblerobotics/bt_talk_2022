#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cstdio>
#include <rclcpp/rclcpp.hpp>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "calculateNearestWallGoal.hpp"
#include "canSeeACloseWall.hpp"
#include "closePointSubscriber.hpp"
#include "isNearAWall.hpp"
#include "moveToPose.hpp"
#include "saySomething.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr g_node = rclcpp::Node::make_shared("floorbot_1_node");

  std::string xml_path;
  g_node->declare_parameter<std::string>("xml_path", "foo");
  g_node->get_parameter("xml_path", xml_path);
  BT::BehaviorTreeFactory factory;

  factory.registerNodeType<CalculateNearestWallGoal>(
      "CalculateNearestWallGoal");
  BT_Talk_CanSeeACloseWall::RegisterNodes(factory);
  BT_Talk_IsNearAWall::RegisterNodes(factory);
  factory.registerNodeType<MoveToPose>("MoveToPose");
  factory.registerNodeType<DummyNodes::SaySomething>("SaySomething");
  
  auto tree = factory.createTreeFromFile(xml_path);
 
  BT::PublisherZMQ publisher_zmq(tree);
  BT::StdCoutLogger logger_cout(tree);
  BT::printTreeRecursively(tree.rootNode());

  // ClosePointSubscriber::SharedPtr cps = ClosePointSubscriber::singleton();
  std::shared_ptr<ClosePointSubscriber> cps = ClosePointSubscriber::singleton();

  RCLCPP_INFO(g_node->get_logger(),
              "[main] Waiting for map->lidar_link transform to be available");
  std::shared_ptr<tf2_ros::TransformListener> transform_listener{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer =
      std::make_unique<tf2_ros::Buffer>(g_node->get_clock());
  transform_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  auto timeout_ms_ = std::chrono::milliseconds(2);
  while (!tf_buffer->canTransform("map", "lidar_link", tf2::TimePointZero) &&
         rclcpp::ok()) {
    std::this_thread::sleep_for(timeout_ms_);
  }

  RCLCPP_INFO(g_node->get_logger(),
              "[main] transform is available, starting behavior tree");

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
