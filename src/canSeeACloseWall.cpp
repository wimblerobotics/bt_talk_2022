#include "canSeeACloseWall.h"

#include <vector>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "closePointSubscriber.h"

BT::NodeStatus CanSeeACloseWall() {
  const vector<laser_lines::msg::ClosestPointToLineSegment>& line_segments =
      ClosePointSubscriber::singleton()->getLineSements();
  for (size_t i = 0; i < line_segments.size(); i++) {
    if (line_segments[i].distance < 6.0) {
      return BT::NodeStatus::SUCCESS;
    } else {
      std::cout << "[CanSeeACloseWall] rejecting distance: "
                << line_segments[i].distance << std::endl;
    }
  }

  return BT::NodeStatus::RUNNING;
}