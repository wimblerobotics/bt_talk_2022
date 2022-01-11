
#include <vector>
#include "behaviortree_cpp_v3/behavior_tree.h"

class BTSimDb {
 public:
  BTSimDb(BTSimDb const&) delete;
  BTSimDb& operator=(BTSimDb const&) = delete;

  static std::shared_ptr<BTSimDb> singleton() {
    static std::shared_ptr<BTSimDb> s_singleton{new BTSimDb()};
    return s_singleton;
  }

 private:
  vector<std::string> names_;
  vector<BT
  BTSimDb(){};
};