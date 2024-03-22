/**
 * @file behavior_tree.cc
 * @author HKUST Aerial Robotics Group
 * @brief
 * @version 0.1
 * @date 2019-07-07
 *
 * @copyright Copyright (c) 2019
 *
 */

#include "eudm_planner/dcp_tree.h"

namespace planning {
DcpTree::DcpTree(const int& tree_height, const decimal_t& layer_time)
    : tree_height_(tree_height), layer_time_(layer_time) {
  last_layer_time_ = layer_time_;
  // 需要先设置根节点决策，因此在这里调用此函数不合适
  GenerateActionScript();
}

DcpTree::DcpTree(const int& tree_height, const decimal_t& layer_time,
                 const decimal_t& last_layer_time)
    : tree_height_(tree_height),
      layer_time_(layer_time),
      last_layer_time_(last_layer_time) {
  // 需要先设置根节点决策，因此在这里调用此函数不合适
  GenerateActionScript();
}

ErrorType DcpTree::UpdateScript() { return GenerateActionScript(); }

// 返回一个新的决策序列，是在决策序列seq_in后面增加n个决策a
// seq_in 不变，重点！！！！
std::vector<DcpTree::DcpAction> DcpTree::AppendActionSequence(
    const std::vector<DcpAction>& seq_in, const DcpAction& a,
    const int& n) const {
  std::vector<DcpAction> seq = seq_in;
  for (int i = 0; i < n; ++i) {
    seq.push_back(a);
  }
  return seq;
}
// 生成DCP Tree, 根节点的横向action和上一帧最优决策一致，纵向action变化。
// 在每一幕episode上，纵向action保持不变，横向action变化一次
ErrorType DcpTree::GenerateActionScript() {
  // 清除历史决策
  action_script_.clear();
  std::vector<DcpAction> ongoing_action_seq;
  for (int lon = 0; lon < static_cast<int>(DcpLonAction::MAX_COUNT); lon++) {
    ongoing_action_seq.clear();
    // ongoing_action_seq的决策在每一步(DCP Tree的每一层)的横纵向action都是一样的
    ongoing_action_seq.push_back(
        DcpAction(DcpLonAction(lon), ongoing_action_.lat, ongoing_action_.t));

    for (int h = 1; h < tree_height_; ++h) {
      for (int lat = 0; lat < static_cast<int>(DcpLatAction::MAX_COUNT); lat++) {
        if (lat != static_cast<int>(ongoing_action_.lat)) {
          // 从DCP Tree的第h层开始，以后的横向action不同于根节点的横向action
          // 纵向action保持不变
          auto actions = AppendActionSequence(
              ongoing_action_seq,
              DcpAction(DcpLonAction(lon), DcpLatAction(lat), layer_time_),
              tree_height_ - h);
          action_script_.push_back(actions);
        }
      }
      // 在DCP Tree的每一层设置为和根节点一样的action
      ongoing_action_seq.push_back(
          DcpAction(DcpLonAction(lon), ongoing_action_.lat, layer_time_));
    }
    // 在整个循环中，这里会push进去 三个episode，每个episode内的action都是一样的
    action_script_.push_back(ongoing_action_seq);
  }
  // override the last layer time
  for (auto& action_seq : action_script_) {
    action_seq.back().t = last_layer_time_;
  }
  return kSuccess;
}

decimal_t DcpTree::planning_horizon() const {
  if (action_script_.empty()) return 0.0;
  decimal_t planning_horizon = 0.0;
  for (const auto& a : action_script_[0]) {
    planning_horizon += a.t;
  }
  return planning_horizon;
}

}  // namespace planning
