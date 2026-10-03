#include <pntos/cobra/fusion/VirtualStateBlockManager.hpp>

namespace pntos::cobra {

using api::LoggingLevel;
using api::Matrix;
using api::Vector;

VirtualStateBlockManager::VirtualStateBlockManager(const VirtualStateBlockManager& o)
    : mediator_(o.mediator_), path_cache_(o.path_cache_), root_map_(o.root_map_), roots_(o.roots_) {
  for (const auto& [id, n] : o.nodes_) {
    Node copy{n.id, n.parent, n.children, n.block ? n.block->clone() : nullptr};
    nodes_.emplace(id, std::move(copy));
  }
}

VirtualStateBlockManager& VirtualStateBlockManager::operator=(const VirtualStateBlockManager& o) {
  if (this != &o) {
    VirtualStateBlockManager tmp(o);
    *this = std::move(tmp);
  }
  return *this;
}

void VirtualStateBlockManager::log(LoggingLevel level, const std::string& msg) const {
  if (mediator_) mediator_->log_message(level, msg);
}

const api::VirtualStateBlock* VirtualStateBlockManager::block(const std::string& target) const {
  auto it = nodes_.find(target);
  return it == nodes_.end() ? nullptr : it->second.block.get();
}

void VirtualStateBlockManager::add_virtual_state_block(std::unique_ptr<api::VirtualStateBlock> vsb) {
  if (!vsb) return;
  const std::string source = vsb->source();
  const std::string target = vsb->target();
  if (source == target) {
    log(LoggingLevel::ERROR, "Source and target tags should not be the same. Virtual state block will not be added.");
    return;
  }
  auto it = nodes_.find(target);
  if (it != nodes_.end()) {
    Node& target_node = it->second;
    if (target_node.parent) {
      log(LoggingLevel::ERROR,
          "Duplicate virtual state block. Target matches an existing block stored in the VirtualStateBlockManager. "
          "Virtual state block will not be added.");
      return;
    }
    target_node.parent = source;
    target_node.block = std::move(vsb);
    roots_.erase(target);
  } else {
    Node n{target, source, {}, std::move(vsb)};
    nodes_.emplace(target, std::move(n));
  }
  auto sit = nodes_.find(source);
  if (sit == nodes_.end()) {
    Node n{source, std::nullopt, {}, nullptr};
    sit = nodes_.emplace(source, std::move(n)).first;
  }
  sit->second.children.insert(target);
  if (!sit->second.parent) roots_.insert(source);
}

void VirtualStateBlockManager::remove_virtual_state_block(const std::string& target) {
  auto it = nodes_.find(target);
  if (it == nodes_.end()) {
    log(LoggingLevel::ERROR, "Block " + target +
                                 " has not been added to or has already been removed from the "
                                 "VirtualStateBlockManager. Cannot remove.");
    return;
  }
  Node node = std::move(it->second);
  nodes_.erase(it);
  log(LoggingLevel::INFO, "Removed " + target + " from the VirtualStateBlockManager.");
  path_cache_.erase(target);
  root_map_.erase(target);
  roots_.erase(target);
  if (node.parent) {
    auto pit = nodes_.find(*node.parent);
    if (pit != nodes_.end()) pit->second.children.erase(node.id);
  }
  for (const auto& child : node.children) remove_virtual_state_block(child);
}

std::optional<std::vector<std::string>> VirtualStateBlockManager::get_path(const std::string& start,
                                                                           const std::string& target) {
  if (!nodes_.count(target)) {
    log(LoggingLevel::ERROR, "Block " + target + " is not being tracked by the VirtualStateBlockManager.");
    return std::nullopt;
  }
  if (auto c = path_cache_.find(target); c != path_cache_.end() && c->second.front() == start) return c->second;
  std::vector<std::string> out;
  const Node* node = &nodes_.at(target);
  while (node->id != start) {
    if (!node->parent) {
      log(LoggingLevel::ERROR, "No path exists from source block " + start + " to target block " + target + ".");
      return std::nullopt;
    }
    out.push_back(node->id);
    node = &nodes_.at(*node->parent);
  }
  out.push_back(node->id);
  std::reverse(out.begin(), out.end());
  path_cache_[target] = out;
  return out;
}

std::optional<api::EstimateWithCovariance> VirtualStateBlockManager::convert(const api::EstimateWithCovariance& orig,
                                                                             const std::string& start,
                                                                             const std::string& target,
                                                                             api::Timestamp time) {
  if (start == target) return orig;
  auto path = get_path(start, target);
  if (!path) return std::nullopt;
  api::EstimateWithCovariance ewc = orig;
  for (const auto& id : *path) {
    auto& node = nodes_.at(id);
    if (!node.block) continue;
    ewc = node.block->convert(ewc, time);
  }
  return ewc;
}

std::optional<Vector> VirtualStateBlockManager::convert_estimate(const Vector& orig, const std::string& start,
                                                                 const std::string& target, api::Timestamp time) {
  if (start == target) return orig;
  auto path = get_path(start, target);
  if (!path) return std::nullopt;
  Vector est = orig;
  for (const auto& id : *path) {
    auto& node = nodes_.at(id);
    if (!node.block) continue;
    est = node.block->convert_estimate(est, time);
  }
  return est;
}

std::optional<Matrix> VirtualStateBlockManager::jacobian(const Vector& orig, const std::string& start,
                                                         const std::string& target, api::Timestamp time) {
  Matrix out = Matrix::Identity(orig.size(), orig.size());
  if (start == target) return out;
  auto path = get_path(start, target);
  if (!path) return std::nullopt;
  Vector est = orig;
  for (const auto& id : *path) {
    auto& node = nodes_.at(id);
    if (!node.block) continue;
    Matrix jac = node.block->jacobian(est, time);
    est = node.block->convert_estimate(est, time);
    out = jac * out;
  }
  return out;
}

std::optional<Matrix> VirtualStateBlockManager::convert_H(api::StandardFusionEngine& engine,
                                                          const std::string& real_label, const std::string& label,
                                                          const Matrix& curr_H) {
  auto real_est = engine.get_state_block_estimate(real_label);
  if (!real_est) return std::nullopt;
  auto real_to_virt = jacobian(*real_est, real_label, label, engine.time());
  if (!real_to_virt) return std::nullopt;
  return curr_H * *real_to_virt;
}

std::optional<std::string> VirtualStateBlockManager::get_start_block_label(const std::string& target) {
  if (!nodes_.count(target)) {
    log(LoggingLevel::ERROR,
        "Block " + target + " has not been added to the VirtualStateBlockManager. No valid starting label exists.");
    return std::nullopt;
  }
  if (auto r = root_map_.find(target); r != root_map_.end()) {
    auto rit = nodes_.find(r->second);
    if (rit != nodes_.end() && !rit->second.parent) return r->second;
  }
  const Node* node = &nodes_.at(target);
  while (node->parent) node = &nodes_.at(*node->parent);
  root_map_[target] = node->id;
  return node->id;
}

std::optional<std::vector<std::string>> VirtualStateBlockManager::get_virtual_state_block_labels() const {
  std::vector<std::string> out;
  for (const auto& [id, n] : nodes_)
    if (!roots_.count(id)) out.push_back(id);
  if (out.empty()) return std::nullopt;
  return out;
}

void VirtualStateBlockManager::give_virtual_state_block_aux_data(const std::string& target,
                                                                 const api::AuxData& aux) {
  auto it = nodes_.find(target);
  if (it == nodes_.end()) {
    log(LoggingLevel::ERROR, "Block " + target +
                                 " has not been added to or has already been removed from the "
                                 "VirtualStateBlockManager. Cannot give aux data.");
    return;
  }
  if (!it->second.block) {
    log(LoggingLevel::ERROR, "Target " + target +
                                 " does not have a VirtualStateBlock associated with it. It likely has not been "
                                 "added to the manager yet.");
    return;
  }
  it->second.block->receive_aux_data(aux);
}

}  // namespace pntos::cobra
