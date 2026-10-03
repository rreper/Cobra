// Port of pntos.cobra.standard_plugins.fusion.VirtualStateBlockManager.
#pragma once

#include <pntos/api/fusion.hpp>

#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace pntos::cobra {

/// Manages VirtualStateBlocks for a StandardFusionEngine as a forest of n-ary trees whose roots
/// are "real" state blocks. Nodes without a parent are treated as roots; the manager cannot tell a
/// real block apart from a virtual block whose own VSB has not been added yet.
class VirtualStateBlockManager {
 public:
  struct Node {
    std::string id;
    std::optional<std::string> parent;
    std::set<std::string> children;
    std::unique_ptr<api::VirtualStateBlock> block;  ///< null for root nodes
  };

  explicit VirtualStateBlockManager(api::Mediator* mediator) : mediator_(mediator) {}
  VirtualStateBlockManager(const VirtualStateBlockManager&);
  VirtualStateBlockManager& operator=(const VirtualStateBlockManager&);
  VirtualStateBlockManager(VirtualStateBlockManager&&) noexcept = default;
  VirtualStateBlockManager& operator=(VirtualStateBlockManager&&) noexcept = default;

  /// Logs an error and drops the block if source == target or target already has a VSB.
  void add_virtual_state_block(std::unique_ptr<api::VirtualStateBlock> vsb);
  /// Removes `target` and prunes every descendant. Logs an error if unknown.
  void remove_virtual_state_block(const std::string& target);

  std::optional<api::EstimateWithCovariance> convert(const api::EstimateWithCovariance& orig,
                                                     const std::string& start, const std::string& target,
                                                     api::Timestamp time);
  std::optional<api::Vector> convert_estimate(const api::Vector& orig, const std::string& start,
                                              const std::string& target, api::Timestamp time);
  /// Chain-rule Jacobian from `start` to `target` evaluated at `orig` (identity if start == target).
  std::optional<api::Matrix> jacobian(const api::Vector& orig, const std::string& start, const std::string& target,
                                      api::Timestamp time);
  /// curr_H (m × n_virtual) mapped back onto the real block: curr_H · d(virtual)/d(real).
  std::optional<api::Matrix> convert_H(api::StandardFusionEngine& engine, const std::string& real_label,
                                       const std::string& label, const api::Matrix& curr_H);

  /// Root label for `target` (cached). nullopt and an error log if `target` is unknown.
  std::optional<std::string> get_start_block_label(const std::string& target);
  /// All non-root labels; nullopt if there are none.
  std::optional<std::vector<std::string>> get_virtual_state_block_labels() const;
  void give_virtual_state_block_aux_data(const std::string& target, const api::AuxData& aux);

  bool has_node(const std::string& label) const { return nodes_.count(label) != 0; }
  std::size_t num_nodes() const { return nodes_.size(); }
  /// Test hook: the VSB stored under `target`, or null.
  const api::VirtualStateBlock* block(const std::string& target) const;

 private:
  void log(api::LoggingLevel level, const std::string& msg) const;
  std::optional<std::vector<std::string>> get_path(const std::string& start, const std::string& target);

  api::Mediator* mediator_;
  std::map<std::string, Node> nodes_;
  std::map<std::string, std::vector<std::string>> path_cache_;
  std::map<std::string, std::string> root_map_;
  std::set<std::string> roots_;
};

}  // namespace pntos::cobra
