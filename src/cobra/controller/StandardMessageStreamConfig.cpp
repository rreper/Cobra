#include <pntos/cobra/controller/StandardMessageStreamConfig.hpp>

namespace pntos::cobra {

void StandardMessageStreamConfig::sequenced_stream_add(api::AspnMessageType t, const std::optional<std::string>& s) {
  std::lock_guard lk(mutex_);
  modes_[{static_cast<int>(t), s}] = BufferMode::SEQUENCED;
}
void StandardMessageStreamConfig::sequenced_stream_remove(api::AspnMessageType t,
                                                          const std::optional<std::string>& s) {
  std::lock_guard lk(mutex_);
  modes_.erase({static_cast<int>(t), s});
}
void StandardMessageStreamConfig::sequenced_stream_all(bool) {
  std::lock_guard lk(mutex_);
  modes_.clear();
  default_mode_ = BufferMode::SEQUENCED;
}
void StandardMessageStreamConfig::immediate_stream_add(api::AspnMessageType t, const std::optional<std::string>& s) {
  std::lock_guard lk(mutex_);
  modes_[{static_cast<int>(t), s}] = BufferMode::IMMEDIATE;
}
void StandardMessageStreamConfig::immediate_stream_remove(api::AspnMessageType t,
                                                          const std::optional<std::string>& s) {
  std::lock_guard lk(mutex_);
  modes_.erase({static_cast<int>(t), s});
}
void StandardMessageStreamConfig::immediate_stream_all(bool) {
  std::lock_guard lk(mutex_);
  modes_.clear();
  default_mode_ = BufferMode::IMMEDIATE;
}

bool StandardMessageStreamConfig::is_sequenced(api::AspnMessageType t, const std::optional<std::string>& s) const {
  std::lock_guard lk(mutex_);
  if (auto it = modes_.find({static_cast<int>(t), s}); it != modes_.end()) return it->second == BufferMode::SEQUENCED;
  if (s) {
    if (auto it = modes_.find({static_cast<int>(t), std::nullopt}); it != modes_.end())
      return it->second == BufferMode::SEQUENCED;
  }
  return default_mode_ == BufferMode::SEQUENCED;
}

}  // namespace pntos::cobra
