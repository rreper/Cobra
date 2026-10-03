#include <pntos/cobra/diagnostics/DiagnosticLogPlugin.hpp>

#include <pntos/cobra/utils/hdf5.hpp>
#include <pntos/cobra/utils/logging.hpp>

namespace pntos::cobra {

using api::LoggingLevel;

void DiagnosticLogPlugin::init_plugin(const std::optional<std::string>&, api::Mediator* mediator) {
  if (!mediator) {
    utils::print_message(LoggingLevel::ERROR, identifier_, "DiagnosticLogPlugin requires a mediator.");
    return;
  }
  mediator_ = mediator;
  auto kv = mediator_->registry().batch(kGroupToWatch);
  kv->request_notify(std::nullopt, [this](const std::string&, const std::vector<std::string>& keys, api::KeyValueStore& s) {
    for (const auto& key : keys)
      if (auto v = s.get(key)) store_[key].push_back(*v);
  });
}

void DiagnosticLogPlugin::shutdown_plugin() {
  if (store_.empty() || !mediator_) return;
  const bool ok = utils::save_to_hdf5_file(output_file_, store_,
                                           [this](LoggingLevel l, const std::string& m) { mediator_->log_message(l, m); });
  if (ok) mediator_->log_message(LoggingLevel::INFO, "Created diagnostics log file: " + output_file_);
}

}  // namespace pntos::cobra
