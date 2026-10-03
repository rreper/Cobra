#include <pntos/cobra/StandardRegistryPlugin.hpp>
#include <pntos/cobra/utils/logging.hpp>

#include <algorithm>
#include <fstream>
#include <sstream>

namespace pntos::cobra {

using api::LoggingLevel;
using api::RegistryValue;

namespace {
constexpr const char* kBatchErr =
    "Tried to use KeyValueStore outside of batch operation. (Make sure to use `batch_start`/`batch_restart` and "
    "`batch_end`)";
constexpr const char* kDefaultPermanencyDir = "./registry_permanency_files";

std::string sanitize(const std::string& group) {
  std::string s = group;
  for (char& c : s)
    if (c == '/' || c == '\\') c = '_';
  return s;
}

// --- tiny typed text serialisation for permanency ---------------------------------------------
std::string escape(const std::string& s) {
  std::string o;
  for (char c : s) {
    if (c == '\\') o += "\\\\";
    else if (c == '\n') o += "\\n";
    else if (c == '\t') o += "\\t";
    else o += c;
  }
  return o;
}
std::string unescape(const std::string& s) {
  std::string o;
  for (std::size_t i = 0; i < s.size(); ++i) {
    if (s[i] == '\\' && i + 1 < s.size()) {
      char n = s[++i];
      o += n == 'n' ? '\n' : n == 't' ? '\t' : n;
    } else {
      o += s[i];
    }
  }
  return o;
}
std::optional<std::string> serialize(const RegistryValue& v) {
  std::ostringstream os;
  os.precision(17);
  switch (v.index()) {
    case 0: os << "str\t" << escape(std::get<std::string>(v)); break;
    case 1: {
      os << "strs";
      for (const auto& s : std::get<api::StringArray>(v)) os << '\t' << escape(s);
      break;
    }
    case 2: os << "int\t" << std::get<std::int64_t>(v); break;
    case 3: os << "bool\t" << (std::get<bool>(v) ? 1 : 0); break;
    case 4: os << "double\t" << std::get<double>(v); break;
    case 5: {
      const auto& m = std::get<api::Matrix>(v);
      os << "matrix\t" << m.rows() << '\t' << m.cols();
      for (Eigen::Index i = 0; i < m.rows(); ++i)
        for (Eigen::Index j = 0; j < m.cols(); ++j) os << '\t' << m(i, j);
      break;
    }
    default: return std::nullopt;  // Message: not persisted
  }
  return os.str();
}
std::optional<RegistryValue> deserialize(const std::string& line) {
  std::vector<std::string> f;
  std::string cur;
  for (char c : line) {
    if (c == '\t') {
      f.push_back(cur);
      cur.clear();
    } else {
      cur += c;
    }
  }
  f.push_back(cur);
  if (f.empty()) return std::nullopt;
  try {
    if (f[0] == "str" && f.size() >= 2) return RegistryValue(unescape(f[1]));
    if (f[0] == "strs") {
      api::StringArray a;
      for (std::size_t i = 1; i < f.size(); ++i) a.push_back(unescape(f[i]));
      return RegistryValue(a);
    }
    if (f[0] == "int" && f.size() >= 2) return RegistryValue(static_cast<std::int64_t>(std::stoll(f[1])));
    if (f[0] == "bool" && f.size() >= 2) return RegistryValue(f[1] == "1");
    if (f[0] == "double" && f.size() >= 2) return RegistryValue(std::stod(f[1]));
    if (f[0] == "matrix" && f.size() >= 3) {
      const auto r = static_cast<Eigen::Index>(std::stoll(f[1]));
      const auto c = static_cast<Eigen::Index>(std::stoll(f[2]));
      api::Matrix m(r, c);
      std::size_t k = 3;
      for (Eigen::Index i = 0; i < r; ++i)
        for (Eigen::Index j = 0; j < c; ++j) m(i, j) = std::stod(f.at(k++));
      return RegistryValue(m);
    }
  } catch (...) {
  }
  return std::nullopt;
}
}  // namespace

// =============================================================================================
// StandardKeyValueStore
// =============================================================================================

StandardKeyValueStore::StandardKeyValueStore(std::string group, LogFunc log,
                                             const std::optional<std::string>& permanency_dir)
    : group_(std::move(group)), log_(std::move(log)) {
  std::filesystem::path dir = permanency_dir ? std::filesystem::path(*permanency_dir)
                                             : std::filesystem::path(kDefaultPermanencyDir);
  permanency_file_ = dir / (sanitize(group_) + ".registry");
  load_permanent();
}

void StandardKeyValueStore::load_permanent() {
  if (!permanency_file_ || !std::filesystem::exists(*permanency_file_)) return;
  std::ifstream in(*permanency_file_);
  std::string line;
  while (std::getline(in, line)) {
    auto tab = line.find('\t');
    if (tab == std::string::npos) continue;
    std::string key = unescape(line.substr(0, tab));
    auto val = deserialize(line.substr(tab + 1));
    if (!val) continue;
    if (!store_.count(key)) order_.push_back(key);
    store_[key] = *val;
  }
}

void StandardKeyValueStore::save_permanent() {
  if (permanent_keys_.empty() || !permanency_file_) return;
  std::error_code ec;
  std::filesystem::create_directories(permanency_file_->parent_path(), ec);
  std::ofstream out(*permanency_file_, std::ios::trunc);
  for (const auto& key : order_) {
    if (!permanent_keys_.count(key)) continue;
    auto it = store_.find(key);
    if (it == store_.end()) continue;
    auto s = serialize(it->second);
    if (!s) {
      log_(LoggingLevel::WARN, "Permanent key '" + key + "' holds a Message, which cannot be persisted; skipping.");
      continue;
    }
    out << escape(key) << '\t' << *s << '\n';
  }
}

void StandardKeyValueStore::check_batch_operation() const {
  if (!batch_live_) log_(LoggingLevel::ERROR, kBatchErr);
}

std::optional<std::vector<std::string>> StandardKeyValueStore::keys() const {
  check_batch_operation();
  if (order_.empty()) return std::nullopt;
  return order_;
}

bool StandardKeyValueStore::has_key(const std::string& key) const {
  check_batch_operation();
  return store_.count(key) > 0;
}

api::RegistryValueType StandardKeyValueStore::get_type(const std::string& key) const {
  check_batch_operation();
  auto it = store_.find(key);
  if (it == store_.end()) return api::RegistryValueType::KEY_DNE;
  return api::registry_value_type(it->second);
}

std::optional<RegistryValue> StandardKeyValueStore::get(const std::string& key) const {
  check_batch_operation();
  auto it = store_.find(key);
  if (it == store_.end()) {
    log_(LoggingLevel::WARN, "The key '" + key + "' is not found in the StandardKeyValueStore.");
    return std::nullopt;
  }
  return it->second;
}

std::optional<std::vector<unsigned char>> StandardKeyValueStore::get_raw(const std::optional<std::string>& key) const {
  check_batch_operation();
  if (!key) {
    log_(LoggingLevel::ERROR, "This implementation requires a key to be passed to get_raw.");
    return std::nullopt;
  }
  auto it = store_.find(*key);
  if (it == store_.end()) {
    log_(LoggingLevel::WARN, "Key " + *key + " does not exist in group " + group_ + ".");
    return std::nullopt;
  }
  auto s = api::registry_value_as<std::string>(it->second);
  if (!s) {
    log_(LoggingLevel::ERROR, "Value at key " + *key + " cannot be converted to string.");
    return std::nullopt;
  }
  return std::vector<unsigned char>(s->begin(), s->end());
}

void StandardKeyValueStore::set(const std::string& key, RegistryValue value) {
  check_batch_operation();
  if (set_permanent_) permanent_keys_.insert(key);
  auto it = store_.find(key);
  if (it == store_.end()) {
    order_.push_back(key);
    store_.emplace(key, std::move(value));
  } else {
    it->second = std::move(value);
  }
  if (std::find(modified_keys_.begin(), modified_keys_.end(), key) == modified_keys_.end())
    modified_keys_.push_back(key);
}

void StandardKeyValueStore::set_raw(const std::optional<std::string>& key, const std::vector<unsigned char>& bytes) {
  check_batch_operation();
  if (!key) {
    log_(LoggingLevel::ERROR, "This implementation requires a key to be passed to set_raw.");
    return;
  }
  set(*key, std::string(bytes.begin(), bytes.end()));
}

bool StandardKeyValueStore::remove_key(const std::string& key) {
  check_batch_operation();
  auto it = store_.find(key);
  if (it == store_.end()) {
    log_(LoggingLevel::ERROR, "Key " + key + " does not exist in store. Unable to delete.");
    return false;
  }
  store_.erase(it);
  order_.erase(std::remove(order_.begin(), order_.end(), key), order_.end());
  if (std::find(modified_keys_.begin(), modified_keys_.end(), key) == modified_keys_.end())
    modified_keys_.push_back(key);
  if (permanent_keys_.count(key) && set_permanent_) permanent_keys_.erase(key);
  return true;
}

void StandardKeyValueStore::clear() {
  check_batch_operation();
  store_.clear();
  order_.clear();
  callbacks_.clear();
  modified_keys_.clear();
  permanent_keys_.clear();
}

std::vector<RegistryValue> StandardKeyValueStore::values() const {
  check_batch_operation();
  std::vector<RegistryValue> out;
  out.reserve(order_.size());
  for (const auto& k : order_) out.push_back(store_.at(k));
  return out;
}

std::vector<std::pair<std::string, RegistryValue>> StandardKeyValueStore::items() const {
  check_batch_operation();
  std::vector<std::pair<std::string, RegistryValue>> out;
  out.reserve(order_.size());
  for (const auto& k : order_) out.emplace_back(k, store_.at(k));
  return out;
}

void StandardKeyValueStore::batch_end() {
  check_batch_operation();
  save_permanent();
  set_permanent_ = false;

  if (!modified_keys_.empty()) {
    // Snapshot callbacks: a callback may (mis)behave and register/remove others.
    std::vector<Callback> snapshot;
    for (const auto& [id, cb] : callbacks_) snapshot.push_back(cb);
    const std::vector<std::string> modified = modified_keys_;
    // Non-keyed callbacks get all modified keys
    for (const auto& cb : snapshot)
      if (!cb.key) cb.fn(group_, modified, *this);
    // Keyed callbacks get only their matching keys (one call per callback)
    for (const auto& cb : snapshot) {
      if (!cb.key) continue;
      std::vector<std::string> mine;
      for (const auto& k : modified)
        if (k == *cb.key) mine.push_back(k);
      if (!mine.empty()) cb.fn(group_, mine, *this);
    }
  }
  modified_keys_.clear();
  batch_live_ = false;
}

void StandardKeyValueStore::batch_restart() {
  if (batch_live_) {
    log_(LoggingLevel::ERROR, "Tried to restart batch while batch was in progress.");
    return;
  }
  batch_live_ = true;
  modified_keys_.clear();
}

std::optional<api::NotifyToken> StandardKeyValueStore::request_notify(const std::optional<std::string>& key,
                                                                      api::KeyNotifyCallback callback) {
  check_batch_operation();
  api::NotifyToken token{next_token_++};
  callbacks_[token.id] = Callback{key, std::move(callback)};
  return token;
}

bool StandardKeyValueStore::remove_notify(api::NotifyToken token) {
  check_batch_operation();
  return callbacks_.erase(token.id) > 0;
}

bool StandardKeyValueStore::set_permanent(bool permanent) {
  check_batch_operation();
  set_permanent_ = permanent;
  return true;
}

// =============================================================================================
// StandardRegistry
// =============================================================================================

StandardRegistry::StandardRegistry(LogFunc log, const std::optional<std::string>& permanency_dir)
    : log_(std::move(log)), permanency_dir_(permanency_dir) {}

std::shared_ptr<api::KeyValueStore> StandardRegistry::batch_start(const std::string& group) {
  std::shared_ptr<StandardKeyValueStore> store;
  std::vector<std::function<void(const std::string&)>> to_notify;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = groups_.find(group);
    if (it == groups_.end()) {
      store = std::make_shared<StandardKeyValueStore>(group, log_, permanency_dir_);
      groups_.emplace(group, store);
      group_order_.push_back(group);
      to_notify = callbacks_;
    } else {
      store = it->second;
    }
  }
  for (auto& cb : to_notify) cb(group);
  if (store->batch_live()) log_(LoggingLevel::ERROR, "Batch \"" + group + "\" already live.");
  store->mark_batch_live();
  return store;
}

std::optional<std::vector<std::string>> StandardRegistry::group_array() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (group_order_.empty()) return std::nullopt;
  return group_order_;
}

bool StandardRegistry::has_group(const std::string& group) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return groups_.count(group) > 0;
}

bool StandardRegistry::request_notify_new_group(std::function<void(const std::string&)> callback) {
  std::lock_guard<std::mutex> lock(mutex_);
  callbacks_.push_back(std::move(callback));
  return true;
}

void StandardRegistry::end_all_batches() {
  std::vector<std::shared_ptr<StandardKeyValueStore>> live;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto& [g, s] : groups_)
      if (s->batch_live()) live.push_back(s);
  }
  for (auto& s : live) s->batch_end();
}

// =============================================================================================
// StandardRegistryPlugin
// =============================================================================================

StandardRegistryPlugin::StandardRegistryPlugin(std::string identifier,
                                               std::vector<std::shared_ptr<const BaseConfig>> config)
    : identifier_(std::move(identifier)), config_(std::move(config)) {}

void StandardRegistryPlugin::init_plugin(const std::optional<std::string>& plugin_resources_location,
                                         api::Mediator* mediator) {
  if (!mediator) {
    log(LoggingLevel::ERROR, "This registry requires a mediator.");
    return;
  }
  mediator_ = mediator;
  plugin_resources_location_ = plugin_resources_location;
}

void StandardRegistryPlugin::shutdown_plugin() {
  for (auto& r : registries_) r->end_all_batches();
}

namespace {
/// A mediator view that swaps in a different registry (Cobra copied the mediator for this).
class RegistryOverrideMediator final : public api::Mediator {
 public:
  RegistryOverrideMediator(api::Mediator* base, api::Registry& reg) : base_(base), reg_(reg) {}
  std::vector<std::string> filter_description_list() const override { return base_->filter_description_list(); }
  std::optional<std::vector<std::optional<api::Message>>> request_solutions(
      const std::vector<api::Timestamp>& t, const std::optional<std::string>& d) override {
    return base_->request_solutions(t, d);
  }
  void process_pntos_message(const api::Message& m) override { base_->process_pntos_message(m); }
  void broadcast_aspn_message(const api::Message& m, const std::optional<std::string>& t,
                              const std::optional<std::string>& d) override {
    base_->broadcast_aspn_message(m, t, d);
  }
  void log_message(LoggingLevel l, const std::string& m) override { base_->log_message(l, m); }
  api::Registry& registry() override { return reg_; }

 private:
  api::Mediator* base_;
  api::Registry& reg_;
};
}  // namespace

std::shared_ptr<api::Registry> StandardRegistryPlugin::new_registry(const std::optional<std::string>& initial_config) {
  if (initial_config)
    log(LoggingLevel::ERROR, "initial_config parameter is unsupported by this implementation; ignoring values.");
  auto out = std::make_shared<StandardRegistry>([this](LoggingLevel l, const std::string& m) { log(l, m); },
                                                plugin_resources_location_);
  if (!config_.empty()) {
    if (!mediator_) {
      log(LoggingLevel::ERROR, "Cannot load config into registry: init_plugin was not called with a mediator.");
    } else {
      RegistryOverrideMediator view(mediator_, *out);
      for (const auto& conf : config_) conf->to_registry(view);
    }
  }
  registries_.push_back(out);
  return out;
}

void StandardRegistryPlugin::log(LoggingLevel level, const std::string& message) const {
  if (mediator_) mediator_->log_message(level, message);
  else utils::print_message(level, "RegistryPlugin", message);
}

}  // namespace pntos::cobra
