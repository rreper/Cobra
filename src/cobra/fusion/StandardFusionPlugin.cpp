#include <pntos/cobra/config/configs.hpp>
#include <pntos/cobra/fusion/StandardFusionPlugin.hpp>
#include <pntos/cobra/utils/aspn.hpp>

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <set>
#include <sstream>

namespace pntos::cobra {

using api::EstimateWithCovariance;
using api::EstimateWithCovarianceType;
using api::LoggingLevel;
using api::Matrix;
using api::Timestamp;
using api::Vector;

namespace {
std::ostream* g_trace = nullptr;
int g_in_peek = 0;
std::ofstream g_trace_file;
bool g_trace_checked = false;
std::ostream* trace() {
  if (!g_trace_checked) {
    g_trace_checked = true;
    if (const char* f = std::getenv("PNTOS_TRACE_FILE")) {
      g_trace_file.open(f);
      if (g_trace_file) g_trace = &g_trace_file;
    }
  }
  return g_trace;
}
void trace_state(const char* kind, const std::string& what, Timestamp a, Timestamp b, api::StandardFusionStrategy* s) {
  auto* os = trace();
  if (!os || !s || g_in_peek > 0) return;
  auto P = s->covariance();
  auto x = s->estimate();
  *os << kind << ' ' << what << ' ' << a.elapsed_nsec << ' ' << b.elapsed_nsec << ' ' << std::setprecision(17)
      << (P ? P->trace() : 0.0);
  if (x && x->size() >= 9) *os << ' ' << (*x)(6) << ' ' << (*x)(7) << ' ' << (*x)(8);
  if (P && P->rows() >= 9) *os << ' ' << (*P)(8, 8);
  if (std::getenv("PNTOS_TRACE_FULL") && x && P) {
    *os << " |";
    for (Eigen::Index i = 0; i < x->size(); ++i) *os << ' ' << (*x)(i);
    *os << " |";
    for (Eigen::Index i = 0; i < P->rows(); ++i) *os << ' ' << (*P)(i, i);
  }
  *os << '\n';
}
std::string secs(Timestamp t) {
  std::ostringstream os;
  os << std::fixed << std::setprecision(9) << t.seconds();
  return os.str();
}
}  // namespace

StandardFusionEngine::StandardFusionEngine(api::Mediator* mediator, bool save_x_and_p_after_prop,
                                           bool save_x_and_p_after_update)
    : mediator_(mediator),
      vsb_manager_(mediator),
      save_after_prop_(save_x_and_p_after_prop),
      save_after_update_(save_x_and_p_after_update) {}

void StandardFusionEngine::log(LoggingLevel level, const std::string& msg) const {
  if (mediator_) mediator_->log_message(level, msg);
}

StandardFusionEngine::StateBlockInfo* StandardFusionEngine::find_block(const std::string& label) {
  for (auto& b : sb_)
    if (b.label == label) return &b;
  return nullptr;
}
const StandardFusionEngine::StateBlockInfo* StandardFusionEngine::find_block(const std::string& label) const {
  for (auto& b : sb_)
    if (b.label == label) return &b;
  return nullptr;
}
api::StandardMeasurementProcessor* StandardFusionEngine::find_processor(const std::string& label) {
  for (auto& p : mp_)
    if (p->label() == label) return p.get();
  return nullptr;
}

void StandardFusionEngine::re_index_stateblocks() {
  Eigen::Index next = 0;
  for (auto& b : sb_) {
    b.start_index = next;
    b.stop_index = next + b.num_states;
    next += b.num_states;
  }
}

api::GenXandP StandardFusionEngine::gen_x_and_p_func() {
  return [this](const std::vector<std::string>& labels) { return generate_x_and_p(labels); };
}

void StandardFusionEngine::save_x_and_p_to_registry() {
  if (!strategy_ || !mediator_) return;
  auto estimate = strategy_->estimate();
  auto covariance = strategy_->covariance();
  if (!estimate || !covariance) return;
  auto kv = mediator_->registry().batch("diagnostics");
  if (saved_state_labels_.empty()) {
    saved_state_labels_.assign(static_cast<std::size_t>(num_states_), "");
    for (const auto& b : sb_)
      for (Eigen::Index i = b.start_index; i < b.stop_index; ++i)
        saved_state_labels_[static_cast<std::size_t>(i)] = b.label + "_state" + std::to_string(i - b.start_index);
    kv->set("state_labels", api::StringArray(saved_state_labels_));
  }
  // Clear the time key so a notify fires even if the time has not changed.
  if (kv->has_key("time")) {
    auto t = kv->get_value<std::int64_t>("time");
    if (t && *t == time_.elapsed_nsec) kv->remove_key("time");
  }
  kv->set("time", time_.elapsed_nsec);
  kv->set("estimate", Matrix(*estimate));
  kv->set("sigma", Matrix(covariance->diagonal().cwiseSqrt()));
}

std::optional<std::vector<std::string>> StandardFusionEngine::state_block_labels() const {
  if (num_states_ > 0) {
    std::vector<std::string> out;
    for (const auto& b : sb_) out.push_back(b.label);
    return out;
  }
  log(LoggingLevel::WARN, "No state blocks added.");
  return std::nullopt;
}

void StandardFusionEngine::add_state_block(std::unique_ptr<api::StandardStateBlock> block,
                                           const EstimateWithCovariance& init,
                                           const std::optional<api::CrossCovariances>& cross_covariances) {
  if (!strategy_) throw std::logic_error("FusionStrategy has not been set");
  if (!block) return;
  if (init.type != EstimateWithCovarianceType::EWC_GENERIC) {
    log(LoggingLevel::ERROR,
        "Current implementation of add_state_block only supports EstimateWithCovarianceType.EWC_GENERIC.");
    return;
  }
  const Eigen::Index num_new = init.estimate.size();
  const Eigen::Index num_cur = static_cast<Eigen::Index>(strategy_->num_states());
  StateBlockInfo info{block->label(), num_new, num_cur, num_cur + num_new, nullptr};

  std::optional<Matrix> cross;
  if (cross_covariances) {
    cross = Matrix::Zero(num_cur, num_new);
    for (std::size_t j = 0; j < cross_covariances->block_labels.size(); ++j) {
      const auto& lbl = cross_covariances->block_labels[j];
      if (const auto* other = find_block(lbl)) {
        const Matrix& cc = cross_covariances->cross_covariances.at(j);
        if (cc.rows() != other->num_states || cc.cols() != num_new)
          throw std::invalid_argument("Cross covariance for block " + lbl + " has the wrong shape.");
        cross->block(other->start_index, 0, other->num_states, num_new) = cc;
      } else {
        log(LoggingLevel::WARN,
            "Cross covariance label (" + lbl + ") requested in add_state_block does not exist.  No action taken.");
      }
    }
  }
  strategy_->add_states(init.estimate, init.covariance, cross);
  num_states_ += num_new;
  info.block = std::move(block);
  sb_.push_back(std::move(info));
  saved_state_labels_.clear();
}

const StandardFusionEngine::StateBlockInfo* StandardFusionEngine::resolve(const std::string& label, bool& is_real,
                                                                          const char* what) {
  if (const auto* b = find_block(label)) {
    is_real = true;
    return b;
  }
  is_real = false;
  auto real = vsb_manager_.get_start_block_label(label);
  if (!real) {
    log(LoggingLevel::ERROR,
        std::string("Unable to obtain the state block ") + what + " for VirtualStateBlock \"" + label + "\".");
    return nullptr;
  }
  const auto* b = find_block(*real);
  if (!b) log(LoggingLevel::ERROR, "Unable to find the state block \"" + *real + "\".");
  return b;
}

std::optional<Vector> StandardFusionEngine::get_state_block_estimate(const std::string& block_label) {
  if (!strategy_) throw std::logic_error("FusionStrategy has not been set");
  bool real = false;
  const auto* sb = resolve(block_label, real, "estimate");
  if (!sb) return std::nullopt;
  auto full = strategy_->estimate();
  if (!full) {
    log(LoggingLevel::ERROR, "Unable to get estimate from strategy.");
    return std::nullopt;
  }
  Vector est = full->segment(sb->start_index, sb->num_states);
  if (real) return est;
  return vsb_manager_.convert_estimate(est, sb->label, block_label, time_);
}

std::optional<Matrix> StandardFusionEngine::get_state_block_covariance(const std::string& block_label) {
  if (!strategy_) throw std::logic_error("FusionStrategy has not been set");
  bool real = false;
  const auto* sb = resolve(block_label, real, "covariance");
  if (!sb) return std::nullopt;
  auto full_cov = strategy_->covariance();
  if (!full_cov) {
    log(LoggingLevel::ERROR, "Unable to get covariance from strategy.");
    return std::nullopt;
  }
  Matrix cov = full_cov->block(sb->start_index, sb->start_index, sb->num_states, sb->num_states);
  if (real) return cov;
  auto full_est = strategy_->estimate();
  if (!full_est) {
    log(LoggingLevel::ERROR, "Unable to get estimate from strategy.");
    return std::nullopt;
  }
  Vector est = full_est->segment(sb->start_index, sb->num_states);
  auto jac = vsb_manager_.jacobian(est, sb->label, block_label, time_);
  if (!jac) return std::nullopt;
  return Matrix(*jac * cov * jac->transpose());
}

std::optional<Matrix> StandardFusionEngine::get_state_block_cross_covariance(const std::string& l1,
                                                                             const std::string& l2) {
  if (!strategy_) throw std::logic_error("FusionStrategy has not been set");
  bool real1 = false, real2 = false;
  const auto* sb1 = resolve(l1, real1, "estimate");
  if (!sb1) return std::nullopt;
  const auto* sb2 = resolve(l2, real2, "estimate");
  if (!sb2) return std::nullopt;
  auto full_cov = strategy_->covariance();
  if (!full_cov) {
    log(LoggingLevel::ERROR, "Unable to get covariance from strategy.");
    return std::nullopt;
  }
  Matrix cov = full_cov->block(sb1->start_index, sb2->start_index, sb1->num_states, sb2->num_states);
  if (real1 && real2) return cov;
  auto full_est = strategy_->estimate();
  if (!full_est) {
    log(LoggingLevel::ERROR, "Unable to get estimate from strategy.");
    return std::nullopt;
  }
  Vector e1 = full_est->segment(sb1->start_index, sb1->num_states);
  Vector e2 = full_est->segment(sb2->start_index, sb2->num_states);
  auto j1 = vsb_manager_.jacobian(e1, sb1->label, l1, time_);
  auto j2 = vsb_manager_.jacobian(e2, sb2->label, l2, time_);
  if (!j1 || !j2) return std::nullopt;
  return Matrix(*j1 * cov * j2->transpose());
}

void StandardFusionEngine::set_state_block_estimate(const std::string& block_label, const Vector& estimate) {
  if (!strategy_) throw std::logic_error("FusionStrategy has not been set");
  auto* sb = find_block(block_label);
  if (!sb) {
    log(LoggingLevel::WARN,
        "block label (" + block_label + ") requested in set_state_block_estimate does not exist. No action taken.");
    return;
  }
  if (sb->num_states != estimate.size()) {
    log(LoggingLevel::WARN, "Size mismatch: The number of states in the stateblock with label (" + block_label +
                                ") is " + std::to_string(sb->num_states) +
                                ", but the provided new estimate is of length " + std::to_string(estimate.size()) +
                                ". No action taken.");
    return;
  }
  strategy_->set_estimate_slice(estimate, static_cast<std::size_t>(sb->start_index));
}

void StandardFusionEngine::set_state_block_covariance(const std::string& block_label, const Matrix& covariance) {
  if (!strategy_) throw std::logic_error("FusionStrategy has not been set");
  auto* sb = find_block(block_label);
  if (!sb) {
    log(LoggingLevel::WARN,
        "block label (" + block_label + ") requested in set_state_block_covariance does not exist. No action taken.");
    return;
  }
  if (covariance.rows() != sb->num_states || covariance.cols() != sb->num_states)
    throw std::invalid_argument("covariance must be " + std::to_string(sb->num_states) + "x" +
                                std::to_string(sb->num_states));
  strategy_->set_covariance_slice(covariance, static_cast<std::size_t>(sb->start_index));
}

void StandardFusionEngine::set_state_block_cross_covariance(const std::string& l1, const std::string& l2,
                                                            const Matrix& covariance) {
  if (!strategy_) throw std::logic_error("FusionStrategy has not been set");
  auto* sb1 = find_block(l1);
  if (!sb1) {
    log(LoggingLevel::WARN,
        "block_label1 (" + l1 + ") requested in set_state_block_cross_covariance does not exist. No action taken.");
    return;
  }
  auto* sb2 = find_block(l2);
  if (!sb2) {
    log(LoggingLevel::WARN,
        "block_label2 (" + l2 + ") requested in set_state_block_cross_covariance does not exist. No action taken.");
    return;
  }
  if (covariance.rows() != sb1->num_states || covariance.cols() != sb2->num_states)
    throw std::invalid_argument("covariance must be " + std::to_string(sb1->num_states) + "x" +
                                std::to_string(sb2->num_states));
  strategy_->set_covariance_slice(covariance, static_cast<std::size_t>(sb1->start_index),
                                  static_cast<std::size_t>(sb2->start_index));
  strategy_->set_covariance_slice(covariance.transpose(), static_cast<std::size_t>(sb2->start_index),
                                  static_cast<std::size_t>(sb1->start_index));
}

void StandardFusionEngine::remove_state_block(const std::string& block_label) {
  if (!strategy_) throw std::logic_error("FusionStrategy has not been set");
  auto it = std::find_if(sb_.begin(), sb_.end(), [&](const StateBlockInfo& b) { return b.label == block_label; });
  if (it == sb_.end()) {
    log(LoggingLevel::WARN, "Stateblock to be removed (" + block_label + ") does not exist.  No action taken.");
    return;
  }
  strategy_->remove_states(static_cast<std::size_t>(it->start_index), static_cast<std::size_t>(it->num_states));
  num_states_ -= it->num_states;
  sb_.erase(it);
  re_index_stateblocks();
  saved_state_labels_.clear();
}

std::optional<std::vector<std::string>> StandardFusionEngine::measurement_processor_labels() const {
  if (mp_.empty()) return std::nullopt;
  std::vector<std::string> out;
  for (const auto& p : mp_) out.push_back(p->label());
  return out;
}

void StandardFusionEngine::add_measurement_processor(std::unique_ptr<api::StandardMeasurementProcessor> processor) {
  if (!processor) return;
  if (find_processor(processor->label())) {
    log(LoggingLevel::WARN,
        "Measurement processor to be added (" + processor->label() + ") already exists.  No action taken.");
    return;
  }
  mp_.push_back(std::move(processor));
}

void StandardFusionEngine::remove_measurement_processor(const std::string& processor_label) {
  auto it = std::find_if(mp_.begin(), mp_.end(), [&](const auto& p) { return p->label() == processor_label; });
  if (it == mp_.end()) {
    log(LoggingLevel::WARN,
        "Measurement processor to be removed (" + processor_label + ") does not exist.  No action taken.");
    return;
  }
  mp_.erase(it);
}

void StandardFusionEngine::propagate(Timestamp time) {
  if (!strategy_) throw std::logic_error("FusionStrategy has not been set");
  if (time.elapsed_nsec < time_.elapsed_nsec) {
    log(LoggingLevel::WARN, "Attempted to propagate backwards in time.  propagate_time = " + secs(time) +
                                ", filter_time = " + secs(time_) + "s.  No action taken.");
    return;
  }
  if (time.elapsed_nsec == time_.elapsed_nsec) return;
  if (num_states_ == 0) {
    log(LoggingLevel::WARN, "Attempted to propagate a filter with zero states. No action taken.");
    return;
  }
  Matrix big_Phi = Matrix::Zero(num_states_, num_states_);
  Matrix big_Qd = Matrix::Zero(num_states_, num_states_);
  struct Piece {
    Eigen::Index start, n;
    std::function<Vector(const Vector&)> g;
  };
  auto pieces = std::make_shared<std::vector<Piece>>();
  const auto gen = gen_x_and_p_func();
  for (auto& b : sb_) {
    auto dyn = b.block->generate_dynamics(gen, time_, time);
    if (!dyn) {
      log(LoggingLevel::ERROR, "Unable to generate dynamics model during propagate.");
      return;
    }
    if (dyn->Phi.rows() != b.num_states || dyn->Phi.cols() != b.num_states || dyn->Qd.rows() != b.num_states ||
        dyn->Qd.cols() != b.num_states)
      throw std::invalid_argument("Dynamics model for block " + b.label + " has the wrong shape.");
    big_Phi.block(b.start_index, b.start_index, b.num_states, b.num_states) = dyn->Phi;
    big_Qd.block(b.start_index, b.start_index, b.num_states, b.num_states) = dyn->Qd;
    pieces->push_back({b.start_index, b.num_states, std::move(dyn->g)});
  }
  api::StandardDynamicsModel big;
  big.g = [pieces](const Vector& x_in) {
    Vector x_out = Vector::Zero(x_in.size());
    for (const auto& p : *pieces) x_out.segment(p.start, p.n) = p.g(x_in.segment(p.start, p.n));
    return x_out;
  };
  big.Phi = std::move(big_Phi);
  big.Qd = std::move(big_Qd);
  strategy_->propagate(big);
  trace_state("P", "prop", time_, time, strategy_.get());
  time_ = time;
  if (save_after_prop_) save_x_and_p_to_registry();
}

std::optional<std::string> StandardFusionEngine::get_real_label(const std::string& label) {
  if (find_block(label)) return label;
  return vsb_manager_.get_start_block_label(label);
}

void StandardFusionEngine::update(const std::string& processor_label, const api::Message& message) {
  if (!strategy_) throw std::logic_error("FusionStrategy has not been set");
  auto* proc = find_processor(processor_label);
  if (!proc) {
    log(LoggingLevel::ERROR, "Attempted process measurement, but measurement processor (" + processor_label +
                                 ") does not exist. No action taken.");
    return;
  }
  auto tov = message.wrapped_message ? utils::time_of_validity(*message.wrapped_message) : std::nullopt;
  if (!tov) throw std::invalid_argument("update(): message has no time of validity");
  propagate(*tov);

  auto mm = proc->generate_model(message, gen_x_and_p_func());
  if (!mm) return;

  // Map each label the processor models onto the full state. A real block consumes its own width
  // of the processor's H; a virtual block consumes the virtual width and is pulled back onto its
  // real block through the chain-rule Jacobian. (The Python original slices by the real width,
  // which only works for same-size virtual blocks; see COBRA_ANALYSIS §12.)
  struct Slice {
    Eigen::Index real_start, n_real, n_mp;
    std::string real, label;
    bool is_virtual;
  };
  auto slices = std::make_shared<std::vector<Slice>>();
  std::set<std::string> vsb_labels;
  if (auto v = virtual_state_block_target_labels()) vsb_labels.insert(v->begin(), v->end());
  auto full_est = strategy_->estimate();
  if (!full_est) {
    log(LoggingLevel::ERROR, "Unable to get estimate from strategy.");
    return;
  }
  Matrix full_H = Matrix::Zero(mm->H.rows(), num_states_);
  Eigen::Index mp_num_states = 0;
  for (const auto& label : proc->state_block_labels()) {
    auto real = get_real_label(label);
    if (!real) {
      log(LoggingLevel::ERROR, "Unable to populate H with the jacobian from block \"" + label + "\"");
      return;
    }
    const auto* sb = find_block(*real);
    if (!sb) {
      log(LoggingLevel::ERROR, "Unable to find the state block \"" + *real + "\".");
      return;
    }
    const bool is_virtual = vsb_labels.count(label) != 0;
    Eigen::Index n_mp = sb->num_states;
    std::optional<Matrix> real_to_virt;
    if (is_virtual) {
      Vector real_est = full_est->segment(sb->start_index, sb->num_states);
      real_to_virt = vsb_manager_.jacobian(real_est, sb->label, label, time_);
      if (!real_to_virt) return;
      n_mp = real_to_virt->rows();
    }
    if (mp_num_states + n_mp > mm->H.cols())
      throw std::invalid_argument("Measurement model H from processor " + processor_label +
                                  " has fewer columns than its state blocks require.");
    Matrix sub_H = mm->H.middleCols(mp_num_states, n_mp);
    if (real_to_virt) sub_H = sub_H * *real_to_virt;
    full_H.middleCols(sb->start_index, sb->num_states) = sub_H;
    slices->push_back({sb->start_index, sb->num_states, n_mp, sb->label, label, is_virtual});
    mp_num_states += n_mp;
  }

  // h over the full state: gather (and virtually convert) each block's slice, then call the processor's h.
  auto h_mp = mm->h;
  const Timestamp t = time_;
  auto* self = this;
  auto full_h = [slices, h_mp, t, self, mp_num_states](const Vector& full_x) {
    Vector x_mp = Vector::Zero(mp_num_states);
    Eigen::Index at = 0;
    for (const auto& s : *slices) {
      Vector est = full_x.segment(s.real_start, s.n_real);
      if (s.is_virtual) {
        auto conv = self->vsb_manager_.convert_estimate(est, s.real, s.label, t);
        if (!conv) {
          self->log(LoggingLevel::ERROR, "Unable to obtain the set of states for block \"" + s.real + "\".");
          return x_mp;
        }
        est = *conv;
      }
      if (est.size() != s.n_mp) throw std::runtime_error("Virtual block " + s.label + " changed size between calls.");
      x_mp.segment(at, s.n_mp) = est;
      at += s.n_mp;
    }
    return h_mp(x_mp);
  };

  api::StandardMeasurementModel big{mm->z, full_h, full_H, mm->R};
  strategy_->update(big);
  trace_state("U", processor_label, *tov, time_, strategy_.get());
  if (save_after_update_) save_x_and_p_to_registry();
}

std::optional<EstimateWithCovariance> StandardFusionEngine::peek_ahead(Timestamp time,
                                                                       const std::vector<std::string>& block_labels) {
  if (time.elapsed_nsec < time_.elapsed_nsec) {
    log(LoggingLevel::WARN,
        "Peek ahead time (" + secs(time) + "s) is before filter time (" + secs(time_) + "s).");
    return std::nullopt;
  }
  if (time.elapsed_nsec == time_.elapsed_nsec) return generate_x_and_p(block_labels);
  if (block_labels.empty()) {
    log(LoggingLevel::WARN, "No block labels for peek_ahead().");
    return std::nullopt;
  }
  for (const auto& l : block_labels) {
    if (!find_block(l)) {
      log(LoggingLevel::WARN, "In peek_ahead(), the state block label \"" + l +
                                  "\" does not match any stateblock loaded into the fusion engine.");
      return std::nullopt;
    }
  }
  auto copy = clone();
  ++g_in_peek;
  copy->propagate(time);
  --g_in_peek;
  return copy->generate_x_and_p(block_labels);
}

std::optional<EstimateWithCovariance> StandardFusionEngine::generate_x_and_p(
    const std::vector<std::string>& block_labels) {
  if (!strategy_) throw std::logic_error("FusionStrategy has not been set");
  if (block_labels.empty()) {
    log(LoggingLevel::WARN, "No block labels for generate_x_and_p().");
    return std::nullopt;
  }
  std::set<std::string> vsb_labels;
  if (auto v = virtual_state_block_target_labels()) vsb_labels.insert(v->begin(), v->end());
  for (const auto& l : block_labels) {
    if (!find_block(l) && !vsb_labels.count(l)) {
      log(LoggingLevel::WARN, "In generate_x_and_p(), the state block label \"" + l +
                                  "\" does not match any stateblock loaded into the fusion engine.");
      return std::nullopt;
    }
  }
  std::vector<Vector> parts;
  Eigen::Index total = 0;
  for (const auto& l : block_labels) {
    auto sub = get_state_block_estimate(l);
    if (!sub) return std::nullopt;
    total += sub->size();
    parts.push_back(std::move(*sub));
  }
  Vector est(total);
  Eigen::Index at = 0;
  for (const auto& p : parts) {
    est.segment(at, p.size()) = p;
    at += p.size();
  }
  auto cov = build_joint_covariance(block_labels, total);
  if (!cov) return std::nullopt;
  return EstimateWithCovariance{EstimateWithCovarianceType::EWC_GENERIC, std::move(est), std::move(*cov)};
}

std::optional<Matrix> StandardFusionEngine::build_joint_covariance(const std::vector<std::string>& block_labels,
                                                                   Eigen::Index size) {
  Matrix out = Matrix::Zero(size, size);
  const std::size_t n = block_labels.size();
  Eigen::Index row0 = 0, col0 = 0, sb0_n = 0;
  for (std::size_t i = 0; i < n; ++i) {
    row0 += sb0_n;
    col0 = row0;
    for (std::size_t j = i; j < n; ++j) {
      std::optional<Matrix> block = (i == j) ? get_state_block_covariance(block_labels[i])
                                             : get_state_block_cross_covariance(block_labels[i], block_labels[j]);
      if (!block) return std::nullopt;
      sb0_n = block->rows();
      const Eigen::Index sb1_n = block->cols();
      out.block(row0, col0, sb0_n, sb1_n) = *block;
      if (i != j) out.block(col0, row0, sb1_n, sb0_n) = block->transpose();
      col0 += sb1_n;
    }
  }
  return out;
}

void StandardFusionEngine::give_state_block_aux_data(const std::string& block_label, const api::AuxData& aux) {
  auto* sb = find_block(block_label);
  if (!sb) {
    log(LoggingLevel::WARN,
        "State block (" + block_label + ") identified in give_state_block_aux_data() does not exist .");
    return;
  }
  sb->block->receive_aux_data(aux);
}

void StandardFusionEngine::give_measurement_processor_aux_data(const std::string& processor_label,
                                                               const api::AuxData& aux) {
  auto* p = find_processor(processor_label);
  if (!p) {
    log(LoggingLevel::WARN, "State block (" + processor_label +
                                ") identified in give_measurement_processor_aux_data() does not exist .");
    return;
  }
  p->receive_aux_data(aux);
}

std::unique_ptr<api::StandardFusionEngine> StandardFusionEngine::clone() const {
  auto c = std::make_unique<StandardFusionEngine>(mediator_, save_after_prop_, save_after_update_);
  c->time_ = time_;
  c->num_states_ = num_states_;
  c->saved_state_labels_ = saved_state_labels_;
  c->strategy_ = strategy_ ? strategy_->clone() : nullptr;
  for (const auto& b : sb_)
    c->sb_.push_back({b.label, b.num_states, b.start_index, b.stop_index, b.block->clone()});
  for (const auto& p : mp_) c->mp_.push_back(p->clone());
  c->vsb_manager_ = vsb_manager_;
  return c;
}

void StandardFusionEngine::set_trace(std::ostream* os) {
  g_trace = os;
  g_trace_checked = true;
}

// ----------------------------------------------------------------------------- plugin

std::unique_ptr<api::StandardFusionEngine> StandardFusionPlugin::new_fusion_engine(api::FusionType type) {
  if (!is_fusion_type_supported(type)) {
    if (mediator_)
      mediator_->log_message(LoggingLevel::ERROR,
                             "Fusion engine type not currently supported. Make sure to call "
                             "FusionPlugin.is_fusion_type_supported before requesting a new fusion engine.");
    return nullptr;
  }
  FusionEngineConfig cfg;
  if (mediator_ && mediator_->registry().has_group(FusionEngineConfig::kGroup)) {
    if (auto c = FusionEngineConfig::from_registry(*mediator_)) cfg = *c;
  }
  return std::make_unique<StandardFusionEngine>(mediator_, cfg.save_x_and_p_after_prop, cfg.save_x_and_p_after_update);
}

}  // namespace pntos::cobra
