// Port of pntos-cobra/tests/test_single_threaded_controller.py plus mediator buffering tests
// (the Python mediator has no direct unit tests; its behaviour is pinned here).
#include <pntos/cobra/EkfFusionStrategyPlugin.hpp>
#include <pntos/cobra/StandardLoggingPlugin.hpp>
#include <pntos/cobra/StandardRegistryPlugin.hpp>
#include <pntos/cobra/config/configs.hpp>
#include <pntos/cobra/controller/StandardControllerPlugin.hpp>
#include <pntos/cobra/dummy/DummyPlugins.hpp>
#include <pntos/cobra/fusion/StandardFusionPlugin.hpp>
#include <pntos/cobra/state_modeling/StandardStateModelingPlugin.hpp>
#include <pntos/cobra/utils/aspn.hpp>

#include "test_support.hpp"

#include <thread>

using namespace pntos;
using namespace pntos::test;
using api::LoggingLevel;
using api::Message;
using api::PluginType;
using api::Timestamp;

namespace {
constexpr std::int64_t kSec = 1'000'000'000;

/// Records every (source, tov, sequenced) delivered to it; solution = last message.
class RecordingOrchestration final : public api::OrchestrationPlugin {
 public:
  struct Rec {
    std::string source;
    std::int64_t tov;
    bool sequenced;
  };
  explicit RecordingOrchestration(std::string id, bool imu_immediate = true)
      : id_(std::move(id)), imu_immediate_(imu_immediate) {}
  void init_plugin(const std::optional<std::string>&, api::Mediator* m) override { mediator_ = m; }
  void shutdown_plugin() override { shutdowns++; }
  const std::string& identifier() const override { return id_; }
  void init_orchestration_plugin(const std::optional<api::PluginList>& plugins,
                                 api::MessageStreamConfig& sc) override {
    plugins_ = plugins.value_or(api::PluginList{});
    sc.sequenced_stream_all(true);
    if (imu_immediate_) sc.immediate_stream_add(ASPN_MEASUREMENT_IMU);
  }
  void process_pntos_message(const Message& m, bool sequenced) override {
    recs.push_back({m.source_identifier, cobra::utils::time_of_validity(*m.wrapped_message)->elapsed_nsec, sequenced});
    last_ = m;
  }
  std::vector<std::string> filter_description_list() const override { return {"BEST_ESTIMATE"}; }
  std::optional<std::vector<std::optional<Message>>> request_solutions(const std::vector<Timestamp>& t,
                                                                       const std::optional<std::string>& d) override {
    if (d && *d != "BEST_ESTIMATE") return std::nullopt;
    requests += t.size();
    return std::vector<std::optional<Message>>(t.size(), last_);
  }
  std::vector<Rec> recs;
  std::size_t requests = 0;
  int shutdowns = 0;
  api::PluginList plugins_;

 private:
  std::string id_;
  bool imu_immediate_;
  api::Mediator* mediator_ = nullptr;
  std::optional<Message> last_;
};

class RecordingTransport final : public api::TransportPlugin {
 public:
  explicit RecordingTransport(std::string id) : id_(std::move(id)) {}
  void init_plugin(const std::optional<std::string>&, api::Mediator* m) override { mediator = m; }
  void shutdown_plugin() override {}
  const std::string& identifier() const override { return id_; }
  void start_listening() override { listening = true; }
  void stop_listening() override { listening = false; }
  void broadcast_message(const Message& m, const std::optional<std::string>& ch) override {
    broadcasts.emplace_back(m, ch.value_or(""));
  }
  api::Mediator* mediator = nullptr;
  bool listening = false;
  std::vector<std::pair<Message, std::string>> broadcasts;

 private:
  std::string id_;
};

class RecordingLogger final : public api::LoggingPlugin {
 public:
  explicit RecordingLogger(std::string id) : id_(std::move(id)) {}
  void init_plugin(const std::optional<std::string>&, api::Mediator*) override { inited = true; }
  void shutdown_plugin() override { shutdown = true; }
  const std::string& identifier() const override { return id_; }
  void log(PluginType t, const std::string& src, LoggingLevel l, const std::string& m) override {
    entries.push_back({t, src, l, m});
  }
  struct E {
    PluginType type;
    std::string source;
    LoggingLevel level;
    std::string message;
  };
  std::vector<E> entries;
  bool inited = false, shutdown = false;

 private:
  std::string id_;
};

class DummyInertialPlugin final : public api::InertialPlugin {
 public:
  explicit DummyInertialPlugin(std::string id) : id_(std::move(id)) {}
  void init_plugin(const std::optional<std::string>&, api::Mediator* m) override { mediator = m; }
  void shutdown_plugin() override {}
  const std::string& identifier() const override { return id_; }
  bool is_inertial_type_supported(api::InertialType) const override { return true; }
  std::unique_ptr<api::CommonInertial> new_inertial(api::InertialType, const Message&,
                                                    const std::optional<std::string>&) override {
    return nullptr;
  }
  api::Mediator* mediator = nullptr;

 private:
  std::string id_;
};

class DummyInitializationPlugin final : public api::InitializationPlugin {
 public:
  explicit DummyInitializationPlugin(std::string id) : id_(std::move(id)) {}
  void init_plugin(const std::optional<std::string>&, api::Mediator* m) override { mediator = m; }
  void shutdown_plugin() override {}
  const std::string& identifier() const override { return id_; }
  bool is_initialization_type_supported(api::InitializationType) const override { return true; }
  std::unique_ptr<api::CommonInitializationStrategy> new_initialization_strategy(
      api::InitializationType, const std::optional<std::string>&) override {
    return nullptr;
  }
  api::Mediator* mediator = nullptr;

 private:
  std::string id_;
};

/// A UI plugin that owns the main thread and returns immediately (Python's raises ExitThread).
class DummyUiPlugin final : public api::UiPlugin {
 public:
  explicit DummyUiPlugin(std::string id) : id_(std::move(id)) {}
  void init_plugin(const std::optional<std::string>&, api::Mediator* m) override { mediator = m; }
  void shutdown_plugin() override {}
  const std::string& identifier() const override { return id_; }
  bool requires_main_thread() const override { return true; }
  void run_main_thread() override { ran = true; }
  api::Mediator* mediator = nullptr;
  bool ran = false;

 private:
  std::string id_;
};

Message pos(std::int64_t tov, const std::string& src) {
  return Message(make_position(tov, 0.5, -1.5, 100, api::Matrix::Identity(3, 3)), src);
}
Message imu(std::int64_t tov, const std::string& src) {
  return Message(make_imu(tov, api::Vector3(0, 0, -9.8), api::Vector3::Zero()), src);
}

// ----------------------------------------------------------------------------- mediator

class MediatorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    ctx = std::make_shared<cobra::MediatorContext>();
    orch = std::make_shared<RecordingOrchestration>("orch");
    transport = std::make_shared<RecordingTransport>("lcm");
    logger = std::make_shared<RecordingLogger>("log");
    ctx->orchestration_plugin = orch;
    ctx->transport_plugins = {transport};
    ctx->logging_plugin = logger;
    ctx->buffer_time_nsec = 2 * kSec;
    ctx->publish_interval_nsec = kSec;
    orch->init_orchestration_plugin(api::PluginList{}, *ctx->stream_config);
    med = std::make_unique<cobra::StandardMediator>(ctx, "transport_plugin", PluginType::TRANSPORT);
  }
  std::shared_ptr<cobra::MediatorContext> ctx;
  std::shared_ptr<RecordingOrchestration> orch;
  std::shared_ptr<RecordingTransport> transport;
  std::shared_ptr<RecordingLogger> logger;
  std::unique_ptr<cobra::StandardMediator> med;
};

TEST_F(MediatorTest, ImmediateAndSequencedRouting) {
  // IMU is immediate: delivered at once with sequenced=false.
  med->process_pntos_message(imu(1 * kSec, "imu"));
  ASSERT_EQ(orch->recs.size(), 1u);
  EXPECT_FALSE(orch->recs[0].sequenced);
  // Positions are sequenced: buffered until 2 s younger than the newest message.
  med->process_pntos_message(pos(3 * kSec, "gps"));
  med->process_pntos_message(pos(2 * kSec, "gps_late"));  // out of order
  EXPECT_EQ(orch->recs.size(), 1u);
  med->process_pntos_message(imu(4 * kSec, "imu"));  // newest = 4 s => release < 2 s: nothing yet
  EXPECT_EQ(orch->recs.size(), 2u);
  med->process_pntos_message(imu(5.5 * kSec, "imu"));  // release < 3.5 s: 2 s and 3 s, in time order
  ASSERT_EQ(orch->recs.size(), 5u);
  EXPECT_EQ(orch->recs[3].source, "gps_late");
  EXPECT_TRUE(orch->recs[3].sequenced);
  EXPECT_EQ(orch->recs[4].source, "gps");
  EXPECT_TRUE(orch->recs[4].sequenced);
  EXPECT_EQ(ctx->messages.size(), 0u);
}

TEST_F(MediatorTest, PublishesSolutionsAtTheInterval) {
  med->process_pntos_message(imu(1 * kSec, "imu"));  // first message only records the time
  EXPECT_EQ(orch->requests, 0u);
  med->process_pntos_message(imu(1.5 * kSec, "imu"));  // 0.5 s: no request
  EXPECT_EQ(orch->requests, 0u);
  med->process_pntos_message(imu(2.5 * kSec, "imu"));  // > 1 s: request + broadcast
  EXPECT_EQ(orch->requests, 1u);
  ASSERT_EQ(transport->broadcasts.size(), 1u);
  EXPECT_EQ(transport->broadcasts[0].second, "imu");  // destination = solution's source identifier
  med->process_pntos_message(imu(3.0 * kSec, "imu"));
  EXPECT_EQ(orch->requests, 1u);
  med->process_pntos_message(imu(3.6 * kSec, "imu"));
  EXPECT_EQ(orch->requests, 2u);
  EXPECT_EQ(med->filter_description_list(), std::vector<std::string>{"BEST_ESTIMATE"});
}

TEST_F(MediatorTest, BroadcastAndLogging) {
  auto second = std::make_shared<RecordingTransport>("ros");
  ctx->transport_plugins.push_back(second);
  med->broadcast_aspn_message(imu(0, "x"));  // all transports
  EXPECT_EQ(transport->broadcasts.size(), 1u);
  EXPECT_EQ(second->broadcasts.size(), 1u);
  med->broadcast_aspn_message(imu(0, "x"), "ros", "chan");
  EXPECT_EQ(transport->broadcasts.size(), 1u);
  EXPECT_EQ(second->broadcasts.size(), 2u);
  EXPECT_EQ(second->broadcasts[1].second, "chan");
  med->broadcast_aspn_message(imu(0, "x"), "nope");
  ASSERT_FALSE(logger->entries.empty());
  EXPECT_EQ(logger->entries.back().level, LoggingLevel::WARN);
  EXPECT_EQ(logger->entries.back().type, PluginType::CONTROLLER);

  med->log_message(LoggingLevel::INFO, "hello");
  EXPECT_EQ(logger->entries.back().type, PluginType::TRANSPORT);
  EXPECT_EQ(logger->entries.back().source, "transport_plugin");
  // ERROR only trips the exit event when a controller is attached.
  med->log_message(LoggingLevel::ERROR, "bad");
  EXPECT_FALSE(ctx->exit_event.is_set());
  cobra::StandardControllerPlugin ctrl("ctrl");
  ctx->controller_plugin = &ctrl;
  med->log_message(LoggingLevel::ERROR, "bad");
  EXPECT_TRUE(ctx->exit_event.is_set());
  EXPECT_EQ(ctx->exit_event.exit_code(), cobra::ExitCode::ERROR);
}

TEST_F(MediatorTest, UiGateBlocksDisabledSources) {
  cobra::StandardRegistryPlugin reg("registry", {});
  TestMediator tm;
  reg.init_plugin(std::nullopt, &tm);
  ctx->registry = reg.new_registry();
  ctx->ui_interface = std::make_unique<cobra::UiMediatorInterface>(ctx->registry);
  med->process_pntos_message(imu(1 * kSec, "imu"));
  EXPECT_EQ(orch->recs.size(), 1u);
  EXPECT_TRUE(ctx->registry->has_group("ui/channel/imu"));
  ctx->registry->batch("ui/channel/imu")->set("enabled_mediator", false);
  med->process_pntos_message(imu(2 * kSec, "imu"));
  EXPECT_EQ(orch->recs.size(), 1u);  // dropped
  ctx->registry->batch("ui/channel/imu")->set("enabled_mediator", true);
  med->process_pntos_message(imu(3 * kSec, "imu"));
  EXPECT_EQ(orch->recs.size(), 2u);
  ctx->ui_interface.reset();
}

TEST(ExitEvent, SetWaitClear) {
  cobra::ExitEvent ev;
  EXPECT_FALSE(ev.wait(std::chrono::milliseconds(10)));
  std::thread t([&] { ev.set(cobra::ExitCode::ERROR); });
  EXPECT_TRUE(ev.wait());
  t.join();
  EXPECT_EQ(ev.exit_code(), cobra::ExitCode::ERROR);
  ev.clear();
  EXPECT_FALSE(ev.is_set());
  EXPECT_EQ(ev.exit_code(), cobra::ExitCode::SUCCESS);
}

// ----------------------------------------------------------------------------- controller

struct Plugins {
  std::shared_ptr<RecordingOrchestration> orch = std::make_shared<RecordingOrchestration>("orchestration");
  std::shared_ptr<DummyInitializationPlugin> init = std::make_shared<DummyInitializationPlugin>("initialization");
  std::shared_ptr<DummyInertialPlugin> inertial = std::make_shared<DummyInertialPlugin>("inertial");
  std::shared_ptr<cobra::StandardFusionPlugin> fusion = std::make_shared<cobra::StandardFusionPlugin>("fusion");
  std::shared_ptr<cobra::EkfFusionStrategyPlugin> strategy = std::make_shared<cobra::EkfFusionStrategyPlugin>("ekf");
  std::shared_ptr<cobra::StandardStateModelingPlugin> sm = std::make_shared<cobra::StandardStateModelingPlugin>("sm");
  std::shared_ptr<cobra::StandardRegistryPlugin> registry;
  std::shared_ptr<RecordingLogger> logger = std::make_shared<RecordingLogger>("logger");
  std::shared_ptr<DummyUiPlugin> ui = std::make_shared<DummyUiPlugin>("ui");
  std::shared_ptr<RecordingTransport> transport = std::make_shared<RecordingTransport>("transport");

  explicit Plugins(std::vector<std::shared_ptr<const cobra::BaseConfig>> configs = {}) {
    if (configs.empty()) configs.push_back(std::make_shared<cobra::ControllerConfig>());
    registry = std::make_shared<cobra::StandardRegistryPlugin>("registry", configs);
  }
  api::PluginList list(bool with_ui = true) const {
    api::PluginList l{orch, init, inertial, fusion, strategy, sm, registry, logger};
    if (with_ui) l.push_back(ui);
    l.push_back(transport);
    return l;
  }
};

TEST(Controller, InitPluginWithAndWithoutMediator) {
  cobra::StandardControllerPlugin c("controller");
  c.init_plugin(std::nullopt, nullptr);
  cobra::DummyMediator dm;
  c.init_plugin(std::nullopt, &dm);  // logs an error to the console, must not throw
}

TEST(Controller, TakeControlWiresEverythingAndShutsDown) {
  Plugins p;
  cobra::StandardControllerPlugin c("controller");
  c.init_plugin(std::nullopt, nullptr);
  c.take_control(p.list());  // UI owns the main thread and returns immediately
  EXPECT_TRUE(p.ui->ran);
  EXPECT_TRUE(p.logger->inited);
  EXPECT_TRUE(p.logger->shutdown);
  EXPECT_EQ(p.orch->shutdowns, 1);
  EXPECT_TRUE(p.transport->listening);
  EXPECT_NE(p.transport->mediator, nullptr);
  EXPECT_NE(p.ui->mediator, nullptr);
  EXPECT_NE(p.inertial->mediator, nullptr);
  // plugins handed to orchestration: fusion, strategy, inertial, initialization, state modeling (no preprocessors)
  EXPECT_EQ(p.orch->plugins_.size(), 5u);
  EXPECT_EQ(p.orch->plugins_[0]->plugin_type(), PluginType::FUSION);
  EXPECT_EQ(p.orch->plugins_[1]->plugin_type(), PluginType::FUSION_STRATEGY);
  EXPECT_EQ(p.orch->plugins_[4]->plugin_type(), PluginType::STATE_MODELING);
  EXPECT_EQ(c.exit_code(), cobra::ExitCode::SUCCESS);
  // config applied
  EXPECT_EQ(c.context()->buffer_time_nsec, 2 * kSec);
  EXPECT_EQ(c.context()->publish_interval_nsec, kSec);
  // transport mediator routes through the stream config set by the orchestration
  bool saw_ctrl_log = false;
  for (const auto& e : p.logger->entries)
    if (e.type == PluginType::CONTROLLER && e.message.find("Shutting down") != std::string::npos) saw_ctrl_log = true;
  EXPECT_TRUE(saw_ctrl_log);
}

TEST(Controller, MissingPluginsThrow) {
  Plugins p;
  cobra::StandardControllerPlugin c("controller");
  api::PluginList l{p.orch, p.registry, p.logger};  // no transport, no fusion
  EXPECT_THROW(c.take_control(l), std::runtime_error);
  EXPECT_FALSE(p.logger->inited);
}

TEST(Controller, ReadyToShutdownFlagEndsTheMainLoop) {
  Plugins p;
  cobra::StandardControllerPlugin c("controller");
  auto l = p.list(/*with_ui=*/false);
  std::thread setter([&] {
    // Wait until the controller is running, then raise the flag from "another plugin".
    for (int i = 0; i < 200 && !p.transport->listening; ++i) std::this_thread::sleep_for(std::chrono::milliseconds(5));
    ASSERT_TRUE(p.transport->listening);
    p.transport->mediator->registry().batch("controller/flags")->set("ready_to_shutdown", true);
  });
  c.take_control(l);
  setter.join();
  EXPECT_EQ(c.exit_code(), cobra::ExitCode::SUCCESS);
  EXPECT_EQ(p.orch->shutdowns, 1);
}

TEST(Controller, ErrorLogSetsErrorExitCode) {
  Plugins p;
  cobra::StandardControllerPlugin c("controller");
  auto l = p.list(false);
  std::thread setter([&] {
    for (int i = 0; i < 200 && !p.transport->listening; ++i) std::this_thread::sleep_for(std::chrono::milliseconds(5));
    p.transport->mediator->log_message(LoggingLevel::ERROR, "transport exploded");
  });
  c.take_control(l);
  setter.join();
  EXPECT_EQ(c.exit_code(), cobra::ExitCode::ERROR);
}

TEST(Controller, MediatorFilterDescriptionList) {
  auto ctx = std::make_shared<cobra::MediatorContext>();
  ctx->orchestration_plugin = std::make_shared<cobra::DummyOrchestrationPlugin>("Dummy orchestration");
  cobra::StandardMediator m(ctx, "controller", PluginType::CONTROLLER);
  EXPECT_EQ(m.filter_description_list(), std::vector<std::string>{"LAST_MESSAGE"});
}

TEST(DummyPlugins, ControllerRunsTransportAndOrchestration) {
  auto orch = std::make_shared<cobra::DummyOrchestrationPlugin>("orch");
  auto transport = std::make_shared<cobra::DummyTransportPlugin>("transport", std::chrono::milliseconds(20));
  auto logger = std::make_shared<RecordingLogger>("logger");
  cobra::DummyControllerPlugin c("controller", std::chrono::milliseconds(120));
  c.take_control({orch, transport, logger});
  EXPECT_GE(transport->messages_sent(), 3u);
  bool processed = false, echoed = false;
  for (const auto& e : logger->entries) {
    if (e.message.find("Orchestration processing message from channel_foo") != std::string::npos) processed = true;
    if (e.message.find("broadcasting on channel_foo_echo") != std::string::npos) echoed = true;
  }
  EXPECT_TRUE(processed);
  EXPECT_TRUE(echoed);
}

TEST(Configs, ControllerAndStreamRoundTrip) {
  cobra::StandardRegistryPlugin reg("registry", {});
  TestMediator tm;
  reg.init_plugin(std::nullopt, &tm);
  tm.set_registry(reg.new_registry());
  cobra::ControllerConfig cc;
  cc.publish_interval = std::nullopt;
  cc.auto_shutdown = false;
  cc.to_registry(tm);
  auto back = cobra::ControllerConfig::from_registry(tm);
  ASSERT_TRUE(back);
  EXPECT_FALSE(back->publish_interval.has_value());
  EXPECT_FALSE(back->auto_shutdown);
  EXPECT_DOUBLE_EQ(back->buffer_length_sec, 2.0);

  auto sc = cobra::default_stream_config();
  sc.to_registry(tm);
  auto sback = cobra::StreamConfig::from_registry(tm, "config/stream_config");
  ASSERT_TRUE(sback);
  EXPECT_EQ(sback->default_buffer_mode, cobra::BufferMode::SEQUENCED);
  ASSERT_TRUE(sback->override_streams);
  ASSERT_EQ(sback->override_streams->size(), 1u);
  EXPECT_EQ((*sback->override_streams)[0].message_type, ASPN_MEASUREMENT_IMU);
  EXPECT_FALSE((*sback->override_streams)[0].source_identifier.has_value());
}

}  // namespace
