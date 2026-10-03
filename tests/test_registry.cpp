// Port of pntos-cobra/tests/test_registry.py (the parts that do not depend on Python config
// dataclasses; config round-trips are tested in test_config.cpp).
#include <pntos/cobra/StandardRegistryPlugin.hpp>

#include "test_support.hpp"

#include <filesystem>

using namespace pntos;
using namespace pntos::test;
using api::Matrix;
using api::Message;
using api::RegistryValue;
using api::StringArray;

namespace {

const std::string kGroup = "generic_test_group";

class RegistryTest : public ::testing::Test {
 protected:
  void SetUp() override {
    tmp = std::filesystem::temp_directory_path() / ("pntos_registry_test_" + std::to_string(::getpid()));
    std::filesystem::remove_all(tmp);
    plugin = std::make_unique<cobra::StandardRegistryPlugin>("Standard registry 1");
    plugin->init_plugin(tmp.string(), &med);
    reg = plugin->new_registry(std::nullopt);
    med.set_registry(reg);

    test_message = Message(make_pva(0, 0, 0, 0, 0, 0, 0, vec({1, 0, 0, 0})), "Genesis planet");
    keys = {"str", "list[str]", "int", "bool", "float", "ndarray", "Message"};
    vals = {RegistryValue(std::string("Wassup")),
            RegistryValue(StringArray{"What", "is", "up"}),
            RegistryValue(std::int64_t{42}),
            RegistryValue(true),
            RegistryValue(3.14159),
            RegistryValue(mat({{0.0, 0.1}, {0.2, 0.3}})),
            RegistryValue(test_message)};
  }
  void TearDown() override { std::filesystem::remove_all(tmp); }

  std::shared_ptr<api::KeyValueStore> set_up_store_with_all_types(const std::string& group = kGroup) {
    auto kv = reg->batch_start(group);
    kv->clear();
    for (std::size_t i = 0; i < keys.size(); ++i) kv->set(keys[i], vals[i]);
    return kv;
  }

  static bool same(const RegistryValue& a, const RegistryValue& b) {
    if (a.index() != b.index()) return false;
    if (a.index() == 5) return std::get<Matrix>(a) == std::get<Matrix>(b);
    if (a.index() == 6)
      return std::get<Message>(a).source_identifier == std::get<Message>(b).source_identifier &&
             std::get<Message>(a).wrapped_message == std::get<Message>(b).wrapped_message;
    return a == b;
  }

  std::filesystem::path tmp;
  TestMediator med;
  std::unique_ptr<cobra::StandardRegistryPlugin> plugin;
  std::shared_ptr<api::Registry> reg;
  Message test_message;
  std::vector<std::string> keys;
  std::vector<RegistryValue> vals;
};

TEST_F(RegistryTest, Clear) {
  auto kv = reg->batch_start(kGroup);
  for (std::size_t i = 0; i < keys.size(); ++i) kv->set(keys[i], vals[i]);
  kv->clear();
  for (const auto& k : keys) EXPECT_FALSE(kv->has_key(k)) << k;
  kv->batch_end();
}

TEST_F(RegistryTest, AddToRegistry) {
  auto kv = reg->batch_start(kGroup);
  for (int i = 0; i < 10; ++i) kv->set("Key " + std::to_string(i), std::int64_t{i});
  kv->batch_end();
  kv = reg->batch_start(kGroup);
  auto key_array = kv->keys();
  ASSERT_TRUE(key_array);
  for (std::size_t i = 0; i < key_array->size(); ++i) {
    EXPECT_EQ((*key_array)[i], "Key " + std::to_string(i));
    EXPECT_EQ(*kv->get_value<std::int64_t>((*key_array)[i]), static_cast<std::int64_t>(i));
  }
  kv->batch_end();
}

TEST_F(RegistryTest, RequestNotifyNewGroup) {
  std::string callbacked;
  ASSERT_TRUE(reg->request_notify_new_group([&](const std::string& g) { callbacked = g; }));
  auto kv = reg->batch_start(kGroup);
  EXPECT_EQ(callbacked, kGroup);
  kv->batch_end();
}

TEST_F(RegistryTest, AnyValueToStr) {
  auto kv = reg->batch_start(kGroup);
  kv->set("str", std::string("Hello There!"));
  kv->set("list[str]", StringArray{"Hello", "There"});
  kv->set("int", std::int64_t{42});
  kv->set("bool", true);
  kv->set("float", 3.1415927);
  kv->set("np_array", Matrix(vec({1.0, 1.1, 1.2})));
  kv->batch_end();

  kv = reg->batch_start(kGroup);
  EXPECT_EQ(*kv->get_value<std::string>("str"), "Hello There!");
  EXPECT_FALSE(kv->get_value<std::string>("list[str]").has_value());  // multi-element list -> no str
  EXPECT_EQ(*kv->get_value<std::string>("int"), "42");
  EXPECT_EQ(*kv->get_value<std::string>("bool"), "True");
  EXPECT_TRUE(kv->get_value<std::string>("float").has_value());
  EXPECT_TRUE(kv->get_value<std::string>("np_array").has_value());
  kv->batch_end();
}

TEST_F(RegistryTest, StrToInt) {
  auto kv = reg->batch_start(kGroup);
  kv->set("str1", std::string("42"));
  kv->set("str2", std::string("Not a number"));
  kv->batch_end();
  kv->batch_restart();
  EXPECT_EQ(*kv->get_value<std::int64_t>("str1"), 42);
  EXPECT_FALSE(kv->get_value<std::int64_t>("str2").has_value());
  kv->batch_end();
}

TEST_F(RegistryTest, MatrixRoundTrip) {
  Matrix a1(20, 1);
  for (int i = 0; i < 20; ++i) a1(i, 0) = i;
  Matrix a2 = mat({{0, 1}, {2, 3}});
  auto kv = reg->batch_start(kGroup);
  kv->set("array1", a1);
  kv->set("array2", a2);
  kv->batch_end();
  kv->batch_restart();
  EXPECT_FALSE(kv->get_value<double>("array1").has_value());
  EXPECT_EQ(*kv->get_value<Matrix>("array1"), a1);
  EXPECT_EQ(*kv->get_value<Matrix>("array2"), a2);
  EXPECT_EQ(kv->get_value<api::Vector>("array1")->size(), 20);
  kv->batch_end();
}

TEST_F(RegistryTest, StrToList) {
  auto kv = reg->batch_start(kGroup);
  kv->set("str1", std::string("Hello there"));
  kv->batch_end();
  kv->batch_restart();
  EXPECT_EQ(*kv->get_value<StringArray>("str1"), StringArray{"Hello there"});
  kv->batch_end();
}

TEST_F(RegistryTest, NumberToList) {
  auto kv = reg->batch_start(kGroup);
  kv->set("t1", std::int64_t{42});
  kv->batch_end();
  kv->batch_restart();
  EXPECT_EQ(*kv->get_value<StringArray>("t1"), StringArray{"42"});
  kv->batch_end();
}

TEST_F(RegistryTest, Messages) {
  auto kv = reg->batch_start(kGroup);
  kv->set("message", test_message);
  kv->batch_end();
  kv->batch_restart();
  EXPECT_FALSE(kv->get_value<std::string>("message").has_value());
  EXPECT_FALSE(kv->get_value<std::int64_t>("message").has_value());
  auto m = kv->get_value<Message>("message");
  ASSERT_TRUE(m);
  EXPECT_EQ(m->source_identifier, "Genesis planet");
  EXPECT_EQ(m->wrapped_message, test_message.wrapped_message);
  kv->batch_end();
}

TEST_F(RegistryTest, RawString) {
  const std::string s = "test string";
  auto kv = reg->batch_start(kGroup);
  kv->set_raw(std::string("key"), std::vector<unsigned char>(s.begin(), s.end()));
  kv->batch_end();
  kv = reg->batch_start(kGroup);
  auto out = kv->get_raw(std::string("key"));
  ASSERT_TRUE(out);
  EXPECT_EQ(std::string(out->begin(), out->end()), s);
  kv->batch_end();
}

TEST_F(RegistryTest, AssignTwice) {
  auto kv = set_up_store_with_all_types();
  for (std::size_t i = 0; i < keys.size(); ++i) kv->set(keys[i], vals[i]);
  EXPECT_EQ(kv->size(), keys.size());
  kv->batch_end();
}

TEST_F(RegistryTest, BatchRules) {
  auto kv = reg->batch_start(kGroup);
  kv->batch_end();
  EXPECT_FALSE(med.has_error());
  kv->set("Enterprise", std::int64_t{1701});
  EXPECT_TRUE(med.has_error());
  EXPECT_NE(med.last_message().find("outside of batch operation"), std::string::npos);
}

TEST_F(RegistryTest, BatchRaiiHelper) {
  {
    auto b = reg->batch("test_context");
    b->set("write", std::int64_t{42});
    EXPECT_EQ(*b->get_value<std::int64_t>("write"), 42);
  }
  // batch closed: restarting must not log an error
  auto kv = reg->batch_start("test_context");
  kv->batch_end();
  EXPECT_FALSE(med.has_error());
}

TEST_F(RegistryTest, RequestNotifyNoKey) {
  std::vector<std::string> test_keys{"Resistance", "is", "futile."};
  auto kv = reg->batch_start(kGroup);
  for (const auto& k : test_keys) kv->set(k, std::int64_t{0});
  kv->batch_end();
  int calls = 0;
  kv->batch_restart();
  ASSERT_TRUE(kv->request_notify(std::nullopt, [&](const std::string& g, const std::vector<std::string>& ks,
                                                   api::KeyValueStore& store) {
    ++calls;
    EXPECT_EQ(g, kGroup);
    EXPECT_EQ(std::set<std::string>(ks.begin(), ks.end()), std::set<std::string>(test_keys.begin(), test_keys.end()));
    EXPECT_EQ(&store, kv.get());
  }));
  for (const auto& k : test_keys) kv->set(k, std::int64_t{1});
  kv->batch_end();
  EXPECT_EQ(calls, 1);
}

TEST_F(RegistryTest, RequestNotifyNoModifiedKeys) {
  auto kv = reg->batch_start(kGroup);
  kv->request_notify(std::nullopt, [](const std::string&, const std::vector<std::string>&, api::KeyValueStore&) {
    FAIL() << "Callback called but no keys were modified.";
  });
  kv->batch_end();
  kv->batch_restart();
  kv->batch_end();
}

TEST_F(RegistryTest, RequestNotifyMultipleKeysAndRemove) {
  std::vector<std::string> test_keys{"Resistance", "is", "futile."};
  auto kv = reg->batch_start(kGroup);
  for (const auto& k : test_keys) kv->set(k, std::int64_t{0});
  kv->batch_end();

  std::string cb_group;
  std::set<std::string> cb_keys;
  int calls = 0;
  kv->batch_restart();
  std::vector<api::NotifyToken> tokens;
  for (const auto& k : test_keys) {
    auto t = kv->request_notify(k, [&](const std::string& g, const std::vector<std::string>& ks, api::KeyValueStore&) {
      ++calls;
      cb_group = g;
      cb_keys.insert(ks.begin(), ks.end());
    });
    ASSERT_TRUE(t);
    tokens.push_back(*t);
  }
  for (const auto& k : test_keys) kv->set(k, std::int64_t{1});
  kv->batch_end();
  EXPECT_EQ(cb_group, kGroup);
  EXPECT_EQ(cb_keys, std::set<std::string>(test_keys.begin(), test_keys.end()));
  EXPECT_EQ(calls, 3);

  kv->batch_restart();
  for (auto t : tokens) EXPECT_TRUE(kv->remove_notify(t));
  calls = 0;
  for (const auto& k : test_keys) kv->set(k, std::int64_t{2});
  kv->batch_end();
  EXPECT_EQ(calls, 0);
}

TEST_F(RegistryTest, GetType) {
  auto kv = set_up_store_with_all_types();
  const api::RegistryValueType expected[] = {
      api::RegistryValueType::STR,    api::RegistryValueType::STR_ARRAY, api::RegistryValueType::INT,
      api::RegistryValueType::BOOL,   api::RegistryValueType::DOUBLE,    api::RegistryValueType::DOUBLE_ARRAY,
      api::RegistryValueType::MESSAGE};
  for (std::size_t i = 0; i < keys.size(); ++i) EXPECT_EQ(kv->get_type(keys[i]), expected[i]) << keys[i];
  EXPECT_EQ(kv->get_type("nope"), api::RegistryValueType::KEY_DNE);
  kv->batch_end();
}

TEST_F(RegistryTest, ContainsGroupAndKey) {
  auto kv = set_up_store_with_all_types("group");
  EXPECT_TRUE(reg->has_group("group"));
  auto groups = reg->group_array();
  ASSERT_TRUE(groups);
  for (const auto& g : *groups) EXPECT_TRUE(reg->has_group(g));
  EXPECT_FALSE(reg->has_group("nonexistent_group"));
  for (const auto& k : keys) EXPECT_TRUE(kv->has_key(k));
  kv->batch_end();
}

TEST_F(RegistryTest, GetItemTypedAndRaw) {
  auto kv = set_up_store_with_all_types();
  EXPECT_EQ(*kv->get_value<std::string>(keys[0]), "Wassup");
  EXPECT_EQ(*kv->get_value<StringArray>(keys[1]), (StringArray{"What", "is", "up"}));
  EXPECT_EQ(*kv->get_value<std::int64_t>(keys[2]), 42);
  EXPECT_EQ(*kv->get_value<bool>(keys[3]), true);
  EXPECT_DOUBLE_EQ(*kv->get_value<double>(keys[4]), 3.14159);
  EXPECT_EQ(*kv->get_value<Matrix>(keys[5]), std::get<Matrix>(vals[5]));
  EXPECT_EQ(kv->get_value<Message>(keys[6])->source_identifier, "Genesis planet");
  for (std::size_t i = 0; i < keys.size(); ++i) EXPECT_TRUE(same(*kv->get(keys[i]), vals[i])) << keys[i];
  EXPECT_EQ(kv->size(), keys.size());
  EXPECT_EQ(*kv->keys(), keys);  // insertion order
  kv->batch_end();
}

TEST_F(RegistryTest, DeleteItemsValues) {
  auto kv = set_up_store_with_all_types();
  auto items = kv->items();
  auto values = kv->values();
  ASSERT_EQ(items.size(), keys.size());
  for (std::size_t i = 0; i < keys.size(); ++i) {
    EXPECT_EQ(items[i].first, keys[i]);
    EXPECT_TRUE(same(items[i].second, vals[i]));
    EXPECT_TRUE(same(values[i], vals[i]));
  }
  for (const auto& k : keys) {
    EXPECT_TRUE(kv->remove_key(k));
    EXPECT_FALSE(kv->has_key(k));
  }
  EXPECT_FALSE(kv->remove_key("never"));
  kv->batch_end();
}

TEST_F(RegistryTest, SetPermanent) {
  const std::string permanent_batch = "permanent", not_permanent_batch = "not_permanent";
  std::vector<std::string> p_keys, np_keys;
  for (const auto& k : keys) {
    p_keys.push_back(k + "_p");
    np_keys.push_back(k + "_np");
  }
  // Messages are not persisted in the C++ port: drop the Message key for this test.
  const std::size_t n = keys.size() - 1;

  {
    TestMediator med1;
    cobra::StandardRegistryPlugin registry("Standard registry 1");
    registry.init_plugin(tmp.string(), &med1);
    auto r = registry.new_registry(std::nullopt);
    auto kvp = r->batch_start(permanent_batch);
    kvp->set_permanent(true);
    for (std::size_t i = 0; i < n; ++i) kvp->set(p_keys[i], vals[i]);
    kvp->set_permanent(false);
    for (std::size_t i = 0; i < n; ++i) kvp->set(np_keys[i], vals[i]);
    kvp->batch_end();
    auto kvnp = r->batch_start(not_permanent_batch);
    kvnp->set_permanent(false);
    for (std::size_t i = 0; i < n; ++i) kvnp->set(np_keys[i], vals[i]);
    kvnp->batch_end();
    registry.shutdown_plugin();
  }
  {
    TestMediator med2;
    cobra::StandardRegistryPlugin registry("Standard registry 2");
    registry.init_plugin(tmp.string(), &med2);
    auto r = registry.new_registry(std::nullopt);
    auto kvp = r->batch_start(permanent_batch);
    auto kvnp = r->batch_start(not_permanent_batch);
    EXPECT_EQ(kvnp->size(), 0u);
    EXPECT_EQ(kvp->size(), n);
    for (std::size_t i = 0; i < n; ++i) {
      EXPECT_FALSE(kvp->has_key(np_keys[i]));
      ASSERT_TRUE(kvp->has_key(p_keys[i])) << p_keys[i];
      EXPECT_TRUE(same(*kvp->get(p_keys[i]), vals[i])) << p_keys[i];
    }
    kvp->batch_end();
    kvnp->batch_end();
  }
}

TEST_F(RegistryTest, BatchStartTwiceLogsError) {
  reg->batch_start("new_group");
  EXPECT_FALSE(med.has_error());
  reg->batch_start("new_group");
  EXPECT_TRUE(med.has_error());
}

}  // namespace
