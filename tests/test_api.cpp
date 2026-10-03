// API compile/behaviour test (port of pntos-cobra-api/tests/test_api.py).
// Verifies the API headers compile, value types behave, and registry value conversions match
// Cobra's StandardKeyValueStore conversion table.
#include <pntos/api.hpp>

#include <gtest/gtest.h>

using namespace pntos::api;

TEST(Api, TimestampOrderingAndSeconds) {
  Timestamp a(1'000'000'000), b(2'500'000'000);
  EXPECT_LT(a, b);
  EXPECT_DOUBLE_EQ(b.seconds(), 2.5);
  EXPECT_EQ(Timestamp::from_seconds(0.01).elapsed_nsec, 10'000'000);
  EXPECT_EQ(Timestamp(a.to_aspn()), a);
}

TEST(Api, RegistryValueTypes) {
  EXPECT_EQ(registry_value_type(RegistryValue(std::string("x"))), RegistryValueType::STR);
  EXPECT_EQ(registry_value_type(RegistryValue(StringArray{"a"})), RegistryValueType::STR_ARRAY);
  EXPECT_EQ(registry_value_type(RegistryValue(std::int64_t{1})), RegistryValueType::INT);
  EXPECT_EQ(registry_value_type(RegistryValue(true)), RegistryValueType::BOOL);
  EXPECT_EQ(registry_value_type(RegistryValue(1.5)), RegistryValueType::DOUBLE);
  EXPECT_EQ(registry_value_type(RegistryValue(Matrix::Zero(2, 2))), RegistryValueType::DOUBLE_ARRAY);
  EXPECT_EQ(registry_value_type(RegistryValue(Message{})), RegistryValueType::MESSAGE);
}

TEST(Api, RegistryValueConversions) {
  // int -> everything but Message
  RegistryValue i{std::int64_t{42}};
  EXPECT_EQ(*registry_value_as<std::string>(i), "42");
  EXPECT_EQ(*registry_value_as<StringArray>(i), StringArray{"42"});
  EXPECT_EQ(*registry_value_as<std::int64_t>(i), 42);
  EXPECT_EQ(*registry_value_as<bool>(i), true);
  EXPECT_DOUBLE_EQ(*registry_value_as<double>(i), 42.0);
  EXPECT_DOUBLE_EQ((*registry_value_as<Matrix>(i))(0, 0), 42.0);
  EXPECT_FALSE(registry_value_as<Message>(i).has_value());

  // float -> int unsupported (as in Cobra), float -> str ok
  RegistryValue f{3.5};
  EXPECT_FALSE(registry_value_as<std::int64_t>(f).has_value());
  EXPECT_TRUE(registry_value_as<std::string>(f).has_value());

  // list[str] of numbers -> matrix, str -> int only if fully numeric
  RegistryValue sa{StringArray{"1", "2.5", "-3"}};
  auto m = registry_value_as<Matrix>(sa);
  ASSERT_TRUE(m.has_value());
  EXPECT_EQ(m->size(), 3);
  EXPECT_DOUBLE_EQ((*m)(1, 0), 2.5);
  EXPECT_FALSE(registry_value_as<Matrix>(RegistryValue{StringArray{"1", "x"}}).has_value());
  EXPECT_EQ(*registry_value_as<std::int64_t>(RegistryValue{std::string("17")}), 17);
  EXPECT_FALSE(registry_value_as<std::int64_t>(RegistryValue{std::string("17abc")}).has_value());

  // matrix -> list[str], matrix -> str unsupported in Cobra? (Cobra: ndarray->str is None)
  RegistryValue mat{Matrix::Constant(2, 1, 1.5)};
  EXPECT_EQ(registry_value_as<StringArray>(mat)->size(), 2u);
  EXPECT_FALSE(registry_value_as<std::int64_t>(mat).has_value());
}
