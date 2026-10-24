/**
 * @file tests/unit/test_abr_params.cpp
 * @brief Test src/abr/params.*
 */
#include "../tests_common.h"

#include <filesystem>
#include <fstream>
#include <src/abr/control.h>
#include <src/abr/params.h>

namespace {
  // A parameters file that exists for the length of one test
  struct params_file_t {
    explicit params_file_t(const std::string &text, const char *name) {
      path = std::filesystem::temp_directory_path() / name;
      std::ofstream(path) << text;
    }

    ~params_file_t() {
      std::error_code ec;
      std::filesystem::remove(path, ec);
    }

    std::filesystem::path path;
  };
}  // namespace

TEST(AbrParamsTest, VariantsRoundTripThroughTheirNames) {
  for (auto v : {abr::Variant::baseline, abr::Variant::patched, abr::Variant::padding, abr::Variant::train}) {
    EXPECT_EQ(abr::variant_by_name(abr::name(v)), v);
  }
  EXPECT_THROW(abr::variant_by_name("nope"), std::invalid_argument);
  EXPECT_STREQ(abr::name(static_cast<abr::Variant>(99)), "?");
}

TEST(AbrParamsTest, EveryReasonHasAName) {
  for (auto r : {abr::Reason::startup, abr::Reason::steady, abr::Reason::no_evidence, abr::Reason::queue_growing, abr::Reason::queue_high, abr::Reason::loss_backoff, abr::Reason::ramping, abr::Reason::reanchored, abr::Reason::probing, abr::Reason::capped}) {
    EXPECT_STRNE(abr::name(r), "?");
  }
  EXPECT_STREQ(abr::name(static_cast<abr::Reason>(99)), "?");
}

TEST(AbrParamsTest, SetsKnownParametersAndRejectsUnknownOnes) {
  abr::Params p;
  EXPECT_TRUE(p.set("Tc", "0.25"));
  EXPECT_DOUBLE_EQ(p.Tc, 0.25);
  EXPECT_TRUE(p.set("mtu", "1200"));
  EXPECT_EQ(p.mtu, 1200);
  EXPECT_TRUE(p.set("variant", "train"));
  EXPECT_EQ(p.variant, abr::Variant::train);
  EXPECT_FALSE(p.set("variant", "nope"));
  EXPECT_FALSE(p.set("not_a_parameter", "1"));
}

TEST(AbrParamsTest, LoadsAFlatTomlFile) {
  params_file_t file("# a comment\n[section]\n\n  Tc = 0.2  # trailing comment\nmtu=1100\nvariant = padding\n", "sunshine_abr_params_ok.toml");
  abr::Params p;
  EXPECT_EQ(p.load(file.path.string()), "");
  EXPECT_DOUBLE_EQ(p.Tc, 0.2);
  EXPECT_EQ(p.mtu, 1100);
  EXPECT_EQ(p.variant, abr::Variant::padding);
}

TEST(AbrParamsTest, ReportsTheLineOfEachMistake) {
  abr::Params p;
  EXPECT_NE(p.load((std::filesystem::temp_directory_path() / "sunshine_abr_params_missing.toml").string()).find("cannot open"), std::string::npos);

  params_file_t no_equals("Tc = 0.1\nmtu\n", "sunshine_abr_params_noeq.toml");
  EXPECT_EQ(p.load(no_equals.path.string()), "line 2: missing =");

  params_file_t unknown("Tc = 0.1\n\nmisspelled = 3\n", "sunshine_abr_params_unknown.toml");
  EXPECT_EQ(p.load(unknown.path.string()), "line 3: unknown parameter 'misspelled'");

  params_file_t variant("variant = fancy\n", "sunshine_abr_params_variant.toml");
  EXPECT_EQ(p.load(variant.path.string()), "line 1: unknown variant 'fancy'");
}

TEST(AbrParamsTest, DumpLoadsBackToTheSameValues) {
  abr::Params tuned;
  ASSERT_TRUE(tuned.set("Tc", "0.37"));
  ASSERT_TRUE(tuned.set("coh_cycles", "7"));
  const std::string text = tuned.dump();
  EXPECT_NE(text.find("Tc = 0.37\n"), std::string::npos);
  EXPECT_NE(text.find("coh_cycles = 7\n"), std::string::npos);

  params_file_t file(text, "sunshine_abr_params_dump.toml");
  abr::Params loaded;
  ASSERT_EQ(loaded.load(file.path.string()), "");
  EXPECT_EQ(loaded.dump(), text);
}
