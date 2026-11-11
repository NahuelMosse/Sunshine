/**
 * @file tests/unit/test_abr_idr_deferred_loss.cpp
 * @brief Test that an IDR request inside the coalescing window is served when the window ends, in a
 * session the controller takes over from the client's loss stats.
 */
// test includes
#include "../tests_common.h"

// standard includes
#include <chrono>
#include <cstdint>
#include <optional>
#include <vector>

// local includes
#include <src/adaptive_bitrate.h>

using namespace std::literals;

namespace {
  // The round trip ENet measures on a path that is not congested: two of them are under 300 ms
  constexpr double rtt_s = 0.060;
  constexpr auto window = 300ms;
  constexpr auto pass = 5ms;

  adaptive_bitrate::session_t make_session() {
    return {adaptive_bitrate::mode_e::full, 20000, 60, 1024, 90, 300000.0};
  }

  void put32le(std::vector<char> &out, std::uint32_t v) {
    for (int shift = 0; shift < 32; shift += 8) {
      out.push_back(char(v >> shift));
    }
  }

  // The client sends its loss stats, so the controller is in charge of the session
  void take_over(adaptive_bitrate::session_t &session, std::chrono::steady_clock::time_point now) {
    // A loss stats message as moonlight-common-c lays it out, little-endian: nothing lost
    std::vector<char> msg;
    for (std::uint32_t v : {0u, 50u, 1000u, 1u, 0u, 0u, 0u, 0x14u}) {
      put32le(msg, v);
    }
    EXPECT_TRUE(session.loss_stats(now, {msg.data(), msg.size()}));
    EXPECT_TRUE(session.active());
  }

  // The control loop's passes from `from` to `to`, every 5 ms: when the first one sends an IDR
  // frame, if any does
  std::optional<std::chrono::steady_clock::time_point> first_idr(adaptive_bitrate::session_t &session, std::chrono::steady_clock::time_point from, std::chrono::steady_clock::time_point to) {
    for (auto now = from; now <= to; now += pass) {
      session.tick(now, rtt_s);
      if (session.take_idr_request()) {
        return now;
      }
    }
    return std::nullopt;
  }

  // How many IDR frames the control loop's passes from `from` to `to` send
  int idrs_sent(adaptive_bitrate::session_t &session, std::chrono::steady_clock::time_point from, std::chrono::steady_clock::time_point to) {
    int sent = 0;
    for (auto now = from; now <= to; now += pass) {
      session.tick(now, rtt_s);
      sent += session.take_idr_request() ? 1 : 0;
    }
    return sent;
  }
}  // namespace

// @ABR-03-S1 @ABR-03-S2 @ABR-03-S3 @ABR-03-S4
TEST(AbrIdrDeferredByLossStatsTest, ALostIdrIsSentAgainWithinTheWindowOfTheSecondRequest) {
  const auto start = std::chrono::steady_clock::now() + 4s;
  auto session = make_session();
  take_over(session, start);

  // The first request is answered; that IDR frame is lost on the way
  const auto first = start + 1s;
  ASSERT_TRUE(session.idr_requested(first));
  // The client asks once more, inside the window: not answered on the spot...
  const auto second = first + 100ms;
  EXPECT_FALSE(session.idr_requested(second)) << "the IDR frame already sent may still be on its way";

  // ...but not dropped either: the control loop sends it once the window ends
  const auto sent = first_idr(session, second, second + window);
  ASSERT_TRUE(sent.has_value()) << "the second request must produce an IDR frame";
  EXPECT_GE(*sent, first + window) << "not before the window ends";
  EXPECT_LE(*sent - second, window) << "within the window length of the second request";
}

// @ABR-03-ASM-01 @ABR-03-S3
TEST(AbrIdrDeferredByLossStatsTest, RequestsInsideOneWindowGetOneIdrWhenItEnds) {
  const auto start = std::chrono::steady_clock::now() + 4s;
  auto session = make_session();
  take_over(session, start);

  const auto first = start + 1s;
  ASSERT_TRUE(session.idr_requested(first));
  for (auto later : {50ms, 100ms, 200ms, 250ms}) {
    EXPECT_FALSE(session.idr_requested(first + later)) << later.count();
  }

  EXPECT_EQ(idrs_sent(session, first + 50ms, first + window - pass), 0) << "nothing before the window ends";
  EXPECT_EQ(idrs_sent(session, first + window, first + 2 * window - pass), 1) << "one IDR frame for every request of the window";
}

// @ABR-03-ASM-02
TEST(AbrIdrDeferredByLossStatsTest, TheDeferredIdrLeavesOnThe5MsPassAfterThe300MsWindowEnds) {
  const auto start = std::chrono::steady_clock::now() + 4s;
  auto session = make_session();
  take_over(session, start);
  EXPECT_EQ(adaptive_bitrate::control_wait(&session), pass) << "the control loop passes every 5 ms";

  const auto first = start + 1s;
  ASSERT_TRUE(session.idr_requested(first));
  ASSERT_FALSE(session.idr_requested(first + 20ms));

  session.tick(first + window - pass, rtt_s);
  EXPECT_FALSE(session.take_idr_request()) << "the window has not ended";
  session.tick(first + window, rtt_s);
  EXPECT_TRUE(session.take_idr_request()) << "the window ended";
  EXPECT_EQ(idrs_sent(session, first + window + pass, first + 3 * window), 0) << "sent once";
}

// @ABR-03-ASM-02
TEST(AbrIdrDeferredByLossStatsTest, ARequestAnsweredAfterTheWindowStandsForTheDeferredOne) {
  const auto start = std::chrono::steady_clock::now() + 4s;
  auto session = make_session();
  take_over(session, start);

  const auto first = start + 1s;
  ASSERT_TRUE(session.idr_requested(first));
  ASSERT_FALSE(session.idr_requested(first + 100ms));
  // The control thread did not pass before the client's next request, after the window
  EXPECT_TRUE(session.idr_requested(first + window + 100ms));

  EXPECT_EQ(idrs_sent(session, first + window + 100ms, first + 4 * window), 0) << "the deferred request was served by that one";
}
