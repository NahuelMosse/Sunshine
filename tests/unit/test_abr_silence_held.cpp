/**
 * @file tests/unit/test_abr_silence_held.cpp
 * @brief Test what the signal meter reads as delivered after a silence of the client's loss
 * summaries: the frames held on the way arrive at once and are not the pipe's rate.
 */
// test includes
#include "../tests_common.h"

// standard includes
#include <algorithm>
#include <cstdint>

// local includes
#include <src/abr/control.h>
#include <src/abr/gamestream/signals.h>
#include <src/abr/params.h>

using abr::gamestream::LossSummary;
using abr::gamestream::SentFrame;
using abr::gamestream::SignalMeter;

// @ABR-10-S1 @ABR-10-ASM-08
TEST(AbrSilenceHeldTest, FramesHeldThroughASilenceAreNotReadAsThePipe) {
  SignalMeter meter;
  const abr::Params params;
  meter.configure(params);
  meter.rtt(0.040);

  constexpr int64_t frame_us = 16667;
  constexpr int64_t start = 1'000'000'000;
  constexpr int64_t silence_from = start + 3'000'000;  // the video and its summaries stop for 400 ms
  constexpr int64_t silence_to = silence_from + 400'000;
  int64_t next_summary = start;
  uint32_t held_from = 0;
  abr::Measurement m;
  double worst = 0;
  double sent_rate = 0;
  for (uint32_t index = 1; index <= 300; ++index) {
    const int64_t now = start + index * frame_us;
    SentFrame c;
    c.index = index;
    c.t_first_us = now;
    c.t_last_us = now + 2000;
    c.packets = 20;
    c.data_packets = 20;
    c.rtp_bytes = 20000;
    c.wire_bytes = 21000;
    meter.sent(c);
    if (now >= silence_from && held_from == 0) {
      held_from = index - 1;
    }
    if (now >= next_summary && (now < silence_from || now >= silence_to)) {
      next_summary = now + 50000;
      LossSummary s;
      // the first summary after the silence still names the last frame before it; the next one,
      // every frame held on the way
      s.last_good = now >= silence_to && now < silence_to + 40000 ? held_from : index;
      meter.summary(now + 20000, s);
    }
    // measured on every frame, as the controller does, so the window holds only the last 250 ms
    const bool measured = meter.measure(now + 20000, m);
    if (measured && now >= silence_to && now < silence_to + 450000) {
      worst = std::max(worst, m.received_rate);
      sent_rate = std::max(sent_rate, m.sent_rate);
    }
  }
  ASSERT_GT(sent_rate, 0);
  EXPECT_LT(worst, 1.3 * sent_rate) << "half a second of held frames read as more than the pipe";
}
