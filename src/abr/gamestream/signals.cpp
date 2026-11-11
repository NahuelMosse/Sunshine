/**
 * @file signals.cpp
 * @brief The signal meter: queue, loss, burst length and delivery from what GameStream carries.
 */
#include "signals.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iterator>

namespace abr {
  namespace gamestream {
    using namespace cal;

    void SignalMeter::sent(const SentFrame &c) {
      if (static_cast<int32_t>(c.index - last_sent_) > 0) {
        last_sent_ = c.index;
      }
      sent_[c.index] = c;
      bits_sent_ += c.wire_bytes * 8.0;
      if (c.packets > 0) {
        bits_data_ += c.wire_bytes * 8.0 * c.data_packets / c.packets;
      }
      packets_tx_ += c.packets;
      data_tx_ += c.data_packets;
      packets_per_frame_ = packets_per_frame_ > 0 ? packets_per_frame_ + kPacketsPerFrameGain * (c.packets - packets_per_frame_) : c.packets;
      for (auto it = sent_.begin(); it != sent_.end();) {
        if (c.t_first_us - it->second.t_first_us > kRetentionUs) {
          it = sent_.erase(it);
        } else {
          break;
        }
      }
    }

    void SignalMeter::configure(const Params &p) {
      use_age_ = p.queue_age != 0;
      age_recent_us_ = static_cast<int64_t>(kAgeRecent * 1e6);
      cadence_max_us_ = static_cast<int64_t>(kAgeCadence * 1e6 / p.fps);
      use_same_frames_ = p.delivery_full > 0;
      alarm_intervals_ = kAlarmIntervals;
      proof_packets_ = kQuietFrameProof > 0 ? static_cast<int>(kQuietFrameMinPackets) : 0;
      proof_packets_back_ = kQuietFrameProof > 0 ? static_cast<int>(kQuietFrameMinPacketsBack) : 0;
      prior_us_ = static_cast<int64_t>(kAlarmPriorS * 1e6);
      span_min_ = p.alarm_span_min;
    }

    void SignalMeter::fec_status(const FecStatus &status) {
      // The report comes from the network: it is only believed up to what the host put on the wire for
      // that frame, once per FEC block of the frame.
      const auto sent_it = sent_.find(status.index);
      if (sent_it == sent_.end()) {
        return;
      }
      // GameStream carries the block in two bits: at most four per frame
      if (status.block < 0 || status.block >= session::kMaxBlocksPerFrame) {
        return;
      }
      if (!fec_seen_.insert({status.index, status.block}).second) {
        return;
      }
      if (fec_seen_.size() > kMaxSeen) {
        fec_seen_.erase(fec_seen_.begin());
      }
      const SentFrame &f = sent_it->second;
      const int data_sent = std::max(0, std::min(f.data_packets, f.packets));
      const int parity_sent = std::max(0, f.packets - data_sent);
      FecStatus e = status;
      // Each block reports its own packets: together they cannot add up to more than the frame had
      auto &claimed = fec_claimed_[status.index];
      e.data_total = std::max(0, std::min(e.data_total, data_sent - claimed.first));
      e.parity_total = std::max(0, std::min(e.parity_total, parity_sent - claimed.second));
      claimed.first += e.data_total;
      claimed.second += e.parity_total;
      if (fec_claimed_.size() > kMaxSeen) {
        fec_claimed_.erase(fec_claimed_.begin());
      }
      e.data_received = std::max(0, std::min(e.data_received, e.data_total));
      e.parity_received = std::max(0, std::min(e.parity_received, e.parity_total));
      const int total = e.data_total + e.parity_total;
      const int received_n = e.data_received + e.parity_received;
      // the gaps behind the highest received cannot be more than what did not arrive
      e.missing = std::min(e.missing, total - received_n);
      // Reassembled, what is missing up to the total is the parity that did not turn out to be needed: what is lost is
      // the gaps. Counting up to the total always gives the block's parity: the burst looks like two to
      // four packets when the channel's is one and a bit, and the parity goes to its ceiling.
      const int missing_n = received_n >= e.data_total && e.missing >= 0 ? std::max(e.missing, e.data_total - e.data_received) : std::max(0, total - received_n);
      if (missing_n > 0) {
        packets_lost_ += missing_n;
        lost_in_frames_ += missing_n;
        frames_with_loss_ += 1;
      }
      // Reed-Solomon reassembles from any k of the n: if fewer arrived than the data,
      // the frame was lost even if part of it arrived
      if (received_n < e.data_total && fec_failed_.insert(status.index).second) {
        ++failed_with_status_;
        if (fec_failed_.size() > kMaxSeen) {
          fec_failed_.erase(fec_failed_.begin());
        }
      }
    }

    void SignalMeter::summary(int64_t arrival_us, const LossSummary &r) {
      // A frame the host has not sent yet cannot have arrived: whatever follows would read a huge advance
      // and pin the budget at its floor.
      if (static_cast<int32_t>(r.last_good - last_sent_) > 0) {
        return;
      }
      if (!had_summary_) {
        had_summary_ = true;
        last_good_ = r.last_good;
        summary_arrival_us_ = arrival_us;
        failed_with_status_ = 0;
        return;
      }
      // After a silence the summaries that follow are the held-up ones arriving together: what was
      // delivered cannot be read from their spacing until a whole window of normal ones has gone by.
      // Nor from the summaries written just after it: if the video was held too, the frames held on
      // the way reach the client at once, and the first summaries name them all —half a second of
      // frames read as twice the pipe—. What arrives within a window of the silence is left out.
      const bool resumed = arrival_us - summary_arrival_us_ > kSilenceUs;
      if (resumed) {
        summaries_.clear();
        window_rx_.clear();
        bits_arrived_ = 0;
        resumed_us_ = arrival_us;
      }
      summary_arrival_us_ = arrival_us;
      const int32_t advance = static_cast<int32_t>(r.last_good - last_good_);
      if (advance <= 0) {
        return;  // no advance, or an old summary
      }

      // What arrived since the previous summary is the frames up to the last good one, minus those
      // lost on the way. The host knows the sizes.
      double bytes = 0;
      int known = 0;
      for (auto it = sent_.upper_bound(last_good_);
           it != sent_.end() && it->first <= r.last_good;
           ++it) {
        bytes += it->second.wire_bytes;
        ++known;
      }
      // If the requested keyframe arrived in the interval, the lost ones the summary counts are those
      // the client discarded while waiting for it, which arrived whole. The loss that led to the request was
      // already reported by the FEC status of the frame that could not be reassembled.
      const bool discards = idr_pending_ && r.last_good >= idr_from_;
      if (discards) {
        idr_pending_ = false;
      }
      // And the client counts as lost every index that did not reach it, including those the host never sent
      // —the ones Sunshine drops when its encoder queue fills up—. Those are not network loss:
      // they are those of the interval the host has no record of.
      const double not_sent = std::max(0.0, static_cast<double>(advance) - known);
      const uint32_t lost = discards ? 0 : static_cast<uint32_t>(std::min(static_cast<double>(advance), std::max(0.0, static_cast<double>(r.lost) - not_sent)));

      const double per_frame = known > 0 ? bytes / known : 0.0;
      if (arrival_us - resumed_us_ >= kWindowUs) {
        bits_arrived_ += std::max(0.0, bytes - lost * per_frame) * 8.0;
      }

      // Frames lost whole send no FEC status: no packet arrived that the client
      // could talk about. They are the lost ones the summary counts and the status does not explain.
      const int whole = std::max(0, static_cast<int>(lost) - failed_with_status_);
      packets_lost_ += whole * packets_per_frame_;
      failed_with_status_ = 0;
      last_good_ = r.last_good;
      lost_total_ += lost;

      // ── what was delivered, over the same frames ─────────────────────────────
      // Comparing what arrives now with what left a trip earlier compares different frames, and the
      // encoder varies from one frame to another considerably more than the pipe: the
      // ratio between the two rates varied between 9 and 17 % in steady state, and that triggered backoffs
      // with the queue empty. Here the frames the last good one advanced over at least 300 ms are taken, and
      // what they weighed is divided by the greater of how long they took to leave and how long they took to
      // arrive: BBR's delivery rate estimate (draft-cheng-iccrg-delivery-rate-estimation).
      // How much slower they arrived than they left says whether the pipe is slowing them down.
      summaries_.push_back({arrival_us, r.last_good, lost_total_});
      while (summaries_.size() > 2 && arrival_us - summaries_[1].arrival_us >= kDeliverySpanUs) {
        summaries_.pop_front();
      }
      const Summary &from = summaries_.front();
      if (arrival_us - from.arrival_us >= kDeliverySpanUs) {
        const auto a = sent_.find(from.good);
        const auto b = sent_.find(r.last_good);
        if (a != sent_.end() && b != sent_.end()) {
          double bytes = 0;
          int n = 0;
          for (auto it = sent_.upper_bound(from.good); it != sent_.end() && it->first <= r.last_good; ++it) {
            bytes += it->second.wire_bytes;
            ++n;
          }
          if (n > 0) {
            bytes -= std::min(static_cast<double>(n), lost_total_ - from.lost) * bytes / n;
          }
          const double out_span = (b->second.t_last_us - a->second.t_last_us) / 1e6;
          const double in_span = (arrival_us - from.arrival_us) / 1e6;
          if (out_span > 0 && in_span > 0 && bytes > 0) {
            same_frames_rate_ = bytes * 8.0 / std::max(out_span, in_span);
            delivery_ = out_span / in_span;
            t_same_frames_us_ = arrival_us;
          }
        }
      }

      // ── the age of the last good frame ────────────────────────────────────────
      // From when it left until the summary that names it arrived: the video's trip plus the return one.
      // It is the only delay measure that crosses the VIDEO's queue: ENet's round trip
      // travels on the control channel, and behind a queue that shares per flow it does not see it.
      // But only with frames going out at their cadence: while the client discards waiting for a
      // keyframe or an invalidation, or the host skips frames (the divider), the last good one
      // stays still and its age adds the wait until the next, which is not queue.
      const auto c = sent_.find(r.last_good);
      if (!resumed && c != sent_.end() && (cadence_max_us_ <= 0 || (!discards && !idr_pending_))) {
        const auto next_it = std::next(c);
        const int64_t next_frame_us = next_it != sent_.end() ? next_it->second.t_first_us : arrival_us;
        if (cadence_max_us_ <= 0 || next_frame_us - c->second.t_first_us <= cadence_max_us_) {
          ages_.push_back({arrival_us, (arrival_us - c->second.t_last_us) / 1000.0});
        }
      }
    }

    bool SignalMeter::plausible(double ms, std::deque<Sample> &history, int &streak) {
      if (history.empty()) {
        return true;
      }
      double delay_floor = history.front().v;
      for (const Sample &x : history) {
        delay_floor = std::min(delay_floor, x.v);
      }
      if (ms >= 0.5 * delay_floor) {
        streak = 0;
        return true;
      }
      if (++streak < kConfirmReadings) {
        return false;
      }
      // Three in a row: the path really got shorter, and the old floor is no longer any use
      streak = 0;
      history.clear();
      return true;
    }

    void SignalMeter::rtt(double seconds) {
      if (seconds <= 0 || !plausible(seconds * 1000.0, rtts_, rtt_low_)) {
        return;
      }
      rtt_ = seconds;
    }

    // The noise of a real path —Wi-Fi, internet— is samples that rise and come back by themselves: the air
    // cuts out for a while, an ack gets delayed. One's own queue does not do that: a pipe that narrows raises it like
    // a ramp that does not come back, and a path that lengthens raises it by a step that stays. What
    // tells the three apart is looking at both sides: a sample counts as noise by how much it stands out
    // over the smaller of before and the smaller of after, within kNoiseSideUs on each side. A ramp does not stand out
    // from the smaller of after (it keeps rising) and neither does a step (it stays). Each sample is judged
    // with that delay. The noise is the height of the jumps: the 90th percentile of the samples that stand out
    // more than kJumpMin in the last kNoiseWindowUs. A Wi-Fi's outages take up few samples —a
    // couple every few seconds—, and the 90th percentile of all fell between them: the noise came from the background
    // swing and the jumps were still read as queue. Without jumps, it is that of all of them.
    void SignalMeter::update_noise(int64_t t_us) {
      constexpr int64_t kNoiseSideUs = 250000, kNoiseWindowUs = 30000000;
      constexpr double kJumpMin = 0.005;
      constexpr size_t kJumpsMin = 5;
      for (size_t c = 0; c < raw_.size(); ++c) {
        const Sample &x = raw_[c];
        if (x.t_us <= noise_center_us_) {
          continue;
        }
        if (t_us - x.t_us < kNoiseSideUs) {
          break;  // the after is not visible yet
        }
        double before = x.v, after = x.v;
        bool has_before = false, has_after = false;
        for (size_t j = 0; j < raw_.size(); ++j) {
          const Sample &y = raw_[j];
          if (y.t_us < x.t_us && x.t_us - y.t_us <= kNoiseSideUs) {
            before = has_before ? std::min(before, y.v) : y.v;
            has_before = true;
          }
          if (y.t_us > x.t_us && y.t_us - x.t_us <= kNoiseSideUs) {
            after = has_after ? std::min(after, y.v) : y.v;
            has_after = true;
          }
        }
        noise_center_us_ = x.t_us;
        if (!has_before || !has_after) {
          continue;
        }
        excursions_.push_back({x.t_us, std::max(0.0, x.v - std::max(before, after)) / 1000.0});
      }
      while (!excursions_.empty() && t_us - excursions_.front().t_us > kNoiseWindowUs) {
        excursions_.pop_front();
      }
      if (excursions_.size() < kMinExcursions) {
        noise_ = 0;
        return;
      }
      std::vector<double> v, jumps;
      v.reserve(excursions_.size());
      for (const Sample &e : excursions_) {
        v.push_back(e.v);
        if (e.v > kJumpMin) {
          jumps.push_back(e.v);
        }
      }
      if (jumps.size() >= kJumpsMin) {
        v.swap(jumps);
      }
      const size_t k = static_cast<size_t>(kNoisePercentile * (v.size() - 1));
      std::nth_element(v.begin(), v.begin() + static_cast<long>(k), v.end());
      noise_ = v[k];
    }

    // How much of the round trip of what arrives around t_us is the uplink's queue: the oldest ping that
    // arrived close to it, over its ten-second minimum. Pings held up behind the same stall arrive
    // together, and the oldest of them is the one that measures it.
    double SignalMeter::upstream_ms(int64_t t_us) const {
      double v_min = 0, near_max = 0;
      bool first = true, seen_near = false;
      for (const Sample &x : ping_drifts_) {
        v_min = first ? x.v : std::min(v_min, x.v);
        first = false;
        if (std::llabs(t_us - x.t_us) > kPingPairUs) {
          continue;
        }
        near_max = seen_near ? std::max(near_max, x.v) : x.v;
        seen_near = true;
      }
      return seen_near ? std::max(0.0, near_max - v_min) : 0.0;
    }

    // The video's one-way delay of each frame, in the shape the queue measurements want: a round trip.
    // It is that of the frame's last packet, which also carries the queue the frame itself made. The
    // clocks are unrelated, so the delay is read from the first frame and kept as its rise over the
    // ten-second minimum (which also takes the clock's drift out), added to the round trip the host had
    // then. The sample is placed at the time the frame would have taken to come back, so the floor and the
    // noise work on it as they would on a ping sent along with the frame. It does not
    // cross the uplink, so there is no queue of the uplink to take off. Being the video's own, the swing
    // of an AQM's queue (fq_codel) also reads as noise, and the queue target grows with it.
    bool SignalMeter::train(const ReceivedTrain &r, double &queue, double &growth, double *sample_ms, int64_t arrival_us) {
      if (sample_ms) {
        *sample_ms = 0;
      }
      const auto it = sent_.find(r.index);
      if (it == sent_.end()) {
        return false;
      }
      // One report per frame: a repeated index would add a sample and a dispersion that are not new.
      if (!train_seen_.insert(r.index).second) {
        return false;
      }
      if (train_seen_.size() > kMaxSeen) {
        train_seen_.erase(train_seen_.begin());
      }
      if (trains_rx_.size() < kMaxTrains) {
        trains_rx_.push_back(r);
      }
      if (rtt_ <= 0 || r.packets == 0) {
        return false;
      }
      const int64_t sent_us = it->second.t_last_us;
      // A frame that arrived in a fraction of the time it took to send was stamped by a receiver that
      // read its packets in a batch: its delay says when the receiver woke up, not what the queue held.
      const int64_t send_span_us = sent_us - it->second.t_first_us;
      if (span_min_ > 0 && r.packets >= 3 && send_span_us > 0 && static_cast<double>(static_cast<uint32_t>(r.t_last_us - r.t_first_us)) < span_min_ * static_cast<double>(send_span_us)) {
        return false;
      }
      const uint32_t delay_us = r.t_last_us - static_cast<uint32_t>(sent_us);
      if (!has_delay_ref_) {
        has_delay_ref_ = true;
        delay_ref_ = delay_us;
        delay_base_ms_ = rtt_ * 1000.0;
      }
      const double delay_ms = static_cast<int32_t>(delay_us - delay_ref_) / 1000.0;
      // A delay that moved by more than everything the meter remembers is a corrupt report or a clock that
      // jumped. One such reading is dropped; the same reading three times in a row is the new path.
      const double window_ms = kRetentionUs / 1000.0;
      if (!delays_.empty() && std::fabs(delay_ms - delays_.back().v) > window_ms) {
        if (delay_jumps_ > 0 && std::fabs(delay_ms - delay_candidate_) <= window_ms) {
          ++delay_jumps_;
        } else {
          delay_jumps_ = 1;
        }
        delay_candidate_ = delay_ms;
        if (delay_jumps_ < kConfirmReadings) {
          return false;
        }
        delays_.clear();
        raw_.clear();
        excursions_.clear();
      }
      delay_jumps_ = 0;
      delays_.push_back({sent_us, delay_ms});
      while (sent_us - delays_.front().t_us > kFloorWindowUs || delays_.size() > kMaxDelays) {
        delays_.pop_front();
      }
      double delay_min = delay_ms;
      for (const Sample &x : delays_) {
        delay_min = std::min(delay_min, x.v);
      }
      const double ms = delay_base_ms_ + delay_ms - delay_min;
      const int64_t t_us = sent_us + static_cast<int64_t>(ms * 1000.0);
      // The sample is placed where a ping would have come back, which is a round trip after the frame left:
      // it cannot be later than the report's arrival plus the return trip it already counts.
      if (arrival_us != INT64_MIN && t_us > arrival_us + static_cast<int64_t>(rtt_ * 1e6)) {
        return false;
      }
      return queue_sample(t_us, ms, queue, growth, sample_ms);
    }

    bool SignalMeter::queue_sample(int64_t t_us, double ms, double &queue, double &growth, double *sample_ms) {
      // At most one sample per kSampleGapUs, and frames that arrive together —held up behind the same
      // stall— are one sample, not several. At the frame rate, the growth over 50 ms would be read from
      // intervals of a frame, and the swing of one frame's burst against the next became growth.
      if (!raw_.empty() && raw_.back().t_us - t_us > kRetentionUs) {
        // far behind everything kept: the timeline restarted, and the old history would reject every sample
        raw_.clear();
        excursions_.clear();
        noise_center_us_ = t_us - 1;
      }
      if (!raw_.empty() && t_us - raw_.back().t_us <= kSampleGapUs) {
        return false;
      }
      raw_.push_back({t_us, ms});
      if (sample_ms) {
        *sample_ms = ms;
      }
      while (t_us - raw_.front().t_us > kFloorWindowUs) {
        raw_.pop_front();
      }
      update_noise(t_us);
      // The growth is that of the last two intervals between samples, the smaller (or that of the last,
      // with kAlarmIntervals 1), brought to 50 ms
      // with the real time of each: with the queue filling the acks space out —from 25 ms to more
      // than 100— and a fixed time window was left empty right in the ramp. Requiring it in both removes
      // the noise —samples from different acks that jump ten milliseconds up and down— and tells apart
      // a pipe that narrows, which makes the queue grow like a ramp at (R−C)/R, from a path that
      // lengthens, which raises it by a step and leaves it flat.
      double delay_floor = ms;
      for (const Sample &x : raw_) {
        delay_floor = std::min(delay_floor, x.v);
      }
      const size_t n = raw_.size();
      if (n < kGrowthSamples) {
        return false;
      }
      const Sample &a = raw_[n - kGrowthSamples];
      const Sample &b = raw_[n - 2];
      if (t_us <= b.t_us || b.t_us <= a.t_us) {
        return false;
      }
      double v2 = a.v;
      queue = (ms - delay_floor) / 1000.0;
      const double last = (ms - b.v) * kGrowthSpanUs / (t_us - b.t_us);
      if (alarm_intervals_ <= 0) {
        // Over a fixed 50 ms window, from the last sample before: with back-to-back pings the
        // window gathers several, and a loose ack's jump ends up spread out instead of multiplied.
        size_t j = n - 1;
        while (j > 0 && t_us - raw_[j].t_us < kGrowthSpanUs) {
          --j;
        }
        if (t_us - raw_[j].t_us < kGrowthSpanUs) {
          return false;
        }
        v2 = raw_[j].v;
        growth = last > 0 ? (ms - v2) * kGrowthSpanUs / (t_us - raw_[j].t_us) / 1000.0 : 0.0;
      } else {
        growth = (alarm_intervals_ == 1 ? last : std::min(last, (b.v - a.v) * kGrowthSpanUs / (b.t_us - a.t_us))) / 1000.0;
      }
      // And only from a nearly empty queue that was also empty a short while ago: a pipe that narrows in
      // steady state starts with no queue, whereas with a download in the pipe the queue was already there —it builds
      // it— and the ten-second minimum is poisoned at its level, so the queue from before
      // looked low without being so. What the download does not do is let the round trip touch the floor.
      // Empty a short while ago means below half of what is admitted before the step: the loop works
      // with ten or twenty milliseconds of its own queue, and requiring it to touch the floor left the alarm mute
      // just when it was most needed, with the pipe stuck.
      // The queue from before is the one from before the ramp: the minimum of the last kAlarmPriorS. The one from two
      // samples back is already part of the ramp when the acks space out, and it passed the limit at the
      // third sample of a drop, before the measured growth reached the threshold.
      if (prior_us_ > 0) {
        v2 = ms;
        for (size_t i = n - 1; i-- > 0 && t_us - raw_[i].t_us <= prior_us_;) {
          v2 = std::min(v2, raw_[i].v);
        }
      }
      if (ms - delay_floor < 0.5 * kAlarmPriorQueue * 1000.0) {
        t_enet_floor_us_ = t_us;
      }
      if ((v2 - delay_floor) / 1000.0 > kAlarmPriorQueue || t_us - t_enet_floor_us_ > kFloorTouchedUs) {
        growth = 0;
      }
      // Nor in the first seconds, when the floor is not yet known: if another flow was already filling the
      // pipe at startup, the first queue seen looks like a step.
      if (t_us - raw_.front().t_us < kFloorKnownUs) {
        growth = 0;
      }
      return true;
    }

    void SignalMeter::client_ping(int64_t arrival_us, uint32_t client_ms) {
      if (client_ms == 0) {
        return;  // a stock client does not set the mark
      }
      if (!has_ping_ref_) {
        has_ping_ref_ = true;
        ping_ref_us_ = arrival_us;
        ping_ref_ms_ = client_ms;
      }
      // How far the arrival drifts from the client's clock, both measured from the first ping. The client's
      // clock is a 32-bit counter: only its difference is taken, as a signed number, so it wraps cleanly.
      const double drift = (arrival_us - ping_ref_us_) / 1000.0 - static_cast<int32_t>(client_ms - ping_ref_ms_);
      const double window_ms = kRetentionUs / 1000.0;
      // A drift that jumped by more than the whole window is a corrupt mark or a client that restarted: one is
      // dropped, the same one three times in a row is the new clock.
      if (!ping_drifts_.empty() && std::fabs(drift - ping_drifts_.back().v) > window_ms) {
        if (ping_jumps_ > 0 && std::fabs(drift - ping_candidate_) <= window_ms) {
          ++ping_jumps_;
        } else {
          ping_jumps_ = 1;
        }
        ping_candidate_ = drift;
        if (ping_jumps_ < kConfirmReadings) {
          return;
        }
        ping_drifts_.clear();
      }
      ping_jumps_ = 0;
      ping_drifts_.push_back({arrival_us, drift});
    }

    bool SignalMeter::measure(int64_t now_us, Measurement &m) {
      if (rtt_ <= 0 || sent_.empty()) {
        return false;
      }
      const double span = last_call_us_ > 0 ? (now_us - last_call_us_) / 1e6 : kFirstSpanS;
      last_call_us_ = now_us;

      auto push = [&](std::deque<Interval> &v, double bits) {
        if (span > 1e-3) {
          v.push_back({now_us, span, bits});
        }
        while (!v.empty() && now_us - v.front().t_us > kWindowUs) {
          v.pop_front();
        }
      };
      auto rate = [](const std::deque<Interval> &v) {
        double bits = 0, dur = 0;
        for (const Interval &x : v) {
          bits += x.bits;
          dur += x.span_s;
        }
        return dur > 1e-3 ? bits / dur : 0.0;
      };
      auto sum = [](const std::deque<Interval> &v) {
        double s = 0;
        for (const Interval &x : v) {
          s += x.bits;
        }
        return s;
      };

      push(window_tx_, bits_sent_);
      push(window_data_, bits_data_);
      push(window_rx_, bits_arrived_);
      push(window_packets_tx_, packets_tx_);
      push(window_packets_lost_, packets_lost_);

      const double sent_count = sum(window_packets_tx_);
      const double lost = sum(window_packets_lost_);

      m = Measurement {};
      m.t = now_us / 1e6;
      m.rtt = rtt_;
      m.noise = noise_;
      m.loss = sent_count > 0 ? std::min(1.0, lost / sent_count) : 0.0;
      m.sent_rate = rate(window_tx_) + audio_bps_;
      m.data_rate = rate(window_data_);
      // With the loss summary, what arrives is measured by the advance of the last good frame. Without
      // it —a stock client— it can only be estimated as what went out minus what was lost. And
      // it is also estimated if a whole window has gone by without any arriving: the summary travels with no
      // delivery guarantee, and reading its absence as "nothing arrived" made the loop back off over a
      // drop that did not exist. The same while the client waits for a keyframe: the last good
      // frame does not advance even though frames arrive.
      // And after a silence, until the held-up summaries and those written right after it have left the
      // window.
      const bool fresh_summary = had_summary_ && !idr_pending_ &&
                                 now_us - summary_arrival_us_ <= kWindowUs &&
                                 now_us - resumed_us_ >= 2 * kWindowUs;
      m.received_rate = fresh_summary ? rate(window_rx_) + audio_bps_ : m.sent_rate * (1.0 - m.loss);
      if (use_same_frames_ && fresh_summary && t_same_frames_us_ > 0 && now_us - t_same_frames_us_ <= kWindowUs) {
        m.received_rate = same_frames_rate_ + audio_bps_;
        m.delivery = delivery_;
      }

      // ── the queue: ENet's round trip's or the frame age's, the larger ──
      // The age counts the video's queue and also the uplink's, through which the
      // summary comes back; that second one is discounted with the ping's drift, which only travels uplink.
      auto prune_floor = [&](std::deque<Sample> &v) {
        while (!v.empty() && now_us - v.front().t_us > kFloorWindowUs) {
          v.pop_front();
        }
      };
      rtts_.push_back({now_us, rtt_ * 1000.0});
      prune_floor(rtts_);
      prune_floor(ages_);
      prune_floor(ping_drifts_);
      // The round trip without what the uplink's queue added to it. With no arrival time for anything, the
      // delay there is the round trip: the queue is how far it came off its minimum, the same computation
      // the control does with the one-way delay.
      double rtt_min = rtts_.front().v;
      for (const Sample &x : rtts_) {
        rtt_min = std::min(rtt_min, x.v);
      }
      double rise_ms = rtt_ * 1000.0 - rtt_min;
      const double upstream = upstream_ms(now_us);
      if (upstream >= kUplinkShare * rise_ms) {
        rise_ms = std::max(0.0, rise_ms - upstream);
      }
      // ENet smooths the round trip by an eighth per ack, so after a stall that held the acks it stays
      // above the floor for a while after the path is back. The delay of the frames that left since says
      // whether there is a queue: the round trip crosses both ways, so it cannot have risen by more than
      // twice what the video's delay did —a path that lengthens does exactly that—, and a stall, which
      // leaves no queue behind, raises neither.
      prune_floor(delays_);
      if (!delays_.empty() && now_us - delays_.back().t_us <= kDelayFreshUs) {
        double recent = delays_.back().v, delay_min = recent, peak = recent;
        for (const Sample &x : delays_) {
          delay_min = std::min(delay_min, x.v);
          if (delays_.back().t_us - x.t_us <= kDelayPeakUs) {
            peak = std::max(peak, x.v);
          }
          if (delays_.back().t_us - x.t_us <= kDelayRecentUs) {
            recent = std::min(recent, x.v);
          }
        }
        const double allowed = 2.0 * std::max(0.0, recent - delay_min);
        // What the round trip rose and the video did not is a rise of the path that came back by itself:
        // noise, for as long as ENet's smoothing takes to forget it. If the video's delay did rise lately, it
        // is the tail of a queue that emptied, which is not noise even though the cap above already cuts it.
        const double shown = std::max(allowed, 2.0 * std::max(0.0, peak - delay_min));
        if (rise_ms - shown > kStallMinMs) {
          stall_ms_ = rise_ms - shown;
          stall_us_ = now_us;
        }
        rise_ms = std::min(rise_ms, allowed);
      }
      if (now_us - stall_us_ < kStallHoldUs) {
        m.noise = std::max(m.noise, stall_ms_ / 1000.0);
      }
      const double rtt_video = rtt_min / 1000.0 + rise_ms / 1000.0;
      m.owd_min = rtt_video;
      m.owd_mean = rtt_video;
      if (use_age_ && !ages_.empty()) {
        double age_min = ages_.front().v, recent = kNoAge;
        for (const Sample &x : ages_) {
          age_min = std::min(age_min, x.v);
          if (now_us - x.t_us <= age_recent_us_) {
            recent = std::min(recent, x.v);
          }
        }
        double uplink = 0;
        if (!ping_drifts_.empty()) {
          double v_min = ping_drifts_.front().v;
          for (const Sample &x : ping_drifts_) {
            v_min = std::min(v_min, x.v);
          }
          uplink = ping_drifts_.back().v - v_min;
        }
        if (recent < kNoAge) {
          const double queue_age = std::max(0.0, recent - age_min - uplink) / 1000.0;
          const double rtt_queue = std::max(0.0, rtt_video - rtt_min / 1000.0);
          if (queue_age > rtt_queue) {
            m.owd_min = m.owd_mean = rtt_video + (queue_age - rtt_queue);
          }
        }
      }
      // ── the dispersion of each frame (Frame Stats) ────────────────────────────
      // Only the frames the bottleneck spread out: one that arrives as spaced as it left did not
      // cross anything narrower than the sender itself, and its quotient measures the sender. The client counts
      // RTP bytes; the bottleneck also charges the headers, in the proportion of the frame.
      // Except right after a silence: a frame that arrived across the stall spans it, which is not what the
      // bottleneck spread it by, and its arrival report comes in the next few summaries.
      if (now_us - resumed_us_ < kWindowUs) {
        trains_rx_.clear();
      }
      for (const ReceivedTrain &r : trains_rx_) {
        const auto it = sent_.find(r.index);
        if (it == sent_.end() || r.bytes == 0) {
          continue;
        }
        const SentFrame &e = it->second;
        const double wire_ratio = e.rtp_bytes > 0 ? e.wire_bytes / e.rtp_bytes : 1.0;
        const double span = static_cast<uint32_t>(r.t_last_us - r.t_first_us) / 1e6;
        const double send_span = (e.t_last_us - e.t_first_us) / 1e6;
        if (span > kMinDispersion * send_span) {
          m.trains.push_back({r.bytes * wire_ratio, span});
        } else if (send_span > 0 && ((proof_packets_ > 0 && r.packets >= static_cast<uint32_t>(proof_packets_)) || (proof_packets_back_ > 0 && r.packets >= static_cast<uint32_t>(proof_packets_back_)))) {
          // A frame that crossed as tight as it left proves the pipe carries at least its rate: n packets
          // open n - 1 gaps, over the longer of the two spans.
          const double n = r.packets;
          const double proof = r.bytes * wire_ratio * 8.0 * (n - 1.0) / n / std::max(span, send_span);
          if (proof_packets_ > 0 && r.packets >= static_cast<uint32_t>(proof_packets_)) {
            m.proof_bps = std::max(m.proof_bps, proof);
          }
          m.proof_back_bps = std::max(m.proof_back_bps, proof);
        }
      }
      trains_rx_.clear();

      m.parity_sent = data_tx_ > 0 ? (packets_tx_ - data_tx_) / data_tx_ : 0.0;
      // Over the frames that lost something, so loss that falls packet by packet already reads above
      // one (1.6 with 5 % over 22 packets). Taking that out also takes it out of real bursts in a dense
      // loss, which then get less parity than they need: it errs on the side of a frame that is not lost.
      m.loss_burst = frames_with_loss_ > 0 ? lost_in_frames_ / frames_with_loss_ : 0.0;
      if (packets_per_frame_ > 0) {
        m.loss_burst = std::min(m.loss_burst, packets_per_frame_);
      }
      m.samples = static_cast<int>(packets_tx_);

      bits_sent_ = bits_data_ = bits_arrived_ = 0;
      packets_tx_ = data_tx_ = packets_lost_ = 0;
      frames_with_loss_ = lost_in_frames_ = 0;
      return true;
    }

  }  // namespace gamestream
}  // namespace abr
