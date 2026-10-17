#include "signals.h"

#include <algorithm>
#include <cstdlib>

namespace abr {
namespace gamestream {
namespace {

// How long what was sent is kept: it has to cover with room to spare the round trip plus the cadence of the
// loss summary, which is when it crosses with what arrived.
constexpr int64_t kRetentionUs = 3000000;

// The window of the rates and the loss. The summary arrives every 50 ms and the FEC status
// only with loss: a quarter of a second gathers several of each.
constexpr int64_t kWindowUs = 250000;

// Over how long the delivery is measured: with the quantization of one frame at each end, 300 ms leaves
// an error of about 2 %.
constexpr int64_t kDeliverySpanUs = 300000;

// A summary this long after the previous one was held up on its way, and whatever it carries
// piled up while the path was silent: it says how far the last good frame advanced, not how fast.
constexpr int64_t kSilenceUs = 250000;

// How close a client ping has to arrive to an ENet sample to say how much of that round trip is
// the uplink's queue: a ping every 100 ms and an ack every 25 ms, plus the spread of one that was held.
constexpr int64_t kPingPairUs = 150000;

// What share of a round trip's rise the ping's drift has to account for to call it the uplink's queue
constexpr double kUplinkShare = 0.75;

// The minima of the age, of the ping drift and of the round trip, like the control's floor.
constexpr int64_t kFloorWindowUs = 10000000;

// How much longer a frame has to take to arrive than to leave for its dispersion to be that of the
// bottleneck and not the sender's.
constexpr double kMinDispersion = 1.1;

}  // namespace

void SignalMeter::sent(const SentFrame& c) {
  sent_[c.index] = c;
  bits_sent_ += c.wire_bytes * 8.0;
  if (c.packets > 0) bits_data_ += c.wire_bytes * 8.0 * c.data_packets / c.packets;
  packets_tx_ += c.packets;
  data_tx_ += c.data_packets;
  packets_per_frame_ = packets_per_frame_ > 0
                             ? packets_per_frame_ + 0.05 * (c.packets - packets_per_frame_)
                             : c.packets;
  for (auto it = sent_.begin(); it != sent_.end();) {
    if (c.t_first_us - it->second.t_first_us > kRetentionUs) it = sent_.erase(it);
    else break;
  }
}

void SignalMeter::fec_status(const FecStatus& e) {
  const int total = e.data_total + e.parity_total;
  const int received_n = e.data_received + e.parity_received;
  // Reassembled, what is missing up to the total is the parity that did not turn out to be needed: what is lost is
  // the gaps. Counting up to the total always gives the block's parity: the burst looks like two to
  // four packets when the channel's is one and a bit, and the parity goes to its ceiling.
  const int missing_n = received_n >= e.data_total && e.missing >= 0
                         ? std::max(e.missing, e.data_total - e.data_received)
                         : std::max(0, total - received_n);
  if (missing_n > 0) {
    packets_lost_ += missing_n;
    lost_in_frames_ += missing_n;
    frames_with_loss_ += 1;
  }
  // Reed-Solomon reassembles from any k of the n: if fewer arrived than the data,
  // the frame was lost even if part of it arrived
  if (received_n < e.data_total) ++failed_with_status_;
}

void SignalMeter::summary(int64_t arrival_us, const LossSummary& r) {
  if (!had_summary_) {
    had_summary_ = true;
    last_good_ = r.last_good;
    summary_arrival_us_ = arrival_us;
    failed_with_status_ = 0;
    return;
  }
  // After a silence the summaries that follow are the held-up ones arriving together: what was
  // delivered cannot be read from their spacing until a whole window of normal ones has gone by.
  const bool resumed = arrival_us - summary_arrival_us_ > kSilenceUs;
  if (resumed) {
    summaries_.clear();
    window_rx_.clear();
    bits_arrived_ = 0;
    resumed_us_ = arrival_us;
  }
  summary_arrival_us_ = arrival_us;
  if (r.last_good <= last_good_) return;         // no advance, or an old summary

  // What arrived since the previous summary is the frames up to the last good one, minus those
  // lost on the way. The host knows the sizes.
  double bytes = 0;
  int known = 0;
  for (auto it = sent_.upper_bound(last_good_);
       it != sent_.end() && it->first <= r.last_good; ++it) {
    bytes += it->second.wire_bytes;
    ++known;
  }
  // If the requested keyframe arrived in the interval, the lost ones the summary counts are those
  // the client discarded while waiting for it, which arrived whole. The loss that led to the request was
  // already reported by the FEC status of the frame that could not be reassembled.
  const bool discards = idr_pending_ && r.last_good >= idr_from_;
  if (discards) idr_pending_ = false;
  // And the client counts as lost every index that did not reach it, including those the host never sent
  // —the ones Sunshine drops when its encoder queue fills up—. Those are not network loss:
  // they are those of the interval the host has no record of.
  const double not_sent = std::max(0.0, static_cast<double>(r.last_good - last_good_) - known);
  const uint32_t lost = discards ? 0
      : static_cast<uint32_t>(std::max(0.0, static_cast<double>(r.lost) - not_sent));

  const double per_frame = known > 0 ? bytes / known : 0.0;
  if (!resumed) bits_arrived_ += std::max(0.0, bytes - lost * per_frame) * 8.0;

  // Frames lost whole send no FEC status: no packet arrived that the client
  // could talk about. They are the lost ones the summary counts and the status does not explain.
  const int whole = std::max(0, static_cast<int>(lost) - failed_with_status_);
  packets_lost_ += whole * packets_per_frame_;
  failed_with_status_ = 0;
  last_good_ = r.last_good;
  lost_total_ += lost;

  // ── what was delivered, over the same frames ─────────────────────────────
  // Comparing what arrives now with what left a trip earlier compares different frames, and the
  // encoder varies from one frame to another considerably more than the pipe: with real Sunshine, the
  // ratio between the two rates varied between 9 and 17 % in steady state, and that triggered backoffs
  // with the queue empty. Here the frames the last good one advanced over at least 300 ms are taken, and
  // what they weighed is divided by the greater of how long they took to leave and how long they took to
  // arrive: BBR's delivery rate estimate (draft-cheng-iccrg-delivery-rate-estimation).
  // How much slower they arrived than they left says whether the pipe is slowing them down.
  summaries_.push_back({arrival_us, r.last_good, lost_total_});
  while (summaries_.size() > 2 && arrival_us - summaries_[1].arrival_us >= kDeliverySpanUs) summaries_.pop_front();
  const Summary& from = summaries_.front();
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
      if (n > 0) bytes -= std::min(static_cast<double>(n), lost_total_ - from.lost) * bytes / n;
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

bool SignalMeter::plausible(double ms, std::deque<Sample>& history, int& streak) {
  if (history.empty()) return true;
  double delay_floor = history.front().v;
  for (const Sample& x : history) delay_floor = std::min(delay_floor, x.v);
  if (ms >= 0.5 * delay_floor) { streak = 0; return true; }
  if (++streak < 3) return false;
  // Three in a row: the path really got shorter, and the old floor is no longer any use
  streak = 0;
  history.clear();
  return true;
}

void SignalMeter::rtt(double seconds) {
  if (seconds <= 0 || !plausible(seconds * 1000.0, rtts_, rtt_low_)) return;
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
    const Sample& x = raw_[c];
    if (x.t_us <= noise_center_us_) continue;
    if (t_us - x.t_us < kNoiseSideUs) break;           // the after is not visible yet
    double before = x.v, after = x.v;
    bool has_before = false, has_after = false;
    for (size_t j = 0; j < raw_.size(); ++j) {
      const Sample& y = raw_[j];
      if (y.t_us < x.t_us && x.t_us - y.t_us <= kNoiseSideUs) { before = has_before ? std::min(before, y.v) : y.v; has_before = true; }
      if (y.t_us > x.t_us && y.t_us - x.t_us <= kNoiseSideUs) { after = has_after ? std::min(after, y.v) : y.v; has_after = true; }
    }
    noise_center_us_ = x.t_us;
    if (!has_before || !has_after) continue;
    excursions_.push_back({x.t_us, std::max(0.0, x.v - std::max(before, after)) / 1000.0});
  }
  while (!excursions_.empty() && t_us - excursions_.front().t_us > kNoiseWindowUs) excursions_.pop_front();
  if (excursions_.size() < 20) { noise_ = 0; return; }
  std::vector<double> v, jumps;
  v.reserve(excursions_.size());
  for (const Sample& e : excursions_) { v.push_back(e.v); if (e.v > kJumpMin) jumps.push_back(e.v); }
  if (jumps.size() >= kJumpsMin) v.swap(jumps);
  const size_t k = static_cast<size_t>(0.9 * (v.size() - 1));
  std::nth_element(v.begin(), v.begin() + static_cast<long>(k), v.end());
  noise_ = v[k];
}

// What the host put on the wire between two of its times: the frames, spread over how long they took
// to leave, and the audio, which goes out evenly.
double SignalMeter::bits_between(int64_t from_us, int64_t to_us) const {
  if (to_us <= from_us) return 0;
  double bits = audio_bps_ * (to_us - from_us) / 1e6;
  for (const auto& [index, c] : sent_) {
    (void)index;
    if (c.t_last_us < from_us) continue;
    if (c.t_first_us > to_us) break;
    const int64_t duration = c.t_last_us - c.t_first_us;
    const double share = duration <= 0 ? 1.0
        : static_cast<double>(std::min(to_us, c.t_last_us) - std::max(from_us, c.t_first_us)) / duration;
    bits += c.wire_bytes * 8.0 * std::max(0.0, share);
  }
  return bits;
}

// The pipe that empties a queue, measured with ENet's ping, which makes the same queue as the video. A
// FIFO queue does not stop serving while it has something: between two pings that found it full,
// everything the host put on the wire between one leaving and the other came out, not a byte more or less.
// How long they took to come back is how long that took to leave. It is BBR's delivery estimate
// (draft-cheng-iccrg-delivery-rate-estimation): with a queue, what is delivered per unit of time is the
// bottleneck. It does not depend on the frame size, so it works just as well with one-packet
// frames, which have no dispersion to measure.
//
// It is measured from the peak until the queue returns to the floor, because that is what says how the pipe ended up:
// when the air cuts out, while the queue grows the pipe delivers almost nothing, and what empties it is the
// usual pipe; when the pipe narrows, what empties it is the new pipe. If in the middle the queue
// emptied for an instant, the span adds time without delivering and the measure comes out low: being wrong is
// falling short. And with another flow in the queue it comes out as the video's part, also short.
void SignalMeter::track_drain(int64_t t_us, double ms, double delay_floor) {
  constexpr double kFullMs = 20, kEmptyMs = 5;        // when there is a queue and when there no longer is
  constexpr int64_t kEpisodeMaxUs = 3000000;
  const double queue_ms = ms - delay_floor;
  const int64_t out_us = t_us - static_cast<int64_t>(ms * 1000.0);
  if (!in_queue_) {
    if (queue_ms < kFullMs) return;
    in_queue_ = true;
    queue_since_us_ = out_us;
    queue_since_ms_ = ms;
    descent_.assign(1, {t_us, out_us, ms});
    return;
  }
  if (t_us - queue_since_us_ > kEpisodeMaxUs) { in_queue_ = false; descent_.clear(); return; }
  if (queue_ms >= kEmptyMs) {
    // Since the last peak: what still rises is the queue filling up. A couple of milliseconds more
    // are the ack's swing, not the queue.
    if (ms >= descent_.front().ms || ms > descent_.back().ms + 2) descent_.clear();
    descent_.push_back({t_us, out_us, ms});
    return;
  }
  in_queue_ = false;
  if (descent_.size() >= 3 && descent_.back().t_us - descent_.front().t_us >= 30000) {
    // The line of what was sent against the return time: its slope is what the pipe delivered
    const Descent& a = descent_.front();
    double st = 0, sb = 0;
    std::vector<double> b(descent_.size());
    for (size_t i = 0; i < descent_.size(); ++i) {
      b[i] = bits_between(a.out_us, descent_[i].out_us);
      st += (descent_[i].t_us - a.t_us) / 1e6;
      sb += b[i];
    }
    const double n = static_cast<double>(descent_.size()), mt = st / n, mb = sb / n;
    double num = 0, den = 0;
    for (size_t i = 0; i < descent_.size(); ++i) {
      const double x = (descent_[i].t_us - a.t_us) / 1e6 - mt;
      num += x * (b[i] - mb);
      den += x * x;
    }
    if (den > 0 && num > 0) {
      drain_rate_ = num / den;
      drain_start_us_ = queue_since_us_;
      drain_peak_us_ = a.out_us;
      drain_height_ = (a.ms - queue_since_ms_) / 1000.0;
    }
  }
  descent_.clear();
}

// How much of the round trip of what arrives around t_us is the uplink's queue: the oldest ping that
// arrived close to it, over its ten-second minimum. Pings held up behind the same stall arrive
// together, and the oldest of them is the one that measures it.
double SignalMeter::upstream_ms(int64_t t_us) const {
  double v_min = 0, near_max = 0;
  bool first = true, near = false;
  for (const Sample& x : ping_drifts_) {
    v_min = first ? x.v : std::min(v_min, x.v);
    first = false;
    if (std::llabs(t_us - x.t_us) > kPingPairUs) continue;
    near_max = near ? std::max(near_max, x.v) : x.v;
    near = true;
  }
  return near ? std::max(0.0, near_max - v_min) : 0.0;
}

bool SignalMeter::enet_sample(int64_t t_us, double ms, double& queue, double& growth, double* accepted_ms) {
  if (accepted_ms) *accepted_ms = 0;
  if (ms <= 0 || !plausible(ms, raw_, enet_low_)) return false;
  // The host reads the same sample several times between two acks; one equal to the previous says
  // nothing the alarm needs —it only matters when the queue grows— and counting it twice put in
  // growth intervals of zero right in the middle of the ramp.
  if (!raw_.empty() && last_raw_ms_ == ms) return false;
  last_raw_ms_ = ms;
  // The ENet round trip crosses both ways, and the ping's drift says how much of what it rose is the
  // uplink's queue, which is not the video's: the sample is stored without it, so the floor, the
  // noise and the draining are those of the video's path alone. Never more than what the round trip
  // rose over its floor, and only if the drift accounts for most of that rise: a path that lengthens
  // adds the same to both directions, so its ping drifts by half of what the round trip rose, and
  // that is a step in the path, not a queue in the uplink.
  {
    double floor_ms = ms;
    for (const Sample& x : raw_) floor_ms = std::min(floor_ms, x.v);
    const double rise = ms - floor_ms, upstream = upstream_ms(t_us);
    if (upstream >= kUplinkShare * rise) ms -= std::min(upstream, rise);
  }
  if (accepted_ms) *accepted_ms = ms;
  raw_.push_back({t_us, ms});
  while (t_us - raw_.front().t_us > kFloorWindowUs) raw_.pop_front();
  update_noise(t_us);
  // The growth is that of the last two intervals between samples, the smaller (or that of the last,
  // with alarm_intervals 1), brought to 50 ms
  // with the real time of each: with the queue filling the acks space out —from 25 ms to more
  // than 100— and a fixed time window was left empty right in the ramp. Requiring it in both removes
  // the noise —samples from different acks that jump ten milliseconds up and down— and tells apart
  // a pipe that narrows, which makes the queue grow like a ramp at (R−C)/R, from a path that
  // lengthens, which raises it by a step and leaves it flat.
  double delay_floor = ms;
  for (const Sample& x : raw_) delay_floor = std::min(delay_floor, x.v);
  track_drain(t_us, ms, delay_floor);
  const size_t n = raw_.size();
  if (n < 3) return false;
  const Sample& a = raw_[n - 3];
  const Sample& b = raw_[n - 2];
  if (t_us <= b.t_us || b.t_us <= a.t_us) return false;
  double v2 = a.v;
  queue = (ms - delay_floor) / 1000.0;
  const double last = (ms - b.v) * 50000.0 / (t_us - b.t_us);
  if (alarm_intervals_ <= 0) {
    // Over a fixed 50 ms window, from the last sample before: with back-to-back pings the
    // window gathers several, and a loose ack's jump ends up spread out instead of multiplied.
    size_t j = n - 1;
    while (j > 0 && t_us - raw_[j].t_us < 50000) --j;
    if (t_us - raw_[j].t_us < 50000) return false;
    v2 = raw_[j].v;
    growth = last > 0 ? (ms - v2) * 50000.0 / (t_us - raw_[j].t_us) / 1000.0 : 0.0;
  } else {
    growth = (alarm_intervals_ == 1 ? last : std::min(last, (b.v - a.v) * 50000.0 / (b.t_us - a.t_us))) / 1000.0;
  }
  // And only from a nearly empty queue that was also empty a short while ago: a pipe that narrows in
  // steady state starts with no queue, whereas with a download in the pipe the queue was already there —it builds
  // it— and the ten-second minimum is poisoned at its level, so the queue from before
  // looked low without being so. What the download does not do is let the round trip touch the floor.
  // Empty a short while ago means below half of what is admitted before the step: the loop works
  // with ten or twenty milliseconds of its own queue, and requiring it to touch the floor left the alarm mute
  // just when it was most needed, with the pipe stuck.
  // The queue from before is the one from before the ramp: the minimum of the last alarm_prior_s. The one from two
  // samples back is already part of the ramp when the acks space out, and it passed the limit at the
  // third sample of a drop, before the measured growth reached the threshold.
  if (prior_us_ > 0) {
    v2 = ms;
    for (size_t i = n - 1; i-- > 0 && t_us - raw_[i].t_us <= prior_us_;) v2 = std::min(v2, raw_[i].v);
  }
  if (ms - delay_floor < 0.5 * prior_queue_max_ * 1000.0) t_enet_floor_us_ = t_us;
  if ((v2 - delay_floor) / 1000.0 > prior_queue_max_ || t_us - t_enet_floor_us_ > 5000000) growth = 0;
  // Nor in the first seconds, when the floor is not yet known: if another flow was already filling the
  // pipe at startup, the first queue seen looks like a step.
  if (t_us - raw_.front().t_us < 3000000) growth = 0;
  return true;
}

void SignalMeter::client_ping(int64_t arrival_us, uint32_t client_ms) {
  if (client_ms == 0) return;    // a stock client does not set the mark
  ping_drifts_.push_back({arrival_us, arrival_us / 1000.0 - client_ms});
}

bool SignalMeter::measure(int64_t now_us, Measurement& m) {
  if (rtt_ <= 0 || sent_.empty()) return false;
  const double span = last_call_us_ > 0 ? (now_us - last_call_us_) / 1e6 : 0.1;
  last_call_us_ = now_us;

  auto push = [&](std::deque<Interval>& v, double bits) {
    if (span > 1e-3) v.push_back({now_us, span, bits});
    while (!v.empty() && now_us - v.front().t_us > kWindowUs) v.pop_front();
  };
  auto rate = [](const std::deque<Interval>& v) {
    double bits = 0, dur = 0;
    for (const Interval& x : v) { bits += x.bits; dur += x.span_s; }
    return dur > 1e-3 ? bits / dur : 0.0;
  };
  auto sum = [](const std::deque<Interval>& v) {
    double s = 0;
    for (const Interval& x : v) s += x.bits;
    return s;
  };

  push(window_tx_, bits_sent_);
  push(window_data_, bits_data_);
  push(window_rx_, bits_arrived_);
  push(window_packets_tx_, packets_tx_);
  push(window_packets_lost_, packets_lost_);

  const double sent_count = sum(window_packets_tx_);
  const double lost = sum(window_packets_lost_);

  m = Measurement{};
  m.t = now_us / 1e6;
  m.rtt = rtt_;
  m.noise = noise_;
  if (drain_rate_ > 0) {
    m.drain_rate = drain_rate_;
    m.drain_start = drain_start_us_ / 1e6;
    m.drain_rise = std::max<int64_t>(0, drain_peak_us_ - drain_start_us_) / 1e6;
    m.drain_height = std::max(0.0, drain_height_);
    drain_rate_ = 0;
  }
  m.loss = sent_count > 0 ? std::min(1.0, lost / sent_count) : 0.0;
  m.sent_rate = rate(window_tx_) + audio_bps_;
  m.data_rate = rate(window_data_);
  // With the loss summary, what arrives is measured by the advance of the last good frame. Without
  // it —a stock client— it can only be estimated as what went out minus what was lost. And
  // it is also estimated if a whole window has gone by without any arriving: the summary travels with no
  // delivery guarantee, and reading its absence as "nothing arrived" made the loop back off over a
  // drop that did not exist. The same while the client waits for a keyframe: the last good
  // frame does not advance even though frames arrive.
  // And after a silence, until the held-up summaries have left the window.
  const bool fresh_summary = had_summary_ && !idr_pending_ &&
                              now_us - summary_arrival_us_ <= kWindowUs &&
                              now_us - resumed_us_ >= kWindowUs;
  m.received_rate = fresh_summary ? rate(window_rx_) + audio_bps_
                                   : m.sent_rate * (1.0 - m.loss);
  if (use_same_frames_ && fresh_summary && t_same_frames_us_ > 0 && now_us - t_same_frames_us_ <= kWindowUs) {
    m.received_rate = same_frames_rate_ + audio_bps_;
    m.delivery = delivery_;
  }

  // ── the queue: ENet's round trip's or the frame age's, the larger ──
  // The age counts the video's queue and also the uplink's, through which the
  // summary comes back; that second one is discounted with the ping's drift, which only travels uplink.
  auto prune_floor = [&](std::deque<Sample>& v) {
    while (!v.empty() && now_us - v.front().t_us > kFloorWindowUs) v.pop_front();
  };
  rtts_.push_back({now_us, rtt_ * 1000.0});
  prune_floor(rtts_);
  prune_floor(ages_);
  prune_floor(ping_drifts_);
  // The round trip without what the uplink's queue added to it. With no arrival time for anything, the
  // delay there is the round trip: the queue is how far it came off its minimum, the same computation
  // the control does with the one-way delay.
  double rtt_min = rtts_.front().v;
  for (const Sample& x : rtts_) rtt_min = std::min(rtt_min, x.v);
  const double rise_ms = rtt_ * 1000.0 - rtt_min, upstream = upstream_ms(now_us);
  const double rtt_video = upstream >= kUplinkShare * rise_ms
      ? rtt_min / 1000.0 + std::max(0.0, rise_ms - upstream) / 1000.0 : rtt_;
  m.owd_min = rtt_video;
  m.owd_mean = rtt_video;
  if (use_age_ && !ages_.empty()) {
    double age_min = ages_.front().v, recent = 1e18;
    for (const Sample& x : ages_) {
      age_min = std::min(age_min, x.v);
      if (now_us - x.t_us <= age_recent_us_) recent = std::min(recent, x.v);
    }
    double uplink = 0;
    if (!ping_drifts_.empty()) {
      double v_min = ping_drifts_.front().v;
      for (const Sample& x : ping_drifts_) v_min = std::min(v_min, x.v);
      uplink = ping_drifts_.back().v - v_min;
    }
    if (recent < 1e17) {
      const double queue_age = std::max(0.0, recent - age_min - uplink) / 1000.0;
      const double rtt_queue = std::max(0.0, rtt_video - rtt_min / 1000.0);
      if (queue_age > rtt_queue) m.owd_min = m.owd_mean = rtt_video + (queue_age - rtt_queue);
    }
  }
  // ── the dispersion of each frame (Frame Stats) ────────────────────────────
  // Only the frames the bottleneck spread out: one that arrives as spaced as it left did not
  // cross anything narrower than the sender itself, and its quotient measures the sender. The client counts
  // RTP bytes; the bottleneck also charges the headers, in the proportion of the frame.
  // Except right after a silence: a frame that arrived across the stall spans it, which is not what the
  // bottleneck spread it by, and its arrival report comes in the next few summaries.
  if (now_us - resumed_us_ < kWindowUs) trains_rx_.clear();
  for (const ReceivedTrain& r : trains_rx_) {
    const auto it = sent_.find(r.index);
    if (it == sent_.end() || r.bytes == 0) continue;
    const SentFrame& e = it->second;
    const double wire_ratio = e.rtp_bytes > 0 ? e.wire_bytes / e.rtp_bytes : 1.0;
    const double span = static_cast<uint32_t>(r.t_last_us - r.t_first_us) / 1e6;
    const double send_span = (e.t_last_us - e.t_first_us) / 1e6;
    if (span > kMinDispersion * send_span) m.trains.push_back({r.bytes * wire_ratio, span});
  }
  trains_rx_.clear();

  m.parity_sent = data_tx_ > 0 ? (packets_tx_ - data_tx_) / data_tx_ : 0.0;
  m.loss_burst = frames_with_loss_ > 0 ? lost_in_frames_ / frames_with_loss_ : 0.0;
  m.samples = static_cast<int>(packets_tx_);

  bits_sent_ = bits_data_ = bits_arrived_ = 0;
  packets_tx_ = data_tx_ = packets_lost_ = 0;
  frames_with_loss_ = lost_in_frames_ = 0;
  return true;
}

}  // namespace gamestream
}  // namespace abr
