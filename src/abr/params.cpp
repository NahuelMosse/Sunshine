#include "params.h"

#include <cstdlib>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace abr {
namespace {

// A flat TOML is enough: key = value, one per line, # for comments. I prefer thirty
// lines of my own to a dependency that then has to be dragged all the way to Sunshine.
struct Field {
  const char* name;
  enum { D, I } kind;
  size_t offset;
};

#define FIELD_D(n) { #n, Field::D, offsetof(Params, n) }
#define FIELD_I(n) { #n, Field::I, offsetof(Params, n) }

const std::vector<Field>& table() {
  static const std::vector<Field> t = {
      FIELD_D(Tc), FIELD_D(fps),
      FIELD_D(filter_slow), FIELD_D(filter_fast), FIELD_D(filter_rtt_cutoff),
      FIELD_I(line_fit_samples),
      FIELD_D(dref_floor), FIELD_D(dref_fraction), FIELD_D(dref_noise), FIELD_D(noise_window), FIELD_D(noise_filter), FIELD_D(drain_episode), FIELD_D(drain_ramp), FIELD_D(drain_deadline),FIELD_I(trip_percentile_dref), FIELD_I(tau_from_floor),
      FIELD_D(delay_floor_window), FIELD_D(capacity_window),
      FIELD_I(capacity_percentile), FIELD_I(trip_percentile),
      FIELD_D(capacity_time_const), FIELD_D(egress_margin), FIELD_D(cross_s), FIELD_D(cross_max_s), FIELD_D(cross_tolerance), FIELD_D(delivery_full), FIELD_I(ceiling_anti_windup), FIELD_I(queue_age), FIELD_D(competitor_threshold), FIELD_I(competitor_percentile), FIELD_D(competitor_sustained), FIELD_D(competitor_dref), FIELD_D(competitor_loss_s), FIELD_I(capacity_sustained), FIELD_D(encoder_release_loss), FIELD_D(age_cadence),FIELD_D(age_recent),FIELD_D(egress_time_const),FIELD_D(ceiling_over_C),      FIELD_D(coh_min_drop), FIELD_D(coh_flat), FIELD_D(coh_margin),
      FIELD_I(coh_cycles),
      FIELD_D(rho), FIELD_D(phase_margin),
      FIELD_D(backoff_factor), FIELD_D(backoff_loss), FIELD_D(backoff_queue_margin), FIELD_D(fast_backoff), FIELD_D(fast_backoff_queue), FIELD_I(fast_backoff_cycles), FIELD_D(congestion_loss), FIELD_D(congestion_loss_s), FIELD_D(blind_s), FIELD_D(backoff_step), FIELD_D(backoff_step_factor), FIELD_D(alarm_step), FIELD_D(alarm_queue), FIELD_D(alarm_prior_queue), FIELD_I(alarm_intervals), FIELD_D(alarm_prior_s),FIELD_D(alarm_retract_s), FIELD_D(alarm_factor), FIELD_D(alarm_drain), FIELD_I(keyframe_on_drop), FIELD_I(frames_in_flight), FIELD_I(alarm_frame_divisor),FIELD_D(delivered_floor), FIELD_I(backoff_trains),
      FIELD_D(ramp_duration), FIELD_D(ramp_exploration), FIELD_D(ramp_convex), FIELD_D(fast_startup), FIELD_D(cruise_gain), FIELD_D(cruise_queue), FIELD_D(cruise_wait), FIELD_D(reexplore_s), FIELD_I(queue_requires_full),
      FIELD_D(anchor_collapse), FIELD_D(anchor_blend),
      FIELD_D(fec_min), FIELD_D(fec_max), FIELD_D(fec_rise),
      FIELD_D(fec_fall_tau), FIELD_D(fec_z), FIELD_D(expected_blocks),
      FIELD_D(probe_interval), FIELD_D(probe_duration), FIELD_D(probe_cut),
      FIELD_D(utilization), FIELD_D(app_limited_threshold), FIELD_D(train_pacing),
      FIELD_D(train_utilization), FIELD_D(train_window), FIELD_I(train_envelope_iters), FIELD_D(train_min_packets),      FIELD_D(train_rise_max), FIELD_D(train_anchor), FIELD_D(train_anchor_wait), FIELD_D(train_expiry),
      FIELD_D(encoder_compensation_max), FIELD_D(encoder_time_const), FIELD_I(encoder_release_cycles),      FIELD_D(bitrate_min), FIELD_D(bitrate_max), FIELD_D(startup_fraction),
      FIELD_I(mtu), FIELD_I(header_bytes), FIELD_D(audio_bps),
      FIELD_I(gf_shards), FIELD_I(blocks_per_frame), FIELD_I(fec_min_packets), FIELD_I(fec_min_clean), FIELD_D(fec_clean_s),
      FIELD_I(parity_in_packets), FIELD_D(parity_block_failure), FIELD_D(parity_repair_failure), FIELD_D(fec_max_repair), FIELD_D(parity_memory), FIELD_D(parity_burst), FIELD_D(parity_startup), FIELD_D(parity_startup_s), FIELD_D(enet_ping_ms),  };
  return t;
}

std::string trim(std::string s) {
  const auto a = s.find_first_not_of(" \t\r\n");
  if (a == std::string::npos) return "";
  const auto b = s.find_last_not_of(" \t\r\n");
  return s.substr(a, b - a + 1);
}

}  // namespace

std::string Params::load(const std::string& file) {
  std::ifstream f(file);
  if (!f) return "cannot open " + file;

  std::string line;
  int n = 0;
  while (std::getline(f, line)) {
    ++n;
    const auto h = line.find('#');
    if (h != std::string::npos) line = line.substr(0, h);
    line = trim(line);
    if (line.empty() || line[0] == '[') continue;

    const auto eq = line.find('=');
    if (eq == std::string::npos) {
      return "line " + std::to_string(n) + ": missing =";
    }
    const std::string key = trim(line.substr(0, eq));
    const std::string value = trim(line.substr(eq + 1));
    if (!set(key, value)) {
      return "line " + std::to_string(n) + (key == "variant" ? ": unknown variant '" + value + "'"
                                                              : ": unknown parameter '" + key + "'");
    }
  }
  return "";
}

// An unknown parameter is an error, not something to ignore: it is almost always a
// misspelled name, and silencing it makes the run use a value nobody set.
bool Params::set(const std::string& key, const std::string& value) {
  if (key == "variant") {
    try { variant = variant_by_name(value); } catch (const std::invalid_argument&) { return false; }
    return true;
  }
  for (const auto& c : table()) {
    if (key != c.name) continue;
    auto* base = reinterpret_cast<char*>(this);
    if (c.kind == Field::D) {
      *reinterpret_cast<double*>(base + c.offset) = std::strtod(value.c_str(), nullptr);
    } else {
      *reinterpret_cast<int*>(base + c.offset) =
          static_cast<int>(std::strtol(value.c_str(), nullptr, 10));
    }
    return true;
  }
  return false;
}

std::string Params::dump() const {
  std::ostringstream o;
  o.precision(10);
  for (const auto& c : table()) {
    const auto* base = reinterpret_cast<const char*>(this);
    o << c.name << " = ";
    if (c.kind == Field::D) o << *reinterpret_cast<const double*>(base + c.offset);
    else                    o << *reinterpret_cast<const int*>(base + c.offset);
    o << "\n";
  }
  return o.str();
}

}  // namespace abr
