// Native tests for the timed A/B segment append (Issue #15).
//
// FakeAxis models the parts of FastAccelStepper 2.0.0 that the feeder depends
// on: moveTimed() command splitting and capacity checks (fas_member/
// move_timed.h), direction-change pauses (pd_esp32/esp32_queue.h) and a queue
// of 32 entries that drains in real time. It does not model the MCPWM/PCNT
// driver, so these tests cannot show a driver stall on the real machine.
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <iostream>

#include "MotionBlock.h"
#include "PlotterConfig.h"
#include "SegmentGenerator.h"
#include "SegmentQueue.h"
#include "TimedSegmentFeeder.h"

namespace {

constexpr uint32_t kTicksPerS = 16000000;
constexpr uint32_t kTicksPerMs = kTicksPerS / 1000;
constexpr uint32_t kTicksPerUs = kTicksPerS / 1000000;
constexpr uint16_t kQueueLen = 32;
constexpr uint32_t kMinCmdTicks = kTicksPerS / 5000;
constexpr uint32_t kDirChangeDelayTicks = DIR_CHANGE_DELAY_US * kTicksPerUs;

constexpr int8_t kOk = 0;
constexpr int8_t kDeviceNotReady = 4;
constexpr int8_t kDirPin2msPauseAdded = 5;
constexpr int8_t kDirChangePauseInjected = 6;
constexpr int8_t kMoveBusy = 7;
constexpr int8_t kMoveEmpty = 8;
constexpr int8_t kTicksTooLow = -1;
constexpr int8_t kTooLarge = -4;

void require(bool condition, const char* message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << "\n";
    std::exit(1);
  }
}

class FakeAxis {
 public:
  struct Entry {
    uint32_t ticks;
    uint8_t steps;
    bool count_up;
  };

  int8_t moveTimed(int16_t steps, uint32_t duration, uint32_t* actual, bool start) {
    ++calls_;
    if (forced_results_left_ > 0) {
      --forced_results_left_;
      if (actual != nullptr) *actual = 0;
      return forced_result_;
    }
    const int8_t ret_ok = queue_.empty() ? kMoveEmpty : kOk;
    if (steps == 0 && duration == 0) {
      if (start && !queue_.empty() && !not_ready_) running_ = true;
      return ret_ok;
    }
    if (not_ready_) return kDeviceNotReady;
    const uint32_t free_entries = kQueueLen - queue_.size();
    if (actual != nullptr) *actual = 0;
    if (steps == 0) {
      if ((duration >> 16) >= kQueueLen) return kTooLarge;
      if ((duration >> 16) >= free_entries) return kMoveBusy;
      while (duration > 0) {
        uint32_t ticks = duration;
        if (duration >= 131072) {
          ticks = 65535;
        } else if (duration > 65535) {
          ticks = duration >> 1;
        }
        const int8_t result = addEntry({ticks, 0, queue_end_count_up_}, start, actual);
        if (result != kOk) return result;
        duration -= ticks;
      }
      return ret_ok;
    }
    const bool count_up = steps > 0;
    uint32_t count = static_cast<uint32_t>(steps < 0 ? -steps : steps);
    const uint32_t rate = duration / count;
    if (rate > 65535) {
      const uint32_t commands_per_step = (rate >> 16) + 1;
      if (commands_per_step >= kQueueLen || count >= kQueueLen) return kTooLarge;
      const uint32_t commands = count * commands_per_step;
      if (commands >= kQueueLen) return kTooLarge;
      if (commands > free_entries) return kMoveBusy;
      for (uint32_t step = 0; step < count; ++step) {
        uint32_t remaining = rate;
        uint8_t entry_steps = 1;
        while (remaining > 0) {
          uint32_t ticks = remaining;
          if (remaining >= 131072) {
            ticks = 65535;
          } else if (remaining > 65535) {
            ticks = remaining / 2;
          }
          remaining -= ticks;
          const int8_t result = addEntry({ticks, entry_steps, count_up}, start, actual);
          if (result != kOk) return result;
          entry_steps = 0;
        }
      }
      return ret_ok;
    }
    if (count > kQueueLen * 255U) return kTooLarge;
    if (count > free_entries * 255U) return kMoveBusy;
    uint32_t ticks = rate;
    int16_t missing = static_cast<int16_t>(duration - rate * count);
    while (count > 0) {
      uint32_t entry_steps = count;
      if (count > 510) {
        entry_steps = 255;
      } else if (count > 255) {
        entry_steps = count / 2;
      }
      if (static_cast<int32_t>(count) <= missing) {
        ++ticks;
        missing = 0;
      }
      const int8_t result =
          addEntry({ticks, static_cast<uint8_t>(entry_steps), count_up}, start, actual);
      if (result != kOk) return result;
      count -= entry_steps;
    }
    return ret_ok;
  }

  uint16_t queueEntries() const { return static_cast<uint16_t>(queue_.size()); }
  bool isRunning() const { return running_; }
  bool isSuspended() const { return false; }

  // Plays the queue for elapsed ticks. An empty queue stops.
  void advance(uint32_t elapsed) {
    if (stalled_) return;
    while (running_ && elapsed > 0) {
      if (queue_.empty()) {
        running_ = false;
        break;
      }
      Entry& entry = queue_.front();
      const uint32_t entry_ticks =
          entry.steps > 1 ? entry.ticks * entry.steps : entry.ticks;
      const uint32_t left = entry_ticks - head_played_ticks_;
      if (elapsed < left) {
        head_played_ticks_ += elapsed;
        played_ticks_ += elapsed;
        return;
      }
      elapsed -= left;
      played_ticks_ += left;
      position_ += entry.count_up ? entry.steps : -static_cast<int32_t>(entry.steps);
      head_played_ticks_ = 0;
      queue_.pop_front();
    }
    if (queue_.empty()) running_ = false;
  }

  void setNotReady(bool not_ready) { not_ready_ = not_ready; }
  // The driver stopped playing entries although the queue counts as running.
  void setStalled(bool stalled) { stalled_ = stalled; }
  void forceResult(int8_t result, uint32_t times) {
    forced_result_ = result;
    forced_results_left_ = times;
  }
  void setQueueEndDirection(bool count_up) { queue_end_count_up_ = count_up; }

  int32_t position() const { return position_; }
  int32_t queuedPosition() const { return queued_position_; }
  uint64_t queuedTicks() const { return queued_ticks_; }
  uint32_t calls() const { return calls_; }
  uint16_t maxEntries() const { return max_entries_; }
  uint32_t entriesUsed() const { return entries_used_; }
  uint32_t pauseEntries() const { return pause_entries_; }

 private:
  // FastAccelStepper::addQueueEntry() with a direction pin on the ESP32
  // MCPWM/PCNT driver: a direction change first needs MIN_CMD_TICKS of pause
  // in the old direction, then the direction-change delay as a pause that
  // carries the new direction. Each call injects one of them and refuses the
  // command itself.
  int8_t addEntry(const Entry& entry, bool start, uint32_t* actual) {
    if (entry.count_up != queue_end_count_up_) {
      if (last_pause_ticks_ < kMinCmdTicks) {
        push({kMinCmdTicks, 0, queue_end_count_up_}, start);
        if (actual != nullptr) *actual += kMinCmdTicks;
        return kDirChangePauseInjected;
      }
      if (entry.steps > 0 || entry.ticks < kDirChangeDelayTicks) {
        push({kDirChangeDelayTicks, 0, entry.count_up}, start);
        if (actual != nullptr) *actual += kDirChangeDelayTicks;
        return kDirChangePauseInjected;
      }
    }
    // StepperQueue::addQueueEntry(): a command shorter than MIN_CMD_TICKS.
    if ((entry.steps > 1 ? entry.ticks * entry.steps : entry.ticks) < kMinCmdTicks) {
      return kTicksTooLow;
    }
    push(entry, start);
    if (actual != nullptr) {
      *actual += entry.steps > 1 ? entry.ticks * entry.steps : entry.ticks;
    }
    return kOk;
  }

  void push(const Entry& entry, bool start) {
    require(queue_.size() < kQueueLen, "model queue overflow");
    const bool direction_changed = entry.count_up != queue_end_count_up_;
    queue_.push_back(entry);
    queue_end_count_up_ = entry.count_up;
    queued_position_ += entry.count_up ? entry.steps : -static_cast<int32_t>(entry.steps);
    queued_ticks_ += entry.steps > 1 ? entry.ticks * entry.steps : entry.ticks;
    ++entries_used_;
    if (entry.steps == 0) ++pause_entries_;
    if (queue_.size() > max_entries_) max_entries_ = static_cast<uint16_t>(queue_.size());
    if (entry.steps > 0 || direction_changed) {
      last_pause_ticks_ = 0;
    } else {
      last_pause_ticks_ += entry.ticks;
    }
    if (start) running_ = true;
  }

  std::deque<Entry> queue_;
  bool running_ = false;
  bool not_ready_ = false;
  bool stalled_ = false;
  uint32_t pause_entries_ = 0;
  bool queue_end_count_up_ = true;
  uint32_t last_pause_ticks_ = 0;
  uint32_t head_played_ticks_ = 0;
  uint64_t played_ticks_ = 0;
  uint64_t queued_ticks_ = 0;
  int32_t position_ = 0;
  int32_t queued_position_ = 0;
  uint32_t calls_ = 0;
  uint32_t entries_used_ = 0;
  uint16_t max_entries_ = 0;
  int8_t forced_result_ = 0;
  uint32_t forced_results_left_ = 0;
};

struct FakeClock {
  FakeAxis* a;
  FakeAxis* b;
  uint32_t now_ms = 0;
  uint32_t nowMs() { return now_ms; }
  void waitMs() {
    ++now_ms;
    a->advance(kTicksPerMs);
    b->advance(kTicksPerMs);
  }
};

timed_segment::Config testConfig() {
  timed_segment::Config config;
  config.queue_len = kQueueLen;
  config.direction_entry_margin = 2;
  config.partial_retry_timeout_ms = 1000;
  config.max_pause_injections = 4;
  return config;
}

// A trapezoid from standstill to standstill, as MotionTask plans one XY move.
MotionBlock planBlock(int32_t a_steps, int32_t b_steps, float feed_mm_min) {
  MotionBlock block{};
  block.a_steps = a_steps;
  block.b_steps = b_steps;
  const float da_mm = a_steps / STEPS_PER_MM;
  const float db_mm = b_steps / STEPS_PER_MM;
  const float dx_mm = (da_mm + db_mm) / 2.0f;
  const float dy_mm = (da_mm - db_mm) / 2.0f;
  block.length_mm = std::sqrt(dx_mm * dx_mm + dy_mm * dy_mm);
  block.acceleration_mm_s2 = DEFAULT_ACCEL_MM_S2;
  const float feed_mm_s = feed_mm_min / 60.0f;
  const float peak_limit_mm_s = std::sqrt(block.acceleration_mm_s2 * block.length_mm);
  block.peak_speed_mm_s = std::min(feed_mm_s, peak_limit_mm_s);
  block.acceleration_time_s = block.peak_speed_mm_s / block.acceleration_mm_s2;
  block.deceleration_time_s = block.acceleration_time_s;
  block.acceleration_distance_mm =
      0.5f * block.acceleration_mm_s2 * block.acceleration_time_s *
      block.acceleration_time_s;
  block.deceleration_distance_mm = block.acceleration_distance_mm;
  block.cruise_distance_mm = std::max(
      0.0f, block.length_mm - block.acceleration_distance_mm -
                block.deceleration_distance_mm);
  block.cruise_time_s = block.cruise_distance_mm / block.peak_speed_mm_s;
  block.trapezoid_planned = true;
  return block;
}

struct RunOutcome {
  bool ok = false;
  uint32_t duration_ms = 0;
};

// MotionTask::executeTimedSegments(): first segment with start=false, start
// both queues, then feed the rest and retry every 1 ms while the queues are
// full.
RunOutcome runMove(timed_segment::Feeder& feeder, FakeAxis& a, FakeAxis& b,
                   FakeClock& clock, const MotionBlock& block) {
  SegmentGenerator generator;
  static SegmentQueue queue;
  require(generator.generate(block, queue), "SegmentGenerator rejected the block");
  RunOutcome outcome;
  const uint32_t started_ms = clock.now_ms;
  bool first = true;
  MotionSegment segment{};
  while (queue.dequeue(segment)) {
    for (;;) {
      const timed_segment::FeedResult result =
          feeder.feed(a, b, clock, segment.a_steps, segment.b_steps,
                      segment.duration_us * kTicksPerUs, !first);
      if (result == timed_segment::FeedResult::QUEUED) break;
      if (result == timed_segment::FeedResult::ERROR) return outcome;
      clock.waitMs();
      require(clock.now_ms - started_ms < 60000, "feeding never finished");
    }
    if (first) {
      a.moveTimed(0, 0, nullptr, true);
      b.moveTimed(0, 0, nullptr, true);
      first = false;
    }
  }
  while (a.isRunning() || b.isRunning()) {
    clock.waitMs();
    require(clock.now_ms - started_ms < 60000, "queues never drained");
  }
  outcome.ok = true;
  outcome.duration_ms = clock.now_ms - started_ms;
  return outcome;
}

void testResultClassification() {
  using timed_segment::ResultClass;
  using timed_segment::classifyResult;
  require(classifyResult(kOk) == ResultClass::QUEUED, "OK is queued");
  require(classifyResult(kMoveEmpty) == ResultClass::QUEUED_AFTER_EMPTY,
          "EMPTY is queued after an empty queue");
  // Issue #15: these two were treated as "queued", but the move is NOT queued.
  require(classifyResult(kDirPin2msPauseAdded) == ResultClass::PAUSE_INJECTED,
          "DirPin2msPauseAdded must not count as queued");
  require(classifyResult(kDirChangePauseInjected) == ResultClass::PAUSE_INJECTED,
          "DirChangePauseInjected must not count as queued");
  require(classifyResult(kMoveBusy) == ResultClass::RETRY, "BUSY is a retry");
  require(classifyResult(kDeviceNotReady) == ResultClass::RETRY,
          "DEVICE_NOT_READY is a retry");
  require(classifyResult(kTooLarge) == ResultClass::ERROR, "negative is an error");
  require(classifyResult(-5) == ResultClass::ERROR, "suspended is an error");
}

// The capacity check must never promise room that moveTimed() then lacks.
void testEstimateCoversModelEntries() {
  for (uint32_t duration_us = 1000; duration_us <= 20000; duration_us += 500) {
    const uint32_t duration = duration_us * kTicksPerUs;
    for (int32_t steps = 0; steps <= 700; ++steps) {
      FakeAxis axis;
      uint32_t actual = 0;
      const int8_t result = axis.moveTimed(static_cast<int16_t>(steps), duration,
                                           &actual, false);
      if (result < 0) continue;
      require(result == kMoveEmpty, "model refused a move on an empty queue");
      require(timed_segment::estimateEntries(steps, duration) >= axis.entriesUsed(),
              "estimateEntries is below the entries moveTimed() used");
      require(timed_segment::estimateEntries(-steps, duration) ==
                  timed_segment::estimateEntries(steps, duration),
              "estimateEntries depends on the sign");
    }
  }
  // Ticks per step around the points where the library splits a pause in two.
  for (const uint32_t rate : {65535U, 65536U, 131070U, 131071U, 131072U, 196606U,
                              196607U, 196608U, 262143U, 262144U}) {
    for (int32_t steps = 1; steps <= 4; ++steps) {
      FakeAxis axis;
      const int8_t result =
          axis.moveTimed(static_cast<int16_t>(steps), rate * steps, nullptr, false);
      require(result == kMoveEmpty, "model refused a boundary move");
      require(timed_segment::estimateEntries(steps, rate * steps) >= axis.entriesUsed(),
              "estimateEntries is below the entries used at a split boundary");
    }
  }
  // 20 ms with 0..4 steps is the slow axis of a near-45-degree move.
  const uint32_t segment_ticks = 20000 * kTicksPerUs;
  require(timed_segment::estimateEntries(0, segment_ticks) == 5, "0 steps: 5 pauses");
  require(timed_segment::estimateEntries(1, segment_ticks) == 5, "1 step: 5 entries");
  require(timed_segment::estimateEntries(4, segment_ticks) == 8, "4 steps: 8 entries");
  require(timed_segment::estimateEntries(5, segment_ticks) == 1, "5 steps: 1 entry");
}

struct MoveCase {
  const char* name;
  int32_t a_steps;
  int32_t b_steps;
  float feed_mm_min;
};

void testNear45DegreeMovesComplete() {
  const MoveCase cases[] = {
      {"issue example 1 (B/A 0.046, F9000)", 9927, -461, 9000.0f},
      {"issue example 2 (B/A 0.018, F15900)", 27842, 503, 15900.0f},
      {"exact 45 degrees, B=0", 20000, 0, 15900.0f},
      {"B=0, slow", 8000, 0, 600.0f},
      {"B/A 0.018", 20000, 360, 9000.0f},
      {"B/A 0.046", 20000, -920, 9000.0f},
      {"B/A 0.052", 20000, 1040, 9000.0f},
      {"B/A 0.1", 20000, -2000, 15900.0f},
      {"A/B 0.018 (the other diagonal)", -360, 20000, 9000.0f},
      {"A=0", 0, -20000, 15900.0f},
      {"pure X", 16000, 16000, 15900.0f},
      {"pure Y", 16000, -16000, 9000.0f},
      {"1 step on B", 12000, 1, 9000.0f},
      {"short move", 40, -3, 600.0f},
  };
  for (const MoveCase& move : cases) {
    FakeAxis a;
    FakeAxis b;
    FakeClock clock{&a, &b};
    timed_segment::Feeder feeder(testConfig());
    const MotionBlock block = planBlock(move.a_steps, move.b_steps, move.feed_mm_min);
    const RunOutcome outcome = runMove(feeder, a, b, clock, block);
    if (!outcome.ok) {
      const timed_segment::Failure& failure = feeder.failure();
      std::cerr << move.name << ": stage=" << timed_segment::stageName(failure.stage)
                << " raw_a=" << static_cast<int>(failure.raw_a)
                << " raw_b=" << static_cast<int>(failure.raw_b)
                << " segment=" << failure.segment_index << "\n";
    }
    require(outcome.ok, move.name);
    require(a.position() == move.a_steps, "A did not reach its step count");
    require(b.position() == move.b_steps, "B did not reach its step count");
    const timed_segment::RunStats& stats = feeder.stats();
    require(stats.underruns_a == 0 && stats.underruns_b == 0,
            "a queue ran dry during the move");
    require(stats.max_b_wait_ms == 0, "B was refused after A was queued");
    // The skew is removed by lengthening segments a little. It must never
    // become a pause on a motor that is stepping.
    // Tick rounding alone is far below one pause; a direction change adds its
    // two pauses (2 % of a 20 ms segment) to the other axis once.
    const bool reverses = move.a_steps < 0 || move.b_steps < 0;
    require(stats.max_stretch_ticks <
                (reverses ? 3 * kMinCmdTicks : kMinCmdTicks),
            "a segment was lengthened by a visible amount");
    if (move.a_steps > 5000 && move.b_steps >= 0 && move.b_steps <= 600) {
      require(a.pauseEntries() == 0, "the fast motor got a zero-step pause");
    }
    const int64_t skew = static_cast<int64_t>(a.queuedTicks()) -
                         static_cast<int64_t>(b.queuedTicks());
    require(skew == stats.skew_ticks, "skew accounting does not match the queues");
    require(std::llabs(skew) < static_cast<int64_t>(kMinCmdTicks),
            "A and B timelines differ by a queueable pause");
    require(a.maxEntries() < kQueueLen && b.maxEntries() < kQueueLen,
            "a queue was filled to the last entry");
  }
}

// Both motors reverse: every axis gets direction-change pauses, and the two
// timelines must still end together with the exact step counts.
void testDirectionChangeKeepsStepsAndTiming() {
  const MoveCase moves[] = {
      {"forward", 9927, 461, 9000.0f},
      {"back", -9927, -461, 9000.0f},
      {"A only reverses", 9927, -461, 9000.0f},
  };
  FakeAxis a;
  FakeAxis b;
  FakeClock clock{&a, &b};
  timed_segment::Feeder feeder(testConfig());
  int32_t expected_a = 0;
  int32_t expected_b = 0;
  uint32_t pauses = 0;
  for (const MoveCase& move : moves) {
    const MotionBlock block = planBlock(move.a_steps, move.b_steps, move.feed_mm_min);
    require(runMove(feeder, a, b, clock, block).ok, move.name);
    expected_a += move.a_steps;
    expected_b += move.b_steps;
    require(a.position() == expected_a, "A lost or doubled steps at a direction change");
    require(b.position() == expected_b, "B lost or doubled steps at a direction change");
    const int64_t skew = static_cast<int64_t>(a.queuedTicks()) -
                         static_cast<int64_t>(b.queuedTicks());
    require(std::llabs(skew) < static_cast<int64_t>(kMinCmdTicks),
            "direction-change pauses left A and B out of step in time");
    pauses += feeder.stats().pause_injections_a + feeder.stats().pause_injections_b;
  }
  require(pauses > 0, "the model never injected a direction-change pause");
}

// The pause answer means "not queued": the move must be sent again, once.
void testPauseInjectedMoveIsSentAgain() {
  for (const int8_t pause_code : {kDirPin2msPauseAdded, kDirChangePauseInjected}) {
    FakeAxis a;
    FakeAxis b;
    FakeClock clock{&a, &b};
    timed_segment::Feeder feeder(testConfig());
    a.forceResult(pause_code, 1);
    require(feeder.feed(a, b, clock, 100, 3, 20000 * kTicksPerUs, false) ==
                timed_segment::FeedResult::QUEUED,
            "segment after a pause answer on A was not queued");
    require(a.queuedPosition() == 100, "A steps were dropped after a pause answer");
    require(b.queuedPosition() == 3, "B steps missing");

    FakeAxis a2;
    FakeAxis b2;
    FakeClock clock2{&a2, &b2};
    timed_segment::Feeder feeder2(testConfig());
    b2.forceResult(pause_code, 2);
    require(feeder2.feed(a2, b2, clock2, 100, 3, 20000 * kTicksPerUs, false) ==
                timed_segment::FeedResult::QUEUED,
            "segment after pause answers on B was not queued");
    require(a2.queuedPosition() == 100, "A was queued twice or not at all");
    require(b2.queuedPosition() == 3, "B steps were dropped after a pause answer");
  }
}

// B refuses while A is already queued: the feeder waits, and a short refusal
// is not an error.
void testShortRefusalOnBRecovers() {
  FakeAxis a;
  FakeAxis b;
  FakeClock clock{&a, &b};
  timed_segment::Feeder feeder(testConfig());
  b.forceResult(kDeviceNotReady, 5);
  require(feeder.feed(a, b, clock, 300, 2, 20000 * kTicksPerUs, false) ==
              timed_segment::FeedResult::QUEUED,
          "a 5 ms refusal on B became an error");
  require(a.queuedPosition() == 300 && b.queuedPosition() == 2, "steps wrong");
  require(feeder.stats().max_b_wait_ms == 5, "B wait time not recorded");
  require(!feeder.failure().valid, "failure recorded without an error");
}

// The failure of Issue #15 as far as it is understood: B keeps refusing with
// A queued. The feeder gives up after the timeout and keeps the evidence.
void testStuckBTimesOutWithSnapshot() {
  FakeAxis a;
  FakeAxis b;
  FakeClock clock{&a, &b};
  timed_segment::Feeder feeder(testConfig());
  const uint32_t ticks = 20000 * kTicksPerUs;
  require(feeder.feed(a, b, clock, 40, 0, ticks, false) ==
              timed_segment::FeedResult::QUEUED,
          "first segment");
  a.moveTimed(0, 0, nullptr, true);
  b.moveTimed(0, 0, nullptr, true);
  require(feeder.feed(a, b, clock, 80, 1, ticks, true) ==
              timed_segment::FeedResult::QUEUED,
          "second segment");
  b.setNotReady(true);
  const uint32_t before_ms = clock.now_ms;
  require(feeder.feed(a, b, clock, 120, 1, ticks, true) ==
              timed_segment::FeedResult::ERROR,
          "a stuck B must end in an error");
  require(clock.now_ms - before_ms == 1000, "timeout is not 1000 ms");
  const timed_segment::Failure& failure = feeder.failure();
  require(failure.valid, "no failure snapshot");
  require(failure.stage == timed_segment::Stage::B_TIMEOUT, "wrong stage");
  require(failure.raw_a == kOk, "raw A result not kept");
  require(failure.raw_b == kDeviceNotReady, "raw B result not kept");
  require(failure.first_raw_b == kDeviceNotReady, "first raw B result not kept");
  require(failure.a_steps == 120 && failure.b_steps == 1, "segment not kept");
  require(failure.duration_ticks == ticks, "ticks not kept");
  require(failure.segment_index == 2, "segment index not kept");
  require(failure.start, "start flag not kept");
  require(failure.waited_ms == 1000, "wait time not kept");
  require(failure.b_retries == 1000, "retry count not kept");
}

void testErrorsAreReportedWithStage() {
  FakeAxis a;
  FakeAxis b;
  FakeClock clock{&a, &b};
  timed_segment::Feeder feeder(testConfig());
  const uint32_t ticks = 20000 * kTicksPerUs;
  require(feeder.feed(a, b, clock, 40000, 0, ticks, false) ==
              timed_segment::FeedResult::ERROR,
          "steps beyond int16 accepted");
  require(feeder.failure().stage == timed_segment::Stage::INVALID_SEGMENT, "stage");
  require(feeder.feed(a, b, clock, 10, 0, 0, false) ==
              timed_segment::FeedResult::ERROR,
          "zero duration accepted");
  require(feeder.feed(a, b, clock, 10, 0, 0x80000000U, false) ==
              timed_segment::FeedResult::ERROR,
          "a duration beyond the tick range was accepted");
  // 30 steps in 2 s: more queue entries than one move may use.
  require(feeder.feed(a, b, clock, 30, 0, 2 * kTicksPerS, false) ==
              timed_segment::FeedResult::ERROR,
          "oversized move accepted");
  require(feeder.failure().stage == timed_segment::Stage::TOO_LARGE, "stage");
  require(a.queueEntries() == 0 && b.queueEntries() == 0,
          "a rejected segment left entries behind");

  a.forceResult(-5, 1);
  require(feeder.feed(a, b, clock, 10, 10, ticks, false) ==
              timed_segment::FeedResult::ERROR,
          "suspended A accepted");
  require(feeder.failure().stage == timed_segment::Stage::A_APPEND, "stage");
  require(feeder.failure().raw_a == -5, "raw A");
  require(b.queueEntries() == 0, "B was queued after A failed");

  b.forceResult(-5, 1);
  require(feeder.feed(a, b, clock, 10, 10, ticks, false) ==
              timed_segment::FeedResult::ERROR,
          "suspended B accepted");
  require(feeder.failure().stage == timed_segment::Stage::B_APPEND, "stage");
  require(feeder.failure().raw_b == -5, "raw B");
}

// A full queue is reported as RETRY before anything is queued on either axis.
void testFullQueueQueuesNothing() {
  FakeAxis a;
  FakeAxis b;
  FakeClock clock{&a, &b};
  timed_segment::Feeder feeder(testConfig());
  const uint32_t ticks = 20000 * kTicksPerUs;
  int32_t queued = 0;
  for (;;) {
    const int32_t a_before = a.queuedPosition();
    const uint16_t entries_a = a.queueEntries();
    const uint16_t entries_b = b.queueEntries();
    const timed_segment::FeedResult result =
        feeder.feed(a, b, clock, 200, 1, ticks, false);
    if (result == timed_segment::FeedResult::RETRY) {
      require(a.queuedPosition() == a_before, "RETRY after A was queued");
      require(a.queueEntries() == entries_a && b.queueEntries() == entries_b,
              "RETRY left entries behind");
      break;
    }
    require(result == timed_segment::FeedResult::QUEUED, "unexpected error");
    require(++queued < 100, "the queue never filled");
  }
  require(queued >= 3, "fewer than three slow segments fit");
}

// B's queue stops being played while it still counts as running: there is
// never room for the next segment. That used to retry forever in silence.
void testStalledQueueIsReported() {
  FakeAxis a;
  FakeAxis b;
  FakeClock clock{&a, &b};
  timed_segment::Feeder feeder(testConfig());
  const uint32_t ticks = 20000 * kTicksPerUs;
  require(feeder.feed(a, b, clock, 200, 1, ticks, false) ==
              timed_segment::FeedResult::QUEUED,
          "first segment");
  a.moveTimed(0, 0, nullptr, true);
  b.moveTimed(0, 0, nullptr, true);
  b.setStalled(true);
  uint32_t queued = 1;
  timed_segment::FeedResult result = timed_segment::FeedResult::RETRY;
  const uint32_t started_ms = clock.now_ms;
  for (;;) {
    result = feeder.feed(a, b, clock, 200, 1, ticks, true);
    if (result == timed_segment::FeedResult::ERROR) break;
    if (result == timed_segment::FeedResult::QUEUED) {
      ++queued;
    } else {
      clock.waitMs();
    }
    require(clock.now_ms - started_ms < 5000, "a stalled queue was never reported");
  }
  const timed_segment::Failure& failure = feeder.failure();
  require(failure.stage == timed_segment::Stage::CAPACITY_STALL, "wrong stage");
  require(failure.waited_ms >= 1000, "reported before the timeout");
  require(failure.running_b, "running state not kept");
  require(failure.entries_b > 20, "entry count not kept");
  require(b.queuedPosition() == static_cast<int32_t>(queued), "segment count wrong");

  // A queue that drains slowly is not a stall: waiting for room is normal.
  FakeAxis a2;
  FakeAxis b2;
  FakeClock clock2{&a2, &b2};
  timed_segment::Feeder feeder2(testConfig());
  const MotionBlock block = planBlock(600, 0, 60.0f);
  require(runMove(feeder2, a2, b2, clock2, block).ok,
          "a slow move was reported as a stall");
}

// The backend filled the queues and they did not start.
void testStartFailureIsRecorded() {
  FakeAxis a;
  FakeAxis b;
  FakeClock clock{&a, &b};
  timed_segment::Feeder feeder(testConfig());
  require(feeder.feed(a, b, clock, 200, 0, 20000 * kTicksPerUs, false) ==
              timed_segment::FeedResult::QUEUED,
          "first segment");
  require(!feeder.failure().valid, "failure before the start");
  b.setNotReady(true);
  a.moveTimed(0, 0, nullptr, true);
  b.moveTimed(0, 0, nullptr, true);
  feeder.recordStartFailure(a, b);
  const timed_segment::Failure& failure = feeder.failure();
  require(failure.valid && failure.stage == timed_segment::Stage::START, "stage");
  require(failure.running_a && !failure.running_b, "which axis did not start");
  require(failure.entries_b == 5, "entry count of the axis that did not start");
}

// An axis that ran dry while the run was going is counted, not hidden.
void testUnderrunIsCounted() {
  FakeAxis a;
  FakeAxis b;
  FakeClock clock{&a, &b};
  timed_segment::Feeder feeder(testConfig());
  const uint32_t ticks = 20000 * kTicksPerUs;
  require(feeder.feed(a, b, clock, 200, 20, ticks, false) ==
              timed_segment::FeedResult::QUEUED,
          "first segment");
  a.moveTimed(0, 0, nullptr, true);
  b.moveTimed(0, 0, nullptr, true);
  require(feeder.stats().underruns_a == 0 && feeder.stats().underruns_b == 0,
          "the first segment counted as an underrun");
  for (int ms = 0; ms < 30; ++ms) clock.waitMs();
  require(feeder.feed(a, b, clock, 200, 20, ticks, true) ==
              timed_segment::FeedResult::QUEUED,
          "segment after the gap");
  require(feeder.stats().underruns_a == 1 && feeder.stats().underruns_b == 1,
          "underrun not counted");
}

}  // namespace

int main() {
  testResultClassification();
  testEstimateCoversModelEntries();
  testNear45DegreeMovesComplete();
  testDirectionChangeKeepsStepsAndTiming();
  testPauseInjectedMoveIsSentAgain();
  testShortRefusalOnBRecovers();
  testStuckBTimesOutWithSnapshot();
  testErrorsAreReportedWithStage();
  testFullQueueQueuesNothing();
  testStalledQueueIsReported();
  testStartFailureIsRecorded();
  testUnderrunIsCounted();
  std::cout << "native timed segment feeder tests passed\n";
  return 0;
}
