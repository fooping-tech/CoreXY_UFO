#pragma once

#include <stdint.h>

// Timed A/B segment append logic of StepperBackendFastAccel (Issue #15).
//
// Kept free of Arduino / FastAccelStepper includes so that the native tests
// can drive it with a model of the stepper queue. This is not a planner: it
// only decides how one already planned segment is appended to the two queues.
//
// Axis must provide:
//   int8_t moveTimed(int16_t steps, uint32_t ticks, uint32_t* actual, bool start);
//   uint16_t queueEntries() const;
//   bool isRunning() const;
// Clock must provide:
//   uint32_t nowMs();
//   void waitMs();   // wait about 1 ms (lets the queues drain)
namespace timed_segment {

// FastAccelStepper MoveTimedResultCode values (2.0.0). StepperBackendFastAccel
// checks these against the library with static_assert.
constexpr int8_t RESULT_OK = 0;
constexpr int8_t RESULT_DIR_PIN_2MS_PAUSE_ADDED = 5;
constexpr int8_t RESULT_DIR_CHANGE_PAUSE_INJECTED = 6;
constexpr int8_t RESULT_MOVE_EMPTY = 8;
constexpr uint32_t MAX_ENTRY_TICKS = 65535;

enum class ResultClass : uint8_t {
  QUEUED,
  // Appended, but the queue had been empty before the call.
  QUEUED_AFTER_EMPTY,
  // The library queued a direction-change pause instead. The move itself was
  // NOT appended and has to be sent again.
  PAUSE_INJECTED,
  RETRY,
  ERROR,
};

constexpr ResultClass classifyResult(int8_t raw) {
  return raw == RESULT_OK
             ? ResultClass::QUEUED
             : raw == RESULT_MOVE_EMPTY
                   ? ResultClass::QUEUED_AFTER_EMPTY
                   : (raw == RESULT_DIR_PIN_2MS_PAUSE_ADDED ||
                      raw == RESULT_DIR_CHANGE_PAUSE_INJECTED)
                         ? ResultClass::PAUSE_INJECTED
                         : raw > 0 ? ResultClass::RETRY : ResultClass::ERROR;
}

// Upper bound of the queue entries moveTimed() uses for one move.
inline uint16_t estimateEntries(int32_t steps, uint32_t duration_ticks) {
  if (steps < 0) steps = -steps;
  if (duration_ticks == 0) return 0;
  if (steps == 0) {
    return static_cast<uint16_t>((duration_ticks + MAX_ENTRY_TICKS - 1) /
                                 MAX_ENTRY_TICKS);
  }
  const uint32_t rate = duration_ticks / static_cast<uint32_t>(steps);
  if (rate > MAX_ENTRY_TICKS) {
    // One step command plus pauses per step. The library splits the last
    // 65536..131071 ticks in two, so this can be one more than rate >> 16.
    const uint32_t commands_per_step = (rate + MAX_ENTRY_TICKS - 1) / MAX_ENTRY_TICKS;
    return static_cast<uint16_t>(static_cast<uint32_t>(steps) * commands_per_step);
  }
  return static_cast<uint16_t>((steps + 254L) / 255L);
}

enum class Stage : uint8_t {
  NONE,
  INVALID_SEGMENT,
  TOO_LARGE,
  A_APPEND,
  B_APPEND,
  B_TIMEOUT,
};

inline const char* stageName(Stage stage) {
  switch (stage) {
    case Stage::NONE: return "none";
    case Stage::INVALID_SEGMENT: return "invalid_segment";
    case Stage::TOO_LARGE: return "too_large";
    case Stage::A_APPEND: return "a_append";
    case Stage::B_APPEND: return "b_append";
    case Stage::B_TIMEOUT: return "b_timeout";
  }
  return "unknown";
}

struct Config {
  uint16_t queue_len = 32;
  // Entries kept free per move for direction-change pauses.
  uint8_t direction_entry_margin = 2;
  // Shortest pause the queue accepts; also the A/B skew that gets padded.
  uint32_t min_pause_ticks = 3200;
  // A is already queued while B is refused: give up after this time.
  uint32_t partial_retry_timeout_ms = 1000;
  // Direction-change pauses accepted per moveTimed() retry sequence.
  uint8_t max_pause_injections = 4;
};

// State when a segment was refused for good. valid stays false until then.
struct Failure {
  bool valid = false;
  Stage stage = Stage::NONE;
  int8_t raw_a = 0;
  int8_t raw_b = 0;
  int32_t a_steps = 0;
  int32_t b_steps = 0;
  uint32_t duration_ticks = 0;
  bool start = false;
  uint16_t entries_a = 0;
  uint16_t entries_b = 0;
  bool running_a = false;
  bool running_b = false;
  uint32_t waited_ms = 0;
  uint32_t b_retries = 0;
  uint32_t pause_ticks_a = 0;
  uint32_t pause_ticks_b = 0;
  int32_t skew_ticks = 0;
  uint32_t segment_index = 0;
};

// Counters since the queues were last idle (one XY move, or one batch).
struct RunStats {
  uint32_t segments = 0;
  // moveTimed() reported an empty queue while the run was already started:
  // that axis ran dry, so A/B timing is no longer guaranteed for this run.
  uint32_t underruns_a = 0;
  uint32_t underruns_b = 0;
  uint32_t pause_injections_a = 0;
  uint32_t pause_injections_b = 0;
  uint32_t pad_ticks_a = 0;
  uint32_t pad_ticks_b = 0;
  uint32_t b_retries = 0;
  uint32_t max_b_wait_ms = 0;
  // Ticks queued on A minus ticks queued on B.
  int32_t skew_ticks = 0;
};

enum class FeedResult : uint8_t {
  QUEUED,
  RETRY,  // Nothing of this segment was queued. Call again later.
  ERROR,  // See failure().
};

class Feeder {
 public:
  explicit Feeder(const Config& config = Config()) : config_(config) {}

  const Failure& failure() const { return failure_; }
  const RunStats& stats() const { return stats_; }

  template <class Axis, class Clock>
  FeedResult feed(Axis& a, Axis& b, Clock& clock, int32_t a_steps,
                  int32_t b_steps, uint32_t duration_ticks, bool start) {
    Attempt attempt(a_steps, b_steps, duration_ticks, start);
    if (a_steps < INT16_MIN || a_steps > INT16_MAX || b_steps < INT16_MIN ||
        b_steps > INT16_MAX || duration_ticks == 0) {
      return fail(Stage::INVALID_SEGMENT, a, b, attempt);
    }
    const uint32_t needed_a =
        estimateEntries(a_steps, duration_ticks) + config_.direction_entry_margin;
    const uint32_t needed_b =
        estimateEntries(b_steps, duration_ticks) + config_.direction_entry_margin;
    if (needed_a >= config_.queue_len || needed_b >= config_.queue_len) {
      return fail(Stage::TOO_LARGE, a, b, attempt);
    }
    if (!start && !a.isRunning() && !b.isRunning() && a.queueEntries() == 0 &&
        b.queueEntries() == 0) {
      stats_ = RunStats();
      failure_ = Failure();
    }
    // Both queues must take the whole segment, so that A is never queued
    // while B has no room.
    if (a.queueEntries() + needed_a >= config_.queue_len ||
        b.queueEntries() + needed_b >= config_.queue_len) {
      return FeedResult::RETRY;
    }

    const Append append_a = append(a, a_steps, duration_ticks, start);
    attempt.raw_a = append_a.raw;
    attempt.pause_ticks_a = append_a.pause_ticks;
    stats_.pause_injections_a += append_a.pause_injections;
    // An injected pause is on A's timeline even if the move is not queued yet.
    stats_.skew_ticks += static_cast<int32_t>(append_a.queued_ticks);
    switch (classifyResult(append_a.raw)) {
      case ResultClass::QUEUED:
        break;
      case ResultClass::QUEUED_AFTER_EMPTY:
        if (start && stats_.segments > 0) ++stats_.underruns_a;
        break;
      case ResultClass::PAUSE_INJECTED:
      case ResultClass::RETRY:
        return FeedResult::RETRY;
      case ResultClass::ERROR:
        return fail(Stage::A_APPEND, a, b, attempt);
    }

    // A is queued. B has to follow; returning RETRY here would queue A twice.
    const uint32_t wait_start_ms = clock.nowMs();
    for (;;) {
      const Append append_b = append(b, b_steps, duration_ticks, start);
      attempt.raw_b = append_b.raw;
      attempt.pause_ticks_b += append_b.pause_ticks;
      stats_.pause_injections_b += append_b.pause_injections;
      stats_.skew_ticks -= static_cast<int32_t>(append_b.queued_ticks);
      const ResultClass result_b = classifyResult(append_b.raw);
      if (result_b == ResultClass::QUEUED) break;
      if (result_b == ResultClass::QUEUED_AFTER_EMPTY) {
        if (start && stats_.segments > 0) ++stats_.underruns_b;
        break;
      }
      attempt.waited_ms = clock.nowMs() - wait_start_ms;
      if (result_b == ResultClass::ERROR) {
        return fail(Stage::B_APPEND, a, b, attempt);
      }
      if (attempt.waited_ms >= config_.partial_retry_timeout_ms) {
        return fail(Stage::B_TIMEOUT, a, b, attempt);
      }
      clock.waitMs();
      ++attempt.b_retries;
    }
    attempt.waited_ms = clock.nowMs() - wait_start_ms;
    stats_.b_retries += attempt.b_retries;
    if (attempt.waited_ms > stats_.max_b_wait_ms) {
      stats_.max_b_wait_ms = attempt.waited_ms;
    }
    ++stats_.segments;
    padLaggingAxis(a, b, start);
    return FeedResult::QUEUED;
  }

 private:
  struct Attempt {
    // Constructor instead of aggregate init: the ESP32 toolchain builds as
    // C++11, where members with default values make this a non-aggregate.
    Attempt(int32_t a_steps_in, int32_t b_steps_in, uint32_t duration_ticks_in,
            bool start_in)
        : a_steps(a_steps_in),
          b_steps(b_steps_in),
          duration_ticks(duration_ticks_in),
          start(start_in) {}
    int32_t a_steps;
    int32_t b_steps;
    uint32_t duration_ticks;
    bool start;
    int8_t raw_a = 0;
    int8_t raw_b = 0;
    uint32_t pause_ticks_a = 0;
    uint32_t pause_ticks_b = 0;
    uint32_t waited_ms = 0;
    uint32_t b_retries = 0;
  };

  struct Append {
    int8_t raw = 0;
    // Ticks this call put on the axis timeline (pauses + the move if queued).
    uint32_t queued_ticks = 0;
    uint32_t pause_ticks = 0;
    uint8_t pause_injections = 0;
  };

  // Sends one move. When the library answers with a direction-change pause,
  // the move was not appended: count the pause and send the move again.
  template <class Axis>
  Append append(Axis& axis, int32_t steps, uint32_t duration_ticks, bool start) {
    Append out;
    for (;;) {
      uint32_t actual_ticks = 0;
      out.raw = axis.moveTimed(static_cast<int16_t>(steps), duration_ticks,
                               &actual_ticks, start);
      const ResultClass result = classifyResult(out.raw);
      if (result == ResultClass::PAUSE_INJECTED) {
        out.queued_ticks += actual_ticks;
        out.pause_ticks += actual_ticks;
        if (++out.pause_injections >= config_.max_pause_injections) return out;
        continue;
      }
      if (result == ResultClass::QUEUED ||
          result == ResultClass::QUEUED_AFTER_EMPTY) {
        out.queued_ticks += actual_ticks;
      }
      return out;
    }
  }

  // Direction-change pauses and tick rounding make one timeline longer than
  // the other. Once the difference is a queueable pause, the shorter one gets
  // it, so the following segments start together again.
  template <class Axis>
  void padLaggingAxis(Axis& a, Axis& b, bool start) {
    const int32_t min_pause = static_cast<int32_t>(config_.min_pause_ticks);
    if (stats_.skew_ticks >= min_pause) {
      const uint32_t padded = pad(b, static_cast<uint32_t>(stats_.skew_ticks), start);
      stats_.pad_ticks_b += padded;
      stats_.skew_ticks -= static_cast<int32_t>(padded);
    } else if (stats_.skew_ticks <= -min_pause) {
      const uint32_t padded = pad(a, static_cast<uint32_t>(-stats_.skew_ticks), start);
      stats_.pad_ticks_a += padded;
      stats_.skew_ticks += static_cast<int32_t>(padded);
    }
  }

  template <class Axis>
  uint32_t pad(Axis& axis, uint32_t ticks, bool start) {
    if (ticks > MAX_ENTRY_TICKS) ticks = MAX_ENTRY_TICKS;
    if (axis.queueEntries() + 1U >= config_.queue_len) return 0;
    uint32_t actual_ticks = 0;
    const ResultClass result =
        classifyResult(axis.moveTimed(0, ticks, &actual_ticks, start));
    // Not queued: the skew stays and is padded after a later segment.
    if (result != ResultClass::QUEUED && result != ResultClass::QUEUED_AFTER_EMPTY) {
      return 0;
    }
    return actual_ticks;
  }

  template <class Axis>
  FeedResult fail(Stage stage, const Axis& a, const Axis& b, const Attempt& attempt) {
    failure_.valid = true;
    failure_.stage = stage;
    failure_.raw_a = attempt.raw_a;
    failure_.raw_b = attempt.raw_b;
    failure_.a_steps = attempt.a_steps;
    failure_.b_steps = attempt.b_steps;
    failure_.duration_ticks = attempt.duration_ticks;
    failure_.start = attempt.start;
    failure_.entries_a = a.queueEntries();
    failure_.entries_b = b.queueEntries();
    failure_.running_a = a.isRunning();
    failure_.running_b = b.isRunning();
    failure_.waited_ms = attempt.waited_ms;
    failure_.b_retries = attempt.b_retries;
    failure_.pause_ticks_a = attempt.pause_ticks_a;
    failure_.pause_ticks_b = attempt.pause_ticks_b;
    failure_.skew_ticks = stats_.skew_ticks;
    failure_.segment_index = stats_.segments;
    return FeedResult::ERROR;
  }

  Config config_;
  Failure failure_;
  RunStats stats_;
};

}  // namespace timed_segment
