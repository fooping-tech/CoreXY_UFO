#pragma once

#include <stdint.h>

// #30: which multi-turn count a servo is at, after its replies were lost during a move.
//
// The servo reports only its angle in the turn (raw 0..4095). Every count with that angle is
// a candidate: raw + k * turn. During the move the servo went from the last tracked count
// toward the target (the runaway guard holds it to that path while it is heard), and since
// the last good read it can have moved at most `reach` counts. A candidate is possible when
// it lies on that path (with `margin` either side) and within `reach` of the last tracked
// count. The count is known only when exactly one candidate is possible: a servo stalled a
// whole turn short of a multi-turn target, at the target's angle, has two (review on #31).
//
// Pure (no hardware), so the native tests check it.
struct StsTurnRecovery {
  bool known = false;
  int candidates = 0;
  int32_t abs_counts = 0;
};

inline StsTurnRecovery stsRecoverTurn(int32_t last_abs, int32_t target, int raw,
                                      int32_t reach, int32_t margin,
                                      int32_t counts_per_turn = 4096) {
  StsTurnRecovery out;
  const int32_t lo = (last_abs < target ? last_abs : target) - margin;
  const int32_t hi = (last_abs < target ? target : last_abs) + margin;
  // the first count >= lo with this angle
  int32_t mod = lo % counts_per_turn;
  if (mod < 0) mod += counts_per_turn;
  int32_t c = lo - mod + raw;
  if (c < lo) c += counts_per_turn;
  for (; c <= hi; c += counts_per_turn) {
    const int32_t from_last = c > last_abs ? c - last_abs : last_abs - c;
    if (from_last > reach) continue;
    out.candidates++;
    out.abs_counts = c;
  }
  out.known = out.candidates == 1;
  return out;
}
