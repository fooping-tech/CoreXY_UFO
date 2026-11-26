// #30: stsRecoverTurn() (the turn count after a lost feedback).
#include <cstdlib>
#include <iostream>

#include "StsTurnRecovery.h"

namespace {

void require(bool condition, const char* message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << "\n";
    std::exit(1);
  }
}

constexpr int32_t kMargin = 256;

void testTheMachinesClawOpen() {
  // 2026-11-26: lost at 1551 on the way from 2456 to 1245; it answered again at raw 1247
  const StsTurnRecovery r = stsRecoverTurn(1551, 1245, 1247, 2400 * 4, kMargin);
  require(r.known && r.candidates == 1 && r.abs_counts == 1247, "claw open: one candidate, 1247");
}

void testAStallAWholeTurnShortIsAmbiguous() {
  // review on #31: target 8192, lost near 4000, stalled at 4096 (raw 0, the target's angle)
  const StsTurnRecovery r = stsRecoverTurn(4000, 8192, 0, 2400 * 5, kMargin);
  require(!r.known && r.candidates == 2, "a stall one turn short at the target's angle is not known");
}

void testReachLimitsTheCandidates() {
  // the same, but lost only 0.25 s before it was heard again: it cannot have gone a turn
  const StsTurnRecovery r = stsRecoverTurn(4000, 8192, 0, 2400 / 4 + 50, kMargin);
  require(r.known && r.abs_counts == 4096, "within reach: only the near candidate");
}

void testArrivedAfterAMultiTurnMove() {
  // lost at 7000 on the way to 8192, arrived (raw 0): 4096 is on the path but out of reach
  const StsTurnRecovery r = stsRecoverTurn(7000, 8192, 0, 2400 * 2, kMargin);
  require(r.known && r.abs_counts == 8192, "arrived: 8192");
}

void testNegativeCounts() {
  const StsTurnRecovery r = stsRecoverTurn(-1000, -1500, 2600, 2400 * 3, kMargin);  // -1496
  require(r.known && r.abs_counts == -1496, "negative counts: -1496");
}

void testOffThePathIsNotACandidate() {
  // the angle seen is nowhere on the path (lost at 1551, target 1245, raw 3000)
  const StsTurnRecovery r = stsRecoverTurn(1551, 1245, 3000, 2400 * 4, kMargin);
  require(!r.known && r.candidates == 0, "off the path: none");
}

}  // namespace

int main() {
  testTheMachinesClawOpen();
  testAStallAWholeTurnShortIsAmbiguous();
  testReachLimitsTheCandidates();
  testArrivedAfterAMultiTurnMove();
  testNegativeCounts();
  testOffThePathIsNotACandidate();
  std::cout << "sts turn recovery tests: OK\n";
  return 0;
}
