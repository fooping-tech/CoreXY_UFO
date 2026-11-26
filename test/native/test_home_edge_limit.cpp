// #32: a home switch ON near the home edge is the end of the stage (no alarm);
// farther away it is a lost position (alarm).
#include <cstdlib>
#include <iostream>

#include "HomeEdgeLimit.h"

namespace {

constexpr float kHome = 0.0f;
constexpr int8_t kDir = -1;  // switch at the minimum, like HOMING_X_DIR/HOMING_Y_DIR
constexpr float kZone = 8.0f;
constexpr float kRelease = 20.0f;

void require(bool condition, const char* message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << "\n";
    std::exit(1);
  }
}

void testZone() {
  require(homeEdgeWithinZone(0.0f, kHome, kDir, kZone), "home itself is the edge");
  require(homeEdgeWithinZone(0.715f, kHome, kDir, kZone), "2026-11-26 jog overshoot Y=0.715 is the edge");
  require(homeEdgeWithinZone(8.0f, kHome, kDir, kZone), "zone boundary is the edge");
  require(!homeEdgeWithinZone(8.01f, kHome, kDir, kZone), "past the zone is not the edge");
  require(!homeEdgeWithinZone(150.0f, kHome, kDir, kZone), "mid stage is not the edge");
  require(homeEdgeWithinZone(-3.0f, kHome, kDir, kZone), "past the home position is the edge");
  // switch at the maximum (+1): home 300, edge is [292, ...]
  require(homeEdgeWithinZone(295.0f, 300.0f, 1, kZone), "+dir: near home is the edge");
  require(!homeEdgeWithinZone(250.0f, 300.0f, 1, kZone), "+dir: far from home is not");
  require(homeEdgeDistanceMm(5.0f, kHome, kDir) == 5.0f, "distance grows into the stage");
}

HomeEdgeEvent step(HomeEdgeLatch& latch, bool on, float pos) {
  return latch.update(on, on, pos, kHome, kDir, kZone, kRelease);
}

void testTouchAndRelease() {
  HomeEdgeLatch latch;
  require(step(latch, false, 50.0f) == HomeEdgeEvent::NONE, "switch off: nothing");
  require(!latch.unexpected(false, 50.0f, kHome, kDir, kZone), "switch off: not unexpected");
  require(step(latch, true, 0.715f) == HomeEdgeEvent::TOUCHED, "ON at 0.715: edge touch");
  require(latch.latched() && latch.touchMm() == 0.715f, "touch point is kept");
  require(!latch.unexpected(true, 0.715f, kHome, kDir, kZone), "edge touch is not an alarm");
  require(step(latch, true, 0.715f) == HomeEdgeEvent::NONE, "touch is reported once");
  // moving away, the switch still ON past the zone: release allowance holds
  require(step(latch, true, 12.0f) == HomeEdgeEvent::NONE, "still ON 11 mm away: allowed");
  require(!latch.unexpected(true, 12.0f, kHome, kDir, kZone), "latched: no away-from-home alarm");
  // debounced OFF but raw still ON: not released yet
  require(latch.update(false, true, 12.5f, kHome, kDir, kZone, kRelease) == HomeEdgeEvent::NONE,
          "raw still ON: not released");
  require(step(latch, false, 13.0f) == HomeEdgeEvent::RELEASED, "OFF: released");
  require(!latch.latched(), "released clears the latch");
}

void testNotReleased() {
  HomeEdgeLatch latch;
  require(step(latch, true, 1.0f) == HomeEdgeEvent::TOUCHED, "touch");
  require(step(latch, true, 20.9f) == HomeEdgeEvent::NONE, "19.9 mm: still allowed");
  require(step(latch, true, 21.0f) == HomeEdgeEvent::NOT_RELEASED,
          "20 mm from the touch point and still ON: alarm");
}

void testAwayFromHome() {
  HomeEdgeLatch latch;
  // lost position: the switch comes ON where the firmware thinks it is far away
  require(step(latch, true, 120.0f) == HomeEdgeEvent::NONE, "far ON: no edge touch");
  require(!latch.latched(), "far ON does not latch");
  require(latch.unexpected(true, 120.0f, kHome, kDir, kZone), "far ON: alarm as before");
  require(latch.unexpected(true, 8.5f, kHome, kDir, kZone), "just past the zone: alarm");
  latch.reset();
  require(!latch.latched(), "reset clears");
}

}  // namespace

int main() {
  testZone();
  testTouchAndRelease();
  testNotReleased();
  testAwayFromHome();
  std::cout << "home edge limit tests passed\n";
  return 0;
}
