// Test page: un-homed motor / XY moves, limit monitor and STS3020 servo control.
// Uses app.js globals (app, $, api, getMachine, isConnected, isJobRunning,
// refreshState, updateStateUI, showError). The firmware enforces the safety
// rules (test mode gate, limit stop, servo hold check); this page only sends
// whitelisted commands through /api/test/*.
(function () {
  const TEST_POLL_MS = 1500;
  const TEST_FEED_MM_MIN = 600;
  const SERVOS = [
    { id: 1, name: "ID1 Z axis (drum)" },
    { id: 2, name: "ID2 Arm" },
  ];

  const test = {
    motorSteps: 400,
    xyStepMm: 5,
    pollTimer: null,
    polling: false,
  };

  function testModeOn() {
    return Boolean(getMachine().testMode);
  }

  function testBusy() {
    return Boolean(app.state?.testBusy);
  }

  function testPageActive() {
    return app.page === "test";
  }

  function canRunTest() {
    return isConnected() && !isJobRunning() && testModeOn() && !testBusy();
  }

  async function sendTest(command) {
    await api("/api/test/command", {
      method: "POST",
      body: JSON.stringify({ command }),
    });
    await refreshState();
  }

  async function stopTest() {
    await api("/api/test/stop", { method: "POST", body: "{}" });
    await refreshState();
  }

  async function pollTest() {
    if (test.polling || !testPageActive() || !isConnected() || isJobRunning()) return;
    if (!$("testAutoPoll").checked) return;
    test.polling = true;
    try {
      const payload = await api("/api/test/poll", { method: "POST", body: "{}", timeoutMs: 12000 });
      if (payload.state) {
        app.state = payload.state;
        updateStateUI();
      }
    } catch (error) {
      // Polling failures are shown once in the console log by the server.
    } finally {
      test.polling = false;
    }
  }

  function setSegment(containerId, attribute, value) {
    document.querySelectorAll(`#${containerId} button`).forEach((button) => {
      button.classList.toggle("active", button.dataset[attribute] === String(value));
    });
  }

  function renderLimit(id, debounced, raw) {
    const element = $(id);
    const active = debounced === "ACTIVE" || debounced === "ON";
    const known = Boolean(debounced) && debounced !== "UNKNOWN";
    element.className = `limit-indicator ${known ? (active ? "on" : "off") : "unknown"}`;
    element.querySelector(".limit-value").textContent = known ? (active ? "ON" : "OFF") : "?";
    element.querySelector(".limit-raw").textContent = `raw ${raw || "?"}`;
  }

  function servoPanelHtml(servo) {
    const id = servo.id;
    return `
      <article class="panel servo-panel" data-servo-id="${id}">
        <div class="panel-header">
          <div>
            <div class="label">Serial Servo</div>
            <h2>${servo.name}</h2>
          </div>
          <button class="secondary" data-servo-action="status" type="button">STATUS</button>
        </div>
        <div class="servo-status muted" id="servoStatus${id}">No status yet</div>
        <div class="servo-row">
          <label>Angle [deg]
            <input id="servoAngle${id}" type="number" min="0" max="359.9" step="1" value="180">
          </label>
          <button class="primary test-servo" data-servo-action="angle" type="button">MOVE TO ANGLE</button>
        </div>
        <div class="servo-row">
          <button class="test-servo" data-servo-action="turn" data-turns="-1" type="button">-1 turn</button>
          <button class="test-servo" data-servo-action="turn" data-turns="-0.25" type="button">-1/4</button>
          <button class="test-servo" data-servo-action="turn" data-turns="0.25" type="button">+1/4</button>
          <button class="test-servo" data-servo-action="turn" data-turns="1" type="button">+1 turn</button>
        </div>
        <div class="servo-row">
          <label>Turns (+ = clockwise, max ±3)
            <input id="servoTurns${id}" type="number" min="-3" max="3" step="0.25" value="3">
          </label>
          <button class="warning test-servo" data-servo-action="turnCustom" type="button">ROTATE</button>
          <button class="danger" data-servo-action="off" type="button">TORQUE OFF</button>
        </div>
        <div class="muted" id="servoResult${id}">-</div>
      </article>`;
  }

  function servoStatusText(servo) {
    if (!servo || !servo.result) return "No status yet";
    if (servo.result !== "OK") return `Status: ${servo.result}`;
    const turns = servo.tracking === "YES" ? ` abs=${servo.abs} (${servo.turns} turn)` : "";
    return `raw=${servo.raw} torque=${servo.torque} limits=${servo.limits} tracking=${servo.tracking}${turns} ` +
      `${servo.voltage}V ${servo.temp}C flags=${servo.flags}`;
  }

  function servoCommand(id, action, button) {
    if (action === "status") return `SERVO_STATUS ${id}`;
    if (action === "off") return `SERVO_OFF ${id}`;
    if (action === "angle") {
      const angle = Number($(`servoAngle${id}`).value);
      if (!Number.isFinite(angle) || angle < 0 || angle >= 360) throw new Error("Angle must be 0 to 359.9");
      return `SERVO_ANGLE ${id} ${angle.toFixed(1)}`;
    }
    const turns = action === "turn" ? Number(button.dataset.turns) : Number($(`servoTurns${id}`).value);
    if (!Number.isFinite(turns) || turns === 0 || Math.abs(turns) > 3) throw new Error("Turns must be non-zero and within ±3");
    return `SERVO_TURN ${id} ${turns.toFixed(2)}`;
  }

  function updateTestModeUI() {
    if (!$("test")) return;
    const machine = getMachine();
    const on = testModeOn();
    const connected = isConnected();
    const busy = testBusy();
    $("testModeState").textContent = on ? "ON" : "OFF";
    $("testModeState").className = on ? "test-on" : "";
    $("testBusyText").textContent = !connected
      ? "Connect serial port first"
      : isJobRunning()
        ? "Job running: test mode unavailable"
        : busy
          ? `Running: ${app.state.testCommand}`
          : machine.alarmed
            ? `Alarm: ${machine.alarmReason || "unknown"} (CLEAR ALARM)`
            : on ? "Ready" : "Enter test mode to move";
    $("testLastResult").textContent = app.state?.lastTestResult || "-";
    renderLimit("testLimitX", machine.limits?.x, machine.limitsRaw?.x);
    renderLimit("testLimitY", machine.limits?.y, machine.limitsRaw?.y);

    $("testEnterBtn").disabled = !connected || isJobRunning() || busy || on;
    $("testExitBtn").disabled = !connected || busy || !on;
    $("testClearAlarmBtn").disabled = !connected || busy || !machine.alarmed;
    $("testStopBtn").disabled = !connected;
    document.querySelectorAll("#test .test-move, #test .test-servo").forEach((button) => {
      button.disabled = !canRunTest() || Boolean(machine.alarmed);
    });
    document.querySelectorAll('#test [data-servo-action="status"], #test [data-servo-action="off"]').forEach((button) => {
      button.disabled = !connected || isJobRunning() || busy;
    });
    SERVOS.forEach(({ id }) => {
      const servo = app.state?.servos?.[String(id)];
      $(`servoStatus${id}`).textContent = servoStatusText(servo);
      $(`servoResult${id}`).textContent = servo?.lastResult
        ? `${servo.lastResult} ${servo.lastDetail || ""}`
        : "-";
    });
  }

  function bindTestUI() {
    $("testServoPanels").innerHTML = SERVOS.map(servoPanelHtml).join("");
    $("testEnterBtn").addEventListener("click", () => sendTest("TEST_MODE ON").catch(showError));
    $("testExitBtn").addEventListener("click", () => sendTest("TEST_MODE OFF").catch(showError));
    $("testClearAlarmBtn").addEventListener("click", () => sendTest("ALARM_CLEAR").catch(showError));
    $("testStopBtn").addEventListener("click", () => stopTest().catch(showError));

    document.querySelectorAll("#testMotorStepControl button").forEach((button) => {
      button.addEventListener("click", () => {
        test.motorSteps = Number(button.dataset.steps);
        setSegment("testMotorStepControl", "steps", test.motorSteps);
      });
    });
    document.querySelectorAll("#testXyStepControl button").forEach((button) => {
      button.addEventListener("click", () => {
        test.xyStepMm = Number(button.dataset.mm);
        setSegment("testXyStepControl", "mm", test.xyStepMm);
        $("testXyStepLabel").textContent = `${test.xyStepMm}mm`;
      });
    });
    document.querySelectorAll("[data-test-motor]").forEach((button) => {
      button.addEventListener("click", () => {
        const steps = Number(button.dataset.sign) * test.motorSteps;
        sendTest(`TEST_${button.dataset.testMotor} ${steps} ${TEST_FEED_MM_MIN}`).catch(showError);
      });
    });
    document.querySelectorAll("[data-test-xy]").forEach((button) => {
      button.addEventListener("click", () => {
        const [sx, sy] = button.dataset.testXy.split(",").map(Number);
        const dx = (sx * test.xyStepMm).toFixed(3);
        const dy = (sy * test.xyStepMm).toFixed(3);
        sendTest(`TEST_JOG ${dx} ${dy} ${TEST_FEED_MM_MIN}`).catch(showError);
      });
    });
    $("testServoPanels").addEventListener("click", (event) => {
      const button = event.target.closest("[data-servo-action]");
      if (!button) return;
      const id = Number(button.closest("[data-servo-id]").dataset.servoId);
      try {
        sendTest(servoCommand(id, button.dataset.servoAction, button)).catch(showError);
      } catch (error) {
        showError(error);
      }
    });
    document.querySelector('.nav[data-page="test"]').addEventListener("click", () => pollTest());
    test.pollTimer = window.setInterval(pollTest, TEST_POLL_MS);
  }

  window.updateTestModeUI = updateTestModeUI;
  bindTestUI();
  updateTestModeUI();
})();
