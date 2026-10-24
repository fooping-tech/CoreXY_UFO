#!/bin/bash
# Pre-PR verification for CoreXY_UFO (same checks as .github/workflows/ci.yml, plus a secret scan).
#
#   verify.sh            native tests + Python tests + secret scan + build of the plotter envs
#   verify.sh --quick    skip the PlatformIO builds
#   verify.sh --all      build every env in platformio.ini (sts3020-test, atoms3-joystick too)
#
# Prints one PASS/FAIL line per step and a summary; exit code is non-zero if any step failed.
# The device steps (upload, SELFTEST) are not here: they need the board and the user's go-ahead.
set -u

ROOT=$(git rev-parse --show-toplevel) || exit 1
cd "$ROOT"
MODE=${1:-default}

PIO=$(command -v pio || echo "$HOME/.platformio/penv/bin/pio")
PLOTTER_ENVS="m5stack-core2 m5stack-core2-sim m5stack-core2-hwtest"
ALL_ENVS=$(sed -nE 's/^\[env:([^]]+)\]/\1/p' platformio.ini | tr '\n' ' ')

LOG_DIR=${TMPDIR:-/tmp}/corexy_verify
mkdir -p "$LOG_DIR"
RESULTS=()
FAILED=0

run_step() {
  local name=$1; shift
  local log="$LOG_DIR/${name//[^A-Za-z0-9_.-]/_}.log"
  if "$@" >"$log" 2>&1; then
    RESULTS+=("PASS  $name")
  else
    RESULTS+=("FAIL  $name  (log: $log)")
    FAILED=1
    tail -30 "$log"
  fi
}

python_tests() {
  if [ ! -x .venv/bin/python ]; then
    python3 -m venv .venv || return 1
  fi
  .venv/bin/pip install -q -r tools/serial_tool/requirements.txt -r tools/qr_tool/requirements.txt \
    -r tools/webui/requirements.txt pytest || return 1
  .venv/bin/python -m pytest -q tests tools/webui
}

# Credentials must never be committed. Scans the branch diff against the default branch and
# anything staged or unstaged.
secret_scan() {
  local base
  base=$(git merge-base HEAD origin/main 2>/dev/null || echo HEAD)
  local hits
  hits=$( { git diff "$base"; git diff --cached; git diff; } | grep -E '^\+' | grep -nE \
    'gh[pousr]_[A-Za-z0-9]{30,}|github_pat_[A-Za-z0-9_]{30,}|AKIA[0-9A-Z]{16}|sk-[A-Za-z0-9_-]{30,}|-----BEGIN [A-Z ]*PRIVATE KEY-----|xox[baprs]-[A-Za-z0-9-]{10,}' )
  if [ -n "$hits" ]; then echo "possible secret in diff:"; echo "$hits"; return 1; fi
  local files
  files=$( { git diff --name-only "$base"; git diff --cached --name-only; } | sort -u | grep -E '(^|/)(\.env|.*\.pem|.*\.key|id_rsa.*|credentials.*|secrets?\..*)$' )
  if [ -n "$files" ]; then echo "credential-like file in diff:"; echo "$files"; return 1; fi
  return 0
}

run_step "secret-scan" secret_scan
run_step "native-tests" sh tools/run_native_motion_tests.sh
run_step "python-tests" python_tests

case "$MODE" in
  --quick) ;;
  --all) for e in $ALL_ENVS; do run_step "pio-$e" "$PIO" run -e "$e"; done ;;
  *) for e in $PLOTTER_ENVS; do run_step "pio-$e" "$PIO" run -e "$e"; done ;;
esac

echo
echo "== verify summary ($MODE) =="
printf '%s\n' "${RESULTS[@]}"
exit $FAILED
