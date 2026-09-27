#!/usr/bin/env bash
# Regression check for the MainKinematic test suites.
#
# Runs ManipTests, ManipSolverTests and PhysicsLibTests from BUILD_DIR and
# compares the failing tests with Tests/known_failures.txt (the tests that
# document still-unfixed bugs):
#   - a test failing now but not listed      -> REGRESSION (exit code 1)
#   - a listed test that passes now          -> FIXED (remove it with --update)
#   - a test binary that crashes / aborts    -> CRASH (exit code 1)
#
# Usage:
#   Tests/run_regression_check.sh BUILD_DIR            # check
#   Tests/run_regression_check.sh BUILD_DIR --update   # accept current state as the new baseline

set -u

BUILD_DIR=${1:?usage: $0 BUILD_DIR [--update]}
MODE=${2:-check}
HERE=$(cd "$(dirname "$0")" && pwd)
BASELINE="$HERE/known_failures.txt"
SUITES="ManipTests ManipSolverTests PhysicsLibTests"

current=$(mktemp)
status=0

for suite in $SUITES; do
    bin="$BUILD_DIR/$suite"
    if [ ! -x "$bin" ]; then
        echo "MISSING: $bin (build it first)"
        status=1
        continue
    fi
    out=$("$bin" --gtest_brief=1 2>/dev/null)
    code=$?
    if ! grep -q "PASSED  ]" <<<"$out"; then
        echo "CRASH: $suite exited with code $code before finishing"
        status=1
    fi
    # "[  FAILED  ] Suite.Name (12 ms)" / "[  FAILED  ] P/Suite.Name/3, where GetParam() = ..."
    grep -E '^\[  FAILED  \] [A-Za-z]' <<<"$out" |
        sed -E 's/^\[  FAILED  \] //; s/, where GetParam.*//; s/ \([0-9]+ ms\)$//' |
        sed "s/^/$suite: /" >>"$current"
    passed=$(grep -oE 'PASSED  \] [0-9]+' <<<"$out" | grep -oE '[0-9]+$')
    echo "$suite: ${passed:-0} passed, $(grep -c "^$suite: " "$current") failing"
done

sort -u -o "$current" "$current"

if [ "$MODE" = "--update" ]; then
    cp "$current" "$BASELINE"
    echo "Baseline updated: $(wc -l <"$BASELINE") known failures in $BASELINE"
    rm -f "$current"
    exit $status
fi

touch "$BASELINE"
regressions=$(comm -13 "$BASELINE" "$current")
fixed=$(comm -23 "$BASELINE" "$current")

if [ -n "$regressions" ]; then
    echo
    echo "REGRESSIONS (failing now, not in known_failures.txt):"
    sed 's/^/  /' <<<"$regressions"
    status=1
fi
if [ -n "$fixed" ]; then
    echo
    echo "FIXED (listed as known failures, passing now) -- run with --update to accept:"
    sed 's/^/  /' <<<"$fixed"
fi
if [ -z "$regressions" ] && [ $status -eq 0 ]; then
    echo
    echo "OK: no regressions ($(wc -l <"$current") known failures remaining)"
fi

rm -f "$current"
exit $status
