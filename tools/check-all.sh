#!/usr/bin/env bash
# Run every CI job locally.
#
#   tools/check-all.sh [build-root]
#
# Jobs are skipped rather than failed when their tool is absent, so this is
# useful with whatever you happen to have installed; what is missing is listed at
# the end. With gcc, clang, clang-format, clang-tidy and arm-none-eabi-gcc
# present it covers everything GitHub Actions runs.
set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${1:-$REPO/.check}"
mkdir -p "$OUT"

rc=0
results=()

have()   { command -v "$1" >/dev/null 2>&1; }
note()   { printf '\n\033[1m:: %s\033[0m\n' "$*"; }
pass()   { results+=("PASS  $*"); printf '   \033[32mPASS\033[0m  %s\n' "$*"; }
fail()   { results+=("FAIL  $*"); printf '   \033[31mFAIL\033[0m  %s\n' "$*"; rc=1; }
skip()   { results+=("SKIP  $*"); printf '   SKIP  %s\n' "$*"; }

# ---------------------------------------------------------------------------
build_and_test()   # <label> <cc> <cxx> <extra cmake args...>
{
    local label="$1" cc="$2" cxx="$3"; shift 3
    local dir="$OUT/$label"

    if ! have "$cxx"; then
        skip "$label (no $cxx)"
        return
    fi

    if ! CC="$cc" CXX="$cxx" cmake -S "$REPO" -B "$dir" "$@" > "$OUT/$label-cfg.log" 2>&1; then
        fail "$label configure"; tail -20 "$OUT/$label-cfg.log"; return
    fi
    if ! cmake --build "$dir" --parallel > "$OUT/$label-build.log" 2>&1; then
        fail "$label build"; grep -E 'error' "$OUT/$label-build.log" | head -20; return
    fi
    pass "$label build"

    if ctest --test-dir "$dir" --output-on-failure > "$OUT/$label-test.log" 2>&1; then
        pass "$label tests ($(grep -oE '[0-9]+% tests passed' "$OUT/$label-test.log" | head -1))"
    else
        fail "$label tests"; tail -30 "$OUT/$label-test.log"
    fi
}

note "gcc"
build_and_test gcc gcc g++ -DCMAKE_BUILD_TYPE=RelWithDebInfo -DAUK_WERROR=ON

note "clang"
build_and_test clang clang clang++ -DCMAKE_BUILD_TYPE=RelWithDebInfo -DAUK_WERROR=ON

note "address + UB sanitizers"
if have clang++; then
    build_and_test asan clang clang++ -DCMAKE_BUILD_TYPE=Debug -DAUK_SANITIZE=ON
else
    build_and_test asan gcc g++ -DCMAKE_BUILD_TYPE=Debug -DAUK_SANITIZE=ON
fi

# ---------------------------------------------------------------------------
note "cross-compile for Cortex-M4"
if have arm-none-eabi-g++; then
    cat > "$OUT/arm.cmake" <<'EOF'
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_C_COMPILER   arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(FLAGS "-mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti")
set(CMAKE_C_FLAGS_INIT   "${FLAGS}")
set(CMAKE_CXX_FLAGS_INIT "${FLAGS}")
EOF
    if cmake -S "$REPO" -B "$OUT/arm" -DCMAKE_TOOLCHAIN_FILE="$OUT/arm.cmake" \
             -DCMAKE_BUILD_TYPE=MinSizeRel > "$OUT/arm-cfg.log" 2>&1 \
       && cmake --build "$OUT/arm" --target auk --parallel > "$OUT/arm-build.log" 2>&1; then
        bytes=$(arm-none-eabi-size -t "$OUT/arm/libauk.a" 2>/dev/null | tail -1 | awk '{print $4}')
        pass "cortex-m4 cross-compile (${bytes:-?} bytes)"
    else
        fail "cortex-m4 cross-compile"; tail -20 "$OUT/arm-build.log"
    fi
else
    skip "cortex-m4 cross-compile (no arm-none-eabi-g++)"
fi

# ---------------------------------------------------------------------------
note "clang-format"
if have clang-format; then
    if find "$REPO/include" "$REPO/src" "$REPO/sim" "$REPO/tests" \
            -name '*.hpp' -o -name '*.cpp' \
       | xargs clang-format --dry-run --Werror > "$OUT/format.log" 2>&1; then
        pass "clang-format"
    else
        fail "clang-format"; head -12 "$OUT/format.log"
    fi
else
    skip "clang-format (not installed)"
fi

# ---------------------------------------------------------------------------
note "clang-tidy"
if have clang-tidy; then
    # clang-tidy needs a compilation database so it analyses with the same flags
    # the real build uses. Configuring is enough; nothing has to be compiled.
    if cmake -S "$REPO" -B "$OUT/tidy" -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
             -DCMAKE_BUILD_TYPE=RelWithDebInfo > "$OUT/tidy-cfg.log" 2>&1; then
        if clang-tidy -p "$OUT/tidy" --quiet --warnings-as-errors='*' \
               "$REPO"/src/core/*.cpp "$REPO"/src/control/*.cpp "$REPO"/src/power/*.cpp \
               "$REPO"/src/safety/*.cpp "$REPO"/src/app/*.cpp > "$OUT/tidy.log" 2>&1; then
            pass "clang-tidy"
        else
            fail "clang-tidy"; grep -E 'warning:|error:' "$OUT/tidy.log" | head -12
        fi
    else
        fail "clang-tidy configure"; tail -10 "$OUT/tidy-cfg.log"
    fi
else
    skip "clang-tidy (not installed)"
fi

# ---------------------------------------------------------------------------
note "simulator"
demo=""
for d in gcc clang asan; do
    [ -x "$OUT/$d/auk-sim-demo" ] && { demo="$OUT/$d/auk-sim-demo"; break; }
done
if [ -n "$demo" ]; then
    if "$demo" > "$OUT/demo.log" 2>&1 && grep -q 'watchdog resets: 0' "$OUT/demo.log"; then
        pass "simulator demo"
    else
        fail "simulator demo"; tail -20 "$OUT/demo.log"
    fi
else
    skip "simulator demo (nothing built)"
fi

printf '\n=============================== summary ===============================\n'
printf '%s\n' "${results[@]}"
printf '=======================================================================\n'
if [ $rc -eq 0 ]; then
    echo "all available checks pass   (logs in $OUT)"
else
    echo "FAILURES -- see $OUT"
fi
exit $rc
