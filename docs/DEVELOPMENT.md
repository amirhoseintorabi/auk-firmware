# Development

## Running every check

```sh
tools/check-all.sh
```

Runs everything CI runs — gcc, clang, sanitizers, the Cortex-M4 cross-compile,
clang-format and clang-tidy — and prints a summary. Logs land in `.check/`.

Missing tools are reported as `SKIP` rather than failing, so the script is useful
with whatever you happen to have. With the full set installed it is exactly what
GitHub Actions does:

```
PASS  gcc build
PASS  gcc tests (100% tests passed)
PASS  clang build
PASS  clang tests (100% tests passed)
PASS  asan build
PASS  asan tests (100% tests passed)
PASS  cortex-m4 cross-compile (3867 bytes)
PASS  clang-format
PASS  clang-tidy
PASS  simulator demo
```

## What you need

Building and testing needs only **CMake 3.16 and a C++17 compiler**. Everything
else below is for reproducing the rest of the CI matrix locally.

### Debian / Ubuntu

```sh
sudo apt install build-essential cmake clang clang-format clang-tidy \
                 gcc-arm-none-eabi
```

### macOS

```sh
brew install cmake llvm arm-none-eabi-gcc
```

Apple's `clang` does not ship `clang-tidy`; the Homebrew `llvm` keg does, under
`$(brew --prefix llvm)/bin`.

### Without root

`apt-get download` and `dpkg-deb -x` work unprivileged, which is enough to get
clang into a user prefix on a machine you do not administer:

```sh
mkdir -p /tmp/clang && cd /tmp/clang
apt-get download clang-18 libclang-common-18-dev libclang-rt-18-dev \
                 clang-format-18 clang-tidy-18
for d in *.deb; do dpkg-deb -x "$d" stage; done
mkdir -p ~/.local/lib
cp -a stage/usr/lib/llvm-18 ~/.local/lib/
ln -sf ~/.local/lib/llvm-18/bin/clang    ~/.local/bin/clang
ln -sf ~/.local/lib/llvm-18/bin/clang++  ~/.local/bin/clang++
ln -sf ~/.local/lib/llvm-18/bin/clang-format ~/.local/bin/clang-format
ln -sf ~/.local/lib/llvm-18/bin/clang-tidy   ~/.local/bin/clang-tidy
```

If `libclang-cpp18` is already installed system-wide — it often is, since other
packages pull it in — the driver will find it. If not, symlink
`/usr/lib/llvm-18/lib/*.so*` into `~/.local/lib/llvm-18/lib/`.

Verify the sanitizer runtimes came across, because a clang that compiles fine can
still be missing them:

```sh
printf '#include <cstdio>\nint main(){int*p=new int[4];p[5]=1;printf("%%d",p[5]);}\n' > /tmp/t.cpp
clang++ -fsanitize=address,undefined -g /tmp/t.cpp -o /tmp/t && /tmp/t
```

That should report a heap-buffer-overflow. Silence means the runtime is absent.

## Individual checks

```sh
# build and test
cmake -B build && cmake --build build
ctest --test-dir build --output-on-failure

# sanitizers
cmake -B build-asan -DCMAKE_BUILD_TYPE=Debug -DAUK_SANITIZE=ON
cmake --build build-asan
ASAN_OPTIONS=detect_leaks=1 UBSAN_OPTIONS=halt_on_error=1 \
  ctest --test-dir build-asan --output-on-failure

# formatting
find include src sim tests -name '*.hpp' -o -name '*.cpp' | xargs clang-format -i

# static analysis
cmake -B build -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
clang-tidy -p build src/*/*.cpp
```

## Conventions

**Warnings are part of the build contract.** The library compiles under
`-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion
-Wdouble-promotion -Wold-style-cast -Werror`. If a warning is wrong, suppress it
narrowly and say why; do not widen the exclusion.

`-Wdouble-promotion` is the one that catches people. This codebase works in
`float`, and any implicit widening to `double` is either a real accident or
belongs in an explicit cast. The test harness absorbs the widening once, in
`CHECK_NEAR`, so assertions can be written naturally.

**clang-tidy is gated, not advisory.** `.clang-tidy` selects the categories that
matter in firmware and excludes four, each with its reason written in the file.
Something new should be fixed, or excluded with a reason — not left to
accumulate.

**Tests state behaviour, not implementation.** `tests/test_safety.cpp` is the
interlock's specification; if you change what the robot is allowed to do, the
diff should delete or change a named case there. That is deliberate: it makes
giving up a safety property visible in review.

**Tolerances are derived, not tuned.** Where a test bounds a physical quantity,
the comment shows the arithmetic. See the bumper stopping distance in
`tests/test_integration.cpp` for the pattern — if such a test fails, check the
model before touching the number.
