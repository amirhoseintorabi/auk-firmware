// SPDX-License-Identifier: MIT
#pragma once

// A test harness small enough to read in one sitting.
//
// No external dependency on purpose: the point of this repository is that it
// builds anywhere with a C++17 compiler and nothing else, and a test framework
// that has to be fetched undermines that. What is here -- named cases, floating
// point comparison with a tolerance, a failure count -- is the whole of what
// these tests need.

#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace auk::test
{

inline int g_failures = 0;
inline int g_checks = 0;
inline const char* g_current_case = "";

inline void begin_case(const char* name)
{
    g_current_case = name;
    std::printf("  %s\n", name);
}

inline void fail(const char* file, int line, const char* expr)
{
    ++g_failures;
    std::printf("    FAIL %s:%d in \"%s\"\n           %s\n", file, line, g_current_case,
                expr);
}

inline bool close(double a, double b, double tolerance)
{
    return std::fabs(a - b) <= tolerance;
}

inline int summary(const char* suite)
{
    std::printf("\n%s: %d checks, %d failure(s)\n", suite, g_checks, g_failures);
    return (g_failures == 0) ? 0 : 1;
}

}  // namespace auk::test

#define CASE(name) ::auk::test::begin_case(name)

#define CHECK(expr)                                       \
    do                                                    \
    {                                                     \
        ++::auk::test::g_checks;                          \
        if (!(expr))                                      \
        {                                                 \
            ::auk::test::fail(__FILE__, __LINE__, #expr); \
        }                                                 \
    } while (0)

// The casts are explicit so that a caller may pass floats -- which is what most
// of this codebase deals in -- without tripping -Wdouble-promotion. Absorbing
// the widening here, once and visibly, is the harness's job; making every call
// site cast would be noise, and turning the warning off would lose it in the
// library code where it is worth having.
#define CHECK_NEAR(a, b, tol)                                                   \
    do                                                                          \
    {                                                                           \
        ++::auk::test::g_checks;                                                \
        if (!::auk::test::close(static_cast<double>(a), static_cast<double>(b), \
                                static_cast<double>(tol)))                      \
        {                                                                       \
            char msg[256];                                                      \
            std::snprintf(msg, sizeof(msg), "%s == %s  (%.6f vs %.6f)", #a, #b, \
                          static_cast<double>(a), static_cast<double>(b));      \
            ::auk::test::fail(__FILE__, __LINE__, msg);                         \
        }                                                                       \
    } while (0)
