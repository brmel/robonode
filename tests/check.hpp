#pragma once

#include <cstdio>
#include <cstdlib>

// assert() vanishes under NDEBUG (Release); tests must fail in every build
// type. Shared by all module test binaries; grows into Catch2 when the
// sim-gate suites land (FR-3.3).
#define CHECK(cond)                                                              \
    do {                                                                         \
        if (!(cond)) {                                                           \
            std::fprintf(stderr, "CHECK failed %s:%d: %s\n", __FILE__, __LINE__, \
                         #cond);                                                 \
            std::abort();                                                        \
        }                                                                        \
    } while (0)
