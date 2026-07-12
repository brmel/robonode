#include <catch2/catch_test_macros.hpp>

#include <thread>
#include <vector>

#include "trajlib/spsc_ring.hpp"

using trajlib::SpscRing;

TEST_CASE("push then pop returns items in order") {
    SpscRing<int, 8> ring;
    for (int i = 0; i < 5; ++i) REQUIRE(ring.push(i));
    for (int i = 0; i < 5; ++i) {
        auto v = ring.pop();
        REQUIRE(v.has_value());
        CHECK(*v == i);
    }
    CHECK_FALSE(ring.pop().has_value());
}

TEST_CASE("full ring rejects push, one slot sacrificed") {
    SpscRing<int, 4> ring;  // usable capacity = 3
    CHECK(ring.push(1));
    CHECK(ring.push(2));
    CHECK(ring.push(3));
    CHECK_FALSE(ring.push(4));
    ring.pop();
    CHECK(ring.push(4));
}

TEST_CASE("two threads transfer everything in order") {
    SpscRing<int, 64> ring;
    constexpr int kN = 100000;

    std::thread producer{[&] {
        for (int i = 0; i < kN; ++i) {
            while (!ring.push(i)) std::this_thread::yield();
        }
    }};

    std::vector<int> got;
    got.reserve(kN);
    while (static_cast<int>(got.size()) < kN) {
        if (auto v = ring.pop()) got.push_back(*v);
    }
    producer.join();

    REQUIRE(static_cast<int>(got.size()) == kN);
    for (int i = 0; i < kN; ++i) {
        if (got[i] != i) FAIL("out of order at " << i);
    }
}
