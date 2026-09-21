#pragma once

#include <expected>
#include <utility>

#include "robonode/core/status.hpp"

namespace robonode {

// A value, or the reason there isn't one. The project is C++23, so this is the
// standard type — no dependency, no hand-rolled variant.
//
// `Status f(Args…, T& out)` forces every caller through the same three lines:
// declare an empty T, pass it in, check the Status, and hope nobody reads the T
// on the failing path. `Result<T>` makes the failing path unreadable instead of
// merely wrong, and lets a function be used in an expression.
//
// Status stays: it is the right answer for a call that returns nothing but can
// fail (`run()`, `stop()`, `save()`). The two compose — a Result carries a
// Status as its error.
template <class T>
using Result = std::expected<T, Status>;

// `return no(…)` at a call site reads as the sentence it is. Named for the
// answer it gives, not for the mechanism, and short enough not to bury it.
inline std::unexpected<Status> no(std::string why) {
    return std::unexpected{Status::failure(std::move(why))};
}

inline std::unexpected<Status> no(Status why) { return std::unexpected{std::move(why)}; }

}  // namespace robonode
