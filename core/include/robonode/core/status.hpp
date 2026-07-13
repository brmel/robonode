#pragma once

#include <string>
#include <utility>

namespace robonode {

// The platform's one error model for non-RT surfaces:
//   - configuration/creation/IO verbs return Status (MIL-style: every verb
//     reports, callers decide),
//   - the RT path stays noexcept and latches faults into state
//     (AxisState.safety) — never allocates, never throws.
// Exceptions do not cross module boundaries.
class [[nodiscard]] Status {
public:
    static Status success() { return Status{true, {}}; }
    static Status failure(std::string message) { return Status{false, std::move(message)}; }

    [[nodiscard]] bool ok() const { return ok_; }
    [[nodiscard]] const std::string& message() const { return message_; }
    explicit operator bool() const { return ok_; }

private:
    Status(bool ok, std::string message) : ok_{ok}, message_{std::move(message)} {}
    bool ok_;
    std::string message_;
};

}  // namespace robonode
