#pragma once

#include <cstdint>
#include <vector>

#include "robonode/core/status.hpp"

namespace robonode::sandbox {

struct SandboxLimits {
    std::size_t max_ops{512};
    std::size_t max_stack{64};
};

enum class Op : std::uint8_t { PushConst, LoadInput, Add, Sub, Mul, Div, Neg, Min, Max, Abs, Clamp };

struct Instr {
    Op op;
    double value{};
    std::uint32_t input{};
};

using ByteCode = std::vector<Instr>;

struct Program {
    std::vector<ByteCode> channels;
    std::size_t input_count{};
};

inline Status eval_channel(const ByteCode& code, const std::vector<double>& in,
                           const SandboxLimits& lim, double& out) {
    std::vector<double> st;
    st.reserve(lim.max_stack);
    const auto push = [&](double v) -> bool {
        if (st.size() >= lim.max_stack) return false;
        st.push_back(v);
        return true;
    };
    std::size_t ops = 0;
    for (const auto& ins : code) {
        if (++ops > lim.max_ops) return Status::failure("sandbox: op budget exceeded");
        switch (ins.op) {
            case Op::PushConst:
                if (!push(ins.value)) return Status::failure("sandbox: stack overflow");
                break;
            case Op::LoadInput:
                if (ins.input >= in.size()) return Status::failure("sandbox: input out of range");
                if (!push(in[ins.input])) return Status::failure("sandbox: stack overflow");
                break;
            case Op::Neg:
            case Op::Abs: {
                if (st.empty()) return Status::failure("sandbox: stack underflow");
                double& a = st.back();
                a = ins.op == Op::Neg ? -a : (a < 0 ? -a : a);
                break;
            }
            case Op::Add:
            case Op::Sub:
            case Op::Mul:
            case Op::Div:
            case Op::Min:
            case Op::Max: {
                if (st.size() < 2) return Status::failure("sandbox: stack underflow");
                const double b = st.back();
                st.pop_back();
                double& a = st.back();
                switch (ins.op) {
                    case Op::Add: a = a + b; break;
                    case Op::Sub: a = a - b; break;
                    case Op::Mul: a = a * b; break;
                    case Op::Div: a = a / b; break;
                    case Op::Min: a = a < b ? a : b; break;
                    default: a = a > b ? a : b; break;
                }
                break;
            }
            case Op::Clamp: {
                if (st.size() < 3) return Status::failure("sandbox: stack underflow");
                const double hi = st.back(); st.pop_back();
                const double lo = st.back(); st.pop_back();
                double& x = st.back();
                x = x < lo ? lo : (x > hi ? hi : x);
                break;
            }
        }
    }
    if (st.size() != 1) return Status::failure("sandbox: expression did not reduce to one value");
    out = st.back();
    return Status::success();
}

inline Status run(const Program& p, const std::vector<double>& in, const SandboxLimits& lim,
                  std::vector<double>& out) {
    if (in.size() != p.input_count) return Status::failure("sandbox: input arity mismatch");
    out.clear();
    out.reserve(p.channels.size());
    for (const auto& c : p.channels) {
        double v = 0.0;
        if (const auto s = eval_channel(c, in, lim, v); !s.ok()) return s;
        out.push_back(v);
    }
    return Status::success();
}

}  // namespace robonode::sandbox
