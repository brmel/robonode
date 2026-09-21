#pragma once

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "robonode/sandbox/program.hpp"

namespace robonode::wasm {

// Emit a WebAssembly module from a sandbox::Program. Each output channel becomes
// an exported function "c<k>" of type (f64,f64,f64)->f64 (the inputs are the
// function params; two extra f64 locals scratch clamp). The bytecode is
// straight-line, so this is a direct 1:1 lowering — no control flow. The result
// runs in Wasmtime with the same numeric meaning as the interpreter, but
// memory-isolated and fuel-limited.
namespace detail {

inline void uleb(std::vector<std::uint8_t>& out, std::uint32_t v) {
    do {
        std::uint8_t b = v & 0x7f;
        v >>= 7;
        if (v) b |= 0x80;
        out.push_back(b);
    } while (v);
}

inline void section(std::vector<std::uint8_t>& out, std::uint8_t id,
                    const std::vector<std::uint8_t>& body) {
    out.push_back(id);
    uleb(out, static_cast<std::uint32_t>(body.size()));
    out.insert(out.end(), body.begin(), body.end());
}

inline void f64_const(std::vector<std::uint8_t>& code, double v) {
    code.push_back(0x44);
    std::uint8_t bytes[8];
    std::memcpy(bytes, &v, 8);
    code.insert(code.end(), bytes, bytes + 8);
}

inline void emit_channel(std::vector<std::uint8_t>& code, const sandbox::ByteCode& bc) {
    for (const auto& ins : bc) {
        switch (ins.op) {
            case sandbox::Op::PushConst: f64_const(code, ins.value); break;
            case sandbox::Op::LoadInput: code.push_back(0x20); uleb(code, ins.input); break;
            case sandbox::Op::Add: code.push_back(0xA0); break;
            case sandbox::Op::Sub: code.push_back(0xA1); break;
            case sandbox::Op::Mul: code.push_back(0xA2); break;
            case sandbox::Op::Div: code.push_back(0xA3); break;
            case sandbox::Op::Neg: code.push_back(0x9A); break;
            case sandbox::Op::Abs: code.push_back(0x99); break;
            case sandbox::Op::Min: code.push_back(0xA4); break;
            case sandbox::Op::Max: code.push_back(0xA5); break;
            case sandbox::Op::Clamp:
                code.push_back(0x21); uleb(code, 4);  // local.set hi (local 4)
                code.push_back(0x21); uleb(code, 3);  // local.set lo (local 3)
                code.push_back(0x20); uleb(code, 3);  // local.get lo
                code.push_back(0xA5);                  // f64.max(x, lo)
                code.push_back(0x20); uleb(code, 4);  // local.get hi
                code.push_back(0xA4);                  // f64.min(., hi)
                break;
        }
    }
    code.push_back(0x0B);  // end
}

}  // namespace detail

inline std::vector<std::uint8_t> emit(const sandbox::Program& prog) {
    using namespace detail;
    const std::uint32_t n = static_cast<std::uint32_t>(prog.channels.size());

    std::vector<std::uint8_t> mod{0x00, 0x61, 0x73, 0x6d, 0x01, 0x00, 0x00, 0x00};

    std::vector<std::uint8_t> type{0x01, 0x60, 0x03, 0x7c, 0x7c, 0x7c, 0x01, 0x7c};
    section(mod, 0x01, type);

    std::vector<std::uint8_t> func;
    uleb(func, n);
    for (std::uint32_t k = 0; k < n; ++k) uleb(func, 0);  // all type 0
    section(mod, 0x03, func);

    std::vector<std::uint8_t> exp;
    uleb(exp, n);
    for (std::uint32_t k = 0; k < n; ++k) {
        const std::string name = "c" + std::to_string(k);
        uleb(exp, static_cast<std::uint32_t>(name.size()));
        exp.insert(exp.end(), name.begin(), name.end());
        exp.push_back(0x00);  // func kind
        uleb(exp, k);
    }
    section(mod, 0x07, exp);

    std::vector<std::uint8_t> codesec;
    uleb(codesec, n);
    for (const auto& ch : prog.channels) {
        std::vector<std::uint8_t> body{0x01, 0x02, 0x7c};  // 1 local group: 2 x f64 (clamp scratch)
        emit_channel(body, ch);
        uleb(codesec, static_cast<std::uint32_t>(body.size()));
        codesec.insert(codesec.end(), body.begin(), body.end());
    }
    section(mod, 0x0A, codesec);

    return mod;
}

}  // namespace robonode::wasm
