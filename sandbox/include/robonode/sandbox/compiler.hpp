#pragma once

#include <cctype>
#include <string>
#include <string_view>
#include <vector>

#include "robonode/core/status.hpp"
#include "robonode/sandbox/program.hpp"

namespace robonode::sandbox {

class Compiler {
public:
    static Status compile(const std::string& source, const std::vector<std::string>& input_names,
                          std::size_t expected_channels, Program& out) {
        Program prog;
        prog.input_count = input_names.size();
        for (const auto& seg : split(source)) {
            Compiler c{seg, input_names};
            ByteCode code;
            if (const auto st = c.parse_channel(code); !st.ok()) return st;
            prog.channels.push_back(std::move(code));
        }
        if (prog.channels.size() != expected_channels) {
            return Status::failure("sandbox: expected " + std::to_string(expected_channels) +
                                   " expression(s), got " + std::to_string(prog.channels.size()));
        }
        out = std::move(prog);
        return Status::success();
    }

private:
    Compiler(std::string_view src, const std::vector<std::string>& names)
        : s_{src}, names_{names} {}

    static std::vector<std::string> split(const std::string& source) {
        std::vector<std::string> segs;
        std::string cur;
        for (const char ch : source) {
            if (ch == ';' || ch == '\n') {
                if (!blank(cur)) segs.push_back(cur);
                cur.clear();
            } else {
                cur.push_back(ch);
            }
        }
        if (!blank(cur)) segs.push_back(cur);
        return segs;
    }
    static bool blank(const std::string& t) {
        for (const char ch : t) if (!std::isspace(static_cast<unsigned char>(ch))) return false;
        return true;
    }

    Status parse_channel(ByteCode& code) {
        code_ = &code;
        if (const auto st = expr(); !st.ok()) return st;
        skip_ws();
        if (i_ != s_.size()) return Status::failure("sandbox: trailing tokens in expression");
        return Status::success();
    }

    Status expr() {
        if (const auto st = term(); !st.ok()) return st;
        for (;;) {
            skip_ws();
            const char op = peek();
            if (op != '+' && op != '-') return Status::success();
            ++i_;
            if (const auto st = term(); !st.ok()) return st;
            emit({op == '+' ? Op::Add : Op::Sub});
        }
    }

    Status term() {
        if (const auto st = factor(); !st.ok()) return st;
        for (;;) {
            skip_ws();
            const char op = peek();
            if (op != '*' && op != '/') return Status::success();
            ++i_;
            if (const auto st = factor(); !st.ok()) return st;
            emit({op == '*' ? Op::Mul : Op::Div});
        }
    }

    Status factor() {
        skip_ws();
        const char ch = peek();
        if (ch == '(') {
            ++i_;
            if (const auto st = expr(); !st.ok()) return st;
            skip_ws();
            if (peek() != ')') return Status::failure("sandbox: missing ')'");
            ++i_;
            return Status::success();
        }
        if (ch == '-') {
            ++i_;
            if (const auto st = factor(); !st.ok()) return st;
            emit({Op::Neg});
            return Status::success();
        }
        if (std::isdigit(static_cast<unsigned char>(ch)) || ch == '.') return number();
        if (std::isalpha(static_cast<unsigned char>(ch)) || ch == '_') return identifier();
        return Status::failure("sandbox: unexpected character in expression");
    }

    Status number() {
        const std::size_t start = i_;
        while (i_ < s_.size() &&
               (std::isdigit(static_cast<unsigned char>(s_[i_])) || s_[i_] == '.')) {
            ++i_;
        }
        try {
            emit({Op::PushConst, std::stod(std::string{s_.substr(start, i_ - start)})});
        } catch (...) {
            return Status::failure("sandbox: bad number literal");
        }
        return Status::success();
    }

    Status identifier() {
        const std::size_t start = i_;
        while (i_ < s_.size() &&
               (std::isalnum(static_cast<unsigned char>(s_[i_])) || s_[i_] == '_')) {
            ++i_;
        }
        const std::string_view id = s_.substr(start, i_ - start);
        skip_ws();
        if (peek() == '(') return call(id);
        for (std::uint32_t k = 0; k < names_.size(); ++k) {
            if (id == names_[k]) {
                emit({Op::LoadInput, 0.0, k});
                return Status::success();
            }
        }
        return Status::failure("sandbox: unknown identifier '" + std::string{id} + "'");
    }

    Status call(std::string_view name) {
        ++i_;
        std::size_t argc = 0;
        if (peek() != ')') {
            for (;;) {
                if (const auto st = expr(); !st.ok()) return st;
                ++argc;
                skip_ws();
                if (peek() == ',') { ++i_; continue; }
                break;
            }
        }
        skip_ws();
        if (peek() != ')') return Status::failure("sandbox: missing ')' in call");
        ++i_;
        if (name == "min" && argc == 2) emit({Op::Min});
        else if (name == "max" && argc == 2) emit({Op::Max});
        else if (name == "abs" && argc == 1) emit({Op::Abs});
        else if (name == "clamp" && argc == 3) emit({Op::Clamp});
        else return Status::failure("sandbox: unknown function '" + std::string{name} + "'");
        return Status::success();
    }

    void emit(Instr ins) { code_->push_back(ins); }
    void skip_ws() {
        while (i_ < s_.size() && std::isspace(static_cast<unsigned char>(s_[i_]))) ++i_;
    }
    [[nodiscard]] char peek() const { return i_ < s_.size() ? s_[i_] : '\0'; }

    std::string_view s_;
    const std::vector<std::string>& names_;
    std::size_t i_{0};
    ByteCode* code_{nullptr};
};

}  // namespace robonode::sandbox
