#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <wasmtime.h>

#include "robonode/core/status.hpp"

namespace robonode::wasm {

// A compiled WebAssembly module run under Wasmtime with fuel (instruction
// budget) enabled — the real untrusted-code sandbox: no WASI, no imports, no
// host access; a runaway guest exhausts fuel and traps (rejected, never hangs).
// One module, many evals; each eval gets a fresh store so guests never share
// state. Off the 1 kHz path (ADR-5).
class WasmModule {
public:
    static Status create(const std::vector<std::uint8_t>& wasm,
                         std::unique_ptr<WasmModule>& out) {
        wasm_config_t* config = wasm_config_new();
        wasmtime_config_consume_fuel_set(config, true);
        wasm_engine_t* engine = wasm_engine_new_with_config(config);
        wasmtime_module_t* module = nullptr;
        if (wasmtime_error_t* err =
                wasmtime_module_new(engine, wasm.data(), wasm.size(), &module)) {
            wasmtime_error_delete(err);
            wasm_engine_delete(engine);
            return Status::failure("wasm: invalid module");
        }
        out.reset(new WasmModule{engine, module});
        return Status::success();
    }

    ~WasmModule() {
        wasmtime_module_delete(module_);
        wasm_engine_delete(engine_);
    }

    WasmModule(const WasmModule&) = delete;
    WasmModule& operator=(const WasmModule&) = delete;

    // Call exports c0..c<n-1> with `inputs` (f64 params); collect one f64 each.
    // Any trap (incl. fuel exhaustion) or missing export fails closed.
    Status eval(const std::vector<double>& inputs, std::size_t n_channels,
                std::vector<double>& out, std::uint64_t fuel = 1'000'000) const {
        wasmtime_store_t* store = wasmtime_store_new(engine_, nullptr, nullptr);
        wasmtime_context_t* ctx = wasmtime_store_context(store);
        wasmtime_context_set_fuel(ctx, fuel);

        wasmtime_instance_t inst;
        wasm_trap_t* trap = nullptr;
        if (wasmtime_error_t* err =
                wasmtime_instance_new(ctx, module_, nullptr, 0, &inst, &trap)) {
            wasmtime_error_delete(err);
            wasmtime_store_delete(store);
            return Status::failure("wasm: instantiate failed");
        }

        std::vector<wasmtime_val_t> args(inputs.size());
        for (std::size_t i = 0; i < inputs.size(); ++i) {
            args[i].kind = WASMTIME_F64;
            args[i].of.f64 = inputs[i];
        }

        out.clear();
        out.reserve(n_channels);
        for (std::size_t k = 0; k < n_channels; ++k) {
            const std::string name = "c" + std::to_string(k);
            wasmtime_extern_t fn;
            if (!wasmtime_instance_export_get(ctx, &inst, name.c_str(), name.size(), &fn)) {
                wasmtime_store_delete(store);
                return Status::failure("wasm: missing channel export");
            }
            wasmtime_val_t result;
            if (wasmtime_error_t* err = wasmtime_func_call(ctx, &fn.of.func, args.data(),
                                                           args.size(), &result, 1, &trap)) {
                wasmtime_error_delete(err);
                wasmtime_store_delete(store);
                return Status::failure("wasm: call failed");
            }
            if (trap) {
                wasm_trap_delete(trap);
                wasmtime_store_delete(store);
                return Status::failure("wasm: trap (fuel/limit)");
            }
            out.push_back(result.of.f64);
        }
        wasmtime_store_delete(store);
        return Status::success();
    }

private:
    WasmModule(wasm_engine_t* engine, wasmtime_module_t* module)
        : engine_{engine}, module_{module} {}

    wasm_engine_t* engine_;
    wasmtime_module_t* module_;
};

}  // namespace robonode::wasm
