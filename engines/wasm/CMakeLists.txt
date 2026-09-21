# robonode wasm engine — vendors the Wasmtime C-API (prebuilt, per platform) and
# exposes it as an imported target behind robonode::wasm. Opt-in
# (ROBONODE_BUILD_WASM) since it downloads a platform-specific artifact.
include(FetchContent)

set(WASMTIME_VERSION v29.0.0)
if(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
  set(_wt_os macos)
else()
  set(_wt_os linux)
endif()
if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm64|aarch64")
  set(_wt_arch aarch64)
else()
  set(_wt_arch x86_64)
endif()
set(_wt_name wasmtime-${WASMTIME_VERSION}-${_wt_arch}-${_wt_os}-c-api)

FetchContent_Declare(wasmtime
  URL https://github.com/bytecodealliance/wasmtime/releases/download/${WASMTIME_VERSION}/${_wt_name}.tar.xz)
FetchContent_MakeAvailable(wasmtime)

add_library(wasmtime_ffi STATIC IMPORTED GLOBAL)
set_target_properties(wasmtime_ffi PROPERTIES
  IMPORTED_LOCATION ${wasmtime_SOURCE_DIR}/lib/libwasmtime.a
  INTERFACE_INCLUDE_DIRECTORIES ${wasmtime_SOURCE_DIR}/include)
if(APPLE)
  target_link_libraries(wasmtime_ffi INTERFACE
    "-framework Security" "-framework CoreFoundation" pthread dl m)
else()
  target_link_libraries(wasmtime_ffi INTERFACE pthread dl m)
endif()

add_library(robonode_wasm INTERFACE)
add_library(robonode::wasm ALIAS robonode_wasm)
target_include_directories(robonode_wasm INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(robonode_wasm INTERFACE
  robonode::core robonode::sandbox robonode::vision wasmtime_ffi)
