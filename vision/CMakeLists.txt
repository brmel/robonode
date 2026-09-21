# robonode::vision — the vision capability seam (ADR-11). Header-only: the
# Detector interface + its ModuleRegistry, and the basic toy version. Depends
# on core (geometry) + motion (ModuleRegistry); never on a vendor CV library —
# real detectors (OpenCV/ONNX, #38) and user WASM versions register from
# outside, so vision stays backend-blind.
add_library(robonode_vision INTERFACE)
add_library(robonode::vision ALIAS robonode_vision)
target_include_directories(robonode_vision INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(robonode_vision INTERFACE robonode::core robonode::motion robonode::sandbox)
