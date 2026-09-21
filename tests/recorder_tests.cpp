// robonode::recorder module tests — depends on core + recorder only
// (module boundary exercised by the include set itself).

#include <cstdio>
#include <vector>

#include "check.hpp"
#include "robonode/recorder/mcap_recorder.hpp"

namespace {

void test_mcap_recorder_writes_valid_file() {
    std::vector<std::vector<robonode::TelemetryRow>> rows{
        {{0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0}, {0.001, 1.1, 2.1, 3.1, 4.1, 5.1, 6.1}}};
    const auto st = robonode::McapRecorder::write(
        "test-recorder.mcap", {"rn/test/axis/MotionAxis/telemetry"}, rows, 1000.0);
    CHECK(st.ok());
    std::FILE* f = std::fopen("test-recorder.mcap", "rb");
    CHECK(f != nullptr);
    char magic[8]{};
    CHECK(std::fread(magic, 1, 8, f) == 8);
    std::fclose(f);
    std::remove("test-recorder.mcap");
    // MCAP magic: \x89 M C A P 0 \r \n
    CHECK(magic[1] == 'M' && magic[2] == 'C' && magic[3] == 'A' && magic[4] == 'P');
}

void test_mcap_recorder_rejects_mismatched_inputs() {
    const auto st = robonode::McapRecorder::write("nope.mcap", {"a", "b"}, {{}}, 1000.0);
    CHECK(!st.ok());
    CHECK(!st.message().empty());
}

}  // namespace

int main() {
    test_mcap_recorder_writes_valid_file();
    test_mcap_recorder_rejects_mismatched_inputs();
    std::puts("robonode recorder: all tests passed");
    return 0;
}
