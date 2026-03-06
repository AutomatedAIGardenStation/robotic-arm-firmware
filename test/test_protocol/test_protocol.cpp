#include <unity.h>
#include <string.h>

char last_event[128] = {0};

#undef ARDUINO
void delay(unsigned long ms) {}
uint32_t millis() { return 0; }

// In `src/protocol.cpp`, `protocol_emit_event` uses `Serial.println`.
// We can define a macro to replace `Serial.println` before including `protocol.cpp`.
// Wait, `Serial.println` is a method call.
// We can rename `Serial` in `protocol.cpp` to `ProtocolSerial` and define a macro.

// Actually, `protocol.cpp` defines `void protocol_emit_event(const char* event)`.
// And it uses `Serial.println(event)`.
// We renamed `SerialDummy` to `ProtocolSerialDummy` and `Serial` to `ProtocolSerial`.
// So inside `protocol.cpp`, `Serial.println` calls `ProtocolSerial.println()`.
// Since we #include "protocol.cpp", we don't need to define `protocol_emit_event_hidden`!
// We can just define `ProtocolSerial` to do our intercepting!

#define PROTOCOL_SERIAL_DUMMY_DEFINED
class ProtocolSerialDummy {
public:
    void print(const char* s) {}
    void println(const char* s) {
        strncpy(last_event, s, sizeof(last_event) - 1);
        last_event[sizeof(last_event) - 1] = '\0';
    }
};
static ProtocolSerialDummy Serial;

#include "../../src/protocol.cpp"


void setUp(void) {
    last_event[0] = '\0';
    for (int i = 0; i < 6; i++) {
        g_drivers[i].reset();
        g_limit_switches[i].setTriggered(false);
        g_encoders[i].reset();
    }
    g_encoder_reader.resetAll();

    // Replace the global g_motion_controller instantiated in protocol.cpp
    // This calls the copy assignment operator or we can just reconstruct it using placement new
    g_motion_controller = MotionController(g_driver_ptrs, &g_safety_monitor, nullptr);

    Command cmd;
    cmd.type = CommandType::ARM_HOME;
    g_motion_controller.execute(cmd);
    while (g_motion_controller.getState() == MotionState::MOVING) {
        g_motion_controller.update();
    }
    last_event[0] = '\0'; // clear ARM_DONE from HOME
}

void tearDown(void) {
}

void test_arm_move_to_parsing(void) {
    bool result = protocol_handle_line("ARM_MOVE_TO:x=100.5:y=200.0:z=50.2\n");
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(CommandType::ARM_MOVE_TO, g_motion_controller.current_cmd.type);
    TEST_ASSERT_TRUE(g_motion_controller.current_cmd.has_x);
    TEST_ASSERT_EQUAL_FLOAT(100.5f, g_motion_controller.current_cmd.x);
    TEST_ASSERT_TRUE(g_motion_controller.current_cmd.has_y);
    TEST_ASSERT_EQUAL_FLOAT(200.0f, g_motion_controller.current_cmd.y);
    TEST_ASSERT_TRUE(g_motion_controller.current_cmd.has_z);
    TEST_ASSERT_EQUAL_FLOAT(50.2f, g_motion_controller.current_cmd.z);
}

void test_wrist_set_parsing(void) {
    bool result = protocol_handle_line("WRIST_SET:pitch=45.0:roll=-45.0\n");
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(CommandType::WRIST_SET, g_motion_controller.current_cmd.type);
    TEST_ASSERT_TRUE(g_motion_controller.current_cmd.has_pitch);
    TEST_ASSERT_EQUAL_FLOAT(45.0f, g_motion_controller.current_cmd.pitch);
    TEST_ASSERT_TRUE(g_motion_controller.current_cmd.has_roll);
    TEST_ASSERT_EQUAL_FLOAT(-45.0f, g_motion_controller.current_cmd.roll);
}

void test_tool_release_parsing(void) {
    // In actual runtime, the newline is trimmed by the serial reader loop, but protocol_handle_line
    // also has a trimming logic at the start. However, our mock might pass the constant string literal
    // and `strlen` gets the full length including `\n`. Wait, `line` is const char*, we can't trim it in-place
    // easily without duplicating it. The original code trims `len` but passes the original string
    // to strtok, which uses `memchr` to find the start of `params`. So the rest of the string is copied.
    bool result = protocol_handle_line("TOOL_RELEASE:tool=CAMERA");
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(CommandType::TOOL_RELEASE, g_motion_controller.current_cmd.type);
    TEST_ASSERT_EQUAL_STRING("CAMERA", g_motion_controller.current_cmd.tool_name);
}

uint32_t g_last_ping_ms = 0;

void test_ping_resets_watchdog(void) {
    g_last_ping_ms = 0;
    bool result = protocol_handle_line("PING\n");
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_UINT32(0, g_last_ping_ms); // Mock millis() returns 0
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_arm_move_to_parsing);
    RUN_TEST(test_wrist_set_parsing);
    RUN_TEST(test_tool_release_parsing);
    RUN_TEST(test_ping_resets_watchdog);
    return UNITY_END();
}
