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

void test_pollinate_sequence_success(void) {
    bool result = protocol_handle_line("P1\n");
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_STRING("EVT:ARM_DONE", last_event);
    TEST_ASSERT_GREATER_THAN(5, g_drivers[2].enable_calls);
}

void test_harvest_sequence_success(void) {
    bool result = protocol_handle_line("H1\n");
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_STRING("EVT:ARM_DONE", last_event);
}

void test_sequence_aborts_on_fault(void) {
    g_limit_switches[0].setTriggered(true);
    // Since the sequence checks isFaulted before running wait_for_motion(),
    // it will emit SEQUENCE_ABORTED if it's already faulted!
    // Let's run P1. Wait, actually `arm_pollinate_sequence()` calls `execute()`.
    // Then `wait_for_motion()` checks `isFaulted()` and emits "EVT:ARM_FAULT:code=SEQUENCE_ABORTED".
    // Wait, let's verify if `execute()` emitted something before.

    // Oh, the problem is `last_event` might not be updated!
    // Wait, in our `SerialDummy` mock inside `test_protocol.cpp`, we did:
    // void println(const char* s) { strncpy(last_event, s, ...) }
    // BUT `protocol_emit_event` uses `Serial.println(event)` which goes to `SerialDummy::println`.
    // Wait! In `test_protocol.cpp`:
    // #define protocol_emit_event protocol_emit_event_hidden
    // #include "../../src/protocol.cpp"
    // #undef protocol_emit_event
    // void protocol_emit_event(const char* event) {
    //     strncpy(last_event, event, sizeof(last_event) - 1); ...
    // }
    //
    // BUT in `protocol.cpp`, we have:
    // void protocol_emit_event(const char* event) {
    //     Serial.println(event);
    // }
    // AND in `wait_for_motion()`, it calls `protocol_emit_event_hidden(...)` !!!
    // Because of the `#define`, everywhere in `protocol.cpp` that calls `protocol_emit_event` now calls `protocol_emit_event_hidden` which is defined at the BOTTOM of `protocol.cpp`.
    // And `protocol_emit_event_hidden` just does `Serial.println(event);`
    // And `Serial` is a `SerialDummy` but wait! `SerialDummy`'s println does NOTHING because we removed it!
    // Ah! I removed `SerialDummy::println` body earlier when I was trying to fix redefining!

    bool result = protocol_handle_line("P1\n");
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_STRING("EVT:ARM_FAULT:code=SEQUENCE_ABORTED", last_event);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_pollinate_sequence_success);
    RUN_TEST(test_harvest_sequence_success);
    RUN_TEST(test_sequence_aborts_on_fault);
    return UNITY_END();
}
