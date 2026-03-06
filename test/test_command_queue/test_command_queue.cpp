#include <unity.h>
#include "CommandQueue.h"

void setUp(void) {
    // empty setup
}

void tearDown(void) {
    // empty teardown
}

void test_initial_state_empty(void) {
    CommandQueue queue;
    TEST_ASSERT_TRUE(queue.isEmpty());
    TEST_ASSERT_FALSE(queue.isFull());
}

void test_enqueue_up_to_capacity_succeeds(void) {
    CommandQueue queue;
    for (size_t i = 0; i < CommandQueue::CAPACITY; i++) {
        Command cmd;
        cmd.type = static_cast<CommandType>(static_cast<int>(CommandType::ARM_HOME) + i); // Distinct types for tracking
        TEST_ASSERT_TRUE(queue.enqueue(cmd));
    }
    TEST_ASSERT_TRUE(queue.isFull());
    TEST_ASSERT_FALSE(queue.isEmpty());
}

void test_enqueue_past_capacity_fails(void) {
    CommandQueue queue;
    for (size_t i = 0; i < CommandQueue::CAPACITY; i++) {
        Command cmd;
        queue.enqueue(cmd);
    }

    Command extra_cmd;
    extra_cmd.type = CommandType::GRIPPER_OPEN;
    TEST_ASSERT_FALSE(queue.enqueue(extra_cmd));
}

void test_dequeue_returns_fifo_order(void) {
    CommandQueue queue;
    Command cmds_in[4];
    cmds_in[0].type = CommandType::ARM_HOME;
    cmds_in[1].type = CommandType::GRIPPER_OPEN;
    cmds_in[2].type = CommandType::GRIPPER_CLOSE;
    cmds_in[3].type = CommandType::ARM_MOVE_TO;

    for (int i = 0; i < 4; i++) {
        queue.enqueue(cmds_in[i]);
    }

    for (int i = 0; i < 4; i++) {
        Command cmd_out;
        TEST_ASSERT_TRUE(queue.dequeue(cmd_out));
        TEST_ASSERT_EQUAL(cmds_in[i].type, cmd_out.type);
    }

    TEST_ASSERT_TRUE(queue.isEmpty());
}

void test_empty_dequeue_returns_false(void) {
    CommandQueue queue;
    Command cmd;
    TEST_ASSERT_FALSE(queue.dequeue(cmd));
}

void test_queue_drains_correctly_cycles(void) {
    CommandQueue queue;

    // Enqueue 2, Dequeue 1 (head=1, tail=2)
    Command c1; c1.type = CommandType::ARM_HOME;
    Command c2; c2.type = CommandType::GRIPPER_OPEN;
    queue.enqueue(c1);
    queue.enqueue(c2);

    Command out;
    TEST_ASSERT_TRUE(queue.dequeue(out));
    TEST_ASSERT_EQUAL(CommandType::ARM_HOME, out.type);

    // Enqueue 3 more (head=1, tail=1, full)
    Command c3; c3.type = CommandType::GRIPPER_CLOSE;
    Command c4; c4.type = CommandType::ARM_MOVE_TO;
    Command c5; c5.type = CommandType::WRIST_SET;

    TEST_ASSERT_TRUE(queue.enqueue(c3));
    TEST_ASSERT_TRUE(queue.enqueue(c4));
    TEST_ASSERT_TRUE(queue.enqueue(c5));
    TEST_ASSERT_TRUE(queue.isFull());

    // Drain remaining 4
    TEST_ASSERT_TRUE(queue.dequeue(out)); TEST_ASSERT_EQUAL(CommandType::GRIPPER_OPEN, out.type);
    TEST_ASSERT_TRUE(queue.dequeue(out)); TEST_ASSERT_EQUAL(CommandType::GRIPPER_CLOSE, out.type);
    TEST_ASSERT_TRUE(queue.dequeue(out)); TEST_ASSERT_EQUAL(CommandType::ARM_MOVE_TO, out.type);
    TEST_ASSERT_TRUE(queue.dequeue(out)); TEST_ASSERT_EQUAL(CommandType::WRIST_SET, out.type);

    TEST_ASSERT_TRUE(queue.isEmpty());
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_initial_state_empty);
    RUN_TEST(test_enqueue_up_to_capacity_succeeds);
    RUN_TEST(test_enqueue_past_capacity_fails);
    RUN_TEST(test_dequeue_returns_fifo_order);
    RUN_TEST(test_empty_dequeue_returns_false);
    RUN_TEST(test_queue_drains_correctly_cycles);
    return UNITY_END();
}
