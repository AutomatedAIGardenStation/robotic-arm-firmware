#ifndef COMMAND_QUEUE_H
#define COMMAND_QUEUE_H

#include "Command.h"
#include <stddef.h>

class CommandQueue {
public:
    static constexpr size_t CAPACITY = 4;

    CommandQueue() : head(0), tail(0), count(0) {}

    bool enqueue(const Command& cmd) {
        if (isFull()) {
            return false;
        }
        buffer[tail] = cmd;
        tail = (tail + 1) % CAPACITY;
        count++;
        return true;
    }

    bool dequeue(Command& cmd) {
        if (isEmpty()) {
            return false;
        }
        cmd = buffer[head];
        head = (head + 1) % CAPACITY;
        count--;
        return true;
    }

    bool isEmpty() const {
        return count == 0;
    }

    bool isFull() const {
        return count == CAPACITY;
    }

private:
    Command buffer[CAPACITY];
    size_t head;
    size_t tail;
    size_t count;
};

#endif // COMMAND_QUEUE_H
