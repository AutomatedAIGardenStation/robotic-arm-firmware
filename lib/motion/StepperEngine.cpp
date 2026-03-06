#include "StepperEngine.h"
#include <stdlib.h>

#ifdef ARDUINO
#include <Arduino.h>
#else
extern uint32_t millis();
extern uint32_t micros();
#endif

StepperEngine::StepperEngine(IMotorDriver* drvs[6]) {
    for (int i = 0; i < 6; i++) {
        if (drvs) {
            drivers[i] = drvs[i];
        } else {
            drivers[i] = nullptr;
        }
        current_steps[i] = 0;
        target_steps[i] = 0;
        delta[i] = 0;
        error[i] = 0;
        step_dir[i] = 1;
    }
    moving = false;
    step_count = 0;
    current_delay_us = 0;
    min_delay_us = 1000; // 1 ms min step delay (1000 steps/sec)
    max_delay_us = 5000; // 5 ms start step delay (200 steps/sec)
    accel_steps = 100;
    decel_steps = 100;
    last_step_time_us = 0;
    max_delta = 0;
}

uint32_t StepperEngine::get_micros() {
#ifdef ARDUINO
    return micros();
#else
    // Mock micros using dummy increment for tests so it doesn't need external millis()
    static uint32_t fake_micros = 0;
    fake_micros += 1000;
    return fake_micros;
#endif
}

void StepperEngine::moveTo(const int32_t start_pos[6], const int32_t target_pos[6]) {
    max_delta = 0;

    for (int i = 0; i < 6; i++) {
        current_steps[i] = start_pos[i];
        target_steps[i] = target_pos[i];

        int32_t diff = target_steps[i] - current_steps[i];
        if (diff > 0) {
            step_dir[i] = 1;
            delta[i] = diff;
        } else {
            step_dir[i] = -1;
            delta[i] = -diff;
        }

        if (delta[i] > max_delta) {
            max_delta = delta[i];
        }

        if (drivers[i] && delta[i] > 0) {
            drivers[i]->enable();
        }
    }

    if (max_delta == 0) {
        moving = false;
        return;
    }

    for (int i = 0; i < 6; i++) {
        error[i] = max_delta / 2;
    }

    step_count = 0;

    // Calculate accel/decel parameters
    // We want 10% acceleration, 10% deceleration, or up to 200 steps
    accel_steps = max_delta / 10;
    if (accel_steps > 200) accel_steps = 200;
    if (accel_steps < 10) accel_steps = 10;

    decel_steps = accel_steps;

    // If the move is too short, we don't accelerate fully
    if (accel_steps + decel_steps > max_delta) {
        accel_steps = max_delta / 2;
        decel_steps = max_delta - accel_steps;
    }

    current_delay_us = max_delay_us;
    last_step_time_us = get_micros();
    moving = true;
}

void StepperEngine::update() {
    if (!moving) {
        return;
    }

    uint32_t now = get_micros();
    if ((now - last_step_time_us) < current_delay_us) {
        return;
    }

    last_step_time_us = now;

    // Perform a step for all axes that need it
    for (int i = 0; i < 6; i++) {
        if (delta[i] > 0) {
            error[i] -= delta[i];
            if (error[i] < 0) {
                if (drivers[i]) {
                    drivers[i]->step(step_dir[i] > 0, 1);
                }
                current_steps[i] += step_dir[i];
                error[i] += max_delta;
            }
        }
    }

    step_count++;

    if (step_count >= max_delta) {
        moving = false;
        for (int i = 0; i < 6; i++) {
            if (drivers[i]) {
                drivers[i]->disable();
            }
        }
        return;
    }

    // Update acceleration profile
    if (step_count < accel_steps) {
        // Linear interpolation of delay (not strictly constant acceleration, but simple and stable)
        uint32_t diff = max_delay_us - min_delay_us;
        current_delay_us = max_delay_us - (diff * step_count) / accel_steps;
    } else if (step_count > (max_delta - decel_steps)) {
        int32_t decel_count = step_count - (max_delta - decel_steps);
        uint32_t diff = max_delay_us - min_delay_us;
        current_delay_us = min_delay_us + (diff * decel_count) / decel_steps;
    } else {
        current_delay_us = min_delay_us;
    }
}

bool StepperEngine::isMoving() const {
    return moving;
}
