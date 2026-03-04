/**
 * arm_controller – board-specific pin assignments.
 *
 * Update this file for your chosen MCU and wiring.
 * Supported targets: Arduino Mega 2560, STM32 Nucleo-F446RE, ESP32.
 */
#ifndef PINS_H
#define PINS_H

#if defined(ARDUINO_AVR_MEGA2560)
  // ── Arduino Mega 2560 ──────────────────────────────────────────────────
  // Stepper / servo signal outputs (one per joint)
  #define PIN_J1_STEP   2
  #define PIN_J1_DIR    3
  #define PIN_J2_STEP   4
  #define PIN_J2_DIR    5
  #define PIN_J3_STEP   6
  #define PIN_J3_DIR    7
  #define PIN_J4_STEP   8
  #define PIN_J4_DIR    9
  #define PIN_J5_STEP   10
  #define PIN_J5_DIR    11
  // Gripper servo PWM
  #define PIN_GRIPPER   12
  // Limit switches (NC, pulled high – LOW = triggered)
  #define PIN_LS_J1_MIN 22
  #define PIN_LS_J1_MAX 23
  #define PIN_LS_J2_MIN 24
  #define PIN_LS_J2_MAX 25

#elif defined(ARDUINO_NUCLEO_F446RE)
  // ── STM32 Nucleo-F446RE ────────────────────────────────────────────────
  #define PIN_J1_STEP   PA0
  #define PIN_J1_DIR    PA1
  #define PIN_J2_STEP   PA2
  #define PIN_J2_DIR    PA3
  #define PIN_GRIPPER   PB6   // TIM4_CH1 – hardware PWM
  #define PIN_LS_J1_MIN PC0
  #define PIN_LS_J1_MAX PC1

#elif defined(ARDUINO_ESP32_DEV)
  // ── ESP32 DevKit ──────────────────────────────────────────────────────
  #define PIN_J1_STEP   18
  #define PIN_J1_DIR    19
  #define PIN_J2_STEP   21
  #define PIN_J2_DIR    22
  #define PIN_GRIPPER   23
  #define PIN_LS_J1_MIN 34   // Input-only
  #define PIN_LS_J1_MAX 35   // Input-only

#else
  #warning "Unknown board – check config/pins.h and define your pin mapping."
#endif

#endif  // PINS_H
