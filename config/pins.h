/**
 * arm_controller – board-specific pin assignments.
 *
 * Update this file for your chosen MCU and wiring.
 * Supported targets: Arduino Mega 2560, STM32 Nucleo-F446RE, ESP32.
 */
#ifndef PINS_H
#define PINS_H

#define POSITION_TOLERANCE_STEPS 5

#ifdef ARM_CONTROLLER

#if defined(ARDUINO_AVR_MEGA2560)
  // ── Arduino Mega 2560 ──────────────────────────────────────────────────
  // Stepper / servo signal outputs (one per joint)
  #define PIN_J1_STEP   2
  #define PIN_J1_DIR    3
  #define PIN_J1_EN     26
  #define PIN_J2_STEP   4
  #define PIN_J2_DIR    5
  #define PIN_J2_EN     27
  #define PIN_J3_STEP   6
  #define PIN_J3_DIR    7
  #define PIN_J3_EN     28
  #define PIN_J4_STEP   8
  #define PIN_J4_DIR    9
  #define PIN_J4_EN     29
  #define PIN_J5_STEP   10
  #define PIN_J5_DIR    11
  #define PIN_J5_EN     30
  #define PIN_J6_STEP   13
  #define PIN_J6_DIR    14
  #define PIN_J6_EN     31
  // Gripper servo PWM
  #define PIN_GRIPPER   12
  // Limit switches (NC, pulled high – LOW = triggered)
  #define PIN_LS_J1_MIN 22
  #define PIN_LS_J1_MAX 23
  #define PIN_LS_J2_MIN 24
  #define PIN_LS_J2_MAX 25
  #define PIN_LS_J3_MIN 32
  #define PIN_LS_J3_MAX 33
  #define PIN_LS_J4_MIN 34
  #define PIN_LS_J4_MAX 35
  #define PIN_LS_J5_MIN 36
  #define PIN_LS_J5_MAX 37
  #define PIN_LS_J6_MIN 38
  #define PIN_LS_J6_MAX 39
  // Encoders
  #define PIN_ENC_J1_A  40
  #define PIN_ENC_J1_B  41
  #define PIN_ENC_J2_A  42
  #define PIN_ENC_J2_B  43
  #define PIN_ENC_J3_A  44
  #define PIN_ENC_J3_B  45
  #define PIN_ENC_J4_A  46
  #define PIN_ENC_J4_B  47
  #define PIN_ENC_J5_A  48
  #define PIN_ENC_J5_B  49
  #define PIN_ENC_J6_A  50
  #define PIN_ENC_J6_B  51

#elif defined(ARDUINO_NUCLEO_F446RE)
  // ── STM32 Nucleo-F446RE ────────────────────────────────────────────────
  #define PIN_J1_STEP   PA0
  #define PIN_J1_DIR    PA1
  #define PIN_J1_EN     PA4
  #define PIN_J2_STEP   PA2
  #define PIN_J2_DIR    PA3
  #define PIN_J2_EN     PA5
  #define PIN_J3_STEP   PA6
  #define PIN_J3_DIR    PA7
  #define PIN_J3_EN     PA8
  #define PIN_J4_STEP   PA9
  #define PIN_J4_DIR    PA10
  #define PIN_J4_EN     PA11
  #define PIN_J5_STEP   PA12
  #define PIN_J5_DIR    PA13
  #define PIN_J5_EN     PA14
  #define PIN_J6_STEP   PA15
  #define PIN_J6_DIR    PB0
  #define PIN_J6_EN     PB1

  #define PIN_GRIPPER   PB6   // TIM4_CH1 – hardware PWM

  #define PIN_LS_J1_MIN PC0
  #define PIN_LS_J1_MAX PC1
  #define PIN_LS_J2_MIN PC2
  #define PIN_LS_J2_MAX PC3
  #define PIN_LS_J3_MIN PC4
  #define PIN_LS_J3_MAX PC5
  #define PIN_LS_J4_MIN PC6
  #define PIN_LS_J4_MAX PC7
  #define PIN_LS_J5_MIN PC8
  #define PIN_LS_J5_MAX PC9
  #define PIN_LS_J6_MIN PC10
  #define PIN_LS_J6_MAX PC11

  #define PIN_ENC_J1_A  PC12
  #define PIN_ENC_J1_B  PD2
  #define PIN_ENC_J2_A  PB3
  #define PIN_ENC_J2_B  PB4
  #define PIN_ENC_J3_A  PB5
  #define PIN_ENC_J3_B  PB7
  #define PIN_ENC_J4_A  PB8
  #define PIN_ENC_J4_B  PB9
  #define PIN_ENC_J5_A  PB10
  #define PIN_ENC_J5_B  PB12
  #define PIN_ENC_J6_A  PB13
  #define PIN_ENC_J6_B  PB14

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

#endif // ARM_CONTROLLER

#endif  // PINS_H
