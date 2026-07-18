#pragma once

#include <Arduino.h>

constexpr int PIN_SENSOR_FRONT = A0;
constexpr int PIN_SENSOR_REAR = A1;
constexpr int PIN_BUTTON = D9;

constexpr int SCREEN_WIDTH = 128;
constexpr int SCREEN_HEIGHT = 64;

// ---- Sensor calibration ----
// Voltage divider on each sensor: R1=10k (top), R2=24k (bottom)
constexpr float DIVIDER_RATIO = 24.0f / (10.0f + 24.0f);
constexpr float ADC_VREF = 3.3f;
constexpr int ADC_MAX = 4095;
constexpr float SENSOR_VMIN = 0.5f;
constexpr float SENSOR_VMAX = 4.5f;
constexpr float PSI_TO_BAR = 0.0689476f;
constexpr float SENSOR_BAR_MAX = 200.0f * PSI_TO_BAR;

// Dip detection tuning
constexpr float NOISE_MARGIN_BAR = 1.5f * PSI_TO_BAR;  // Ignore jitter smaller than this
constexpr float MIN_DIP_DEPTH_BAR = 5.0f * PSI_TO_BAR; // Min drop to count rotor face
constexpr int FACES_PER_ROTOR = 3;
constexpr int SAMPLE_INTERVAL_MS = 2;

// Engine warm-up dips
constexpr int WARMUP_DIPS = 3;

// Test completion: at least this many post-warm-up pulses per rotor
// before we're willing to call it done (matches documented rotary
// compression test procedure).
constexpr int MIN_STEADY_PULSES = 10;
constexpr unsigned long MAX_TEST_MS = 10000;

// RPM normalization
// 250 RPM is the Mazda-standard reference cranking speed for
// rotary compression tests. Readings are compared at this speed so
// results from different combination of starters and batteries wont affect.
constexpr float REFERENCE_RPM = 250.0f;
// Empirical linear correction (per RPM of difference from the reference).
// Feel free to tune this coefficient if you calibrate against a known-good engine.
constexpr float RPM_CORRECTION_COEFF = 0.00285f;

// Standard sea level pressure in Pa
constexpr float SEA_LEVEL_PA = 101325.0f;
