/**
 * TrajectoryGenerator.h
 *
 * Simple internal sine wave trajectory generator for testing servo reads while moving.
 *
 * Usage:
 *   TrajectoryGenerator traj(2048, 500, 0.5f, 40);  // center, amplitude, freq_hz, update_rate_hz
 *   traj.begin();
 *
 *   In loop():
 *     int32_t target = traj.update();
 *     if (target >= 0) {
 *       enqueue_goal_cmd(SERVO_ID, target, 0);
 *     }
 *
 * Author: Alex + Claude
 * Date: 2025-12-07
 */

#ifndef TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_H

#include <Arduino.h>

class TrajectoryGenerator {
public:
    /**
     * Constructor
     * @param center_pos Center position (e.g., 2048 for STS3215)
     * @param amplitude Sine wave amplitude in position units (e.g., ±500)
     * @param sine_freq_hz Sine wave frequency in Hz (e.g., 0.5 = 2 sec period)
     * @param update_rate_hz Command update rate in Hz (e.g., 40 Hz)
     */
    TrajectoryGenerator(int32_t center_pos,
                       int32_t amplitude,
                       float sine_freq_hz,
                       uint32_t update_rate_hz)
        : center_pos_(center_pos)
        , amplitude_(amplitude)
        , sine_freq_hz_(sine_freq_hz)
        , update_rate_hz_(update_rate_hz)
        , update_period_us_(1000000UL / update_rate_hz)
        , enabled_(false)
        , start_us_(0)
        , last_update_us_(0)
    {
    }

    /**
     * Enable/disable trajectory generation
     */
    void setEnabled(bool enabled) {
        enabled_ = enabled;
        if (enabled && start_us_ == 0) {
            begin();  // Auto-initialize if not already done
        }
    }

    bool isEnabled() const {
        return enabled_;
    }

    /**
     * Initialize/reset the trajectory generator
     * Call this before starting trajectory generation
     */
    void begin() {
        start_us_ = micros();
        last_update_us_ = start_us_;
    }

    /**
     * Update trajectory and return target position
     * Call this every loop iteration
     *
     * @return Target position (0-4095), or -1 if not time to update yet
     */
    int32_t update() {
        // Disabled - return no update
        if (!enabled_) {
            return -1;
        }

        uint32_t now_us = micros();

        // Not time for next update yet
        if (now_us - last_update_us_ < update_period_us_) {
            return -1;
        }

        last_update_us_ = now_us;

        // Compute time since start in seconds
        float t_sec = (now_us - start_us_) * 1e-6f;

        // Simple sine: position = center + A*sin(2π f t)
        float angle = 2.0f * PI * sine_freq_hz_ * t_sec;
        int32_t target = center_pos_ + (int32_t)(amplitude_ * sinf(angle));

        // Clamp to servo range (0-4095 for STS3215)
        if (target < 0)    target = 0;
        if (target > 4095) target = 4095;

        return target;
    }

    /**
     * Get a static target position (for Step 1: static target test)
     * @return Center position
     */
    int32_t getStaticTarget() const {
        return center_pos_;
    }

    // Configuration getters
    int32_t getCenterPos() const { return center_pos_; }
    int32_t getAmplitude() const { return amplitude_; }
    float getSineFreqHz() const { return sine_freq_hz_; }
    uint32_t getUpdateRateHz() const { return update_rate_hz_; }

private:
    // Configuration
    int32_t center_pos_;
    int32_t amplitude_;
    float sine_freq_hz_;
    uint32_t update_rate_hz_;
    uint32_t update_period_us_;

    // State
    bool enabled_;
    uint32_t start_us_;
    uint32_t last_update_us_;
};

#endif // TRAJECTORY_GENERATOR_H
