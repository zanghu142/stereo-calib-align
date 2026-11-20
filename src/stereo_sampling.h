#pragma once

#include <cstddef>

// Accessed under model_mutex by the capture, analysis and UI paths.
struct StereoSamplingProgress {
    static constexpr std::size_t auto_target = 12;
    std::size_t samples = 0;
    std::size_t valid_samples = 0;
    std::size_t last_attempt_samples = 0;
    std::size_t solved_samples = 0;
    bool calculating = false;
    bool auto_completed = false;

    void recordSample(bool accepted) {
        ++samples;
        if (accepted) ++valid_samples;
    }

    bool needsAutoCalculation() const {
        return !calculating && !auto_completed && valid_samples >= auto_target &&
               valid_samples != last_attempt_samples;
    }

    void beginCalculation() {
        calculating = true;
        last_attempt_samples = valid_samples;
    }

    void finishCalculation(bool success) {
        calculating = false;
        if (success) solved_samples = last_attempt_samples;
        auto_completed = success && last_attempt_samples >= auto_target;
    }
};
