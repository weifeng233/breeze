//! Shared pieces of the filter module.
//!
//! The C headers repeat the same alpha clamp in four files and, more
//! interestingly, derive alpha from a time constant in *two different
//! directions*: the low pass divides the sample time by the sum, the high pass
//! divides the time constant by it. Both are named here so the difference is
//! visible in one place instead of being a detail of two separate headers.

/// Returned when a time constant or sample time is not strictly positive.
///
/// The C versions treat that case as a silent no-op: the filter keeps whatever
/// alpha it had. A caller cannot tell "I set it" from "I failed to set it", so
/// the port reports it. `testdata/math_corpus.txt` keeps the C outcome
/// (`*_alpha_invalid_tc_unchanged`) so the difference is on the record.
pub const InvalidTimeConstant = error{InvalidTimeConstant};

/// The clamp every alpha entry point in the C headers performs.
pub fn clampAlpha(alpha: f32) f32 {
    if (alpha < 0.0) return 0.0;
    if (alpha > 1.0) return 1.0;
    return alpha;
}

/// `sample_time / (time_constant + sample_time)` - the low pass and EWMA form.
pub fn alphaFromSampleTime(time_constant: f32, sample_time: f32) InvalidTimeConstant!f32 {
    if (!(time_constant > 0.0) or !(sample_time > 0.0)) return error.InvalidTimeConstant;
    return clampAlpha(sample_time / (time_constant + sample_time));
}

/// `time_constant / (time_constant + sample_time)` - the high pass form, which
/// divides the other way round.
pub fn alphaFromTimeConstant(time_constant: f32, sample_time: f32) InvalidTimeConstant!f32 {
    if (!(time_constant > 0.0) or !(sample_time > 0.0)) return error.InvalidTimeConstant;
    return clampAlpha(time_constant / (time_constant + sample_time));
}
