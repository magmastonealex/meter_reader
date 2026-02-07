#[derive(Debug, Clone, Copy, PartialEq)]
enum SignalState {
    High,
    Low,
}

pub struct PulseCounter {
    // Configuration
    center_point: i16,
    
    // Internal State
    state: SignalState,
    total_pulses: u32,

    threshold_high: i16,
    threshold_low: i16
}

impl PulseCounter {
    /// Creates a new PulseCounter.
    ///
    /// * `center_point`: The approximate midpoint of your sinusoidal wave.
    /// * `delta`: The hysteresis "buffer". The signal must rise above `center + delta`
    ///   and fall below `center - delta` to register a change. This filters out noise.
    ///   Start with a delta that is about 10-20% of your expected amplitude.
    pub fn new(center_point: i16, delta: u16, total_pulses: u32) -> Self {
        Self {
            center_point,
            threshold_high: center_point.saturating_add_unsigned(delta),
            threshold_low: center_point.saturating_sub_unsigned(delta),
            // We assume Low initially, or you could take an initial sample to decide.
            state: SignalState::Low, 
            total_pulses,
        }
    }

    /// Feeds a new sample into the analyzer.
    /// Returns `Some(total_count)` if a new pulse was just completed, or `None` otherwise.
    pub fn update(&mut self, sample: i16) -> Option<u32> {
        match self.state {
            SignalState::Low => {
                // Look for rising edge
                if sample > self.threshold_high {
                    self.state = SignalState::High;
                }
                None
            },
            SignalState::High => {
                // Look for falling edge
                if sample < self.threshold_low {
                    self.state = SignalState::Low;
                    self.total_pulses += 1;
                    Some(self.total_pulses)
                } else {
                    None
                }
            }
        }
    }

    /// Returns the current total count without updating state.
    pub fn count(&self) -> u32 {
        self.total_pulses
    }
    pub fn set_count(&mut self, count: u32){
        self.total_pulses = count;
    }
}