//! Named enum values shared with the Venus stack. Mirrors systemcalc's
//! `delegates/systemstate.py` (`SystemState`, `BL`, `SOCG` classes) and the
//! `/Settings/CGwacs/Hub4Mode` setting. The raw numbers live ONLY here — every
//! comparison elsewhere uses these names. (Golden-test data tables keep the raw
//! captured values: they are telemetry, not logic.)

/// systemcalc `/SystemState/State` — the ESS range. Plain vebus states
/// (0x00 Off … 0x0B Psu) pass through below these and are not enumerated here.
///
/// Note: on a DVCC/managed-battery install, `EXTERNAL_CONTROL` is the *default*
/// label whenever no more-specific ESS state matches (e.g. battery idle near the
/// setpoint — DISCHARGING needs battery power < −30 W), and it is also published
/// unconditionally while `Hub4Mode == EXTERNAL`. It carries no control behaviour
/// of its own; it is a GUI classification computed from measurements.
pub mod sysstate {
    pub const EXTERNAL_CONTROL: i64 = 0xFC;
    pub const DISCHARGING: i64 = 0x100;
    pub const SUSTAIN: i64 = 0x101;
    pub const RECHARGE: i64 = 0x102;
    pub const SCHEDULED_CHARGE: i64 = 0x103;
    pub const BATTERY_PROTECT: i64 = 0x105;
}

/// `/Settings/CGwacs/BatteryLife/State`. Values 0–8 are the BatteryLife (`BL`)
/// ladder; 9–12 are the "Optimized without BatteryLife" (`SOCG`) ladder.
pub mod bl {
    pub const DISABLED: i32 = 0;
    pub const RESTART: i32 = 1;
    pub const DEFAULT: i32 = 2;
    pub const ABSORPTION: i32 = 3;
    pub const FLOAT: i32 = 4;
    pub const DISCHARGED: i32 = 5;
    pub const FORCE_CHARGE: i32 = 6;
    pub const SUSTAIN: i32 = 7;
    pub const LOW_SOC_CHARGE: i32 = 8;
    pub const SOCG_KEEP_CHARGED: i32 = 9;
    pub const SOCG_DEFAULT: i32 = 10;
    pub const SOCG_DISCHARGED: i32 = 11;
    pub const SOCG_LOW_SOC_CHARGE: i32 = 12;
}

/// `/Settings/CGwacs/Hub4Mode`.
pub mod hub4mode {
    pub const PHASE_COMPENSATION: i32 = 1;
    pub const INDIVIDUAL_PHASE: i32 = 2;
    /// An external controller owns the grid-setpoint loop (that's us, in takeover).
    pub const EXTERNAL: i32 = 3;
}
