use std::time::Duration;

use evian::{
    control::loops::{AngularPid, Pid},
    prelude::Tolerances,
};

// Curvature Drive
pub const TURN_NONLINEARITY: f64 = 0.65;
pub const TURN_SENSITIVITY: f64 = 0.8;
pub const DEADZONE: f64 = 4.0 / 100.0;
pub const SLEW: f64 = 0.3;
pub const NEGATIVE_INERTIA_SCALAR: f64 = 4.0;

// PID
// TODO: Tune
pub const LINEAR_PID: Pid = Pid::new(0.0, 0.0, 0.0, None);
pub const ANGULAR_PID: AngularPid = AngularPid::new(0.0, 0.0, 0.0, None);

// Tolerances
pub const LINEAR_TOLERANCES: Tolerances = Tolerances::new()
    .error(1.0)
    .velocity(0.25)
    .duration(Duration::from_millis(15));
pub const ANGULAR_TOLERANCES: Tolerances = Tolerances::new()
    .error(f64::to_radians(8.0))
    .velocity(0.05)
    .duration(Duration::from_millis(15));

// Tracking
// TODO: replace placeholders
pub const PARA_WHEEL_OFFSET: f64 = 0.5;
pub const PERP_WHEEL_OFFSET: f64 = 0.5;
pub const DISTANCE_SENSOR_OFFSET: f64 = 0.5;

// Intake
pub const BLOCK_PROXIMITY_THRESHOLD: f64 = 0.5;
pub const BLOCK_HUE_TOLERANCE: f64 = 30.0;
pub const BLOCK_FILTER_INTERVAL: Duration = Duration::from_millis(250);
pub const REVERSE_INTERVAL: Duration = Duration::from_millis(250);

pub const RED_HUE: f64 = 0.0;
pub const BLUE_HUE: f64 = -120.0;

pub const MATCHLOADER_CLEAR_TIME: Duration = Duration::from_millis(500);
pub const HALF_MATCHLOADER_CLEAR_TIME: Duration = Duration::from_millis(500);
