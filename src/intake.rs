use std::time::{Duration, Instant};

use vexide::{prelude::*, smart::PortError};

use crate::Alliance;

const BALL_PROXIMITY_THRESHOLD: f64 = 0.5;
const BALL_HUE_TOLERANCE: f64 = 30.0;
const BALL_FILTER_INTERVAL: Duration = Duration::from_millis(250);

trait Ball {
    // hues are assumed to be normalized to (-180, 180], including the hue passed to Ball::check_hue
    const HUE: f64;
    const ALLEGIANCE: Alliance;

    fn check_hue(hue: f64, bot_allegiance: Alliance) -> bool {
        if Self::ALLEGIANCE == bot_allegiance {
            return false;
        }

        (Self::HUE - hue).abs() < BALL_HUE_TOLERANCE
    }
}

struct Red;
struct Blue;

impl Ball for Red {
    const HUE: f64 = 0.0;
    const ALLEGIANCE: Alliance = Alliance::Red;
}

impl Ball for Blue {
    const HUE: f64 = -120.0;
    const ALLEGIANCE: Alliance = Alliance::Blue;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum BallFilter {
    Filtering { until: Instant },
    Idle,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum State {
    Active {
        seal_back: bool,
        ball_filter: BallFilter,
    },
    Idle,
}

pub struct Intake {
    stage0: Motor,
    stage1: Motor,
    stage2: Motor,
    optical: OpticalSensor,

    allegiance: Alliance,
    state: State,
}

impl Intake {
    pub fn new(
        stage0: Motor,
        stage1: Motor,
        stage2: Motor,
        optical: OpticalSensor,
        allegiance: Alliance,
    ) -> Self {
        Self {
            stage0,
            stage1,
            stage2,
            optical,
            allegiance,
            state: State::Idle,
        }
    }

    pub fn set_active(&mut self) {
        if self.state == State::Idle {
            self.state = State::Active {
                seal_back: false,
                ball_filter: BallFilter::Idle,
            };
        }
    }

    pub fn update(&mut self) -> Result<(), PortError> {
        if let State::Active {
            seal_back,
            ball_filter,
        } = &mut self.state
        {
            self.stage0.set_voltage(12.0)?;
            self.stage1.set_voltage(12.0)?;

            if *ball_filter == BallFilter::Idle
                && self.optical.proximity()? > BALL_PROXIMITY_THRESHOLD
            {
                let hue = self.optical.hue()?;
                let wrapped_hue = (hue + 180.0).rem_euclid(360.0) - 180.0;

                let filter = Red::check_hue(wrapped_hue, self.allegiance)
                    | Blue::check_hue(wrapped_hue, self.allegiance);
                if filter {
                    *ball_filter = BallFilter::Filtering {
                        until: Instant::now() + BALL_FILTER_INTERVAL,
                    };
                }
            }

            if *seal_back {
                // TODO: add pneumatic seal
            }

            if let BallFilter::Filtering { until } = *ball_filter {
                if Instant::now() > until {
                    *ball_filter = BallFilter::Idle;
                } else {
                    self.stage2.set_voltage(-12.0)?;
                }
            // filtering and scoring should be exclusive, filtering takes priority
            // TODO: maybe refactor both into a single enum
            } else if !*seal_back {
                self.stage2.set_voltage(12.0)?;
            }
        }

        Ok(())
    }
}
