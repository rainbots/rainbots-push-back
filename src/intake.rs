use std::{cell::Cell, f64, rc::Rc, time::Instant};

use log::{error, info, warn};
use vexide::{
    prelude::*,
    smart::{PortError, motor::BrakeMode},
};

use crate::{Alliance, consts, wing::Wing};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Command {
    Collect,
    ScoreLong,
    ScoreMiddle,
    ScoreLow,
    #[default]
    Stop,
}

pub type CommandCell = Rc<Cell<Command>>;

fn hue_alliance(raw_hue: f64) -> Option<Alliance> {
    let wrapped_hue = (raw_hue + 180.0).rem_euclid(360.0) - 180.0;
    if (consts::RED_HUE - wrapped_hue).abs() < consts::BLOCK_HUE_TOLERANCE {
        Some(Alliance::Red)
    } else if (consts::BLUE_HUE - raw_hue).abs() < consts::BLOCK_HUE_TOLERANCE {
        Some(Alliance::Blue)
    } else {
        None
    }
}

#[derive(Debug, Clone, Copy)]
struct Detection {
    time: Instant,
}

impl Detection {
    fn now() -> Self {
        Self {
            time: Instant::now(),
        }
    }

    fn filter_until(&self) -> Instant {
        self.time + consts::BLOCK_FILTER_INTERVAL
    }
}

pub struct Intake {
    stage0: Motor,
    stage1: Motor,
    stage2: Motor,
    optical: OpticalSensor,

    command: Rc<Cell<Command>>,
    detection: Option<Detection>,
    allegiance: Rc<Cell<Option<Alliance>>>,
    reverse_until: Option<Instant>,
}

impl Intake {
    pub fn new(
        stage0: Motor,
        stage1: Motor,
        stage2: Motor,
        optical: OpticalSensor,
        allegiance: Rc<Cell<Option<Alliance>>>,
    ) -> Self {
        Self {
            stage0,
            stage1,
            stage2,
            optical,
            command: Rc::new(Cell::new(Command::Stop)),
            detection: None,
            allegiance,
            reverse_until: None,
        }
    }

    pub fn command(&self) -> CommandCell {
        Rc::clone(&self.command)
    }

    fn stage0_in(&mut self) {
        self.stage0
            .set_voltage(12.0)
            .unwrap_or_else(|e| error!("couldn't drive stage0, {e}"));
    }

    fn stage0_out(&mut self) {
        self.stage0
            .set_voltage(-12.0)
            .unwrap_or_else(|e| error!("couldn't drive stage0, {e}"));
    }

    fn stage1_in(&mut self) {
        self.stage0
            .set_voltage(12.0)
            .unwrap_or_else(|e| error!("couldn't drive stage1, {e}"));
    }

    fn stage1_out(&mut self) {
        self.stage0
            .set_voltage(-12.0)
            .unwrap_or_else(|e| error!("couldn't drive stage1, {e}"));
    }

    fn stage2_upper(&mut self) {
        self.stage0
            .set_voltage(12.0)
            .unwrap_or_else(|e| error!("couldn't drive stage2 {e}"));
    }

    fn stage2_lower(&mut self) {
        self.stage0
            .set_voltage(-12.0)
            .unwrap_or_else(|e| error!("couldn't drive stage2 {e}"));
    }

    fn stage2_hold(&mut self) {
        self.stage0
            .brake(BrakeMode::Hold)
            .unwrap_or_else(|e| error!("couldn't drive stage2 {e}"));
    }

    fn handle_detection(
        &mut self,
        on_detected: impl FnOnce(&mut Self),
        on_detection_end: impl FnOnce(&mut Self),
    ) {
        // check if color sorting is enabled
        let allegiance = match self.allegiance.get() {
            // if it is, continue to the rest of the function
            Some(a) => a,
            // if it is not:
            None => {
                // just run post-filter code and return
                on_detection_end(self);
                return;
            }
        };

        // if something has already been detected
        if let Some(detection) = self.detection {
            // if the filter interval has not already passed
            if detection.filter_until() > Instant::now() {
                // call filter code
                on_detected(self);
            } else {
                // if the filter interval has passed, invalidate the detection and call end code
                self.detection = None;
                info!("filter has ended");
                on_detection_end(self);
            }
        } else {
            // if something has not been detected:

            // check if something is in the intake
            let proximity = match self.optical.proximity() {
                Ok(p) => p,
                Err(e) => {
                    warn!("couldn't get optical proximity: {e}");
                    return;
                }
            };

            // if there is not, return
            if proximity > consts::BLOCK_PROXIMITY_THRESHOLD {
                return;
            }

            info!("detected object in intake");

            // there is, check its hue
            let hue = match self.optical.hue() {
                Ok(h) => h,
                Err(e) => {
                    warn!("couldn't get optical hue: {e}");
                    return;
                }
            };

            info!("object has hue {hue}");

            // try to match the hue with an alliance
            let block_alliance = hue_alliance(hue);

            // if the hue was matched and it's the opposite alliance
            if let Some(block_alliance) = block_alliance
                && block_alliance != allegiance
            {
                // validate detection
                self.detection = Some(Detection::now());
                // call filter code
                info!("detected block, filtering now");
                on_detected(self);
            }
        }
    }

    fn should_reverse(&mut self) -> bool {
        let now = Instant::now();

        now < *self
            .reverse_until
            .get_or_insert(now + consts::REVERSE_INTERVAL)
    }

    pub fn update(&mut self) -> Result<(), PortError> {
        let command = self.command.get();
        if !matches!(command, Command::ScoreMiddle | Command::ScoreLong) {
            self.reverse_until = None;
        }

        match command {
            Command::Stop => {
                self.stage0.set_voltage(0.0)?;
                self.stage1.set_voltage(0.0)?;
                self.stage2.set_voltage(0.0)?;
            }
            Command::Collect => {
                self.stage0_in();
                self.stage1_in();
                // end is closed so no blocks will exit
                self.stage2_hold();
            }
            Command::ScoreLow => {
                self.stage0_out();
                self.stage1_out();
                // might cause middle scoring if blocks are present at the end of the intake
                self.stage2_lower();
            }
            Command::ScoreMiddle => {
                // if self.should_reverse() {
                //     self.stage1_out();
                //     self.stage2_lower();
                // } else {
                //     self.handle_detection(
                //         // bad block detected, redirect to upper
                //         Self::stage2_upper,
                //         // bad block is out, go back to lower
                //         Self::stage2_lower,
                //     );
                //     self.stage1_in();
                // }

                self.stage2_lower();
                self.stage1_in();
                self.stage0_in();
            }
            Command::ScoreLong => {
                // if self.should_reverse() {
                //     self.stage1_out();
                //     self.stage2_lower();
                // } else {
                //     self.handle_detection(
                //         // bad block detected, redirect to lower
                //         Self::stage2_lower,
                //         // bad block is out, go back to upper
                //         Self::stage2_upper,
                //     );
                //     self.stage1_in();
                // }
                self.stage2_upper();
                self.stage1_in();

                self.stage0_in();
            }
        };

        Ok(())
    }
}
