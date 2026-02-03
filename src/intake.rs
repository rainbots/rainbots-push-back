use std::{cell::RefCell, f64, rc::Rc, time::Instant};

use log::{error, info, trace, warn};
use vexide::{prelude::*, smart::PortError};

use crate::{Alliance, consts, wing::Wing};

pub fn hue_alliance(raw_hue: f64) -> Option<Alliance> {
    let wrapped_hue = (raw_hue + 180.0).rem_euclid(360.0) - 180.0;
    if (consts::RED_HUE - wrapped_hue).abs() < consts::BLOCK_HUE_TOLERANCE {
        Some(Alliance::Red)
    } else if (consts::BLUE_HUE - raw_hue).abs() < consts::BLOCK_HUE_TOLERANCE {
        Some(Alliance::Blue)
    } else {
        None
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Command {
    Collect,
    ScoreLong,
    ScoreMiddle,
    ScoreLow,
    #[default]
    Stop,
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
    wing: Wing,

    allegiance: Alliance,
    command: Rc<RefCell<Command>>,
    detection: Option<Detection>,
}

impl Intake {
    pub fn new(
        stage0: Motor,
        stage1: Motor,
        stage2: Motor,
        optical: OpticalSensor,
        wing: Wing,
        allegiance: Alliance,
    ) -> Self {
        Self {
            stage0,
            stage1,
            stage2,
            optical,
            wing,
            allegiance,
            command: Rc::new(RefCell::new(Command::Stop)),
            detection: None,
        }
    }

    pub fn command(&self) -> Rc<RefCell<Command>> {
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

    fn handle_detection(
        &mut self,
        on_detected: impl FnOnce(&mut Self),
        on_detection_end: impl FnOnce(&mut Self),
    ) {
        if let Some(detection) = self.detection {
            if detection.filter_until() > Instant::now() {
                on_detected(self);
            } else {
                self.detection = None;
                info!("detected block");
                on_detection_end(self);
            }
        } else {
            let proximity = match self.optical.proximity() {
                Ok(p) => p,
                Err(e) => {
                    warn!("couldn't get optical proximity: {e}");
                    return;
                }
            };

            if proximity > consts::BLOCK_PROXIMITY_THRESHOLD {
                return;
            }

            let hue = match self.optical.hue() {
                Ok(h) => h,
                Err(e) => {
                    warn!("couldn't get optical hue: {e}");
                    return;
                }
            };

            let block_alliance = hue_alliance(hue);

            if let Some(block_alliance) = block_alliance
                && block_alliance != self.allegiance
            {
                self.detection = Some(Detection::now());
                on_detected(self);
            }
        }
    }

    fn open_end(&mut self) {
        trace!("opening intake end");
        self.wing.retract();
    }

    fn close_end(&mut self) {
        trace!("closing intake end");
        self.wing.extend();
    }

    pub fn update(&mut self) -> Result<(), PortError> {
        let command = *self.command.borrow();
        match command {
            Command::Stop => {
                self.close_end();
                self.stage0.set_voltage(0.0)?;
                self.stage1.set_voltage(0.0)?;
                self.stage2.set_voltage(0.0)?;
            }
            Command::Collect => {
                self.close_end();
                self.stage0_in();
                self.stage1_in();
                // end is closed so no blocks will exit
                self.stage2_upper();
            }
            Command::ScoreLow => {
                self.stage0_out();
                self.stage1_out();
                // might cause middle scoring if blocks are present at the end of the intake
                self.stage2_lower();
            }
            Command::ScoreMiddle => {
                // so bad blocks can be thrown out
                self.open_end();

                self.handle_detection(
                    // bad block detected, redirect to upper
                    Self::stage2_upper,
                    // bad block is out, go back to lower
                    Self::stage2_lower,
                );

                self.stage0_in();
                self.stage1_in();
            }
            Command::ScoreLong => {
                // so bad blocks can be thrown out
                self.open_end();

                self.handle_detection(
                    // bad block detected, redirect to lower
                    Self::stage2_lower,
                    // bad block is out, go back to upper
                    Self::stage2_upper,
                );

                self.stage0_in();
                self.stage1_in();
            }
        };

        Ok(())
    }
}
