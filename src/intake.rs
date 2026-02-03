use std::{cell::RefCell, rc::Rc, time::Instant};

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

    fn stage0_in(&mut self) -> Result<(), PortError> {
        self.stage0.set_voltage(12.0)
    }

    fn stage0_out(&mut self) -> Result<(), PortError> {
        self.stage0.set_voltage(-12.0)
    }

    fn stage1_in(&mut self) -> Result<(), PortError> {
        self.stage0.set_voltage(12.0)
    }

    fn stage1_out(&mut self) -> Result<(), PortError> {
        self.stage0.set_voltage(-12.0)
    }

    fn stage2_upper(&mut self) -> Result<(), PortError> {
        self.stage0.set_voltage(12.0)
    }

    fn stage2_lower(&mut self) -> Result<(), PortError> {
        self.stage0.set_voltage(-12.0)
    }

    fn handle_detection(
        &mut self,
        on_detected: impl FnOnce(&mut Self) -> Result<(), PortError>,
        on_detection_end: impl FnOnce(&mut Self) -> Result<(), PortError>,
    ) -> Result<(), PortError> {
        if let Some(detection) = self.detection {
            if detection.filter_until() > Instant::now() {
                on_detected(self)?;
            } else {
                self.detection = None;
                on_detection_end(self)?;
            }
        } else {
            let proximity = self.optical.proximity()?;
            if proximity <= consts::BLOCK_PROXIMITY_THRESHOLD {
                let hue = self.optical.hue()?;
                let block_alliance = hue_alliance(hue);

                if let Some(block_alliance) = block_alliance
                    && block_alliance != self.allegiance
                {
                    self.detection = Some(Detection::now());
                    on_detected(self)?;
                }
            }
        }

        Ok(())
    }

    fn open_end(&mut self) -> Result<(), PortError> {
        self.wing.retract()
    }

    fn close_end(&mut self) -> Result<(), PortError> {
        self.wing.extend()
    }

    pub fn update(&mut self) -> Result<(), PortError> {
        let command = *self.command.borrow();
        match command {
            Command::Stop => {
                self.close_end()?;
                self.stage0.set_voltage(0.0)?;
                self.stage1.set_voltage(0.0)?;
                self.stage2.set_voltage(0.0)?;
            }
            Command::Collect => {
                self.close_end()?;
                self.stage0_in()?;
                self.stage1_in()?;
                // end is closed so no blocks will exit
                self.stage2_upper()?;
            }
            Command::ScoreLow => {
                self.stage0_out()?;
                self.stage1_out()?;
                // might cause middle scoring if blocks are present at the end of the intake
                self.stage2_lower()?;
            }
            Command::ScoreMiddle => {
                // so bad blocks can be thrown out
                self.open_end()?;

                self.handle_detection(
                    // bad block detected, redirect to upper
                    Self::stage2_upper,
                    // bad block is out, go back to lower
                    Self::stage2_lower,
                )?;

                self.stage0_in()?;
                self.stage1_in()?;
            }
            Command::ScoreLong => {
                // so bad blocks can be thrown out
                self.open_end()?;

                self.handle_detection(
                    // bad block detected, redirect to lower
                    Self::stage2_lower,
                    // bad block is out, go back to upper
                    Self::stage2_upper,
                )?;

                self.stage0_in()?;
                self.stage1_in()?;
            }
        };

        Ok(())
    }
}
