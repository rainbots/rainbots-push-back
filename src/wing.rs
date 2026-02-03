use log::error;
use vexide::{adi::AdiPort, prelude::*};

pub struct Wing {
    solenoid: AdiDigitalOut,
    extended: bool,
}

impl Wing {
    pub fn new(port: AdiPort) -> Self {
        Self {
            solenoid: AdiDigitalOut::new(port),
            extended: false,
        }
    }

    pub fn extend(&mut self) {
        if !self.extended {
            self.solenoid
                .set_high()
                .unwrap_or_else(|e| error!("couldn't extend wing, {e}"));
            self.extended = true;
        }
    }

    pub fn retract(&mut self) {
        if self.extended {
            self.solenoid
                .set_low()
                .unwrap_or_else(|e| error!("couldn't retract wing, {e}"));
            self.extended = false;
        }
    }
}
