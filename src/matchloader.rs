use vexide::{adi::AdiPort, prelude::*, smart::PortError};

pub struct Matchloader {
    solenoid: AdiDigitalOut,
    extended: bool,
}

impl Matchloader {
    pub fn new(port: AdiPort) -> Self {
        Self {
            solenoid: AdiDigitalOut::new(port),
            extended: false,
        }
    }

    pub fn extend(&mut self) -> Result<(), PortError> {
        if !self.extended {
            self.solenoid.set_high()?;
            self.extended = true;
        }
        Ok(())
    }

    pub fn retract(&mut self) -> Result<(), PortError> {
        if self.extended {
            self.solenoid.set_low()?;
            self.extended = false;
        }
        Ok(())
    }
}
