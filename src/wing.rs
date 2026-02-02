use vexide::{adi::AdiPort, prelude::*, smart::PortError};

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

    pub fn extend(&mut self) -> Result<(), PortError> {
        if !self.extended {
            self.solenoid.set_high()?;
        }
        Ok(())
    }

    pub fn retract(&mut self) -> Result<(), PortError> {
        if self.extended {
            self.solenoid.set_low()?;
        }
        Ok(())
    }
}
