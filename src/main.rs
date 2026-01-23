mod auton;
mod banner;

use std::time::Duration;

use evian::{
    drivetrain::model::Differential,
    math::Angle,
    prelude::*,
    tracking::wheeled::{TrackingWheel, WheeledTracking},
};
use vexide::{prelude::*, smart::SmartPort};

use crate::banner::THEME_RAINBOTS;

struct Jodio {
    dt: Drivetrain<Differential, WheeledTracking>,
    ctrl: Controller,
}

impl Compete for Jodio {
    async fn autonomous(&mut self) {
        auton::auton(self).await;
    }

    async fn driver(&mut self) {
        loop {
            let state = self.ctrl.state().unwrap_or_default();
            self.dt
                .model
                .drive_arcade(state.left_stick.y(), state.left_stick.x())
                .expect("couldn't set drivetrain voltages");
            sleep(Duration::from_millis(10)).await;
        }
    }
}

#[vexide::main(banner(theme = THEME_RAINBOTS))]
async fn main(peris: Peripherals) {
    fn motor(port: SmartPort) -> Motor {
        Motor::new(port, Gearset::Blue, Direction::Forward)
    }

    let jodio = Jodio {
        dt: Drivetrain {
            model: Differential::new(
                [
                    motor(peris.port_8),
                    motor(peris.port_2),
                    motor(peris.port_1),
                ],
                [
                    motor(peris.port_20),
                    motor(peris.port_15),
                    motor(peris.port_14),
                ],
            ),
            tracking: WheeledTracking::new(
                (0.0, 0.0),
                Angle::from_radians(0.0),
                [TrackingWheel::new(
                    RotationSensor::new(peris.port_11, Direction::Forward),
                    2.75,
                    0.5,
                    None,
                )],
                [TrackingWheel::new(
                    RotationSensor::new(peris.port_12, Direction::Forward),
                    2.75,
                    0.5,
                    None,
                )],
                Some({
                    let mut imu = InertialSensor::new(peris.port_13);
                    let _ = imu.calibrate().await;
                    imu
                }),
            ),
        },
        ctrl: peris.primary_controller,
    };

    jodio.compete().await;
}
