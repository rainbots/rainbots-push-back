mod auton;
mod banner;
mod curvature;
mod intake;

use std::time::Duration;

use evian::{
    drivetrain::model::Differential,
    math::Angle,
    prelude::*,
    tracking::wheeled::{TrackingWheel, WheeledTracking},
};
use vexide::{prelude::*, smart::SmartPort};

use crate::{banner::THEME_RAINBOTS, curvature::CurvatureDrive, intake::Intake};

const TURN_NONLINEARITY: f64 = 0.65;
const TURN_SENSITIVITY: f64 = 0.8;
const DEADZONE: f64 = 4.0 / 100.0;
const SLEW: f64 = 0.3;
const NEGATIVE_INERTIA_SCALAR: f64 = 4.0;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Alliance {
    Red,
    Blue,
}

struct Jodio {
    dt: Drivetrain<Differential, WheeledTracking>,
    intake: intake::Intake,
    curvature: CurvatureDrive,
    ctrl: Controller,
}

impl Compete for Jodio {
    async fn autonomous(&mut self) {
        auton::auton(self).await;
    }

    async fn driver(&mut self) {
        self.intake.set_active();
        loop {
            let state = self.ctrl.state().unwrap_or_default();
            self.curvature
                .update(&mut self.dt, state.left_stick.y(), state.right_stick.x())
                .expect("couldn't set drivetrain voltages");
            // self.dt
            //     .model
            //     .drive_arcade(state.left_stick.y(), state.right_stick.x())
            //     .expect("couldn't set drivetrain voltages");

            self.intake.update().expect("couldn't drive intake");
            sleep(Duration::from_millis(10)).await;
        }
    }
}

fn select_allegiance(controller: &Controller) -> Alliance {
    loop {
        let state = controller.state().expect("couldn't read controller state");
        if state.button_right.is_now_pressed() {
            return Alliance::Red;
        } else if state.button_left.is_now_pressed() {
            return Alliance::Blue;
        }
    }
}

#[vexide::main(banner(theme = THEME_RAINBOTS))]
async fn main(peris: Peripherals) {
    fn motor(port: SmartPort) -> Motor {
        Motor::new(port, Gearset::Blue, Direction::Forward)
    }

    let alliance = select_allegiance(&peris.primary_controller);

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
        curvature: CurvatureDrive::new(
            TURN_NONLINEARITY,
            DEADZONE,
            SLEW,
            NEGATIVE_INERTIA_SCALAR,
            TURN_SENSITIVITY,
        ),
        intake: Intake::new(
            Motor::new(peris.port_17, Gearset::Blue, Direction::Forward),
            Motor::new(peris.port_18, Gearset::Blue, Direction::Forward),
            Motor::new(peris.port_19, Gearset::Blue, Direction::Forward),
            OpticalSensor::new(peris.port_21),
            alliance,
        ),
        ctrl: peris.primary_controller,
    };

    jodio.compete().await;
}
