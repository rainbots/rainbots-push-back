mod auton;
mod banner;
mod consts;
mod curvature;
mod intake;
mod wing;

use std::{cell::RefCell, rc::Rc, time::Duration};

use evian::{
    drivetrain::model::Differential,
    math::Angle,
    prelude::*,
    tracking::wheeled::{TrackingWheel, WheeledTracking},
};
use vexide::{prelude::*, smart::SmartPort, task::Task};

use crate::{
    banner::THEME_RAINBOTS,
    curvature::CurvatureDrive,
    intake::{Command, Intake},
    wing::Wing,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Alliance {
    Red,
    Blue,
}

struct Jodio {
    dt: Drivetrain<Differential, WheeledTracking>,
    _intake_task: Task<()>,
    intake_command: Rc<RefCell<Command>>,
    curvature: CurvatureDrive,
    distance_sensor: DistanceSensor,
    ctrl: Controller,
}

impl Jodio {
    fn set_intake_command(&self, command: Command) {
        *self.intake_command.borrow_mut() = command;
    }
}

impl Compete for Jodio {
    async fn autonomous(&mut self) {
        auton::auton(self).await;
    }

    async fn driver(&mut self) {
        let mut collecting = false;
        loop {
            let state = self.ctrl.state().unwrap_or_default();
            self.curvature
                .update(&mut self.dt, state.left_stick.y(), state.right_stick.x())
                .expect("couldn't set drivetrain voltages");

            // Priority:
            // L2 => Score Long
            // L1 => Score Middle
            // R2 => Score Lower
            // R1 => Toggle Collect

            if state.button_l2.is_pressed() {
                collecting = false;
                self.set_intake_command(Command::ScoreLong);
            } else if state.button_l1.is_pressed() {
                collecting = false;
                self.set_intake_command(Command::ScoreMiddle);
            } else if state.button_r2.is_pressed() {
                collecting = false;
                self.set_intake_command(Command::ScoreLow);
            } else if !collecting {
                self.set_intake_command(Command::Stop);
            }

            if state.button_r1.is_now_pressed() {
                collecting = !collecting;
                if collecting {
                    self.set_intake_command(Command::Collect);
                } else {
                    self.set_intake_command(Command::Stop);
                }
            }

            sleep(Duration::from_millis(10)).await;
        }
    }
}

fn select_allegiance(controller: &Controller) -> Alliance {
    loop {
        let state = controller.state().unwrap_or_default();
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

    let mut intake = Intake::new(
        Motor::new(peris.port_17, Gearset::Blue, Direction::Forward),
        Motor::new(peris.port_18, Gearset::Blue, Direction::Forward),
        Motor::new(peris.port_19, Gearset::Blue, Direction::Forward),
        OpticalSensor::new(peris.port_21),
        Wing::new(peris.adi_a),
        alliance,
    );
    let intake_command = intake.command();

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
            consts::TURN_NONLINEARITY,
            consts::DEADZONE,
            consts::SLEW,
            consts::NEGATIVE_INERTIA_SCALAR,
            consts::TURN_SENSITIVITY,
        ),
        _intake_task: spawn(async move {
            loop {
                let _ = intake.update();
                sleep(Duration::from_millis(10)).await;
            }
        }),
        intake_command,
        distance_sensor: DistanceSensor::new(peris.port_3),
        ctrl: peris.primary_controller,
    };

    jodio.compete().await;
}
