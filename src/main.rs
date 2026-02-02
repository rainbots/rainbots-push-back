mod auton;
mod banner;
mod consts;
mod curvature;
mod intake;
mod logger;
mod matchloader;
mod wing;

use std::{cell::RefCell, rc::Rc, time::Duration};

use autons::{prelude::*, route, simple::SimpleSelect};
use evian::{
    drivetrain::model::Differential,
    math::Angle,
    prelude::*,
    tracking::wheeled::{TrackingWheel, WheeledTracking},
};
use log::{LevelFilter, warn};
use vexide::{
    display::{Rect, TouchState},
    prelude::*,
    smart::SmartPort,
    task::Task,
};

use crate::{
    banner::THEME_RAINBOTS,
    curvature::CurvatureDrive,
    intake::{Command, Intake},
    logger::RobotLogger,
    matchloader::Matchloader,
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
    matchloader: Matchloader,
    ctrl: Controller,
}

impl Jodio {
    fn set_intake_command(&self, command: Command) {
        *self.intake_command.borrow_mut() = command;
    }
}

impl SelectCompete for Jodio {
    async fn driver(&mut self) {
        let mut collecting = false;
        loop {
            let state = self.ctrl.state().unwrap_or_default();
            self.curvature
                .update(&mut self.dt, state.left_stick.y(), state.right_stick.x())
                .unwrap_or_else(|e| warn!("couldn't drive drivetrain: {e}"));

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

async fn select_allegiance(display: &mut Display) -> Alliance {
    display.fill(
        &Rect::new(
            [0, 0],
            [
                Display::HORIZONTAL_RESOLUTION / 2,
                Display::VERTICAL_RESOLUTION,
            ],
        ),
        (255, 0, 0),
    );

    display.fill(
        &Rect::new(
            [Display::HORIZONTAL_RESOLUTION / 2, 0],
            [Display::HORIZONTAL_RESOLUTION, Display::VERTICAL_RESOLUTION],
        ),
        (0, 0, 255),
    );

    loop {
        let touch = display.touch_status();
        if touch.state == TouchState::Pressed {
            display.erase((0, 0, 0));
            return if touch.point.x < Display::HORIZONTAL_RESOLUTION / 2 {
                Alliance::Red
            } else {
                Alliance::Blue
            };
        }
        sleep(Duration::from_millis(10)).await;
    }
}

#[vexide::main(banner(theme = THEME_RAINBOTS))]
async fn main(mut peris: Peripherals) {
    fn motor(port: SmartPort) -> Motor {
        Motor::new(port, Gearset::Blue, Direction::Forward)
    }

    let mut imu = InertialSensor::new(peris.port_13);
    let _ = imu.calibrate().await;

    let allegiance = select_allegiance(&mut peris.display).await;

    let mut intake = Intake::new(
        Motor::new(peris.port_17, Gearset::Blue, Direction::Forward),
        Motor::new(peris.port_18, Gearset::Blue, Direction::Forward),
        Motor::new(peris.port_19, Gearset::Blue, Direction::Forward),
        OpticalSensor::new(peris.port_21),
        Wing::new(peris.adi_a),
        allegiance,
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
                    consts::PARA_WHEEL_OFFSET,
                    None,
                )],
                [TrackingWheel::new(
                    RotationSensor::new(peris.port_12, Direction::Forward),
                    2.75,
                    consts::PERP_WHEEL_OFFSET,
                    None,
                )],
                Some(imu),
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
        matchloader: Matchloader::new(peris.adi_b),
        ctrl: peris.primary_controller,
    };

    jodio
        .compete(SimpleSelect::new(
            peris.display,
            [route!("Spin", auton::auton)],
        ))
        .await;
}
