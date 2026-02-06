mod auton;
mod banner;
mod consts;
mod curvature;
mod intake;
mod logger;
mod matchloader;
mod wing;

use std::{cell::Cell, rc::Rc, time::Duration};

use autons::{prelude::*, route, simple::SimpleSelect};
use evian::{
    drivetrain::model::Differential,
    math::Angle,
    prelude::*,
    tracking::{
        shared_motors,
        wheeled::{TrackingWheel, WheeledTracking},
    },
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
    intake::{Command, CommandCell, Intake},
    logger::RobotLogger,
    matchloader::Matchloader,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Alliance {
    Red,
    Blue,
}

struct Jodio {
    dt: Drivetrain<Differential, WheeledTracking>,
    _intake_task: Task<()>,
    intake_command: CommandCell,
    curvature: CurvatureDrive,
    matchloader: Matchloader,
    ctrl: Controller,
    allegiance: Rc<Cell<Option<Alliance>>>,
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
                self.intake_command.set(Command::ScoreLong);
            } else if state.button_l1.is_pressed() {
                collecting = false;
                self.intake_command.set(Command::ScoreMiddle);
            } else if state.button_r2.is_pressed() {
                collecting = false;
                self.intake_command.set(Command::ScoreLow);
            } else if !collecting {
                self.intake_command.set(Command::Stop);
            }

            if state.button_r1.is_now_pressed() {
                collecting = !collecting;
                if collecting {
                    self.intake_command.set(Command::Collect);
                } else {
                    self.intake_command.set(Command::Stop);
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
    RobotLogger.init(LevelFilter::max()).unwrap();

    let allegiance = Rc::new(Cell::new(Some(select_allegiance(&mut peris.display).await)));
    allegiance.set(None);

    let mut intake = Intake::new(
        Motor::new(peris.port_2, Gearset::Blue, Direction::Forward),
        Motor::new_exp(peris.port_1, Direction::Reverse),
        Motor::new_exp(peris.port_11, Direction::Forward),
        OpticalSensor::new(peris.port_21),
        Rc::clone(&allegiance),
    );
    let intake_command = intake.command();

    let left_motors = shared_motors![Motor::new(peris.port_3, Gearset::Green, Direction::Reverse),];
    let right_motors =
        shared_motors![Motor::new(peris.port_4, Gearset::Green, Direction::Reverse),];
    let jodio = Jodio {
        dt: Drivetrain {
            model: Differential::new([], []),
            tracking: WheeledTracking::forward_only(
                (0.0, 0.0),
                Angle::from_radians(0.0),
                [
                    TrackingWheel::new(left_motors.clone(), 3.25, 0.0, Some(36.0 / 48.0)),
                    TrackingWheel::new(right_motors.clone(), 3.25, 0.0, Some(36.0 / 48.0)),
                ],
                {
                    let a: Option<InertialSensor> = None;
                    a
                },
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
        matchloader: Matchloader::new(peris.adi_b),
        ctrl: peris.primary_controller,
        allegiance,
    };

    jodio
        .compete(SimpleSelect::new(
            peris.display,
            [
                route!("Right Safe", auton::right_safe),
                route!("Left Safe", auton::left_safe),
                route!("Right AWP", auton::awp),
                route!("Skills", auton::skills),
            ],
        ))
        .await;
}
