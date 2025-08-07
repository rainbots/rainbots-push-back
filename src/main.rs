#![no_main]
#![no_std]

mod banner;

use core::time::Duration;

use evian::{drivetrain::model::Differential, motion::CurvatureDrive, prelude::*};
use vexide::prelude::*;

use crate::banner::THEME_RAINBOTS;

struct Jodio {
    dt: Drivetrain<Differential, ()>,
    ctrl: Controller,
    curvature: CurvatureDrive,
}

impl Compete for Jodio {
    async fn autonomous(&mut self) {
        // 黄金の回転
        self.dt.model.drive_tank(-1.0, 1.0).expect("couldn't 回転");
    }

    async fn driver(&mut self) {
        let state = self.ctrl.state().expect("couldn't get controller state");
        self.curvature
            .update(&mut self.dt, state.left_stick.y(), state.right_stick.x())
            .expect("couldn't set drivetrain voltages");
        sleep(Duration::from_millis(10)).await;
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
                    motor(peris.port_1),
                    motor(peris.port_2),
                    motor(peris.port_3),
                    motor(peris.port_4),
                    motor(peris.port_5),
                    motor(peris.port_6),
                ],
                [
                    motor(peris.port_11),
                    motor(peris.port_12),
                    motor(peris.port_13),
                    motor(peris.port_14),
                    motor(peris.port_15),
                    motor(peris.port_16),
                ],
            ),
            tracking: (),
        },
        ctrl: peris.primary_controller,
        // TODO: tune consts
        curvature: CurvatureDrive::new(
            0.0, // turn_nonlinearity,
            0.0, // deadzone,
            0.0, // slew,
            0.0, // negative_inertia_scalar,
            0.0, // turn_sensitivity,
        ),
    };

    jodio.compete().await;
}
