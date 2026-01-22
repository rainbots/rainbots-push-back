mod banner;

use std::time::Duration;

use evian::{drivetrain::model::Differential, prelude::*};
use vexide::{prelude::*, smart::SmartPort};

use crate::banner::THEME_RAINBOTS;

struct Jodio {
    dt: Drivetrain<Differential, ()>,
    ctrl: Controller,
}

impl Compete for Jodio {
    async fn autonomous(&mut self) {
        loop {
            // 黄金の回転
            self.dt.model.drive_tank(-1.0, 1.0).expect("couldn't 回転");
            sleep(Duration::from_millis(10)).await;
        }
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
            tracking: (),
        },
        ctrl: peris.primary_controller,
    };

    jodio.compete().await;
}
