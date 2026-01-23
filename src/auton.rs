use std::time::Duration;

use evian::prelude::*;
use vexide::time::sleep;

use crate::Jodio;

pub async fn auton(jodio: &mut Jodio) {
    loop {
        // 黄金の回転
        jodio.dt.model.drive_tank(-1.0, 1.0).expect("couldn't 回転");
        sleep(Duration::from_millis(10)).await;
    }
}
