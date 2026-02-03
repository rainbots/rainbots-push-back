use std::time::Duration;

use evian::{math::Vec2, motion::Basic, prelude::*};
use vexide::time::sleep;

use crate::{Jodio, consts, intake::Command};

pub async fn awp(jodio: &mut Jodio) {
    let mut basic = Basic {
        linear_controller: consts::LINEAR_PID,
        angular_controller: consts::ANGULAR_PID,
        linear_tolerances: consts::LINEAR_TOLERANCES,
        angular_tolerances: consts::ANGULAR_TOLERANCES,
        // TODO: add timeout
        timeout: None,
    };

    jodio.dt.tracking.set_position((-44.866, 0.0)); // TODO: correct y coord
    jodio.dt.tracking.set_heading(90.0.deg());

    // PHASE 1: Long goal scoring

    jodio.set_intake_command(Command::Collect);

    // drive down to right matchloader and long goal
    basic.drive_distance(&mut jodio.dt, 30.119).await;
    basic.turn_to_heading(&mut jodio.dt, 270.0.deg()).await;
    jodio.matchloader.extend();
    basic.drive_distance(&mut jodio.dt, 9.583).await;
    // wait for blocks to be collected from loader
    sleep(Duration::from_millis(500)).await; // TODO: placeholder duration

    // drive to long goal
    basic.drive_distance(&mut jodio.dt, -23.861).await;
    jodio.set_intake_command(Command::ScoreLong);
    sleep(Duration::from_millis(500)).await; // TODO: placeholder duration

    // PHASE 2: Middle goal scoring

    jodio.matchloader.retract();
    // face line of blocks
    // TODO: this needs some more work probably
    basic.turn_to_heading(&mut jodio.dt, 0.0.deg()).await;
    jodio.set_intake_command(Command::Collect);
    // collect blocks
    basic.drive_distance(&mut jodio.dt, 66.301).await;
    // face middle goal
    basic.turn_to_heading(&mut jodio.dt, 315.0.deg()).await;
    basic.drive_distance(&mut jodio.dt, -15.55).await;
    jodio.set_intake_command(Command::ScoreMiddle);
    // don't score too many blocks
    sleep(Duration::from_millis(250)).await; // TODO: placeholder duration
    jodio.set_intake_command(Command::Stop);

    // PHASE 3: Left long  goal scoring

    // drive to long goal
    basic.drive_distance(&mut jodio.dt, -45.52).await;
    basic.turn_to_heading(&mut jodio.dt, 270.0.deg()).await;
    basic.drive_distance(&mut jodio.dt, -12.712).await;
    jodio.set_intake_command(Command::ScoreLong);
    loop {
        sleep(Duration::from_millis(10)).await;
    }
}

// TODO: refactor to work for left
pub async fn safe(jodio: &mut Jodio, _left: bool) {
    let mut basic = Basic {
        linear_controller: consts::LINEAR_PID,
        angular_controller: consts::ANGULAR_PID,
        linear_tolerances: consts::LINEAR_TOLERANCES,
        angular_tolerances: consts::ANGULAR_TOLERANCES,
        // TODO: add timeout
        timeout: None,
    };

    type Point = Vec2<f64>;
    let point0: Point = (-44.866, -14.0).into();

    jodio.dt.tracking.set_position(point0);
    jodio.dt.tracking.set_heading(90.0.deg());

    jodio.set_intake_command(Command::Collect);
    let point1: Point = (-22.374, -21.827).into();

    basic.turn_to_point(&mut jodio.dt, point1).await;
    basic
        .drive_distance(&mut jodio.dt, point1.distance(point0))
        .await;

    let point2: Point = (-13.769, -13.612).into();
    basic.turn_to_point(&mut jodio.dt, point2).await;
    basic
        .drive_distance(&mut jodio.dt, point2.distance(point1))
        .await;
    jodio.set_intake_command(Command::ScoreLow);
    sleep(Duration::from_millis(500)).await; // TODO: placeholder duration
    jodio.set_intake_command(Command::Collect);

    let point3: Point = (-47.213, -47.056).into();
    basic
        .drive_distance(&mut jodio.dt, -point3.distance(point2))
        .await;

    basic.turn_to_heading(&mut jodio.dt, 270.0.deg()).await;
    jodio.matchloader.extend();
    basic.drive_distance(&mut jodio.dt, 6.063).await;

    jodio.matchloader.retract();
    basic.turn_to_heading(&mut jodio.dt, 90.0.deg()).await;
    basic.drive_distance(&mut jodio.dt, 23.079).await;
    jodio.set_intake_command(Command::ScoreLong);
}

pub async fn right_safe(jodio: &mut Jodio) {
    safe(jodio, false).await;
}

pub async fn left_safe(jodio: &mut Jodio) {
    safe(jodio, true).await;
}

pub async fn skills(_jodio: &mut Jodio) {}

pub async fn auton(jodio: &mut Jodio) {
    loop {
        // 黄金の回転
        jodio.dt.model.drive_tank(-1.0, 1.0).expect("couldn't 回転");
        sleep(Duration::from_millis(10)).await;
    }
}
