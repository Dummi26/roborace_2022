use std::{sync::mpsc, thread, time::Duration};

use ev3dev_lang_rust::{motors::{LargeMotor, MediumMotor}, sensors::{ColorSensor, GyroSensor}};

use super::Robot;

#[cfg(feature="pc_test")]
use crate::test::virtual_robot::VirtualRequest;

pub struct Thread {
    pub sender: mpsc::Sender<Task>,
    pub receiver: mpsc::Receiver<Task>,
    pub erkennung_sender: Option<mpsc::Sender<super::erkennung::Task>>,
    pub config: Config,
    pub max_number_of_retries_on_communication_failure: (u32, Duration),
    #[cfg(not(feature="pc_test"))]
    pub motor_l1: LargeMotor,
    #[cfg(not(feature="pc_test"))]
    pub motor_l2: LargeMotor,
    #[cfg(not(feature="pc_test"))]
    pub motor_med: MediumMotor,
    #[cfg(not(feature="pc_test"))]
    pub sensor_color: ColorSensor,
    #[cfg(not(feature="pc_test"))]
    pub sensor_gyro: GyroSensor,
    sleep_duration: Duration,
    old_state: Option<LinienfolgerState>,
    current_state: LinienfolgerState,
    target_lane: Lane,
    target_lane_updated: bool,
    #[cfg(feature="pc_test")]
    pub virtual_robot: std::sync::mpsc::Sender<crate::test::virtual_robot::VirtualRequest>,
}
#[derive(Clone, )]
pub struct Config {
    /// [60] The speed at which the robot moves forward.
    pub default_speed_in_percent: i32,
    /// [2.5] If 0, steering is proportional to brightness
    pub importance_of_brightness_change: f32,
    /// [30.0] Steering angle when changing from one lane to another, in degrees
    pub angle_lane_change_start: f32,
    /// [3.0] Like seconds_to_straighten_out, but this is preferred.
    pub rotations_to_straighten_out: f32,
    /// [0.7] The steering angle will be changed from the start angle to a lesser angle during this time. If this is too high, the robot will probably overshoot the line. It is more acceptable to set this to a value that is a little bit lower than the optimal one, just to be safe. This should probably be dependant on the robot's speed in the future! [TODO]
    pub seconds_to_straighten_out: f32,
    /// [6.0] After enough time has elapsed, the angle between movement direction and line will be this. Set this too high and we might miss the line, set it too low and lane changes take longer.
    pub angle_lane_change_final: f32,
    /// [60] This brightness will trigger the switch from angle_lane_change_final to angle_lane_change_encounter.
    pub brightness_entering_black_line: i32,
    /// [2.0] After detecting the black line, this is the new target angle
    pub angle_lane_change_encounter: f32,
    /// [40] Read brightness_after_black_line!
    pub brightness_on_black_line: i32,
    /// [50] If brightness was <= brightness_on_black_line and later >= this, assume we are on the outer side of the line (we have moved past the line's center) and switch back to Line-Following mode.
    pub brightness_after_black_line: i32,
}

impl super::ThreadedFeature for Thread {
    type InitError = InitError;
    fn init(robot: &mut Robot) -> Result<Self, InitError> {
        #[cfg(not(feature="pc_test"))]
        let motor_l1 = match robot.motor_l1.take() { Some(v) => v, None => {
            return Err(InitError::MissingDevice(Device::LargeMotor1)); } };
        #[cfg(not(feature="pc_test"))]
        let motor_l2 = match robot.motor_l2.take() { Some(v) => v, None => {
            robot.motor_l1 = Some(motor_l1);
            return Err(InitError::MissingDevice(Device::LargeMotor2)); } };
        #[cfg(not(feature="pc_test"))]
        let motor_med = match robot.motor_med.take() { Some(v) => v, None => {
            robot.motor_l1 = Some(motor_l1);
            robot.motor_l2 = Some(motor_l2);
            return Err(InitError::MissingDevice(Device::MediumMotor)); } };
        #[cfg(not(feature="pc_test"))]
        let sensor_color = match robot.sensor_color.take() { Some(v) => v, None => {
            robot.motor_l1 = Some(motor_l1);
            robot.motor_l2 = Some(motor_l2);
            robot.motor_med = Some(motor_med);
            return Err(InitError::MissingDevice(Device::ColorSensor)); } };
        #[cfg(not(feature="pc_test"))]
        let sensor_gyro = match robot.sensor_gyro.take() { Some(v) => v, None => {
            robot.motor_l1 = Some(motor_l1);
            robot.motor_l2 = Some(motor_l2);
            robot.motor_med = Some(motor_med);
            robot.sensor_color = Some(sensor_color);
            return Err(InitError::MissingDevice(Device::GyroSensor)); } };
        #[cfg(not(feature="pc_test"))] {
            match sensor_color.set_mode_col_reflect() {
                Ok(()) => {},
                Err(_e) => return Err(InitError::MissingDevice(Device::ColorSensor)),
            }
            match motor_med.set_speed_sp(motor_med.get_max_speed().unwrap()) {
                Ok(()) => {},
                Err(_e) => return Err(InitError::MissingDevice(Device::MediumMotor)),
            }
            match motor_l1.run_direct() {
                Ok(()) => {},
                Err(_e) => return Err(InitError::MissingDevice(Device::LargeMotor1)),
            }
            match motor_l2.run_direct() {
                Ok(()) => {},
                Err(_e) => return Err(InitError::MissingDevice(Device::LargeMotor2)),
            }
            match motor_med.set_stop_action("hold") {
                Ok(()) => {},
                Err(e) => { println!("Could not set motor stop action: {e}"); return Err(InitError::MissingDevice(Device::MediumMotor)); }
            }
        }

        let (sender, receiver) = mpsc::channel();

        Ok(Self {
            sender,
            receiver,
            erkennung_sender: None,
            config: robot.config.0.clone(),
            max_number_of_retries_on_communication_failure: robot.max_number_of_retries_on_communication_failure.clone(),
            #[cfg(not(feature="pc_test"))]
            motor_l1,
            #[cfg(not(feature="pc_test"))]
            motor_l2,
            #[cfg(not(feature="pc_test"))]
            motor_med,
            #[cfg(not(feature="pc_test"))]
            sensor_color,
            #[cfg(not(feature="pc_test"))]
            sensor_gyro,
            sleep_duration: Duration::from_millis(4), // max: 250Hz
            old_state: None,
            current_state: LinienfolgerState::Following(Lane::Center), // NOTE: Start lane defined here
            target_lane: Lane::Center,
            target_lane_updated: false,
            #[cfg(feature="pc_test")]
            virtual_robot: robot.pc_test_thread.1.clone(),
        })
    }
    fn clean(self, robot: &mut Robot) {
        #[cfg(not(feature="pc_test"))] {
            robot.motor_l1 = Some(self.motor_l1);
            robot.motor_l2 = Some(self.motor_l2);
            robot.motor_med = Some(self.motor_med);
            robot.sensor_color = Some(self.sensor_color);
        }
    }
}

impl Thread {
    /// Attempts to read a color from the color sensor, breaking if the sensor fails to respond multiple times.
    #[cfg(not(feature="pc_test"))]
    fn read_color_sensor(&self) -> Result<i32, StopReason> {
        let mut retries = 0;
        Ok(loop {
            match self.sensor_color.get_color() { Ok(v) => break v, Err(_) => if retries < self.max_number_of_retries_on_communication_failure.0 {
                retries += 1; thread::sleep(self.max_number_of_retries_on_communication_failure.1);
            } else { return Err(StopReason::DeviceFailedToRespond(Device::ColorSensor)) }, };
        })
    }
    #[cfg(feature="pc_test")]
    fn read_color_sensor(&self) -> Result<i32, StopReason> {
        let (s, r) = std::sync::mpsc::channel();
        self.virtual_robot.send(VirtualRequest::ColorSensorBrightness(s)).expect("Robot channel broke.");
        Ok(r.recv().expect("Answer channel broke."))
    }

    /// (in deg) https://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-jessie/sensor_data.html#lego-ev3-gyro
    #[cfg(not(feature="pc_test"))]
    fn read_gyro_sensor(&self) -> Result<i32, StopReason> {
        let mut retries = 0;
        Ok(loop {
            match self.sensor_gyro.get_angle() { Ok(v) => break v, Err(_) => if retries < self.max_number_of_retries_on_communication_failure.0 {
                retries += 1; thread::sleep(self.max_number_of_retries_on_communication_failure.1);
            } else { return Err(StopReason::DeviceFailedToRespond(Device::GyroSensor)) }, };
        })
    }
    #[cfg(feature="pc_test")]
    fn read_gyro_sensor(&self) -> Result<i32, StopReason> {
        let (s, r) = std::sync::mpsc::channel();
        self.virtual_robot.send(VirtualRequest::GetGyroSensorDirection(s)).expect("Robot channel broke.");
        Ok(r.recv().expect("Answer channel broke."))
    }

    #[cfg(not(feature="pc_test"))]
    fn set_steering_angle(&self, angle_deg: f32) -> Result<(), StopReason> {
        // println!("Steering {}°", angle_deg);
        match self.motor_med.run_to_abs_pos(Some((angle_deg * 12.0).round() as i32)) {
            Ok(_) => Ok(()),
            Err(_) => Err(StopReason::DeviceFailedToRespond(Device::MediumMotor)),
        }
    }
    #[cfg(feature="pc_test")]
    fn set_steering_angle(&self, angle_deg: f32) -> Result<(), StopReason> {
        // println!("Steering {}°", angle_deg);
        self.virtual_robot.send(VirtualRequest::SetTurningAngleDeg(angle_deg)).expect("Robot channel broke.");
        Ok(())
    }

    #[cfg(not(feature="pc_test"))]
    fn set_speed(&self, speed: i32) -> Result<(), StopReason> {
        self.motor_l1.run_direct().ok();
        self.motor_l2.run_direct().ok();
        let (speed1, speed2) = (speed, speed);
        let mut attempts = 0;
        loop {
            match (self.motor_l1.set_duty_cycle_sp(speed1), self.motor_l2.set_duty_cycle_sp(speed2)) {
                (Ok(()), Ok(())) => return Ok(()),
                (Err(_), Ok(())) => loop {
                    if attempts < self.max_number_of_retries_on_communication_failure.0 {
                        attempts += 1;
                        std::thread::sleep(self.max_number_of_retries_on_communication_failure.1);
                        if self.motor_l1.set_duty_cycle_sp(speed1).is_ok() { return Ok(()); }
                    } else {
                        return Err(StopReason::DeviceFailedToRespond(Device::LargeMotor1));
                    }
                },
                (Ok(()), Err(_)) => loop {
                    if attempts < self.max_number_of_retries_on_communication_failure.0 {
                        attempts += 1;
                        std::thread::sleep(self.max_number_of_retries_on_communication_failure.1);
                        if self.motor_l2.set_duty_cycle_sp(speed2).is_ok() { return Ok(()); }
                    } else {
                        return Err(StopReason::DeviceFailedToRespond(Device::LargeMotor2));
                    }
                },
                (Err(_), Err(_)) => {
                    if attempts < self.max_number_of_retries_on_communication_failure.0 {
                        attempts += 1;
                        std::thread::sleep(self.max_number_of_retries_on_communication_failure.1);
                    } else {
                        return Err(StopReason::DeviceFailedToRespond(Device::LargeMotor1)); // TODO: Two-Device-Failure!
                    }
                },
            }
        }
    }
    #[cfg(feature="pc_test")]
    fn set_speed(&self, speed: i32) -> Result<(), StopReason> {
        _ = self.virtual_robot.send(crate::test::virtual_robot::VirtualRequest::SetSpeed(speed));
        Ok(())
    }

    #[cfg(not(feature="pc_test"))]
    fn stop_motors(&self) {
        self.motor_l1.stop().ok();
        self.motor_l2.stop().ok();
        self.motor_med.stop().ok();
    }
    #[cfg(feature="pc_test")]
    fn stop_motors(&self) {
        println!("Stopping motors.");
    }

    fn set_state(&mut self, state: LinienfolgerState) {
        println!("Switching state: {:?}", state);
        self.old_state = Some(std::mem::replace(&mut self.current_state, state));
    }

    fn run_sync(&mut self) -> Result<(), StopReason> {
        // loop { thread::sleep(Duration::from_secs_f64(0.1)); println!("Angle: {}", self.read_gyro_sensor()?); }
        self.stop_motors();
        std::thread::sleep(Duration::from_secs_f64(0.5));
        #[cfg(not(feature="pc_test"))]
        match self.sensor_gyro.set_mode_gyro_cal() { Ok(_) => (), Err(_) => return Err(StopReason::DeviceFailedToRespond(Device::GyroSensor)), };
        #[cfg(not(feature="pc_test"))]
        std::thread::sleep(Duration::from_secs_f64(2.5));
        #[cfg(not(feature="pc_test"))]
        match self.sensor_gyro.set_mode_gyro_ang() { Ok(_) => (), Err(_) => return Err(StopReason::DeviceFailedToRespond(Device::GyroSensor)), };
        std::thread::sleep(Duration::from_secs_f64(0.25));
        let mut right_side_of_line_mode = match {
            let max_angle_deg = 42.5;
            thread::sleep(Duration::from_secs_f64(0.3));
            self.set_steering_angle(-max_angle_deg)?;
            thread::sleep(Duration::from_secs_f64(1.0));
            self.set_steering_angle(max_angle_deg)?;
            thread::sleep(Duration::from_secs_f64(1.0));
            self.set_steering_angle(0.0)?;
            thread::sleep(Duration::from_secs_f64(1.0));
            let mut detected_dirs = [None, None]; // false -> this direction = darker, true -> brighter
            for (i, angles) in [&[-10.0, -20.0, -30.0, -40.0], &[10.0, 20.0, 30.0, 40.0]].into_iter().enumerate() {
                let center_brightness = self.read_color_sensor()?;
                let mut prev_brightness = center_brightness;
                for angle in angles {
                    self.set_steering_angle(*angle)?;
                    thread::sleep(Duration::from_secs_f64(0.2));
                    let brightness = self.read_color_sensor()?;
                    if (brightness > prev_brightness && prev_brightness < center_brightness)
                    || (brightness < prev_brightness && prev_brightness > center_brightness) { // direction reversed => we can exit the loop early
                        break;
                    }
                    prev_brightness = brightness;
                }
                if detected_dirs[i].is_none() {
                    if prev_brightness < center_brightness {
                        detected_dirs[i] = Some(false);
                    } else if prev_brightness > center_brightness {
                        detected_dirs[i] = Some(true);
                    }
                }
                self.set_steering_angle(0.0)?;
                thread::sleep(Duration::from_secs_f64(0.5));
            }
            match detected_dirs { // false = left, true = right of line
                [None, None] => None,
                [None, Some(false)] => Some(false),
                [None, Some(true)] => Some(true),
                [Some(false), None] => Some(true),
                [Some(true), None] => Some(false),
                [Some(true), Some(false)] => Some(false),
                [Some(false), Some(true)] => Some(true),
                [Some(true), Some(true)] => None,
                [Some(false), Some(false)] => None,
            }
        } { None => { println!("Couldn't determine which side of the line I am on, using default value."); true }, Some(v) => {println!("It seems I am on the {} side of the line.", if v { "right" } else { "left" }); v } };

        // VARS

        // Following
        let mut previous_brightness = 50;
        let mut avg_change = 0.0;

        // Switching
        let mut switching_start_time: Option<std::time::Instant> = None;
        // If this is true, switching_start_time (or motor rotations) will be used by the line follower to limit how far the steering will go.
        let mut limit_angle = false;
        let mut switching_start_angle = 0.0;
        let mut switching_state = 0; // TODO?: Enum

        // START

        self.set_speed(self.config.default_speed_in_percent)?;

        loop {
            while let Ok(recv) = self.receiver.try_recv() {
                match recv {
                    Task::Stop => {
                        self.stop_motors();
                        self.set_steering_angle(0.0)?;
                        return Err(StopReason::RequestedStop);
                    },
                    Task::StopNoChange => {
                        return Err(StopReason::RequestedStopNoChange);
                    },
                    Task::SwitchToLane(lane) => {
                        let old_lane = std::mem::replace(&mut self.target_lane, lane.clone());
                        self.target_lane_updated = true; // TODO: is self.target_lane even necessary? It's part of Following(..) and Switching(..)!
                        self.set_state(LinienfolgerState::Switching(old_lane, lane, 0.0));
                    },
                    Task::GetLane(sender) => _ = sender.send(self.target_lane.clone()),
                    Task::GetState(sender) => _ = sender.send(self.current_state.clone()),
                    Task::SetErkennungSender(s) => self.erkennung_sender = s,
                }
            }
            let mut new_state = None;
            match &self.current_state { // ~NOTE~ This is where stuff happens ~NOTE~
                LinienfolgerState::Stopped => return Err(StopReason::GracefullyStoppedForInternalReasons()),
                LinienfolgerState::Following(_lane) => {
                    // Linienverfolgung
                    {
                        let detected_brightness = self.read_color_sensor()?;
                        avg_change = avg_change * 0.7 + 0.3 * ((detected_brightness - previous_brightness) as f32);
                        let expected_brightness_soon = (detected_brightness + previous_brightness) as f32 / 2.0 + (self.config.importance_of_brightness_change + 0.5 /* because we average with previous_brightness */) * avg_change;
                        let brightness_diff = expected_brightness_soon - 50.0;
                        let max_angle = if limit_angle {
                            if let Some(time) = switching_start_time {
                                let el = time.elapsed().as_secs_f32();
                                if el > 0.5 { limit_angle = false; println!("[following] angle limit removed - time"); }
                                10.0
                            } else {
                                // TODO!
                                if self.motor_l1.get_position().unwrap() as f32 / self.motor_l1.get_count_per_rot().unwrap() as f32 > 0.8 {
                                    limit_angle = false;
                                    println!("[following] angle limit removed - motor rotations");
                                }
                                10.0
                            }
                        } else {
                            30.0
                        };
                        let steer = brightness_diff.min(max_angle).max(-max_angle) as f32;
                        self.set_steering_angle(if right_side_of_line_mode { -steer } else { steer })?;
                        previous_brightness = detected_brightness;
                    }
                    // {
                    //     let detected_color = self.read_color_sensor()?;
                    //     // println!("[Color Sensor] read {}.", detected_color);
                    //     // println!("Change: {}", avg_change);
                    // 
                    //     // Bright -> Away
                    //     // Dark -> Line
                    //     avg_change = avg_change * 0.8 + 0.2 * ((detected_color - previous_brightness) as f32 / 10.0).max(-1.0).min(1.0);
                    //     let brightness_diff = detected_color - 50;
                    //     let turn_factor = (brightness_diff as f32 / 40.0).max(-1.0).min(1.0);
                    //     // POSITIVE -> Steer normal, NEGATIVE -> Steer away from line
                    //     let change_factor = if avg_change >= 0.0 && turn_factor >= 0.0 {
                    //         if avg_change > 0.4 {
                    //             println!("going the wrong way... (line :: {:.2})", avg_change);
                    //         }
                    //         1.0 + 12.0 * avg_change * avg_change
                    //     } else if avg_change >= 0.0 {
                    //         0.5 - 1.5 * avg_change.abs()
                    //     } else if turn_factor >= 0.0 {
                    //         0.5 - 1.5 * avg_change.abs()
                    //     } else {
                    //         if -avg_change > 0.4 {
                    //             println!("going the wrong way... (line :: {:.2})", -avg_change);
                    //         }
                    //         1.0 + 12.0 * avg_change * avg_change
                    //     };
                    //     // println!("Factor: {:.2} | Turn: {:.2}", change_factor, turn_factor);
                    //     let angle = (change_factor * turn_factor * 20.0).max(-40.0).min(40.0);
                    //     self.set_steering_angle(if right_side_of_line_mode { -angle } else { angle })?;
                    //     previous_brightness = detected_color;
                    // }
                },
                LinienfolgerState::Switching(prev_lane, target_lane, _max_distance) => {
                    if let Some(_old_state) = self.old_state.take() {
                        if let Ok(_) = self.motor_l1.set_position(0) {
                            switching_start_time = None;
                        } else {
                            switching_start_time = Some(std::time::Instant::now());
                        }
                        switching_state = 0;
                        // begin
                        switching_start_angle = self.config.angle_lane_change_start *
                            match (prev_lane, target_lane) {
                                (Lane::Left, Lane::Left) => 0.0,
                                (Lane::Left, Lane::Center) => 1.0,
                                (Lane::Left, Lane::Right) => 1.0,
                                (Lane::Center, Lane::Left) => -1.0,
                                (Lane::Center, Lane::Center) => 0.0,
                                (Lane::Center, Lane::Right) => 1.0,
                                (Lane::Right, Lane::Left) => -1.0,
                                (Lane::Right, Lane::Center) => -1.0,
                                (Lane::Right, Lane::Right) => 0.0,
                            }
                        ;
                        if switching_start_angle == 0.0 {
                            new_state = Some(LinienfolgerState::Following(target_lane.clone()));
                        } else {
                            self.set_steering_angle(switching_start_angle.min(30.0).max(-30.0))?;
                        }
                    }
                    // println!("switching_state: {}", switching_state);
                    let steer = match &switching_state {
                        0 => {
                            //                    TODO/NOTE: This value depends on the robot's speed!
                            let elapsed = if let Some(time) = switching_start_time {
                                let elapsed_seconds = time.elapsed().as_secs_f32();
                                elapsed_seconds / self.config.seconds_to_straighten_out
                            } else {
                                // TODO: Change unwrap()s!
                                let rotations = self.motor_l1.get_position().unwrap() as f32 / self.motor_l1.get_count_per_rot().unwrap() as f32;
                                rotations / self.config.rotations_to_straighten_out
                            }.min(1.0);
                            if elapsed >= 1.0 {
                                self.set_steering_angle(0.0)?;
                                if let Some(_) = switching_start_time {
                                    println!("[lane switch] switch init ended: time");
                                } else {
                                    println!("[lane switch] switch init ended: motor rotations");
                                }
                                switching_state += 1;
                                Some(self.config.angle_lane_change_final.copysign(switching_start_angle))
                            } else {
                                Some(
                                    (switching_start_angle * (1.0 - elapsed)
                                    + elapsed * self.config.angle_lane_change_final.copysign(switching_start_angle))
                                    // .min(30.0).max(-30.0) /* since this isn't directly influencing the steering angle, we don't necessarily need this (although it might be useful) */
                                    // NOTE: I REMOVED THIS WITHOUT TESTING
                                )
                            }
                        },
                        1 => {
                            let brightness = self.read_color_sensor()?;
                            if brightness <= self.config.brightness_entering_black_line { // encountered a black line
                                if brightness <= self.config.brightness_on_black_line {
                                    println!("[lane switch] found line");
                                    switching_state += 1;
                                }
                                Some(self.config.angle_lane_change_encounter.copysign(switching_start_angle))
                            } else {
                                Some(self.config.angle_lane_change_final.copysign(switching_start_angle))
                            }
                        },
                        2 => {
                            if self.read_color_sensor()? >= self.config.brightness_after_black_line { // slowly leaving black line again
                                right_side_of_line_mode = switching_start_angle.is_sign_positive();
                                println!("[lane switch] leaving line");
                                switching_state += 1;
                                Some(0.0)
                            } else {
                                Some(self.config.angle_lane_change_encounter.copysign(switching_start_angle))
                            }
                        },
                        _ => {
                            new_state = Some(LinienfolgerState::Following(target_lane.clone()));
                            limit_angle = true;
                            None
                        },
                    };
                    match steer {
                        Some(dir) => {
                            let rotation_deg = self.read_gyro_sensor()? as f32;
                            println!("{} / {:.1}", rotation_deg, dir);
                            let rotation_off = dir - rotation_deg;
                            println!("-> {:.1}", rotation_off);
                            self.set_steering_angle((2.0 * rotation_off).max(-30.0).min(30.0))?;
                        },
                        None => {},
                    }
                },
            }
            if let Some(new_state) = new_state { self.set_state(new_state); }

            thread::sleep(self.sleep_duration);
        };
    }
}

impl super::ThreadedRun<(mpsc::Sender<Task>, std::thread::JoinHandle<(Self, StopReason)>)> for Thread {
    fn run(self) -> (mpsc::Sender<Task>, std::thread::JoinHandle<(Self, StopReason)>) {
        (self.sender.clone(), self.run())
    }
}
impl super::ThreadedRun<std::thread::JoinHandle<(Self, StopReason)>> for Thread {
    fn run(mut self) -> std::thread::JoinHandle<(Self, StopReason)> {
        std::thread::spawn(move || {
            let run = self.run_sync().unwrap_err();
            match &run {
                StopReason::RequestedStop |
                StopReason::RequestedStopNoChange |
                StopReason::GracefullyStoppedForInternalReasons() => {},
                StopReason::DeviceFailedToRespond(..) |
                StopReason::DeviceFailedToRespondWithError(..) |
                StopReason::DeviceFailedToRespondWithErrors(..) => self.stop_motors(),
            }
            (self, run)
        })
    }
}

pub enum Task {
    /// Stops the robot, then returns.
    Stop,
    /// The thread will return almost immedeately, but the motors will not be stopped. This is not recommended.
    StopNoChange,
    SwitchToLane(Lane),
    GetLane(std::sync::mpsc::Sender<Lane>),
    GetState(std::sync::mpsc::Sender<LinienfolgerState>),
    SetErkennungSender(Option<mpsc::Sender<super::erkennung::Task>>),
}

#[derive(std::fmt::Debug)]
pub enum StopReason {
    RequestedStop,
    RequestedStopNoChange,
    GracefullyStoppedForInternalReasons(),
    DeviceFailedToRespond(Device),
    DeviceFailedToRespondWithError(Device, ev3dev_lang_rust::Ev3Error),
    DeviceFailedToRespondWithErrors(Device, Vec<ev3dev_lang_rust::Ev3Error>),
}

#[derive(std::fmt::Debug)]
pub enum InitError {
    /// Threads "reserve" devices by .take()ing them from the Option<_>s that store them. If .take() is called on the None variant, this error is thrown. This can happen when starting a thread which wants to take() a certain device after another thread has already taken it.
    MissingDevice(Device),
}

#[derive(std::fmt::Debug)]
pub enum Device {
    LargeMotor1,
    LargeMotor2,
    MediumMotor,
    ColorSensor,
    GyroSensor,
}

#[derive(std::fmt::Debug)]
#[derive(Clone)]
pub enum Lane {
    Left,
    Center,
    Right,
}

#[derive(Clone, std::fmt::Debug)]
pub enum LinienfolgerState {
    Stopped,
    Following(Lane),
    /// Switching from [one lane |1] to [another |2], moving at most [3]m forward (if possible).
    Switching(Lane, Lane, f64),
}
